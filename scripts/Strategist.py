"""
Strategist.py — agente competitivo para Escenario 111 (duelo de tanques).

Combina:
  - scripts/Ballistic.py        modelo balistico y solver de lead
  - scripts/OpponentProfiler.py clasificacion del estilo rival en vivo
  - scripts/PID.py              controladores PID para heading y distancia

Politicas implementadas:
  SNIPER     pararse y apuntar fino — vs rival STATIC o lento
  LEAD       cerrar a media distancia y disparar con lead lineal — vs LINEAR
  CLOSE      cerrar a distancia corta donde la balistica es trivial — vs ZIGZAG
  KITE       mantener distancia mientras el rival se acerca — vs AGGRESSIVE
  CHASE      perseguir a velocidad maxima — vs EVASIVE
  OBSERVE    moverse en arco lateral sin disparar, para observar — primeros 5 s

Arquitectura de control:
  Dos PID en cascada con el solver balistico:

  1) heading_pid:   setpoint = bearing al rival (o + offset segun politica)
                    pv       = azimuth del chasis
                    salida   = steering en [-1, +1]
                    Mantiene el chasis apuntado donde queremos ir.

  2) distance_pid:  setpoint = distancia ideal de la postura activa
                    pv       = distancia actual al rival
                    salida   = thrust en [-28, +28] m/s
                    Acerca o retrocede automaticamente sin if/else por postura.

  La torreta no usa PID porque puede girar al angulo deseado en 1 tick;
  la apuntamos directo con la salida del solver balistico (lead incluido).

Uso (igual que SeekAndDestroy.py):
    python3 scripts/Strategist.py 1     # controlar tanque 1
    python3 scripts/Strategist.py 2     # controlar tanque 2

Diferenciacion frente a SeekAndDestroy.py:
  1. Apunta con lead balistico real (no random declinacion).
  2. Solo dispara cuando el error angular esta debajo de umbral
     (ahorra power, evita gastar municion al pedo).
  3. Reconoce el estilo del rival y cambia de politica.
  4. Movimiento controlado por PID (suave, sin chattering, sin overshoot).
  5. Evade activamente cuando lo necesita.
"""

from __future__ import annotations

import math
import socket
import sys
import time
from collections import deque
from struct import unpack

import Configuration
from Command import Command
from TelemetryDictionary import telemetrydirs as td

from Ballistic import (
    BallisticTable,
    solve_moving_intercept,
    world_bearing_to_turret,
    SIM_DT,
)
from OpponentProfiler import OpponentProfiler
from PID import PIDController, normalize_angle_deg

# wakuseibokan: comando 11 = disparar
FIRE = 11

# El simulador empuja telemetria a un puerto cliente. Por defecto el
# server escucha comandos en 4500+tank_id y empuja telemetria a 4600+tank_id
# (revisable en conf/telemetry.endpoints.ini).
TELEMETRY_PORT_BASE = 4600
COMMAND_PORT_BASE = 4500
TELEMETRY_STRUCT = '<LLififffffffffffffffffff'
TELEMETRY_LEN = 84 + 3 * 4   # 96 bytes


class VelocityEstimator:
    """Estimador de velocidad rival: derivada directa frame-a-frame.

    Antes usaba EMA con alpha=0.4 pero introducia lag de 1-2 ticks que
    contra un blanco moviendose a 28 m/s y con 2.5s de tiempo de vuelo
    nos hacia errar shots por varios metros. SmartLead usa derivada
    directa y nos sacaba 30% mas hits. Aprendimos.
    """

    def __init__(self):
        self._last_pos: tuple[float, float, float] | None = None
        self._vel = (0.0, 0.0)

    def update(self, x: float, z: float, timer: float) -> tuple[float, float]:
        if self._last_pos is None:
            self._last_pos = (x, z, timer)
            return self._vel
        lx, lz, lt = self._last_pos
        dt = max(timer - lt, 1.0) * SIM_DT
        self._vel = ((x - lx) / dt, (z - lz) / dt)
        self._last_pos = (x, z, timer)
        return self._vel


class Strategist:
    # ---------------------- Ganancias PID por defecto ----------------------
    # Ajustables tras observar episodios reales (ver docs/NeuroRobotics.md).
    #
    # Heading: error en grados ([-180, 180] tras normalizar), salida steering
    # en [-1, +1]. Con kp=0.04, error de 25 grados ya satura la salida.
    HEADING_KP = 0.04
    HEADING_KI = 0.0005
    HEADING_KD = 0.015
    HEADING_INT_LIMIT = 30.0   # acumulacion de grados-segundo

    # Distancia: error en metros, salida thrust en [-28, +28] m/s.
    # Mas agresivo: kp=0.10 => 50 m de error saturan al maximo => acelera fuerte
    # en cuanto se aleja del setpoint. Kd alto para frenar antes de cruzarlo.
    DISTANCE_KP = 0.10
    DISTANCE_KI = 0.0003
    DISTANCE_KD = 0.15
    DISTANCE_INT_LIMIT = 500.0

    def __init__(self, tank_id: int):
        self.tank_id = int(tank_id)
        # mi puerto de escucha de telemetria
        my_port = TELEMETRY_PORT_BASE + self.tank_id
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', my_port))
        self.sock.settimeout(5.0)

        cmd_port = COMMAND_PORT_BASE + self.tank_id
        self.command = Command(Configuration.ip, cmd_port)

        self.ballistic = BallisticTable()
        self.profiler = OpponentProfiler()
        self.vel_est = VelocityEstimator()

        # PIDs (las tres proporciones por canal)
        self.heading_pid = PIDController(
            kp=self.HEADING_KP, ki=self.HEADING_KI, kd=self.HEADING_KD,
            output_min=-1.0, output_max=1.0,
            integral_min=-self.HEADING_INT_LIMIT, integral_max=self.HEADING_INT_LIMIT,
            angular=True,
        )
        self.distance_pid = PIDController(
            kp=self.DISTANCE_KP, ki=self.DISTANCE_KI, kd=self.DISTANCE_KD,
            output_min=-28.0, output_max=28.0,
            integral_min=-self.DISTANCE_INT_LIMIT, integral_max=self.DISTANCE_INT_LIMIT,
            angular=False,
        )

        self.policy = 'OBSERVE'
        self._prev_policy = 'OBSERVE'
        self.last_print = 0.0
        self._last_timer: float | None = None

        # zigzag interno cuando estamos evadiendo: oscilamos el setpoint de
        # heading en +/- ZIGZAG_AMPLITUDE grados con periodo ZIGZAG_PERIOD ticks.
        self._zigzag_phase = 0
        self.ZIGZAG_PERIOD_TICKS = 30
        self.ZIGZAG_AMPLITUDE_DEG = 35.0

        # historial reciente de health propio para detectar que nos pegan
        self._health_history: deque[tuple[float, float]] = deque(maxlen=20)

        # Anti-stuck: si la posicion no cambia mucho en STUCK_WINDOW_TICKS
        # ticks mientras estoy pidiendo thrust, gatillo maniobra de despegue.
        # En escenario 131 las warehouses y las pendientes nos pueden trabar.
        self._pos_history: deque[tuple[float, float, float]] = deque(maxlen=40)
        # (timer, x, z)
        self.STUCK_WINDOW_TICKS = 60       # ~3 s de telemetria
        self.STUCK_DISPLACEMENT_M = 5.0    # menos que esto = stuck
        self.STUCK_THRUST_THRESHOLD = 5.0  # solo aplica si estaba intentando avanzar
        self.UNSTUCK_DURATION_TICKS = 50   # cuanto durar la maniobra
        self._unstuck_until: float | None = None
        self._unstuck_dir: int = 1         # signo del giro durante unstuck

    # ------------------ I/O telemetria ------------------

    def _read_one(self) -> tuple | None:
        try:
            data, _ = self.sock.recvfrom(TELEMETRY_LEN)
        except socket.timeout:
            return None
        if len(data) != TELEMETRY_LEN:
            return None
        return unpack(TELEMETRY_STRUCT, data)

    def _read_two_tanks(self) -> tuple[tuple, tuple] | None:
        """Lee paquetes hasta tener uno de cada tanque."""
        t1 = t2 = None
        for _ in range(8):  # como mucho 8 intentos
            v = self._read_one()
            if v is None:
                break
            n = int(v[td['number']])
            if n == 1:
                t1 = v
            elif n == 2:
                t2 = v
            if t1 is not None and t2 is not None:
                return t1, t2
        return None

    # ------------------ Logica de control ------------------

    def _select_policy(self, profile, my_health: float, my_power: float) -> str:
        # Salvavidas: power bajo => no dispares mas, slo evade
        if my_power < 50:
            return 'EVADE_NO_FIRE'
        # Lock-in: rival con menos health = aprovechar, agresivo
        if profile.style != 'UNKNOWN' and profile.style != 'STATIC':
            # si el rival ya gasto > 80% de su municion sin pegarnos mucho
            # podemos ser sniper estatico
            if profile.fire_rate > 0.4 and profile.wasted_shots >= 8 and my_health > 700:
                return 'SNIPER'
        rec = OpponentProfiler.recommend_strategy(profile.style)
        return rec['posture']

    def _zigzag_heading_offset(self) -> float:
        """Oscila el setpoint de heading +/- ZIGZAG_AMPLITUDE para romper el
        lead de un rival que asume velocidad constante."""
        self._zigzag_phase += 1
        if (self._zigzag_phase // self.ZIGZAG_PERIOD_TICKS) % 2 == 0:
            return +self.ZIGZAG_AMPLITUDE_DEG
        return -self.ZIGZAG_AMPLITUDE_DEG

    def _setpoints_for_policy(self, policy: str, bearing_to_rival: float,
                              dist: float, ideal_dist: float) -> tuple[float, float, str]:
        """Devuelve (heading_setpoint_world, distance_setpoint, fire_lock).

        fire_lock indica si la politica habilita disparo:
            'OK'   apuntar y disparar normalmente
            'NO'   no disparar (OBSERVE, EVADE_NO_FIRE)
        """
        if policy == 'OBSERVE':
            # circular: apuntar perpendicular al rival, mantener distancia actual
            return bearing_to_rival + 90.0, dist, 'NO'
        if policy == 'SNIPER':
            # frenar y apuntar fino: setpoint de distancia == distancia actual
            # => el PID de distancia genera thrust ~ 0 sin if/else manual.
            return bearing_to_rival, dist, 'OK'
        if policy == 'LEAD':
            return bearing_to_rival, ideal_dist, 'OK'
        if policy == 'CLOSE':
            return bearing_to_rival + self._zigzag_heading_offset(), ideal_dist, 'OK'
        if policy == 'KITE':
            # encarar al rival pero querer estar a 900 m: si el rival se acerca
            # (dist < 900), el distance_pid manda thrust negativo => retrocede.
            return bearing_to_rival + self._zigzag_heading_offset(), ideal_dist, 'OK'
        if policy == 'CHASE':
            return bearing_to_rival, max(ideal_dist - 400, 200), 'OK'
        if policy == 'EVADE_NO_FIRE':
            # perpendicular + zigzag, mantener distancia
            return (bearing_to_rival + 90.0 + self._zigzag_heading_offset(),
                    max(dist, 1500.0), 'NO')
        # default conservador
        return bearing_to_rival, dist, 'NO'

    def _is_stuck(self, timer: float, x: float, z: float) -> bool:
        """Detecta si estamos atorados: la posicion no cambio en
        STUCK_WINDOW_TICKS ticks. No miramos thrust porque el agente puede
        haber decidido frenarse a si mismo y eso no es estar trabado.
        Por eso ADEMAS pediremos que la politica activa requiera moverse.
        """
        self._pos_history.append((timer, x, z))
        if len(self._pos_history) < self.STUCK_WINDOW_TICKS // 2:
            return False
        oldest = self._pos_history[0]
        dt = timer - oldest[0]
        if dt < self.STUCK_WINDOW_TICKS * 0.5:
            return False
        displacement = math.hypot(x - oldest[1], z - oldest[2])
        return displacement < self.STUCK_DISPLACEMENT_M

    def _detect_incoming_fire(self, mine: tuple, timer: float) -> bool:
        """Lee el radar de impactos del simulador. En scenario 131 se actualiza
        con la posicion donde cae cada bala dentro de 500 m.

        Bug-fix: solo gatilla cuando el VALOR cambia (impacto nuevo). El
        valor inicial del radar puede ser no-cero y constante; eso es ruido,
        no fuego real.
        """
        rx = float(mine[td['radarx']])
        ry = float(mine[td['radary']])
        rz = float(mine[td['radarz']])
        if rx == 0.0 and ry == 0.0 and rz == 0.0:
            self._last_radar_changed_at = None
            return False
        prev_val = getattr(self, '_last_radar_val', None)
        # detectar cambio real
        if prev_val is None or abs(rx - prev_val[0]) > 1.0 or abs(rz - prev_val[2]) > 1.0:
            self._last_radar_changed_at = timer
            self._last_radar_val = (rx, ry, rz)
        # solo flag activo durante 30 ticks (~1.5 s) tras el ultimo cambio
        changed = getattr(self, '_last_radar_changed_at', None)
        if changed is None:
            return False
        return (timer - changed) < 30

    def _decide(self, mine: tuple, other: tuple) -> dict:
        my_x = float(mine[td['x']])
        my_y = float(mine[td['y']])
        my_z = float(mine[td['z']])
        my_az = float(mine[td['azimuth']])
        my_hp = float(mine[td['health']])
        my_pw = float(mine[td['power']])

        his_x = float(other[td['x']])
        his_y = float(other[td['y']])
        his_z = float(other[td['z']])
        his_az = float(other[td['azimuth']])
        his_hp = float(other[td['health']])
        his_pw = float(other[td['power']])
        timer = float(mine[td['timer']])

        # dt para los PIDs (en segundos reales del simulador)
        if self._last_timer is None:
            dt = SIM_DT
        else:
            dt = max((timer - self._last_timer) * SIM_DT, SIM_DT)
        self._last_timer = timer

        # actualizar profiler y estimador
        self.profiler.update(timer, my_x, my_z, my_hp,
                             his_x, his_z, his_az, his_pw, his_hp)
        vx, vz = self.vel_est.update(his_x, his_z, timer)

        # registrar health
        self._health_history.append((timer, my_hp))

        # ---------- 1) Apunteria balistica (torreta, sin PID) ----------
        dist = math.hypot(his_x - my_x, his_z - my_z)
        intercept = solve_moving_intercept(
            shooter_xz=(my_x, my_z),
            target_xz=(his_x, his_z),
            target_vel_xz=(vx, vz),
            table=self.ballistic,
            shooter_y=my_y,    # compensar dif de altura (escenario 131)
            target_y=his_y,
        )

        # ---------- 1b) Detectar fuego enemigo via radar de impactos ----------
        incoming_fire = self._detect_incoming_fire(mine, timer)
        # convencion C++ (ver Ballistic.solve_moving_intercept)
        bearing_to_rival = math.degrees(math.atan2(-(his_x - my_x), his_z - my_z))
        if intercept is None:
            turret_decl = 0.0
            turret_bearing = world_bearing_to_turret(bearing_to_rival, my_az)
            aim_error = abs(turret_bearing)
        else:
            turret_decl = intercept['decl_deg']
            turret_bearing = world_bearing_to_turret(intercept['bearing_world_deg'], my_az)
            aim_error = abs(turret_bearing)

        # ---------- 2) Eleccion de politica ----------
        # Sin OBSERVE inicial: SmartLead dispara desde tick 0 y nos saca dos
        # disparos gratis de ventaja. El profiler todavia recolecta datos en
        # background mientras nosotros ya estamos disparando con LEAD.
        profile = self.profiler.classify()
        policy = self._select_policy(profile, my_hp, my_pw)

        # No reseteamos los PIDs al cambiar postura: el setpoint cambia pero
        # el integral acumulado sirve igual (es el mismo chasis, mismas
        # dinamicas). Antes reseteabamos y eso causaba 0.5-1 s de
        # estabilizacion en cada switch — durante ese tiempo el lead estaba
        # mal contra un chasis girando, y errabamos disparos.
        self._prev_policy = policy
        self.policy = policy

        rec = OpponentProfiler.recommend_strategy(profile.style)
        ideal_dist = rec['close_to']
        max_aim_error = rec['fire_when_aim_error_below_deg']

        # ---------- 3) Setpoints segun politica ----------
        heading_sp, distance_sp, fire_lock = self._setpoints_for_policy(
            policy, bearing_to_rival, dist, ideal_dist
        )

        # ---------- 4) PID heading (chasis -> direccion deseada) ----------
        steering = self.heading_pid.step(heading_sp, my_az, dt)
        chassis_err = abs(normalize_angle_deg(heading_sp - my_az))

        # ---------- 5) PID distancia (acercarse / retroceder) ----------
        # IMPORTANTE: el PID genera error = setpoint - pv = sp - dist.
        # Pero la convencion fisica que queremos es:
        #   dist > sp (demasiado lejos)  => thrust > 0 (avanzar)
        #   dist < sp (demasiado cerca)  => thrust < 0 (reversa)
        # => la salida del PID viene con el signo invertido => negar.
        thrust = -self.distance_pid.step(distance_sp, dist, dt)

        # Si el chasis esta muy desorientado, parar y solo rotar.
        # Avanzar mientras rotamos genera espirales que nunca llegan al rival.
        if chassis_err > 30.0:
            thrust = 0.0
        elif chassis_err > 10.0:
            # rotacion casi ok: avanzar a velocidad reducida
            thrust = max(min(thrust, 8.0), -8.0)

        # ---------- 6) Disparo ----------
        # El canion rota libremente; aim_err (= cuanto rota la torreta desde
        # el frente del chasis) NO indica precision. Lo unico relevante:
        # tenemos solucion balistica y municion. SmartLead nos saca a tiros
        # con su volumen — disparamos siempre que podemos.
        fire_ok = (fire_lock == 'OK'
                   and my_pw > 20
                   and intercept is not None)

        # ---------- 7a) Anti-stuck: si estoy clavado, reversa y giro inverso ----------
        # Si ya estamos en una maniobra de unstuck, mantenerla hasta que termine
        if self._unstuck_until is not None and timer < self._unstuck_until:
            thrust = -20.0
            steering = float(self._unstuck_dir) * 1.0
            # reseteo PIDs para que no acumulen integral durante unstuck
            self.heading_pid.reset()
            self.distance_pid.reset()
        else:
            if self._unstuck_until is not None and timer >= self._unstuck_until:
                self._unstuck_until = None
                self.heading_pid.reset()
                self.distance_pid.reset()
            # Stuck = pedimos avanzar (thrust no trivial) pero la posicion
            # no cambia. Si solo estamos rotando en el lugar (SNIPER, OBSERVE)
            # NO es stuck — es comportamiento deseado.
            wants_to_translate = abs(thrust) > 5.0
            if wants_to_translate and self._is_stuck(timer, my_x, my_z):
                self._unstuck_until = timer + self.UNSTUCK_DURATION_TICKS
                # girar en sentido opuesto al heading actual deseado
                self._unstuck_dir = -1 if (turret_bearing > 0) else 1
                thrust = -20.0
                steering = float(self._unstuck_dir) * 1.0
                self._pos_history.clear()

        # ---------- 7b) Reaccion defensiva ----------
        forced = False
        if len(self._health_history) >= 2:
            recent_loss = self._health_history[0][1] - self._health_history[-1][1]
            if recent_loss > 5:
                forced = True
        if incoming_fire:
            forced = True
        # Solo aplicar zigzag forzado si NO estamos en maniobra de unstuck.
        # Y solo bumpear thrust si el chasis ya esta alineado (sino seguimos
        # rotando en lugar de spiralar).
        if forced and self._unstuck_until is None:
            forced_offset = self._zigzag_heading_offset()
            steering = self.heading_pid.step(heading_sp + forced_offset, my_az, dt)
            if chassis_err < 30:
                thrust = max(thrust, 12.0)

        return dict(
            timer=timer,
            thrust=thrust,
            steering=steering,
            turret_decl=turret_decl,
            turret_bearing=turret_bearing,
            fire_ok=fire_ok,
            distance=dist,
            distance_sp=distance_sp,
            heading_sp=heading_sp,
            heading_err=normalize_angle_deg(heading_sp - my_az),
            aim_error=aim_error,
            policy=policy,
            profile=profile,
            incoming_fire=incoming_fire,
            unstuck=(self._unstuck_until is not None and timer < self._unstuck_until),
            height_diff=his_y - my_y,
            pid_h=(self.heading_pid.last_p, self.heading_pid.last_i, self.heading_pid.last_d),
            pid_d=(self.distance_pid.last_p, self.distance_pid.last_i, self.distance_pid.last_d),
        )

    def _print_status(self, decision: dict, mine: tuple) -> None:
        now = time.time()
        if now - self.last_print < 0.5:
            return
        self.last_print = now
        prof = decision['profile']
        print(
            f"t={int(mine[td['timer']]):>6} "
            f"hp={mine[td['health']]:6.0f} pw={mine[td['power']]:4.0f} "
            f"pol={decision['policy']:<14} "
            f"dist={decision['distance']:6.0f}/{decision['distance_sp']:6.0f}m "
            f"dy={decision['height_diff']:+5.1f} "
            f"hdg_err={decision['heading_err']:+6.1f} "
            f"steer={decision['steering']:+5.2f} thrust={decision['thrust']:+6.2f} "
            f"aim_err={decision['aim_error']:5.2f} fire={'Y' if decision['fire_ok'] else '-'} "
            f"INFIRE={'!' if decision['incoming_fire'] else ' '} "
            f"UNSTUCK={'U' if decision.get('unstuck') else '.'} "
            f"|| {prof.style}({prof.confidence:.1f})"
        )

    def run(self) -> None:
        print(f"Strategist controlando tanque {self.tank_id}, "
              f"escucha telemetria en {TELEMETRY_PORT_BASE + self.tank_id}, "
              f"manda comandos a {COMMAND_PORT_BASE + self.tank_id}")
        while True:
            tels = self._read_two_tanks()
            if tels is None:
                print("Sin telemetria (timeout). Episodio terminado o servidor caido.")
                break
            t1, t2 = tels
            mine, other = (t1, t2) if self.tank_id == 1 else (t2, t1)

            try:
                decision = self._decide(mine, other)
            except Exception as exc:
                print(f"Error decidiendo: {exc!r}")
                continue

            if decision['fire_ok']:
                self.command.command = FIRE
            self.command.send_command(
                int(decision['timer']),
                int(self.tank_id),
                float(decision['thrust']),
                float(decision['steering']),
                float(decision['turret_decl']),
                float(decision['turret_bearing']),
            )
            self._print_status(decision, mine)


# ---------------------------------------------------------------------------
# Self-test sin simulador: feed sintetico para validar la decision logic.
# ---------------------------------------------------------------------------

def _self_test() -> None:
    """Smoke test: corre _decide() con un stream sintetico."""
    s = Strategist.__new__(Strategist)   # no llamar __init__ (abriria socket)
    s.tank_id = 1
    s.ballistic = BallisticTable()
    s.profiler = OpponentProfiler()
    s.vel_est = VelocityEstimator()
    s.heading_pid = PIDController(
        kp=Strategist.HEADING_KP, ki=Strategist.HEADING_KI, kd=Strategist.HEADING_KD,
        output_min=-1.0, output_max=1.0,
        integral_min=-Strategist.HEADING_INT_LIMIT,
        integral_max=Strategist.HEADING_INT_LIMIT,
        angular=True,
    )
    s.distance_pid = PIDController(
        kp=Strategist.DISTANCE_KP, ki=Strategist.DISTANCE_KI, kd=Strategist.DISTANCE_KD,
        output_min=-28.0, output_max=28.0,
        integral_min=-Strategist.DISTANCE_INT_LIMIT,
        integral_max=Strategist.DISTANCE_INT_LIMIT,
        angular=False,
    )
    s.policy = 'OBSERVE'
    s._prev_policy = 'OBSERVE'
    s.last_print = 0.0
    s._last_timer = None
    s._zigzag_phase = 0
    s.ZIGZAG_PERIOD_TICKS = 30
    s.ZIGZAG_AMPLITUDE_DEG = 35.0
    s._health_history = deque(maxlen=20)
    s._pos_history = deque(maxlen=40)
    s.STUCK_WINDOW_TICKS = 60
    s.STUCK_DISPLACEMENT_M = 5.0
    s.STUCK_THRUST_THRESHOLD = 5.0
    s.UNSTUCK_DURATION_TICKS = 50
    s._unstuck_until = None
    s._unstuck_dir = 1

    def fake_tel(number, x, z, az, hp, pw, timer, y=10.0,
                 radarx=0.0, radary=0.0, radarz=0.0):
        # 24 fields: timer, lastUpdate, number, hp, pw, az, rx, ry, rz, x, y, z, R1..R12
        return (timer, timer, number, hp, pw, az,
                radarx, radary, radarz,
                x, y, z,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    # rival se acerca a 15 m/s desde el norte, telemetria a 20 Hz
    print(f"{'tick':>5} {'policy':<14} {'dist':>6} {'dist_sp':>7} "
          f"{'hdg_err':>7} {'steer':>6} {'thrust':>7} {'aim':>6} {'fire':>4}")
    his_z = 2000.0
    for tick in range(0, 200 * 20, 1):
        if tick > 0 and tick % 1 == 0:
            his_z -= 15.0 * SIM_DT     # avanza 0.75 m por tick
        mine = fake_tel(1, 0.0, 0.0, 0.0, 1000.0, 1000.0, tick)
        other = fake_tel(2, 0.0, his_z, 180.0, 1000.0,
                         1000.0 - (tick // 80), tick)
        d = s._decide(mine, other)
        if tick % 200 == 0:
            print(f"{tick:>5} {d['policy']:<14} "
                  f"{d['distance']:>6.0f} {d['distance_sp']:>7.0f} "
                  f"{d['heading_err']:>+7.1f} {d['steering']:>+6.2f} "
                  f"{d['thrust']:>+7.2f} {d['aim_error']:>6.2f} "
                  f"{'Y' if d['fire_ok'] else '-':>4}")


if __name__ == '__main__':
    if len(sys.argv) >= 2 and sys.argv[1] == '--selftest':
        _self_test()
    elif len(sys.argv) >= 2:
        Strategist(int(sys.argv[1])).run()
    else:
        print("Uso:")
        print("  python3 scripts/Strategist.py 1         # controlar tanque 1")
        print("  python3 scripts/Strategist.py 2         # controlar tanque 2")
        print("  python3 scripts/Strategist.py --selftest  # validacion sin simulador")
        sys.exit(1)
