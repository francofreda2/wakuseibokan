"""
Strategist.py — agente competitivo para Escenario 111 (duelo de tanques).

Combina:
  - scripts/Ballistic.py        modelo balistico y solver de lead
  - scripts/OpponentProfiler.py clasificacion del estilo rival en vivo

Politicas implementadas:
  SNIPER     pararse y apuntar fino — vs rival STATIC o lento
  LEAD       cerrar a media distancia y disparar con lead lineal — vs LINEAR
  CLOSE      cerrar a distancia corta donde la balistica es trivial — vs ZIGZAG
  KITE       mantener distancia mientras el rival se acerca — vs AGGRESSIVE
  CHASE      perseguir a velocidad maxima — vs EVASIVE
  OBSERVE    moverse en arco lateral sin disparar, para observar — primeros 5 s

Uso (igual que SeekAndDestroy.py):
    python3 scripts/Strategist.py 1     # controlar tanque 1
    python3 scripts/Strategist.py 2     # controlar tanque 2

Diferenciacion frente a SeekAndDestroy.py:
  1. Apunta con lead balistico real (no random declinacion).
  2. Solo dispara cuando el error angular esta debajo de umbral
     (ahorra power, evita gastar municion al pedo).
  3. Reconoce el estilo del rival y cambia de politica.
  4. Evade activamente cuando lo necesita.
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
    """Estimador de velocidad rival con EMA (filtro exponencial)."""

    def __init__(self, alpha: float = 0.4):
        self._alpha = alpha
        self._last_pos: tuple[float, float, float] | None = None
        self._vel = (0.0, 0.0)

    def update(self, x: float, z: float, timer: float) -> tuple[float, float]:
        if self._last_pos is None:
            self._last_pos = (x, z, timer)
            return self._vel
        lx, lz, lt = self._last_pos
        dt = max(timer - lt, 1.0) * SIM_DT
        vx_inst = (x - lx) / dt
        vz_inst = (z - lz) / dt
        a = self._alpha
        self._vel = (a * vx_inst + (1 - a) * self._vel[0],
                     a * vz_inst + (1 - a) * self._vel[1])
        self._last_pos = (x, z, timer)
        return self._vel


class Strategist:
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

        self.policy = 'OBSERVE'
        self.last_print = 0.0

        # zigzag interno cuando estamos evadiendo
        self._zigzag_phase = 0
        self._zigzag_period_ticks = 30

        # historial reciente de health propio para detectar que nos pegan
        self._health_history: deque[tuple[float, float]] = deque(maxlen=20)

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

    def _zigzag_steering(self, base_steer: float) -> float:
        self._zigzag_phase += 1
        if (self._zigzag_phase // self._zigzag_period_ticks) % 2 == 0:
            return base_steer + 0.6
        return base_steer - 0.6

    def _decide(self, mine: tuple, other: tuple) -> dict:
        my_x = float(mine[td['x']])
        my_z = float(mine[td['z']])
        my_az = float(mine[td['azimuth']])
        my_hp = float(mine[td['health']])
        my_pw = float(mine[td['power']])

        his_x = float(other[td['x']])
        his_z = float(other[td['z']])
        his_az = float(other[td['azimuth']])
        his_hp = float(other[td['health']])
        his_pw = float(other[td['power']])
        timer = float(mine[td['timer']])

        # actualizar profiler y estimador
        self.profiler.update(timer, my_x, my_z, my_hp,
                             his_x, his_z, his_az, his_pw, his_hp)
        vx, vz = self.vel_est.update(his_x, his_z, timer)

        # registrar health
        self._health_history.append((timer, my_hp))

        # distancia y lead point
        dist = math.hypot(his_x - my_x, his_z - my_z)
        intercept = solve_moving_intercept(
            shooter_xz=(my_x, my_z),
            target_xz=(his_x, his_z),
            target_vel_xz=(vx, vz),
            table=self.ballistic,
        )

        # politica
        profile = self.profiler.classify()
        if timer < 100:        # primeros ~5 s siempre OBSERVE
            policy = 'OBSERVE'
        else:
            policy = self._select_policy(profile, my_hp, my_pw)
        self.policy = policy

        rec = OpponentProfiler.recommend_strategy(profile.style)
        ideal_dist = rec['close_to']
        max_aim_error = rec['fire_when_aim_error_below_deg']

        # apuntado
        if intercept is None:
            # fuera de rango: encarar al rival y avanzar
            turret_decl = 0.0
            world_bearing = math.degrees(math.atan2(his_x - my_x, his_z - my_z))
            turret_bearing = world_bearing_to_turret(world_bearing, my_az)
            aim_error = abs(turret_bearing)
            fire_ok = False
        else:
            turret_decl = intercept['decl_deg']
            turret_bearing = world_bearing_to_turret(intercept['bearing_world_deg'], my_az)
            aim_error = abs(turret_bearing)
            fire_ok = aim_error < max_aim_error and my_pw > 20

        # movimiento por politica
        thrust = 0.0
        steering = 0.0

        if policy == 'OBSERVE':
            # arco lateral suave, sin disparar
            thrust = 10.0
            steering = 0.3
            fire_ok = False
        elif policy == 'SNIPER':
            # parar y apuntar
            thrust = 0.0
            steering = 0.0
        elif policy == 'LEAD':
            if dist > ideal_dist + 100:
                thrust = 15.0
            elif dist < ideal_dist - 100:
                thrust = -8.0
            steering = 0.2 if turret_bearing > 0 else -0.2
        elif policy == 'CLOSE':
            # acercarse rapido, despues frenar a ideal_dist
            thrust = 20.0 if dist > ideal_dist else 0.0
            steering = self._zigzag_steering(0.0)
        elif policy == 'KITE':
            # rival viene, alejarse manteniendo apuntado
            if dist < ideal_dist:
                thrust = -15.0
            else:
                thrust = 5.0
            steering = self._zigzag_steering(0.2)
        elif policy == 'CHASE':
            thrust = 28.0
            # apuntar el chasis al rival mientras corro
            world_bearing = math.degrees(math.atan2(his_x - my_x, his_z - my_z))
            chassis_err = world_bearing_to_turret(world_bearing, my_az)
            steering = 0.5 if chassis_err > 0 else -0.5
        elif policy == 'EVADE_NO_FIRE':
            thrust = 15.0
            steering = self._zigzag_steering(0.0)
            fire_ok = False
        else:
            # default conservador
            thrust = 0.0
            steering = 0.0

        # si nos pegaron en los ultimos 2 s, zigzag forzado
        if len(self._health_history) >= 2:
            recent_loss = self._health_history[0][1] - self._health_history[-1][1]
            if recent_loss > 5:
                steering = self._zigzag_steering(steering)
                thrust = max(thrust, 12.0)

        return dict(
            timer=timer,
            thrust=thrust,
            steering=steering,
            turret_decl=turret_decl,
            turret_bearing=turret_bearing,
            fire_ok=fire_ok,
            distance=dist,
            aim_error=aim_error,
            policy=policy,
            profile=profile,
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
            f"dist={decision['distance']:6.0f}m "
            f"aim_err={decision['aim_error']:5.2f}deg "
            f"fire={'Y' if decision['fire_ok'] else '-'} "
            f"|| profile={prof.style}({prof.confidence:.1f}) "
            f"sp={prof.avg_speed:4.1f} appr={prof.approach_rate:+4.1f}"
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
                decision['timer'],
                self.tank_id,
                decision['thrust'],
                decision['steering'],
                decision['turret_decl'],
                decision['turret_bearing'],
            )
            self._print_status(decision, mine)


# ---------------------------------------------------------------------------
# Self-test sin simulador: feed sintetico para validar la decision logic.
# ---------------------------------------------------------------------------

def _self_test() -> None:
    """Smoke test: corre _decide() con un stream sintetico."""
    class _StubSock:
        def settimeout(self, x): pass
        def bind(self, x): pass

    s = Strategist.__new__(Strategist)   # no llamar __init__ (abriria socket)
    s.tank_id = 1
    s.ballistic = BallisticTable()
    s.profiler = OpponentProfiler()
    s.vel_est = VelocityEstimator()
    s.policy = 'OBSERVE'
    s.last_print = 0.0
    s._zigzag_phase = 0
    s._zigzag_period_ticks = 30
    s._health_history = deque(maxlen=20)

    # construir telemetrias "fake" con todos los campos en el orden de TelemetryDictionary
    def fake_tel(number, x, z, az, hp, pw, timer):
        # 24 fields: timer, lastUpdate, number, hp, pw, az, rx, ry, rz, x, y, z, R1..R12
        return (timer, timer, number, hp, pw, az,
                0.0, 0.0, 0.0,
                x, 10.0, z,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    # Escenario sintetico: rival cierra distancia a 15 m/s desde el norte
    print(f"{'tick':>5} {'policy':<14} {'dist':>6} {'aim_err':>7} {'fire':>4}")
    for tick in range(0, 6000, 100):
        his_z = 2000.0 - 15.0 * tick * SIM_DT
        mine = fake_tel(1, 0.0, 0.0, 0.0, 1000.0, 1000.0, tick)
        other = fake_tel(2, 0.0, his_z, 180.0, 1000.0, 1000.0 - (tick // 200), tick)
        d = s._decide(mine, other)
        if tick % 500 == 0:
            print(f"{tick:>5} {d['policy']:<14} "
                  f"{d['distance']:>6.0f} {d['aim_error']:>7.2f} {'Y' if d['fire_ok'] else '-':>4}")


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
