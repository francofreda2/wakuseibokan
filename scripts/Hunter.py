"""
Hunter.py — agente simple y agresivo que GANA.

Filosofia (aprendido a las patadas tras 4 commits perdiendo 0-3):
  - Sin PIDs. Sin posturas. Sin profiler.
  - Bang-bang: heading_err grande => girar fuerte, sino mover.
  - Lead balistico iterativo (3-4 iteraciones) — mejor que SmartLead.
  - Velocidad rival = derivada frame-a-frame (sin EMA).
  - Disparar CADA vez que tenemos solucion y municion.
  - Zigzag random en aproach para romper el lead del rival.

Uso:
  python3 scripts/Hunter.py 1     # controla tanque 1
  python3 scripts/Hunter.py 2     # controla tanque 2
"""

from __future__ import annotations

import math
import random
import socket
import sys
from struct import unpack

import Configuration
from Ballistic import (
    BallisticTable,
    solve_moving_intercept,
    world_bearing_to_turret,
    SIM_DT,
)
from Command import Command
from TelemetryDictionary import telemetrydirs as td

FIRE = 11
TELEMETRY_PORT_BASE = 4600
COMMAND_PORT_BASE = 4500
TELEMETRY_STRUCT = '<LLififffffffffffffffffff'
TELEMETRY_LEN = 96

# Distancia ideal de combate. Demasiado cerca => peligro de embestida y
# que el rival tambien pegue facil. Demasiado lejos => tiempo de vuelo
# alto y el lead se rompe ante zigzag.
IDEAL_DIST = 900.0
DIST_TOLERANCE = 150.0

# Si el rival esta a < CLOSE_RANGE, dejar de avanzar para no atravesarlo.
CLOSE_RANGE = 400.0

# Threshold para girar el chasis. Si heading_err > este, frenamos para no
# espirales.
HEADING_GATE = 25.0

# Cuanto rota el chasis a steering=1 en el sim real (~60 deg/s) y mock
# (igual). 1 tick = 0.05 s. Para heading_err > este valor, mandar full.
HEADING_FULL_LOCK = 8.0


class Hunter:
    def __init__(self, tank_id: int):
        self.tank_id = int(tank_id)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except (AttributeError, OSError):
            pass
        self.sock.bind(('0.0.0.0', TELEMETRY_PORT_BASE + self.tank_id))
        self.sock.settimeout(5.0)
        self.command = Command(Configuration.ip, COMMAND_PORT_BASE + self.tank_id)
        self.table = BallisticTable()
        self._last_other_pos: tuple[float, float, float] | None = None
        self._other_vel = (0.0, 0.0)
        # zigzag: oscilamos un offset al heading objetivo
        self._zigzag_phase = 0
        self._zigzag_period = 25
        # detector simple de daño
        self._last_hp = 1000.0
        self._tick_count = 0
        self._shots = 0

    def _read_one(self):
        try:
            data, _ = self.sock.recvfrom(TELEMETRY_LEN)
        except socket.timeout:
            return None
        if len(data) != TELEMETRY_LEN:
            return None
        return unpack(TELEMETRY_STRUCT, data)

    def _read_two_tanks(self):
        t1 = t2 = None
        for _ in range(8):
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

    def decide(self, mine, other):
        my_x = float(mine[td['x']]); my_z = float(mine[td['z']])
        my_y = float(mine[td['y']])
        my_az = float(mine[td['azimuth']])
        my_hp = float(mine[td['health']]); my_pw = float(mine[td['power']])
        ox = float(other[td['x']]); oz = float(other[td['z']])
        oy = float(other[td['y']])
        timer = float(mine[td['timer']])

        # Velocidad rival por derivada directa
        if self._last_other_pos is not None:
            lt, lx, lz = self._last_other_pos
            dt2 = max(timer - lt, 1.0) * SIM_DT
            self._other_vel = ((ox - lx) / dt2, (oz - lz) / dt2)
        self._last_other_pos = (timer, ox, oz)

        # Detectar daño reciente (=> rival esta pegando, zigzag mas fuerte)
        recent_damage = my_hp < self._last_hp - 0.5
        self._last_hp = my_hp

        # Distancia y bearing convencion C++
        dx = ox - my_x; dz = oz - my_z
        dist = math.hypot(dx, dz)
        # Convencion compass del simulador: azimuth = atan2(-dx, dz)
        world_bearing = math.degrees(math.atan2(-dx, dz))

        # Lead iterativo
        intercept = solve_moving_intercept(
            shooter_xz=(my_x, my_z),
            target_xz=(ox, oz),
            target_vel_xz=self._other_vel,
            table=self.table,
            shooter_y=my_y,
            target_y=oy,
            max_iter=4,
            tol=0.5,
        )

        if intercept is None:
            # Fuera de rango => apuntar plano, decl=0, simplemente perseguir
            turret_decl = 0.0
            target_bearing_world = world_bearing
        else:
            turret_decl = float(intercept['decl_deg'])
            target_bearing_world = float(intercept['bearing_world_deg'])

        # Turret relativo al chasis
        turret_bearing = world_bearing_to_turret(target_bearing_world, my_az)

        # Chasis: queremos mirar al rival (al bearing actual, no al lead).
        # Eso hace que avancemos hacia el donde ESTA, lo cual nos acerca
        # naturalmente y reduce el tiempo de vuelo.
        heading_target_world = world_bearing
        # Si nos estan pegando, oscilar +/- 30 grados para romper el lead rival
        if recent_damage:
            self._zigzag_phase += 1
            if (self._zigzag_phase // self._zigzag_period) % 2 == 0:
                heading_target_world += 30
            else:
                heading_target_world -= 30
            # cambiar periodo seguido para no ser predecible
            if self._zigzag_phase % 100 == 0:
                self._zigzag_period = random.randint(15, 40)

        heading_err = heading_target_world - my_az
        # normalizar a [-180, 180]
        while heading_err > 180.0: heading_err -= 360.0
        while heading_err <= -180.0: heading_err += 360.0
        abs_herr = abs(heading_err)

        # Steering bang-bang: cuanto necesitamos rotar
        if abs_herr > HEADING_FULL_LOCK:
            steering = 1.0 if heading_err > 0 else -1.0
        else:
            # ajuste fino proporcional
            steering = heading_err / HEADING_FULL_LOCK

        # Thrust:
        # - Si el chasis no esta mirando al rival, parar y solo rotar (evitar espirales)
        # - Sino, regulamos por distancia
        if abs_herr > HEADING_GATE:
            thrust = 0.0
        else:
            if dist > IDEAL_DIST + DIST_TOLERANCE:
                thrust = 28.0          # full forward
            elif dist < CLOSE_RANGE:
                thrust = -15.0         # alejarse para no chocar
            elif dist < IDEAL_DIST - DIST_TOLERANCE:
                thrust = -8.0          # leve reversa para mantener distancia
            else:
                thrust = 6.0           # mantener leve forward
            # Zigzag de avance: pequeña oscilacion del thrust para no ir recto
            if recent_damage:
                self._zigzag_phase += 1
                thrust *= 0.7

        # Disparo: simplemente cuando tenemos solucion y municion.
        fire_ok = (intercept is not None and my_pw > 20)

        return dict(
            timer=int(timer),
            thrust=float(thrust),
            steering=float(steering),
            turret_decl=float(turret_decl),
            turret_bearing=float(turret_bearing),
            fire=fire_ok,
            dist=dist,
            heading_err=heading_err,
        )

    def run(self):
        print(f"[Hunter] tank={self.tank_id} "
              f"tel={TELEMETRY_PORT_BASE + self.tank_id} "
              f"cmd={COMMAND_PORT_BASE + self.tank_id}", flush=True)
        last_print = 0.0
        import time
        while True:
            tels = self._read_two_tanks()
            if tels is None:
                print("[Hunter] timeout, fin de episodio", flush=True)
                break
            t1, t2 = tels
            mine, other = (t1, t2) if self.tank_id == 1 else (t2, t1)
            d = self.decide(mine, other)

            if d['fire']:
                self.command.command = FIRE
                self._shots += 1
            self.command.send_command(
                d['timer'], int(self.tank_id),
                d['thrust'], d['steering'],
                d['turret_decl'], d['turret_bearing'],
            )

            self._tick_count += 1
            now = time.time()
            if now - last_print > 0.5:
                hp = float(mine[td['health']])
                pw = float(mine[td['power']])
                print(f"t={d['timer']:>5} hp={hp:6.0f} pw={pw:4.0f} "
                      f"dist={d['dist']:6.0f} herr={d['heading_err']:+6.1f} "
                      f"thr={d['thrust']:+6.1f} st={d['steering']:+5.2f} "
                      f"fire={'Y' if d['fire'] else '-'} shots={self._shots}",
                      flush=True)
                last_print = now


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("uso: python3 scripts/Hunter.py {1|2}")
        sys.exit(1)
    Hunter(int(sys.argv[1])).run()
