"""
OpponentsZoo.py — colleccion de rivales sinteticos para benchmark.

Lo que vas a enfrentar en clase (mejor estimacion, basado en SeekAndDestroy.py
+ ControlPID.py + Subsumption.py que vienen como ejemplos):

  1. SmartLead   La mayoria va a hacer SeekAndDestroy + lead balistico.
                 Avanza al rival, frena cerca, apunta liderando el tiro.
                 Estimado: 60% de los compañeros usaran algo asi.

  2. Sniper      Algunos se quedan quietos y apuntan fino con PID.
                 Apuesta a la precision contra la movilidad.
                 Estimado: 15%.

  3. Rusher      Otros van a maxima velocidad encima del rival.
                 Apuesta a que a distancia corta la balistica es trivial.
                 Estimado: 15%.

  4. Zigzag      Quienes piensan en anti-prediccion: zigzag continuo
                 mientras disparan. Estimado: 10%.

Uso:
    python3 scripts/OpponentsZoo.py sniper   1   # spawn Sniper como tank 1
    python3 scripts/OpponentsZoo.py rusher   2
    python3 scripts/OpponentsZoo.py zigzag   2
    python3 scripts/OpponentsZoo.py smartlead 2

Cada uno usa el mismo protocolo UDP (Command + telemetria), asi que pueden
correr contra cualquier otro agente (incluido el Strategist).
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
from PID import PIDController
from TelemetryDictionary import telemetrydirs as td

FIRE = 11
TELEMETRY_PORT_BASE = 4600
COMMAND_PORT_BASE = 4500
TELEMETRY_STRUCT = '<LLififffffffffffffffffff'
TELEMETRY_LEN = 96


class _Base:
    """Esqueleto comun: socket UDP + loop de control."""

    def __init__(self, tank_id: int):
        self.tank_id = int(tank_id)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', TELEMETRY_PORT_BASE + self.tank_id))
        self.sock.settimeout(5.0)
        self.command = Command(Configuration.ip, COMMAND_PORT_BASE + self.tank_id)
        self._last_timer = None
        self.name = self.__class__.__name__

    # I/O ----------------------------------------------------------------

    def _read_one(self) -> tuple | None:
        try:
            data, _ = self.sock.recvfrom(TELEMETRY_LEN)
        except socket.timeout:
            return None
        if len(data) != TELEMETRY_LEN:
            return None
        return unpack(TELEMETRY_STRUCT, data)

    def _read_two_tanks(self) -> tuple[tuple, tuple] | None:
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

    def _dt(self, timer: float) -> float:
        if self._last_timer is None:
            self._last_timer = timer
            return SIM_DT
        dt = max((timer - self._last_timer) * SIM_DT, SIM_DT)
        self._last_timer = timer
        return dt

    # Override --------------------------------------------------------

    def decide(self, mine: tuple, other: tuple, dt: float) -> dict:
        raise NotImplementedError

    def run(self) -> None:
        print(f"[{self.name}] tank={self.tank_id} "
              f"tel={TELEMETRY_PORT_BASE + self.tank_id} "
              f"cmd={COMMAND_PORT_BASE + self.tank_id}")
        while True:
            tels = self._read_two_tanks()
            if tels is None:
                print(f"[{self.name}] timeout, fin de episodio")
                break
            t1, t2 = tels
            mine, other = (t1, t2) if self.tank_id == 1 else (t2, t1)
            timer = float(mine[td['timer']])
            dt = self._dt(timer)
            d = self.decide(mine, other, dt)
            if d.get('fire'):
                self.command.command = FIRE
            self.command.send_command(
                int(timer), int(self.tank_id),
                float(d['thrust']), float(d['steering']),
                float(d['turret_decl']), float(d['turret_bearing']),
            )


# ===========================================================================
#  1) SmartLead — SeekAndDestroy + lead balistico real
# ===========================================================================

class SmartLead(_Base):
    """Avanza al rival, frena cerca, apunta liderando el tiro.
    Lo que la mayoria va a terminar usando."""

    def __init__(self, tank_id: int):
        super().__init__(tank_id)
        self.table = BallisticTable()
        self._last_other = None
        self._other_vel = (0.0, 0.0)

    def decide(self, mine, other, dt):
        my_x = float(mine[td['x']]); my_z = float(mine[td['z']])
        my_az = float(mine[td['azimuth']]); my_pw = float(mine[td['power']])
        ox = float(other[td['x']]); oz = float(other[td['z']])

        # estimacion simple de velocidad rival (no EMA)
        if self._last_other is not None:
            lt, lx, lz = self._last_other
            dt2 = max((float(mine[td['timer']]) - lt) * SIM_DT, SIM_DT)
            self._other_vel = ((ox - lx) / dt2, (oz - lz) / dt2)
        self._last_other = (float(mine[td['timer']]), ox, oz)

        dist = math.hypot(ox - my_x, oz - my_z)
        intercept = solve_moving_intercept((my_x, my_z), (ox, oz),
                                           self._other_vel, self.table)
        if intercept is None:
            world_bearing = math.degrees(math.atan2(ox - my_x, oz - my_z))
            turret_decl = 0.0
            turret_bearing = world_bearing_to_turret(world_bearing, my_az)
        else:
            turret_decl = intercept['decl_deg']
            turret_bearing = world_bearing_to_turret(intercept['bearing_world_deg'], my_az)

        # control naive: avanzar si lejos, frenar si cerca, steering bang-bang
        thrust = 15.0 if dist > 1500 else (10.0 if dist > 800 else 0.0)
        steering = 0.5 if turret_bearing > 0 else (-0.5 if turret_bearing < 0 else 0.0)
        fire = abs(turret_bearing) < 0.8 and my_pw > 20
        return dict(thrust=thrust, steering=steering,
                    turret_decl=turret_decl, turret_bearing=turret_bearing,
                    fire=fire)


# ===========================================================================
#  2) Sniper — estatico, PID de heading, apuntado fino
# ===========================================================================

class Sniper(_Base):
    """No se mueve. Solo apunta fino y dispara cuando el error es minimo."""

    def __init__(self, tank_id: int):
        super().__init__(tank_id)
        self.table = BallisticTable()
        self.heading_pid = PIDController(
            kp=0.05, ki=0.001, kd=0.02,
            output_min=-1.0, output_max=1.0, angular=True,
        )
        self._last_other = None
        self._other_vel = (0.0, 0.0)

    def decide(self, mine, other, dt):
        my_x = float(mine[td['x']]); my_z = float(mine[td['z']])
        my_az = float(mine[td['azimuth']]); my_pw = float(mine[td['power']])
        ox = float(other[td['x']]); oz = float(other[td['z']])

        if self._last_other is not None:
            lt, lx, lz = self._last_other
            dt2 = max((float(mine[td['timer']]) - lt) * SIM_DT, SIM_DT)
            self._other_vel = ((ox - lx) / dt2, (oz - lz) / dt2)
        self._last_other = (float(mine[td['timer']]), ox, oz)

        intercept = solve_moving_intercept((my_x, my_z), (ox, oz),
                                           self._other_vel, self.table)
        world_bearing = math.degrees(math.atan2(ox - my_x, oz - my_z))
        if intercept is None:
            turret_decl = 0.0
            turret_bearing = world_bearing_to_turret(world_bearing, my_az)
        else:
            turret_decl = intercept['decl_deg']
            turret_bearing = world_bearing_to_turret(intercept['bearing_world_deg'], my_az)

        # apenas un toque de steering para apuntar el chasis al rival
        steering = self.heading_pid.step(world_bearing, my_az, dt)
        thrust = 0.0
        fire = abs(turret_bearing) < 0.3 and my_pw > 30
        return dict(thrust=thrust, steering=steering,
                    turret_decl=turret_decl, turret_bearing=turret_bearing,
                    fire=fire)


# ===========================================================================
#  3) Rusher — full thrust al rival, dispara a quemarropa
# ===========================================================================

class Rusher(_Base):
    """Maxima velocidad encima del rival, fire a corta distancia."""

    def __init__(self, tank_id: int):
        super().__init__(tank_id)
        self.table = BallisticTable()
        self.heading_pid = PIDController(
            kp=0.06, ki=0.0005, kd=0.02,
            output_min=-1.0, output_max=1.0, angular=True,
        )

    def decide(self, mine, other, dt):
        my_x = float(mine[td['x']]); my_z = float(mine[td['z']])
        my_az = float(mine[td['azimuth']]); my_pw = float(mine[td['power']])
        ox = float(other[td['x']]); oz = float(other[td['z']])

        dist = math.hypot(ox - my_x, oz - my_z)
        world_bearing = math.degrees(math.atan2(ox - my_x, oz - my_z))
        aim = self.table.aim(dist) if dist > 0 else None
        turret_decl = aim[0] if aim is not None else 0.0
        turret_bearing = world_bearing_to_turret(world_bearing, my_az)

        steering = self.heading_pid.step(world_bearing, my_az, dt)
        thrust = 28.0   # maximo siempre
        # dispara a corta distancia con tolerancia angular grande
        fire = (dist < 700 and abs(turret_bearing) < 2.0 and my_pw > 10)
        return dict(thrust=thrust, steering=steering,
                    turret_decl=turret_decl, turret_bearing=turret_bearing,
                    fire=fire)


# ===========================================================================
#  4) Zigzag — anti-prediccion, cambia direccion cada N ticks
# ===========================================================================

class Zigzag(_Base):
    """Zigzaguea agresivamente; mantiene distancia media; dispara cuando puede."""

    def __init__(self, tank_id: int):
        super().__init__(tank_id)
        self.table = BallisticTable()
        self._zigzag_phase = 0
        self._period = random.randint(20, 40)
        self.heading_pid = PIDController(
            kp=0.05, ki=0.0005, kd=0.02,
            output_min=-1.0, output_max=1.0, angular=True,
        )
        self._last_other = None
        self._other_vel = (0.0, 0.0)

    def decide(self, mine, other, dt):
        my_x = float(mine[td['x']]); my_z = float(mine[td['z']])
        my_az = float(mine[td['azimuth']]); my_pw = float(mine[td['power']])
        ox = float(other[td['x']]); oz = float(other[td['z']])

        if self._last_other is not None:
            lt, lx, lz = self._last_other
            dt2 = max((float(mine[td['timer']]) - lt) * SIM_DT, SIM_DT)
            self._other_vel = ((ox - lx) / dt2, (oz - lz) / dt2)
        self._last_other = (float(mine[td['timer']]), ox, oz)

        intercept = solve_moving_intercept((my_x, my_z), (ox, oz),
                                           self._other_vel, self.table)
        world_bearing = math.degrees(math.atan2(ox - my_x, oz - my_z))
        if intercept is None:
            turret_decl = 0.0
            turret_bearing = world_bearing_to_turret(world_bearing, my_az)
        else:
            turret_decl = intercept['decl_deg']
            turret_bearing = world_bearing_to_turret(intercept['bearing_world_deg'], my_az)

        # zigzag: alterna heading +/- 45 deg del bearing al rival
        self._zigzag_phase += 1
        if (self._zigzag_phase // self._period) % 2 == 0:
            heading_sp = world_bearing + 45.0
        else:
            heading_sp = world_bearing - 45.0
        # cada tantos ticks cambia el periodo para no ser predecible
        if self._zigzag_phase % 200 == 0:
            self._period = random.randint(20, 40)

        steering = self.heading_pid.step(heading_sp, my_az, dt)
        dist = math.hypot(ox - my_x, oz - my_z)
        thrust = 20.0 if dist > 600 else 5.0
        fire = abs(turret_bearing) < 1.0 and my_pw > 20
        return dict(thrust=thrust, steering=steering,
                    turret_decl=turret_decl, turret_bearing=turret_bearing,
                    fire=fire)


# ===========================================================================
#  CLI
# ===========================================================================

REGISTRY = {
    'sniper': Sniper,
    'rusher': Rusher,
    'zigzag': Zigzag,
    'smartlead': SmartLead,
}


def main(argv: list[str]) -> int:
    if len(argv) < 3:
        print(f"Uso: python3 scripts/OpponentsZoo.py {{{ '|'.join(REGISTRY) }}} {{1|2}}")
        return 1
    name = argv[1].lower()
    if name not in REGISTRY:
        print(f"Rival desconocido: {name}. Disponibles: {list(REGISTRY)}")
        return 1
    cls = REGISTRY[name]
    cls(int(argv[2])).run()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
