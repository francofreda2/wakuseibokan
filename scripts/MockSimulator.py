"""
MockSimulator.py — simulador minimalista compatible con el protocolo UDP
del waku/testcase real, para iterar el agente sin crashes ni GUI.

NO depende de ODE, OpenGL, GLUT. Es pura fisica analitica:
  - Tanques en isla plana a altura 10 m.
  - Movimiento newtoniano amortiguado: thrust -> velocidad -> posicion.
  - Steering rota el azimuth del chasis a velocidad proporcional.
  - Balas con trayectoria parabolica con gravedad y damping (igual modelo
    que scripts/Ballistic.py).
  - Daño: si una bala impacta a < 25 m del centro de un tanque, le hace 80 HP.
  - Cooldown 20 ticks (1 s) entre disparos.
  - Match dura DEFAULT_MATCH_DURATION ticks (= 5000). En `-episodes`
    resetea posiciones cuando termina.

Protocolo UDP (identico al simulador real):
  - Comandos: escucha en 4501 + (faction-1). Formato pack('<i6fiiiifffiiI', ...) = 68 bytes
  - Telemetria: envia a los puertos definidos en conf/telemetry.endpoints.ini.
                Formato pack('<LLififffffffffffffffffff', ...) = 96 bytes.
  - Mensajes de fin de match en stdout: "Faction: N, Walrus N : Health: H, Power: P, Travelled Distance: D"
  - Mensaje de reset: "Cleaning up sceneario to start it over again."

Uso:
  python3 scripts/MockSimulator.py -episodes
  python3 scripts/MockSimulator.py -episodes --speedup 5    # corre a 5x

El --speedup acelera la sim (util para tirar muchos matches rapido).
Default es 1x = 20 ticks/s real (igual que el sim real).
"""

from __future__ import annotations

import argparse
import math
import os
import random
import re
import socket
import struct
import sys
import time
from dataclasses import dataclass, field

# Constantes — replican los parametros del simulador real (extraidos del
# C++ en commits previos).
GRAVITY = 9.81
LINEAR_DAMPING = 0.01
SIM_DT = 0.05
MUZZLE_SPEED = 600.0
MUZZLE_FORWARD = 40.0
MUZZLE_HEIGHT = 2.3
BULLET_DAMAGE = 80.0
# El tanque real mide ~6 m x 3 m x 12 m. ODE con explosion shrapnel puede
# pegar fuera del bounding box. 35 m es una aproximacion razonable.
BULLET_HIT_RADIUS = 35.0
BULLET_TTL = 500
DEFAULT_MATCH_DURATION = 5000
SPAWN_HEIGHT = 10.0
MAX_TANK_SPEED = 28.0
MAX_TURN_RATE_DEG_PER_S = 60.0  # grados/s a steering=1
TANK_DAMPING_PER_S = 1.5         # m/s^2 deceleracion natural
FIRE_COOLDOWN_TICKS = 20

TELEMETRY_PORT_BASE = 4600
COMMAND_PORT_BASE = 4500
TELEMETRY_STRUCT = '<LLififffffffffffffffffff'
COMMAND_STRUCT = '<i6fiiiifffiiI'
TELEMETRY_LEN = struct.calcsize(TELEMETRY_STRUCT)
COMMAND_LEN = struct.calcsize(COMMAND_STRUCT)

# Indices del paquete de telemetria (debe matchear TelemetryDictionary.py)
TEL_TIMER = 0
TEL_LASTUPDATE = 1
TEL_NUMBER = 2
TEL_HEALTH = 3
TEL_POWER = 4
TEL_AZIMUTH = 5
TEL_RADARX = 6
TEL_RADARY = 7
TEL_RADARZ = 8
TEL_X = 9
TEL_Y = 10
TEL_Z = 11
TEL_R1 = 12  # 12 floats restantes para R1..R12 (rotation matrix)


@dataclass
class Tank:
    faction: int
    x: float = 0.0
    z: float = 0.0
    y: float = SPAWN_HEIGHT
    vx: float = 0.0
    vz: float = 0.0
    azimuth_deg: float = 0.0
    health: float = 1000.0
    power: float = 1000.0
    travelled: float = 0.0     # acumulado speed * dt
    # Comando vigente (refrescado por UDP)
    cmd_thrust: float = 0.0
    cmd_steering: float = 0.0
    cmd_turret_decl: float = 0.0
    cmd_turret_bearing: float = 0.0
    cmd_fire: bool = False
    last_fire_tick: int = -1000
    radar: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass
class Bullet:
    faction: int                # de quien lo disparo (no daña al dueño)
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    ttl: int = BULLET_TTL


class MockSimulator:
    def __init__(self, episodes: bool = False, speedup: float = 1.0,
                 num_tanks: int = 2, seed: int | None = None):
        self.episodes = episodes
        self.speedup = max(speedup, 0.1)
        self.num_tanks = num_tanks
        self.seed = seed
        self.timer = 0
        self.match_n = 0
        self.endtimer = 0       # 0 = matchend not signaled yet
        self.tanks: list[Tank] = []
        self.bullets: list[Bullet] = []

        # Socket binds (recibir comandos) y lista de destinos de telemetria
        self.cmd_socks: list[socket.socket] = []
        for i in range(num_tanks):
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except (AttributeError, OSError):
                pass
            s.bind(('0.0.0.0', COMMAND_PORT_BASE + i + 1))
            s.setblocking(False)
            self.cmd_socks.append(s)

        # endpoints de telemetria desde conf
        self.tel_endpoints = self._read_telemetry_endpoints()
        self.tel_send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._init_match()

    # ---- conf ----

    def _read_telemetry_endpoints(self) -> list[tuple[str, int]]:
        path = 'conf/telemetry.endpoints.ini'
        eps: list[tuple[str, int]] = []
        if not os.path.exists(path):
            return [('127.0.0.1', TELEMETRY_PORT_BASE + i + 1) for i in range(self.num_tanks)]
        ips: dict[int, str] = {}
        ports: dict[int, int] = {}
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line.startswith('#') or '=' not in line:
                    continue
                k, v = line.split('=', 1)
                m_ep = re.match(r'endpoint@(\d+)', k.strip())
                m_pt = re.match(r'port@(\d+)', k.strip())
                if m_ep:
                    ips[int(m_ep.group(1))] = v.strip()
                elif m_pt:
                    ports[int(m_pt.group(1))] = int(v.strip())
        for idx in sorted(ips):
            if idx in ports:
                eps.append((ips[idx], ports[idx]))
        return eps or [('127.0.0.1', TELEMETRY_PORT_BASE + i + 1) for i in range(self.num_tanks)]

    # ---- init / reset ----

    def _init_match(self) -> None:
        if self.seed is not None:
            random.seed(self.seed + self.match_n)
        self.timer = 0
        self.endtimer = 0
        self.bullets = []
        # spawn tanks at random positions on un area de 1500 m. Distancia
        # promedio entre los dos ~ 1500 m, alcance medio del canion =>
        # los agentes pueden engancharse sin tener que cerrar 2 km.
        side = 750
        self.tanks = []
        for i in range(self.num_tanks):
            t = Tank(faction=i + 1)
            angle = i * 2 * math.pi / self.num_tanks + random.uniform(-0.4, 0.4)
            t.x = side * math.cos(angle)
            t.z = side * math.sin(angle)
            # face the centro
            target_azimuth = math.degrees(math.atan2(-t.x, -t.z))
            t.azimuth_deg = target_azimuth + random.uniform(-30, 30)
            t.health = 1000.0
            t.power = 1000.0
            t.travelled = 0.0
            self.tanks.append(t)
        print(f"[mock] Match {self.match_n + 1} start: "
              + ", ".join(f"T{t.faction} at ({t.x:.0f},{t.z:.0f}) az={t.azimuth_deg:.0f}"
                          for t in self.tanks), flush=True)

    # ---- net ----

    def _recv_commands(self) -> None:
        for i, sock in enumerate(self.cmd_socks):
            faction = i + 1
            try:
                while True:
                    data, _ = sock.recvfrom(2048)
                    if len(data) != COMMAND_LEN:
                        continue
                    fields = struct.unpack(COMMAND_STRUCT, data)
                    # fields: controllingid, thrust, roll, pitch, yaw, precesion, bank,
                    #         faction, command, spawnid, typeofisland, x, y, z, target, weapon, timer
                    cmd_faction = fields[7]
                    if cmd_faction != faction:
                        # comando para otra faction llego aca por error => ignorar
                        continue
                    tank = self.tanks[i]
                    tank.cmd_thrust = float(fields[1])
                    tank.cmd_steering = float(fields[2])    # roll
                    tank.cmd_turret_decl = float(fields[3])  # pitch
                    tank.cmd_turret_bearing = float(fields[5])  # precesion
                    cmd_code = int(fields[8])
                    tank.cmd_fire = (cmd_code == 11)
            except BlockingIOError:
                pass
            except Exception as exc:
                # Skip malformed packets but log so we notice schema drift.
                print(f"[mock] recv error tank{faction}: {exc!r}", flush=True)

    def _send_telemetry(self) -> None:
        for t in self.tanks:
            # Identidad para R1..R12 (no usamos rotation matrix completa)
            r = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
            packet = struct.pack(
                TELEMETRY_STRUCT,
                self.timer,                 # L  recordtimer
                self.timer,                 # L  lastUpdateTimer
                t.faction,                  # i  number
                t.health,                   # f  health
                int(t.power),               # i  power
                t.azimuth_deg,              # f  azimuth
                t.radar[0], t.radar[1], t.radar[2],  # fff radar/landing
                t.x, t.y, t.z,              # fff position
                *r,                         # 12f rotation matrix
            )
            for ip, port in self.tel_endpoints:
                try:
                    self.tel_send_sock.sendto(packet, (ip, port))
                except OSError:
                    pass

    # ---- fisica ----

    def _step_tank(self, t: Tank) -> None:
        if t.health <= 0:
            return
        # Steering: cambia azimuth a tasa proporcional. clamp [-1, +1]
        s = max(-1.0, min(1.0, t.cmd_steering))
        t.azimuth_deg += s * MAX_TURN_RATE_DEG_PER_S * SIM_DT
        # normalizar
        while t.azimuth_deg > 180.0:
            t.azimuth_deg -= 360.0
        while t.azimuth_deg <= -180.0:
            t.azimuth_deg += 360.0

        # Forward del chasis en mundo
        az_rad = math.radians(t.azimuth_deg)
        fx = -math.sin(az_rad)
        fz = math.cos(az_rad)

        # Thrust en dirección del chasis. clamp ±max_speed*algo. Definimos
        # aceleracion proporcional al comando con un cap.
        thrust = max(-MAX_TANK_SPEED, min(MAX_TANK_SPEED, t.cmd_thrust))
        # target velocidad
        tv_x = thrust * fx
        tv_z = thrust * fz
        # aproximamos como first-order: v += (target_v - v) * k * dt
        # k=2 => responde en ~0.5 s
        k = 2.0
        t.vx += (tv_x - t.vx) * k * SIM_DT
        t.vz += (tv_z - t.vz) * k * SIM_DT
        # damping natural
        speed = math.hypot(t.vx, t.vz)
        if speed > 0:
            decel = TANK_DAMPING_PER_S * SIM_DT
            if speed <= decel:
                t.vx = t.vz = 0.0
            else:
                factor = 1 - decel / speed
                t.vx *= factor
                t.vz *= factor

        # Update posicion
        dx = t.vx * SIM_DT
        dz = t.vz * SIM_DT
        t.x += dx
        t.z += dz
        t.travelled += math.hypot(dx, dz)

        # Clamp arena (isla finita) — no penalizamos por estar cerca del borde
        # (en el sim real eso pasa pero confunde el benchmark: el agente perderia
        # por caer al agua en vez de por mal combate). Si queres simular agua,
        # descomenta el bloque de damage abajo.
        ARENA = 1800.0
        t.x = max(-ARENA, min(ARENA, t.x))
        t.z = max(-ARENA, min(ARENA, t.z))

    def _try_fire(self, t: Tank) -> None:
        if not t.cmd_fire:
            return
        if t.power <= 0:
            return
        if self.timer - t.last_fire_tick < FIRE_COOLDOWN_TICKS:
            return
        # Direccion torreta en mundo: chasis + turret_bearing
        world_az = t.azimuth_deg + t.cmd_turret_bearing
        el = t.cmd_turret_decl
        az_rad = math.radians(world_az)
        el_rad = math.radians(el)
        fx = -math.sin(az_rad) * math.cos(el_rad)
        fz = math.cos(az_rad) * math.cos(el_rad)
        fy = math.sin(el_rad)
        # Muzzle: tank_pos + 40 * forward
        mx = t.x + MUZZLE_FORWARD * fx
        my = t.y + MUZZLE_FORWARD * fy + MUZZLE_HEIGHT
        mz = t.z + MUZZLE_FORWARD * fz
        # Velocity
        vx = MUZZLE_SPEED * fx
        vy = MUZZLE_SPEED * fy
        vz = MUZZLE_SPEED * fz
        self.bullets.append(Bullet(faction=t.faction, x=mx, y=my, z=mz,
                                   vx=vx, vy=vy, vz=vz))
        t.power -= 1
        t.last_fire_tick = self.timer
        t.cmd_fire = False   # consumir el flag (re-armado en proximo comando)

    def _step_bullets(self) -> None:
        damping_factor = 1.0 - LINEAR_DAMPING * SIM_DT
        new_bullets: list[Bullet] = []
        for b in self.bullets:
            # integrate
            b.vy += -GRAVITY * SIM_DT
            b.vx *= damping_factor
            b.vy *= damping_factor
            b.vz *= damping_factor
            b.x += b.vx * SIM_DT
            b.y += b.vy * SIM_DT
            b.z += b.vz * SIM_DT
            b.ttl -= 1
            # check hit on cada step (bala atraviesa el tanque, no solo al caer)
            consumed = False
            for t in self.tanks:
                if t.health <= 0 or t.faction == b.faction:
                    continue
                # tank box ~ 6 m wide, 3 m tall, 12 m long. Aproximamos con
                # esfera de radio BULLET_HIT_RADIUS centrada en t.x,t.y,t.z.
                # Solo contamos hit si la bala esta cerca en 3D Y debajo de
                # altura razonable (15 m).
                dx = t.x - b.x
                dz = t.z - b.z
                dy = (t.y + 2.0) - b.y     # centro del tanque ~y+2
                if (b.y < t.y + 8.0
                        and dx * dx + dz * dz + dy * dy
                        <= BULLET_HIT_RADIUS * BULLET_HIT_RADIUS):
                    t.health -= BULLET_DAMAGE
                    t.health = max(0.0, t.health)
                    print(f"[mock] HIT! T{b.faction} -> T{t.faction} "
                          f"(d2D={math.hypot(dx, dz):.1f}m, "
                          f"hp={t.health:.0f})", flush=True)
                    consumed = True
                    break
            if consumed:
                continue
            # impacto contra suelo (y < 0): actualizar radar de tanques cercanos
            if b.y <= 0.0 or b.ttl <= 0:
                for t in self.tanks:
                    d = math.hypot(t.x - b.x, t.z - b.z)
                    if d <= 500.0:
                        t.radar = (b.x, 0.0, b.z)
                continue
            new_bullets.append(b)
        self.bullets = new_bullets

    # ---- match lifecycle ----

    def _check_end_of_match(self) -> None:
        # Si alguno murio y endtimer no esta seteado, agendar fin en 60 ticks
        dead = [t for t in self.tanks if t.health <= 0]
        if dead and self.endtimer == 0:
            self.endtimer = self.timer + 60
        # Timeout normal
        if self.timer >= DEFAULT_MATCH_DURATION and self.endtimer == 0:
            self.endtimer = self.timer + 10
        if self.endtimer > 0 and self.timer >= self.endtimer:
            # Imprimir fin de match igual que el simulador real
            for t in self.tanks:
                # ODE Simulation wraps one real time step in 20 ticks => trvld/20
                print(f"Faction: {t.faction}, Walrus {t.faction} : "
                      f"Health: {t.health:8.2f}, Power: {int(t.power):8.0f}, "
                      f"Travelled Distance: {t.travelled / 20.0:8.2f}",
                      flush=True)
            if self.episodes:
                print("[src/tests/testcase_131.cpp:314] Cleaning up sceneario "
                      "to start it over again.", flush=True)
                self.match_n += 1
                self._init_match()
            else:
                # match end normal: exit
                sys.exit(0)

    # ---- loop principal ----

    def run(self) -> None:
        print(f"[mock] Hooking up to telemetry endpoint at "
              + ", ".join(f"{ip}:{p}" for ip, p in self.tel_endpoints),
              flush=True)
        print(f"[mock] Listening for commands on ports "
              + ", ".join(str(COMMAND_PORT_BASE + i + 1) for i in range(self.num_tanks)),
              flush=True)
        target_dt = SIM_DT / self.speedup
        next_tick_at = time.time()
        while True:
            self._recv_commands()
            for t in self.tanks:
                self._step_tank(t)
                self._try_fire(t)
            self._step_bullets()
            self._send_telemetry()
            self._check_end_of_match()
            self.timer += 1
            # control de ritmo
            next_tick_at += target_dt
            sleep_for = next_tick_at - time.time()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                # behind schedule, no sleep
                next_tick_at = time.time()


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description='Mock simulator UDP-compatible')
    p.add_argument('-episodes', action='store_true',
                   help='loop matches (mismo flag que el sim real)')
    p.add_argument('--speedup', type=float, default=1.0,
                   help='factor de aceleracion (default 1.0 = realtime)')
    p.add_argument('--num-tanks', type=int, default=2)
    p.add_argument('--seed', type=int, default=None)
    args, _ = p.parse_known_args(argv[1:])
    sim = MockSimulator(episodes=args.episodes, speedup=args.speedup,
                        num_tanks=args.num_tanks, seed=args.seed)
    try:
        sim.run()
    except KeyboardInterrupt:
        print("\n[mock] terminado por usuario", flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
