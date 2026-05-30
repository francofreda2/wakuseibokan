"""
Ballistic.py — modelo balistico analitico para el canion del Walrus/Otter
(Escenario 111 / 119).

Parametros fisicos extraidos del codigo C++:
    src/keplerivworld.cpp        gravity = -9.81 m/s2 en y
                                 linearDamping = 0.01 por step de mundo
    main game loop step          dt = 0.05 s  (20 ticks por segundo)
    src/units/AdvancedWalrus.h   firepower = 600 m/s  (velocidad inicial)
    src/units/AdvancedWalrus.cpp firingpos.y = 2.3 m  (altura de la boca)
                                 boca a 40 m adelante del centro de masa
    src/actions/ArtilleryAmmo    mass = 10.01 kg, ttl = 500 ticks (25 s)

Convencion de ejes (OpenGL): y arriba, plano horizontal = (x, z).
Convencion del juego para el comando turretdeclination (pitch):
    0   = horizontal adelante
    +90 = cenit
Convencion del comando turretbearing (precesion):
    relativo al chasis, derecha positivo.

Este modulo es puro Python + numpy. No depende del simulador.
Ejecutalo directo (`python3 scripts/Ballistic.py`) para tests y plot.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


# Parametros fisicos del mundo (fuente: codigo C++ del simulador)
GRAVITY = 9.81                  # m/s2 (positivo, se aplica como -y)
LINEAR_DAMPING = 0.01           # ODE world linear damping
SIM_DT = 0.05                   # segundos por step de simulacion (20 Hz)
MUZZLE_SPEED = 600.0            # m/s, modulo de la velocidad inicial
MUZZLE_FORWARD = 40.0           # m, offset adelante del centro de masa
MUZZLE_HEIGHT = 2.3             # m, altura adicional sobre el centro de masa
BULLET_TTL_TICKS = 500          # ticks de vida del proyectil

# Per-step damping factor (formula de ODE simplificada):
# v_new = v_old * (1 - linear_damping * dt) por step.
DAMPING_FACTOR = 1.0 - LINEAR_DAMPING * SIM_DT   # ~= 0.9995


@dataclass
class Trajectory:
    """Resultado de simular un disparo."""
    decl_deg: float            # angulo de elevacion del cano (grados)
    bearing_deg: float         # angulo horizontal relativo (grados)
    times: np.ndarray          # tiempos sample-eados (s)
    pos: np.ndarray            # posiciones (N, 3) en (x, y, z)
    vel: np.ndarray            # velocidades (N, 3)
    impact_t: float | None     # tiempo al que cruza target_height (s) o None
    impact_xz: tuple[float, float] | None
    impact_distance: float | None   # distancia horizontal recorrida hasta impacto


def _direction_vector(decl_deg: float, bearing_deg: float) -> np.ndarray:
    """Replica toVectorInFixedSystem(0, 0, 1, azimuth=bearing, inclination=decl)
    del C++ (src/math/yamathutil.cpp:65)."""
    az = math.radians(bearing_deg)
    el = math.radians(decl_deg)
    x = -math.cos(el) * math.sin(az)
    y = math.sin(el)
    z = math.cos(el) * math.cos(az)
    return np.array([x, y, z], dtype=np.float64)


def simulate_trajectory(decl_deg: float,
                        bearing_deg: float = 0.0,
                        shooter_pos: tuple[float, float, float] = (0.0, 10.0, 0.0),
                        target_height: float | None = None,
                        dt: float = SIM_DT,
                        max_steps: int = BULLET_TTL_TICKS) -> Trajectory:
    """Simula el vuelo del proyectil tick a tick replicando la integracion ODE.

    target_height: si se pasa, encuentra el primer cruce descendente con esa
    altura mundial (y absoluta). Default = altura del shooter (impacto a la
    misma cota — util para duelos en isla plana).
    """
    if target_height is None:
        target_height = shooter_pos[1]

    forward = _direction_vector(decl_deg, bearing_deg)
    muzzle = np.array(shooter_pos, dtype=np.float64) + MUZZLE_FORWARD * forward
    muzzle[1] += MUZZLE_HEIGHT

    p = muzzle.copy()
    v = MUZZLE_SPEED * forward.copy()

    times = [0.0]
    pos = [p.copy()]
    vel = [v.copy()]
    impact_t = None
    impact_xz = None
    impact_distance = None

    g = np.array([0.0, -GRAVITY, 0.0])
    prev_y = p[1]
    horizontal_start = np.array([muzzle[0], muzzle[2]])

    for step in range(1, max_steps + 1):
        # Integracion semi-implicita: aplicar gravedad, despues damping, despues posicion.
        v = v + g * dt
        v = v * DAMPING_FACTOR
        p = p + v * dt
        t = step * dt
        times.append(t)
        pos.append(p.copy())
        vel.append(v.copy())

        # Cruce con target_height (primer cruce, ascendente o descendente).
        # Si target esta arriba del muzzle, el primer cruce es ascendente
        # (tiro directo); si esta abajo, el unico cruce es descendente.
        if impact_t is None and (
            (prev_y <= target_height <= p[1]) or (p[1] <= target_height <= prev_y)
        ):
            prev_p = pos[-2]
            denom = p[1] - prev_p[1]
            if abs(denom) > 1e-9:
                alpha = (target_height - prev_p[1]) / denom
                hit = prev_p + alpha * (p - prev_p)
                impact_t = times[-2] + alpha * (t - times[-2])
                impact_xz = (float(hit[0]), float(hit[2]))
                impact_distance = float(np.linalg.norm(
                    np.array([hit[0], hit[2]]) - horizontal_start
                ))
        prev_y = p[1]

    return Trajectory(
        decl_deg=decl_deg,
        bearing_deg=bearing_deg,
        times=np.asarray(times),
        pos=np.asarray(pos),
        vel=np.asarray(vel),
        impact_t=impact_t,
        impact_xz=impact_xz,
        impact_distance=impact_distance,
    )


def build_range_table(decl_min: float = 0.0,
                      decl_max: float = 60.0,
                      n_samples: int = 601,
                      shooter_height: float = 10.0,
                      target_height: float | None = None) -> np.ndarray:
    """Devuelve un array (n, 3): [decl_deg, distance_m, time_of_flight_s].
    Util para grabar una tabla de calibracion y para interpolar despues.
    """
    decls = np.linspace(decl_min, decl_max, n_samples)
    rows = []
    for d in decls:
        traj = simulate_trajectory(
            decl_deg=float(d),
            shooter_pos=(0.0, shooter_height, 0.0),
            target_height=target_height,
        )
        if traj.impact_distance is not None:
            rows.append((float(d), traj.impact_distance, traj.impact_t))
    return np.array(rows, dtype=np.float64)


class BallisticTable:
    """Tabla precomputada distancia -> declinacion + tiempo de vuelo.

    Por defecto usa la rama de tiro raso (declinacion baja). Si te interesa
    el tiro de mortero (arco alto, util para sorprender), construi otra
    instancia con decl_min mas alto.
    """

    def __init__(self,
                 decl_min: float = -5.0,
                 decl_max: float = 10.0,
                 n_samples: int = 1501,
                 shooter_height: float = 10.0):
        table = build_range_table(decl_min, decl_max, n_samples,
                                  shooter_height=shooter_height,
                                  target_height=shooter_height)
        # solo ramo en el que la distancia es creciente con la elevacion
        # (tiro raso). En tiro bajo, a mayor elevacion mayor alcance hasta
        # ~45 grados sin damping; con damping el optimo cae un poco.
        decls = table[:, 0]
        dists = table[:, 1]
        tofs = table[:, 2]
        # ordenar por distancia
        order = np.argsort(dists)
        self._dists = dists[order]
        self._decls = decls[order]
        self._tofs = tofs[order]
        self.max_range = float(self._dists.max())
        self.min_range = float(self._dists.min())

    def aim(self, distance: float) -> tuple[float, float] | None:
        """Para una distancia horizontal entre tanques, devuelve (decl_deg, tof_s)
        usando interpolacion lineal. None si la distancia supera el alcance maximo.
        """
        if distance < self.min_range or distance > self.max_range:
            return None
        decl = float(np.interp(distance, self._dists, self._decls))
        tof = float(np.interp(distance, self._dists, self._tofs))
        return decl, tof

    def aim_with_height(self, distance: float,
                        shooter_y: float, target_y: float) -> tuple[float, float] | None:
        """Variante de aim() que compensa por diferencia de altura entre tanque
        y blanco. Necesario en escenario 131 (terreno con elevacion).

        Si la diferencia es chica (<3 m) usa la tabla. Si no, refina con
        biseccion sobre la simulacion para encontrar el decl que hace que la
        bala cruce target_y a la distancia pedida.
        """
        dh = target_y - shooter_y
        # caso facil: terreno plano
        if abs(dh) < 3.0:
            return self.aim(distance)
        # arrancamos con la estimacion plana y agregamos correccion geometrica
        flat = self.aim(distance)
        if flat is None:
            return None
        decl0, tof0 = flat
        # correccion de primer orden: extra angulo para subir o bajar dh
        # delta_decl ~ atan(dh / distancia) en grados
        delta = math.degrees(math.atan2(dh, distance))
        guess = decl0 + delta
        # refinar con simulacion: variar decl hasta que la trayectoria cruce
        # target_y_absoluto = shooter_y + dh dentro de tolerancia
        target_y_abs = shooter_y + dh
        best = guess
        best_err = float('inf')
        best_tof = tof0
        # busqueda local 4 grados alrededor de guess, paso 0.05 deg
        for d_offset in np.arange(-4.0, 4.0, 0.05):
            d_try = guess + d_offset
            traj = simulate_trajectory(
                decl_deg=float(d_try),
                shooter_pos=(0.0, shooter_y, 0.0),
                target_height=target_y_abs,
            )
            if traj.impact_distance is None:
                continue
            err = abs(traj.impact_distance - distance)
            if err < best_err:
                best_err = err
                best = float(d_try)
                best_tof = float(traj.impact_t)
            if err < 2.0:   # convergencia
                break
        if best_err > 50.0:   # no encontro algo razonable
            return None
        return best, best_tof


def solve_moving_intercept(shooter_xz: tuple[float, float],
                           target_xz: tuple[float, float],
                           target_vel_xz: tuple[float, float],
                           table: BallisticTable,
                           shooter_y: float | None = None,
                           target_y: float | None = None,
                           max_iter: int = 5,
                           tol: float = 0.5) -> dict | None:
    """Resuelve el punto de impacto contra un blanco que se mueve linealmente.

    Itera: dado el target en su posicion actual, estima tiempo de vuelo,
    proyecta el target a (pos + vel * tof), recalcula tiempo de vuelo, repite.

    En escenario 131 (terreno variado) pasa shooter_y y target_y para que el
    solver compense la diferencia de altura. Si no se pasan, usa la tabla
    plana (asume misma altura).

    Devuelve dict con decl_deg, bearing_world_deg (azimuth absoluto al
    punto liderado), tof_s, lead_point_xz, distance_m; o None si fuera de rango.
    """
    sx, sz = shooter_xz
    tx, tz = target_xz
    vx, vz = target_vel_xz

    use_height = (shooter_y is not None and target_y is not None
                  and abs(shooter_y - target_y) >= 3.0)

    def _aim_for(d: float) -> tuple[float, float] | None:
        if use_height:
            return table.aim_with_height(d, shooter_y, target_y)
        return table.aim(d)

    lead_x, lead_z = tx, tz
    decl, tof = None, None
    for _ in range(max_iter):
        dx = lead_x - sx
        dz = lead_z - sz
        d = math.hypot(dx, dz)
        aim = _aim_for(d)
        if aim is None:
            return None
        new_decl, new_tof = aim
        new_lead_x = tx + vx * new_tof
        new_lead_z = tz + vz * new_tof
        if (decl is not None and abs(new_decl - decl) < 1e-3
                and math.hypot(new_lead_x - lead_x, new_lead_z - lead_z) < tol):
            decl, tof, lead_x, lead_z = new_decl, new_tof, new_lead_x, new_lead_z
            break
        decl, tof, lead_x, lead_z = new_decl, new_tof, new_lead_x, new_lead_z

    bearing_world_deg = math.degrees(math.atan2(lead_x - sx, lead_z - sz))
    return {
        'decl_deg': decl,
        'bearing_world_deg': bearing_world_deg,
        'tof_s': tof,
        'lead_point_xz': (lead_x, lead_z),
        'distance_m': math.hypot(lead_x - sx, lead_z - sz),
        'height_corrected': use_height,
    }


def world_bearing_to_turret(bearing_world_deg: float, chassis_azimuth_deg: float) -> float:
    """El comando turretbearing es relativo al chasis. Convertir un azimuth
    absoluto al frame del chasis.
    """
    rel = bearing_world_deg - chassis_azimuth_deg
    # normalizar a (-180, 180]
    while rel > 180.0:
        rel -= 360.0
    while rel <= -180.0:
        rel += 360.0
    return rel


# ---------------------------------------------------------------------------
# Self-test / demo cuando se corre como script
# ---------------------------------------------------------------------------

def _print_table(table: np.ndarray, step_m: float = 100.0) -> None:
    print(f"{'decl (deg)':>10} {'distance (m)':>14} {'tof (s)':>10}")
    print('-' * 38)
    last_bucket = -1
    for row in table:
        d, dist, tof = row
        bucket = int(dist // step_m)
        if bucket != last_bucket:
            print(f"{d:>10.3f} {dist:>14.1f} {tof:>10.3f}")
            last_bucket = bucket


def _self_test() -> None:
    print("== Trayectorias de muestra ==")
    for d in (1.0, 5.0, 10.0, 20.0, 30.0, 45.0):
        traj = simulate_trajectory(d)
        if traj.impact_distance is None:
            print(f"  decl {d:5.1f} deg -> sin impacto (TTL agotado)")
        else:
            print(f"  decl {d:5.1f} deg -> alcance {traj.impact_distance:7.1f} m"
                  f"  tof {traj.impact_t:5.2f} s")

    print("\n== Tabla resumida (saltos de ~100 m) ==")
    full = build_range_table()
    _print_table(full, step_m=100.0)

    print("\n== Lookup ==")
    bt = BallisticTable()
    for dist in (200, 500, 800, 1200, 1700, 2200):
        aim = bt.aim(dist)
        if aim is None:
            print(f"  d={dist} m -> fuera de rango (max {bt.max_range:.0f} m)")
        else:
            print(f"  d={dist} m -> decl={aim[0]:5.3f} deg, tof={aim[1]:.2f} s")

    print("\n== Lead solver vs blanco a 1000 m moviendose a 20 m/s lateral ==")
    sol = solve_moving_intercept(
        shooter_xz=(0.0, 0.0),
        target_xz=(0.0, 1000.0),
        target_vel_xz=(20.0, 0.0),
        table=bt,
    )
    print(f"  {sol}")

    print("\n== Tabla guardada en /tmp/wakuseibokan_ballistic_table.csv ==")
    np.savetxt(
        '/tmp/wakuseibokan_ballistic_table.csv',
        full,
        delimiter=',',
        header='decl_deg,distance_m,tof_s',
        comments='',
    )


if __name__ == '__main__':
    _self_test()
