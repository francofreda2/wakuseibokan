"""
OpponentProfiler.py — clasifica el estilo del rival en los primeros segundos
para elegir contra-estrategia.

La idea: los primeros ~5 s del combate son baratos (a 2800 m maxima distancia
ningun disparo va a pegar en menos de 5 s y vale la pena estudiar al rival
antes de gastar municion). Mientras tanto:

  - registramos sus posiciones tick a tick,
  - estimamos velocidad media, varianza de heading, taza de fuego,
  - lo etiquetamos como uno de:
        STATIC      apenas se mueve (sniper estatico)
        LINEAR      cruza recto a velocidad constante
        ZIGZAG      cambia de direccion seguido (anti-prediction)
        AGGRESSIVE  viene encima nuestro
        EVASIVE     se aleja
        UNKNOWN     todavia insuficiente data

Adicional: tracking de su `power`. Cada disparo del rival baja power en 1.
Si vemos que su power decrementa sin que nos pegue (no baja nuestro health)
entonces sabemos que esta disparando feo => podemos pasarnos a sniper estatico
con confianza.

Uso programatico:

    profiler = OpponentProfiler(self_id=1)
    while True:
        # llamar cada tick con telemetria propia y del rival
        profiler.update(timer, my_tel, his_tel)
        if profiler.ready:
            style = profiler.classify()
            # ...elegir politica
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field


# Cuanto historial mantener (en samples). 100 samples ~ 5 segundos al ritmo
# tipico de telemetria (20 Hz).
HISTORY_LEN = 100

# Minimo de samples para emitir clasificacion.
MIN_SAMPLES_FOR_CLASSIFY = 30

# Umbrales (ajustables tras observar partidas).
SPEED_STATIC_THRESHOLD = 1.5         # m/s — bajo esto = static
SPEED_FAST_THRESHOLD = 18.0          # m/s — sobre esto = high mobility
HEADING_STD_ZIGZAG_DEG = 25.0        # std dev de heading > esto = zigzag
APPROACH_RATE_AGGRESSIVE = -3.0      # distancia disminuye > 3 m/s
APPROACH_RATE_EVASIVE = 3.0          # distancia aumenta > 3 m/s


@dataclass
class _Snapshot:
    timer: float
    my_x: float
    my_z: float
    his_x: float
    his_z: float
    his_azimuth: float
    his_power: float
    his_health: float
    my_health: float


@dataclass
class ProfileResult:
    style: str
    confidence: float                # 0..1, 1 = mucha info
    avg_speed: float                 # m/s del rival
    heading_std_deg: float           # variabilidad de heading
    approach_rate: float             # m/s con que se acerca a nosotros
    distance: float                  # distancia actual
    fire_rate: float                 # disparos/s del rival (estimado por power decay)
    wasted_shots: int                # disparos del rival que no nos pegaron
    notes: list[str] = field(default_factory=list)

    def __str__(self) -> str:
        return (
            f"[Profile] style={self.style:<10} conf={self.confidence:.2f} "
            f"speed={self.avg_speed:5.1f} m/s "
            f"hdg_std={self.heading_std_deg:5.1f} deg "
            f"approach={self.approach_rate:+5.1f} m/s "
            f"dist={self.distance:6.0f} m "
            f"fire_rate={self.fire_rate:.2f}/s "
            f"wasted={self.wasted_shots}"
        )


class OpponentProfiler:
    def __init__(self, history_len: int = HISTORY_LEN) -> None:
        self._buf: deque[_Snapshot] = deque(maxlen=history_len)
        self._last_his_power: float | None = None
        self._last_my_health: float | None = None
        self._fire_events: deque[tuple[float, bool]] = deque(maxlen=50)
        # (timer_of_fire, hit_us_within_window)

    @property
    def ready(self) -> bool:
        return len(self._buf) >= MIN_SAMPLES_FOR_CLASSIFY

    @property
    def n_samples(self) -> int:
        return len(self._buf)

    def update(self,
               timer: float,
               my_x: float, my_z: float, my_health: float,
               his_x: float, his_z: float,
               his_azimuth: float,
               his_power: float, his_health: float) -> None:
        snap = _Snapshot(
            timer=timer,
            my_x=my_x, my_z=my_z,
            his_x=his_x, his_z=his_z,
            his_azimuth=his_azimuth,
            his_power=his_power, his_health=his_health,
            my_health=my_health,
        )
        self._buf.append(snap)

        # detectar disparo del rival por caida de power
        if self._last_his_power is not None and his_power < self._last_his_power - 0.5:
            # registrar; chequearemos despues si nos hizo dano
            self._fire_events.append([timer, False])
        self._last_his_power = his_power

        # detectar hit reciente (nuestro health bajo)
        if self._last_my_health is not None and my_health < self._last_my_health - 0.5:
            # marcar los disparos recientes (hasta 6 s atras) como hit
            for ev in self._fire_events:
                if timer - ev[0] < 6.0:
                    ev[1] = True
        self._last_my_health = my_health

    # ---- analisis ----

    def _avg_speed(self) -> float:
        if len(self._buf) < 2:
            return 0.0
        total = 0.0
        count = 0
        for a, b in zip(list(self._buf)[:-1], list(self._buf)[1:]):
            dt = max(b.timer - a.timer, 1e-6) * 0.05  # timer es en ticks, dt en s
            dx = b.his_x - a.his_x
            dz = b.his_z - a.his_z
            total += math.hypot(dx, dz) / dt
            count += 1
        return total / max(count, 1)

    def _heading_std(self) -> float:
        if len(self._buf) < 3:
            return 0.0
        headings = []
        snaps = list(self._buf)
        for a, b in zip(snaps[:-1], snaps[1:]):
            dx = b.his_x - a.his_x
            dz = b.his_z - a.his_z
            if math.hypot(dx, dz) > 0.05:        # ignorar ruido si esta quieto
                headings.append(math.degrees(math.atan2(dx, dz)))
        if len(headings) < 3:
            return 0.0
        # std circular en grados (simple, no exacto, suficiente para clasificar)
        mean = sum(headings) / len(headings)
        var = sum((h - mean) ** 2 for h in headings) / len(headings)
        return math.sqrt(var)

    def _approach_rate(self) -> float:
        """Velocidad con la que cambia la distancia rival-yo (negativa = se acerca)."""
        if len(self._buf) < 2:
            return 0.0
        first = self._buf[0]
        last = self._buf[-1]
        d0 = math.hypot(first.his_x - first.my_x, first.his_z - first.my_z)
        d1 = math.hypot(last.his_x - last.my_x, last.his_z - last.my_z)
        dt = max(last.timer - first.timer, 1e-6) * 0.05
        return (d1 - d0) / dt

    def _current_distance(self) -> float:
        if not self._buf:
            return 0.0
        s = self._buf[-1]
        return math.hypot(s.his_x - s.my_x, s.his_z - s.my_z)

    def _fire_rate(self) -> float:
        if len(self._buf) < 2:
            return 0.0
        timespan = (self._buf[-1].timer - self._buf[0].timer) * 0.05
        if timespan < 0.5:
            return 0.0
        return len(self._fire_events) / timespan

    def _wasted_shots(self) -> int:
        return sum(1 for ev in self._fire_events if not ev[1] and ev[0] < self._buf[-1].timer - 6.0)

    def classify(self) -> ProfileResult:
        speed = self._avg_speed()
        hdg_std = self._heading_std()
        approach = self._approach_rate()
        dist = self._current_distance()
        fire_rate = self._fire_rate()
        wasted = self._wasted_shots()

        notes: list[str] = []
        confidence = min(len(self._buf) / HISTORY_LEN, 1.0)

        if not self.ready:
            return ProfileResult('UNKNOWN', confidence, speed, hdg_std,
                                 approach, dist, fire_rate, wasted,
                                 ['datos insuficientes'])

        style = 'LINEAR'
        if speed < SPEED_STATIC_THRESHOLD:
            style = 'STATIC'
            notes.append(f'velocidad media {speed:.1f} m/s bajo umbral {SPEED_STATIC_THRESHOLD}')
        elif hdg_std > HEADING_STD_ZIGZAG_DEG:
            style = 'ZIGZAG'
            notes.append(f'std de heading {hdg_std:.1f} deg supera umbral {HEADING_STD_ZIGZAG_DEG}')
        elif approach < APPROACH_RATE_AGGRESSIVE:
            style = 'AGGRESSIVE'
            notes.append(f'se acerca a {-approach:.1f} m/s')
        elif approach > APPROACH_RATE_EVASIVE:
            style = 'EVASIVE'
            notes.append(f'se aleja a {approach:.1f} m/s')

        if wasted >= 5 and fire_rate > 0.3:
            notes.append('rival desperdicia municion — sniper estatico es seguro')

        return ProfileResult(style, confidence, speed, hdg_std,
                             approach, dist, fire_rate, wasted, notes)

    # ---- politicas sugeridas segun estilo ----

    @staticmethod
    def recommend_strategy(style: str) -> dict:
        """Devuelve hints estrategicos como dict — interpreta el Strategist."""
        recs = {
            # Tuneo: cerrar mas que antes (mas agresivo), bajar el umbral
            # de error de aim para disparar mas seguido cuando estamos cerca.
            'STATIC':     dict(posture='SNIPER',     close_to=1200, evasion=False, fire_when_aim_error_below_deg=0.5),
            'LINEAR':     dict(posture='LEAD',       close_to=900,  evasion=False, fire_when_aim_error_below_deg=0.7),
            'ZIGZAG':     dict(posture='CLOSE',      close_to=500,  evasion=True,  fire_when_aim_error_below_deg=1.0),
            'AGGRESSIVE': dict(posture='KITE',       close_to=700,  evasion=True,  fire_when_aim_error_below_deg=0.6),
            'EVASIVE':    dict(posture='CHASE',      close_to=400,  evasion=False, fire_when_aim_error_below_deg=0.7),
            # Antes era OBSERVE: ahora LEAD. Default seguro que dispara mientras
            # el profiler recolecta datos en background. No regalamos disparos.
            'UNKNOWN':    dict(posture='LEAD',       close_to=1000, evasion=False, fire_when_aim_error_below_deg=0.7),
        }
        return recs.get(style, recs['UNKNOWN'])


# ---------------------------------------------------------------------------
# Self-test sintetico (sin simulador): simulamos un rival con varios estilos.
# ---------------------------------------------------------------------------

def _synthetic_run(style: str) -> ProfileResult:
    import random
    random.seed(42)
    p = OpponentProfiler()
    # estado inicial: yo en origen, rival a 2000 m al N
    his_x, his_z = 0.0, 2000.0
    his_az = 180.0  # mira hacia mi (sur)
    his_power = 1000.0
    his_health = 1000.0
    my_health = 1000.0
    for tick in range(0, 100 * 20, 20):  # 100 samples cada 20 ticks (1 s)
        if style == 'static':
            pass
        elif style == 'linear':
            his_x += 0.5 * 20 * 0.05      # 0.5 m/tick fwd
            his_z -= 15.0 * 20 * 0.05     # 15 m/s hacia el sur (acercandose)
        elif style == 'zigzag':
            phase = (tick // 60) % 2
            his_x += (8 if phase else -8) * 20 * 0.05
            his_z -= 5 * 20 * 0.05
        elif style == 'aggressive':
            his_z -= 25 * 20 * 0.05       # se acerca rapido
        elif style == 'evasive':
            his_z += 10 * 20 * 0.05       # se aleja
        # rival dispara cada ~40 ticks (cooldown), fallando todos los tiros
        if tick % 40 == 0 and tick > 0:
            his_power -= 1.0
        p.update(tick, 0.0, 0.0, my_health,
                 his_x, his_z, his_az,
                 his_power, his_health)
    return p.classify()


def _self_test() -> None:
    for s in ('static', 'linear', 'zigzag', 'aggressive', 'evasive'):
        res = _synthetic_run(s)
        print(f"input='{s:11}' => {res}")
        print(f"            recomienda: {OpponentProfiler.recommend_strategy(res.style)}")


if __name__ == '__main__':
    _self_test()
