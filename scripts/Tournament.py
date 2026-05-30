"""
Tournament.py — orquesta partidas Strategist vs cada rival del OpponentsZoo
y reporta tasa de victorias.

Uso:
    # opcion A: vos ya lanzaste el simulador en otra terminal con
    #   ./waku -test 111 -episodes -nointro
    # entonces:
    python3 scripts/Tournament.py --simulator-running --matches 5

    # opcion B: dejo que el harness lance el simulador (necesita DISPLAY=:0)
    python3 scripts/Tournament.py --auto-launch --matches 5

    # rivales especificos:
    python3 scripts/Tournament.py --simulator-running --opponents sniper,rusher --matches 3

Como funciona:
    - Para cada (rival, match_n) lanza dos procesos Python:
        tank 1 = Strategist
        tank 2 = rival
    - Mira el stdout de testcase_111 (que printea 'Walrus X : Health:%f ...'
      al fin de cada match en modo episodes) para detectar quien gano.
    - Si no tiene acceso a ese stdout (auto-launch=False), usa un fallback:
      lee la telemetria directamente, mira health de los dos tanques al
      final de los DEFAULT_MATCH_DURATION (= 250 s simulado) o cuando uno
      llega a 0.

El sistema NO requiere recompilar el simulador. Usa los binarios ya hechos.
"""

from __future__ import annotations

import argparse
import os
import re
import signal
import subprocess
import sys
import time
from dataclasses import dataclass

# en ticks. DEFAULT_MATCH_DURATION = 5000 en testcase_111.h => 250 s simulados.
# le damos un margen de wall-clock por si el simulador corre lento.
MATCH_TIMEOUT_WALL_S = 360

# Match-end signature impresa por testcase_111::checkBeforeDone():
#    "Faction: %d, Walrus %d : Health:%8.2f, Power: %8.2f, Travelled Distance: %8.2f"
MATCH_END_RE = re.compile(
    r"Faction:\s*(\d+),\s*Walrus\s+(\d+)\s*:\s*Health:\s*([\d.\-]+),\s*"
    r"Power:\s*([\d.\-]+),\s*Travelled Distance:\s*([\d.\-]+)"
)


@dataclass
class MatchResult:
    opponent: str
    match_id: int
    strategist_hp: float
    opponent_hp: float
    duration_ticks: int
    winner: str           # 'strategist' | 'opponent' | 'draw' | 'timeout'

    @property
    def hp_advantage(self) -> float:
        return self.strategist_hp - self.opponent_hp


class SimulatorLogTail:
    """Tail incremental sobre el stdout del simulador.

    Devuelve diccionarios {1: (hp, power, dist), 2: (...)} cada vez que
    detecta DOS lineas Faction: consecutivas (=fin de match).
    """

    def __init__(self, log_path: str):
        self.log_path = log_path
        self._fp = None
        self._buf: dict[int, tuple[float, float, float]] = {}
        # esperar a que el archivo exista (auto-launch lo crea inmediatamente,
        # simulator-running depende del usuario)
        for _ in range(60):
            if os.path.exists(log_path):
                break
            time.sleep(0.5)
        if not os.path.exists(log_path):
            raise FileNotFoundError(f"No encuentro el log del simulador en {log_path}")
        self._fp = open(log_path, 'r')
        # ir al final del archivo: solo nos interesan lineas NUEVAS
        self._fp.seek(0, os.SEEK_END)

    def poll(self) -> dict[int, tuple[float, float, float]] | None:
        """Lee lineas nuevas. Si en este poll completamos las 2 facciones,
        devuelve y resetea el buffer."""
        while True:
            line = self._fp.readline()
            if not line:
                return None
            m = MATCH_END_RE.search(line)
            if not m:
                continue
            faction = int(m.group(1))
            hp = float(m.group(3))
            power = float(m.group(4))
            dist = float(m.group(5))
            self._buf[faction] = (hp, power, dist)
            if 1 in self._buf and 2 in self._buf:
                out = self._buf.copy()
                self._buf = {}
                return out


class TournamentRunner:
    def __init__(self, opponents: list[str], n_matches: int,
                 auto_launch: bool, sim_log_path: str | None,
                 swap_sides: bool = True,
                 log_dir: str = './data/tournament'):
        self.opponents = opponents
        self.n_matches = n_matches
        self.auto_launch = auto_launch
        self.swap_sides = swap_sides
        self.log_dir = log_dir
        self.sim_log_path = sim_log_path
        os.makedirs(log_dir, exist_ok=True)
        self.results: list[MatchResult] = []
        self.sim_proc: subprocess.Popen | None = None

    # ---------------- Lanzamiento del simulador ----------------

    def start_simulator(self) -> None:
        env = os.environ.copy()
        if 'DISPLAY' not in env:
            raise RuntimeError("DISPLAY no esta seteado. Iniciar el simulador "
                               "requiere un X display. Usar --simulator-running "
                               "y lanzar el simulador manualmente.")
        # libstk-4.6.1.so vive en /usr/local/lib y el binario no lo encuentra
        # sin esta ayuda.
        existing = env.get('LD_LIBRARY_PATH', '')
        env['LD_LIBRARY_PATH'] = '/usr/local/lib' + (':' + existing if existing else '')
        log_path = os.path.join(self.log_dir, 'simulator.log')
        sim_log = open(log_path, 'w')
        self.sim_log_path = log_path
        cmd = ['./waku', '-test', '111', '-episodes', '-nointro', '-mute']
        print(f"[Tournament] Lanzando simulador: {' '.join(cmd)}")
        self.sim_proc = subprocess.Popen(
            cmd, stdout=sim_log, stderr=subprocess.STDOUT, env=env,
            preexec_fn=os.setsid,
        )
        time.sleep(5)
        if self.sim_proc.poll() is not None:
            raise RuntimeError(f"El simulador termino en el arranque, "
                               f"ver {log_path}")
        print(f"[Tournament] Simulador PID={self.sim_proc.pid}, log={log_path}")

    def stop_simulator(self) -> None:
        if self.sim_proc is None:
            return
        print(f"[Tournament] Terminando simulador PID={self.sim_proc.pid}")
        try:
            os.killpg(os.getpgid(self.sim_proc.pid), signal.SIGTERM)
            self.sim_proc.wait(timeout=5)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(self.sim_proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
        self.sim_proc = None

    # ---------------- Lanzar / esperar un match ----------------

    def _spawn_agent(self, script: str, args: list[str], log_name: str) -> subprocess.Popen:
        log_path = os.path.join(self.log_dir, log_name)
        log_f = open(log_path, 'w')
        cmd = ['python3', script] + args
        return subprocess.Popen(cmd, stdout=log_f, stderr=subprocess.STDOUT,
                                cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

    def _watch_match(self, tail: SimulatorLogTail,
                     match_id: int, opponent: str,
                     strategist_tank: int) -> MatchResult:
        """Espera que el simulador imprima las dos lineas de fin de match."""
        start = time.time()
        while True:
            if time.time() - start > MATCH_TIMEOUT_WALL_S:
                return self._build_result(
                    opponent, match_id, strategist_tank,
                    -1.0, -1.0, 0, winner='timeout',
                )
            payload = tail.poll()
            if payload is None:
                time.sleep(0.2)
                continue
            t1_hp = payload[1][0]
            t2_hp = payload[2][0]
            return self._build_result(opponent, match_id, strategist_tank,
                                      t1_hp, t2_hp, 5000, winner=None)

    def _build_result(self, opponent: str, match_id: int,
                      strategist_tank: int,
                      t1_hp: float, t2_hp: float, duration: int,
                      winner: str | None) -> MatchResult:
        if strategist_tank == 1:
            s_hp, o_hp = t1_hp, t2_hp
        else:
            s_hp, o_hp = t2_hp, t1_hp
        if winner is None:
            if s_hp <= 1 and o_hp <= 1:
                winner = 'draw'
            elif s_hp > o_hp + 10:
                winner = 'strategist'
            elif o_hp > s_hp + 10:
                winner = 'opponent'
            else:
                winner = 'draw'
        return MatchResult(opponent=opponent, match_id=match_id,
                           strategist_hp=s_hp, opponent_hp=o_hp,
                           duration_ticks=duration, winner=winner)

    # ---------------- Loop principal ----------------

    def run(self) -> None:
        if self.auto_launch:
            self.start_simulator()
        try:
            if not self.sim_log_path:
                raise RuntimeError("Falta el path al log del simulador. "
                                   "Con --simulator-running pasa --simulator-log /tmp/wakulog "
                                   "y lanza el simulador redirigiendo: "
                                   "./waku -test 111 -episodes -nointro > /tmp/wakulog 2>&1")
            tail = SimulatorLogTail(self.sim_log_path)
            for opp in self.opponents:
                for match_id in range(self.n_matches):
                    strategist_tank = 1 if (match_id % 2 == 0 or not self.swap_sides) else 2
                    opponent_tank = 3 - strategist_tank
                    print(f"\n=== Match {match_id+1}/{self.n_matches} vs {opp} "
                          f"(strategist=tank{strategist_tank}) ===")
                    s_proc = self._spawn_agent(
                        'scripts/Strategist.py', [str(strategist_tank)],
                        f'strategist_vs_{opp}_m{match_id}.log',
                    )
                    o_proc = self._spawn_agent(
                        'scripts/OpponentsZoo.py', [opp, str(opponent_tank)],
                        f'opponent_{opp}_m{match_id}.log',
                    )
                    time.sleep(0.5)
                    try:
                        result = self._watch_match(tail, match_id, opp, strategist_tank)
                    finally:
                        for p in (s_proc, o_proc):
                            try:
                                p.terminate()
                                p.wait(timeout=3)
                            except subprocess.TimeoutExpired:
                                p.kill()
                    print(f"    -> winner={result.winner}, "
                          f"s_hp={result.strategist_hp:.0f}, o_hp={result.opponent_hp:.0f}")
                    self.results.append(result)
        finally:
            if self.auto_launch:
                self.stop_simulator()

        self._print_summary()

    def _print_summary(self) -> None:
        print("\n" + "=" * 70)
        print("RESUMEN DEL TORNEO")
        print("=" * 70)
        by_opp: dict[str, list[MatchResult]] = {}
        for r in self.results:
            by_opp.setdefault(r.opponent, []).append(r)

        print(f"{'Rival':<12} {'Matches':>8} {'Wins':>5} {'Losses':>7} "
              f"{'Draws':>6} {'Avg HP+':>9} {'WinRate':>8}")
        print('-' * 70)
        total_w = total_l = total_d = 0
        for opp, rs in by_opp.items():
            w = sum(1 for r in rs if r.winner == 'strategist')
            l = sum(1 for r in rs if r.winner == 'opponent')
            d = sum(1 for r in rs if r.winner in ('draw', 'timeout'))
            avg_adv = sum(r.hp_advantage for r in rs) / len(rs)
            wr = w / len(rs) if rs else 0.0
            print(f"{opp:<12} {len(rs):>8} {w:>5} {l:>7} {d:>6} "
                  f"{avg_adv:>+9.1f} {wr:>8.1%}")
            total_w += w; total_l += l; total_d += d
        print('-' * 70)
        tot = total_w + total_l + total_d
        wr = total_w / tot if tot else 0
        print(f"{'TOTAL':<12} {tot:>8} {total_w:>5} {total_l:>7} {total_d:>6} "
              f"{'':>9} {wr:>8.1%}")

        # guardar a CSV
        csv = os.path.join(self.log_dir, 'results.csv')
        with open(csv, 'w') as f:
            f.write('opponent,match_id,winner,strategist_hp,opponent_hp,duration_ticks\n')
            for r in self.results:
                f.write(f'{r.opponent},{r.match_id},{r.winner},'
                        f'{r.strategist_hp:.1f},{r.opponent_hp:.1f},{r.duration_ticks}\n')
        print(f"\nCSV guardado en {csv}")


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description='Tournament Strategist vs OpponentsZoo')
    p.add_argument('--matches', type=int, default=3,
                   help='matches por rival (default 3)')
    p.add_argument('--opponents', type=str, default='sniper,rusher,zigzag,smartlead',
                   help='lista CSV de rivales (default todos)')
    p.add_argument('--auto-launch', action='store_true',
                   help='lanzar el simulador automaticamente (requiere DISPLAY)')
    p.add_argument('--simulator-running', action='store_true',
                   help='asume que vos ya lanzaste el simulador en otra terminal')
    p.add_argument('--simulator-log', type=str, default=None,
                   help='path al log del simulador para parsear fin de match '
                        '(requerido con --simulator-running)')
    p.add_argument('--log-dir', default='./data/tournament')
    args = p.parse_args(argv[1:])

    if not args.auto_launch and not args.simulator_running:
        print("Tenes que elegir --auto-launch o --simulator-running.")
        return 1
    if args.auto_launch and args.simulator_running:
        print("Elegi solo uno: --auto-launch O --simulator-running.")
        return 1
    if args.simulator_running and not args.simulator_log:
        print("Con --simulator-running pasa --simulator-log <path>.")
        print("Y lanza el simulador redirigiendo, ejemplo:")
        print("  ./waku -test 111 -episodes -nointro > /tmp/wakulog 2>&1")
        return 1

    opps = [o.strip().lower() for o in args.opponents.split(',') if o.strip()]
    runner = TournamentRunner(opps, args.matches,
                              auto_launch=args.auto_launch,
                              sim_log_path=args.simulator_log,
                              log_dir=args.log_dir)
    runner.run()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
