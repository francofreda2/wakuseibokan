#!/bin/bash
# run-matches.sh — orquesta multiples matches del Strategist vs un rival.
#
# Uso:
#   scripts/run-matches.sh [num_matches] [rival] [scenario]
#
# Defaults:
#   num_matches=5
#   rival=smartlead  (sniper|rusher|zigzag|smartlead)
#   scenario=131     (111|131)
#
# Output:
#   data/run/sim.log         stdout completo del simulador
#   data/run/strat.log       stdout del Strategist
#   data/run/opp.log         stdout del oponente
#   data/run/results.csv     una linea por match con HP final
#
# Requiere que el binario "testcase" haya sido compilado con el escenario
# elegido (make testcase TC=131). El script chequea y avisa si esta
# compilado con otro scenario.

set -u

cd "$(dirname "$0")/.."

NUM_MATCHES="${1:-5}"
RIVAL="${2:-smartlead}"
SCENARIO="${3:-131}"

mkdir -p data/run
RESULTS=data/run/results.csv

# Reset CSV
echo "match_id,scenario,rival,strategist_hp,opponent_hp,duration_ticks" > "$RESULTS"

# Sanity check: el binario testcase tiene el escenario correcto?
COMPILED_TC=$(strings testcase 2>/dev/null | grep -oE "TestCase_[0-9]+" | head -1 | grep -oE "[0-9]+")
if [ "$COMPILED_TC" != "$SCENARIO" ]; then
  echo "AVISO: testcase compilado con TC=$COMPILED_TC pero pediste $SCENARIO"
  echo "       recompila con: make testcase TC=$SCENARIO"
  echo "       sigo igual con TC=$COMPILED_TC."
  SCENARIO=$COMPILED_TC
fi

cleanup() {
  echo ""
  echo "[wrapper] limpiando procesos..."
  pkill -9 -P $$ 2>/dev/null
  pkill -9 -f "scripts/Strategist.py" 2>/dev/null
  pkill -9 -f "scripts/OpponentsZoo.py" 2>/dev/null
  pkill -9 -f "./testcase" 2>/dev/null
  exit 0
}
trap cleanup INT TERM

start_simulator() {
  : > data/run/sim.log
  LD_LIBRARY_PATH=/usr/local/lib stdbuf -oL ./testcase -episodes -nointro -mute \
    >> data/run/sim.log 2>&1 &
  SIM_PID=$!
  echo "[wrapper] simulador PID=$SIM_PID (TC=$SCENARIO, -episodes)"
  sleep 5
  if ! kill -0 $SIM_PID 2>/dev/null; then
    echo "[wrapper] ERROR: simulador no arranco"
    tail data/run/sim.log
    return 1
  fi
}

start_agents() {
  : > data/run/strat.log
  : > data/run/opp.log
  python3 -u scripts/Strategist.py 1 >> data/run/strat.log 2>&1 &
  STRAT_PID=$!
  python3 -u scripts/OpponentsZoo.py "$RIVAL" 2 >> data/run/opp.log 2>&1 &
  OPP_PID=$!
  echo "[wrapper] Strategist PID=$STRAT_PID, $RIVAL PID=$OPP_PID"
}

wait_for_match_end() {
  # Espero hasta que aparezcan las dos lineas Faction:N para el match actual.
  # Despues de "Cleaning up sceneario to start it over again" arranca el proximo.
  local prev_factions
  prev_factions=$(grep "^Faction: " data/run/sim.log 2>/dev/null | wc -l)
  local timeout=400  # segundos wall-clock por match
  local elapsed=0
  while [ $elapsed -lt $timeout ]; do
    sleep 5
    elapsed=$((elapsed + 5))
    if ! kill -0 $SIM_PID 2>/dev/null; then
      echo "[wrapper] simulador murio durante el match"
      return 1
    fi
    local now_factions
    now_factions=$(grep "^Faction: " data/run/sim.log 2>/dev/null | wc -l)
    if [ "$now_factions" -ge $((prev_factions + 2)) ]; then
      return 0
    fi
  done
  echo "[wrapper] timeout wall-clock esperando fin de match"
  return 1
}

parse_last_match() {
  # Devuelve HP1, HP2, duration (ticks) del ultimo match completo
  local last_two
  last_two=$(grep "^Faction: " data/run/sim.log | tail -2)
  if [ "$(echo "$last_two" | wc -l)" -lt 2 ]; then
    echo "0,0,0"
    return
  fi
  local hp1 hp2 dist
  hp1=$(echo "$last_two" | grep "Faction: 1" | sed -E 's/.*Health: *([0-9.]+).*/\1/')
  hp2=$(echo "$last_two" | grep "Faction: 2" | sed -E 's/.*Health: *([0-9.]+).*/\1/')
  dist=5000   # DEFAULT_MATCH_DURATION
  echo "${hp1:-0},${hp2:-0},$dist"
}

# ---- Main ----

start_simulator || exit 1
start_agents

declare -i wins=0 losses=0 draws=0

for i in $(seq 1 $NUM_MATCHES); do
  echo ""
  echo "=== Match $i/$NUM_MATCHES (Strategist vs $RIVAL) ==="
  wait_for_match_end
  if [ $? -ne 0 ]; then
    echo "[wrapper] match $i fallo, intentando reiniciar..."
    pkill -9 -f "scripts/Strategist.py" 2>/dev/null
    pkill -9 -f "scripts/OpponentsZoo.py" 2>/dev/null
    pkill -9 -f "./testcase" 2>/dev/null
    sleep 2
    start_simulator || exit 1
    start_agents
    continue
  fi

  IFS=',' read -r hp1 hp2 dur <<<"$(parse_last_match)"
  printf "%d,%s,%s,%s,%s,%s\n" "$i" "$SCENARIO" "$RIVAL" "$hp1" "$hp2" "$dur" >> "$RESULTS"

  if (( $(echo "$hp1 > $hp2 + 10" | bc -l) )); then
    echo "[wrapper] Match $i: Strategist GANA (hp1=$hp1 vs hp2=$hp2)"
    wins=$((wins+1))
  elif (( $(echo "$hp2 > $hp1 + 10" | bc -l) )); then
    echo "[wrapper] Match $i: $RIVAL gana (hp1=$hp1 vs hp2=$hp2)"
    losses=$((losses+1))
  else
    echo "[wrapper] Match $i: EMPATE (hp1=$hp1 vs hp2=$hp2)"
    draws=$((draws+1))
  fi
done

echo ""
echo "=========================================="
echo "RESUMEN final ($NUM_MATCHES matches vs $RIVAL en TC=$SCENARIO):"
echo "  Strategist: $wins wins, $losses losses, $draws draws"
echo "  WinRate: $(echo "scale=1; $wins*100/$NUM_MATCHES" | bc)%"
echo "=========================================="
echo "CSV: $RESULTS"

cleanup
