#!/bin/bash
# hunter-battery.sh — corre Hunter contra cada rival del OpponentsZoo
# y registra resultados.
#
# Uso: scripts/hunter-battery.sh [matches_per_rival]
#
# Salida:
#   data/battery/<rival>.csv      win/loss/draw por rival
#   data/battery/summary.csv      resumen agregado
#   data/battery/<rival>_<n>.log  log del match individual

set -u

cd "$(dirname "$0")/.."

MATCHES_PER_RIVAL="${1:-5}"
RIVALS="smartlead sniper rusher zigzag"
SPEEDUP=5

mkdir -p data/battery
rm -f data/battery/*

SUMMARY=data/battery/summary.csv
echo "rival,matches,hunter_wins,rival_wins,draws,hunter_total_hits,rival_total_hits,hunter_avg_hp,rival_avg_hp" > "$SUMMARY"

cleanup() {
  pkill -9 -f "MockSimulator" 2>/dev/null
  pkill -9 -f "Hunter.py" 2>/dev/null
  pkill -9 -f "OpponentsZoo.py" 2>/dev/null
}
trap cleanup INT TERM EXIT

for rival in $RIVALS; do
  echo ""
  echo "===================================================="
  echo "Hunter vs $rival ($MATCHES_PER_RIVAL matches)"
  echo "===================================================="

  RIVAL_CSV=data/battery/${rival}.csv
  echo "match_id,hunter_hp,rival_hp,hunter_hits,rival_hits,winner" > "$RIVAL_CSV"

  hunter_wins=0
  rival_wins=0
  draws=0
  total_h_hits=0
  total_r_hits=0
  total_h_hp=0
  total_r_hp=0

  # Lanzar mock en episodes mode
  cleanup
  sleep 1
  python3 -u scripts/MockSimulator.py -episodes --speedup $SPEEDUP \
    > data/battery/mock_${rival}.log 2>&1 &
  MOCK_PID=$!
  sleep 2

  if ! kill -0 $MOCK_PID 2>/dev/null; then
    echo "[error] mock no arranco para rival $rival"
    continue
  fi

  python3 -u scripts/Hunter.py 1 > data/battery/${rival}_hunter.log 2>&1 &
  HUNTER_PID=$!
  python3 -u scripts/OpponentsZoo.py "$rival" 2 > data/battery/${rival}_opp.log 2>&1 &
  RIVAL_PID=$!

  # Esperar matches
  prev_match=0
  prev_hits_h=0
  prev_hits_r=0
  start=$(date +%s)
  while [ $prev_match -lt $MATCHES_PER_RIVAL ]; do
    sleep 4
    now=$(date +%s)
    elapsed=$((now - start))
    if [ $elapsed -gt 300 ]; then
      echo "[timeout] $rival: solo $prev_match/$MATCHES_PER_RIVAL matches completados"
      break
    fi
    matches=$(grep "^Faction: " data/battery/mock_${rival}.log 2>/dev/null | wc -l)
    matches=$((matches / 2))
    if [ "$matches" -gt "$prev_match" ]; then
      # nuevo match completo
      match_id=$matches
      hp1=$(grep "^Faction: 1" data/battery/mock_${rival}.log | tail -1 | sed -E 's/.*Health: *([0-9.]+).*/\1/')
      hp2=$(grep "^Faction: 2" data/battery/mock_${rival}.log | tail -1 | sed -E 's/.*Health: *([0-9.]+).*/\1/')
      hits_h_now=$(grep -c "HIT! T1 -> T2" data/battery/mock_${rival}.log 2>/dev/null)
      hits_r_now=$(grep -c "HIT! T2 -> T1" data/battery/mock_${rival}.log 2>/dev/null)
      hits_h_match=$((hits_h_now - prev_hits_h))
      hits_r_match=$((hits_r_now - prev_hits_r))
      prev_hits_h=$hits_h_now
      prev_hits_r=$hits_r_now

      winner="draw"
      if [ "$(echo "$hp1 > $hp2 + 10" | bc -l)" = "1" ]; then
        winner="hunter"
        hunter_wins=$((hunter_wins+1))
      elif [ "$(echo "$hp2 > $hp1 + 10" | bc -l)" = "1" ]; then
        winner="rival"
        rival_wins=$((rival_wins+1))
      else
        draws=$((draws+1))
      fi

      echo "  match $match_id: hunter=$hp1 / $rival=$hp2 hits(H=$hits_h_match R=$hits_r_match) -> $winner"
      echo "$match_id,$hp1,$hp2,$hits_h_match,$hits_r_match,$winner" >> "$RIVAL_CSV"

      total_h_hits=$((total_h_hits + hits_h_match))
      total_r_hits=$((total_r_hits + hits_r_match))
      total_h_hp=$(echo "$total_h_hp + $hp1" | bc -l)
      total_r_hp=$(echo "$total_r_hp + $hp2" | bc -l)
      prev_match=$match_id
    fi
  done

  # Calcular promedios
  if [ $prev_match -gt 0 ]; then
    avg_h_hp=$(echo "scale=1; $total_h_hp / $prev_match" | bc -l)
    avg_r_hp=$(echo "scale=1; $total_r_hp / $prev_match" | bc -l)
  else
    avg_h_hp="0"; avg_r_hp="0"
  fi

  echo ""
  echo "Resumen vs $rival: H=$hunter_wins / R=$rival_wins / D=$draws  "
  echo "  hits totales H=$total_h_hits  R=$total_r_hits  "
  echo "  HP promedio H=$avg_h_hp  R=$avg_r_hp"

  echo "$rival,$prev_match,$hunter_wins,$rival_wins,$draws,$total_h_hits,$total_r_hits,$avg_h_hp,$avg_r_hp" >> "$SUMMARY"

  cleanup
  sleep 1
done

echo ""
echo "===================================================="
echo "BATERIA COMPLETA"
echo "===================================================="
cat "$SUMMARY"
