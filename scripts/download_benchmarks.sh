#!/usr/bin/env bash
# Download the official MovingAI MAPF benchmark maps and scenario files.
# Provenance: https://www.movingai.com/benchmarks/mapf/index.html
# Citation: Stern et al., "Multi-Agent Pathfinding: Definitions, Variants, and Benchmarks",
#           SoCS 2019.  (Downloaded 2026-09-13)
set -euo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"
cd "$HERE/benchmarks"
BASE="https://www.movingai.com/benchmarks/mapf"

curl -L -o mapf-map.zip          "$BASE/mapf-map.zip"
curl -L -o mapf-scen-even.zip    "$BASE/mapf-scen-even.zip"
curl -L -o mapf-scen-random.zip  "$BASE/mapf-scen-random.zip"

unzip -q mapf-map.zip         -d maps
unzip -q mapf-scen-even.zip   -d scen
unzip -q mapf-scen-random.zip -d scen
rm -f mapf-map.zip mapf-scen-even.zip mapf-scen-random.zip
echo "Done: $(ls maps | wc -l) maps, $(ls scen/scen-even | wc -l) even scen, $(ls scen/scen-random 2>/dev/null | wc -l) random scen"
