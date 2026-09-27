#!/usr/bin/env bash
# Full reproducible pipeline: build, implementation validation, benchmark, analysis.
set -euo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"
cd "$HERE"
CP="classes:lib/jgrapht-core-1.5.2.jar:lib/jheaps-0.14.jar"

echo "== 0. build =="
javac -cp "lib/jgrapht-core-1.5.2.jar:lib/jheaps-0.14.jar" -d classes $(find src -name "*.java")

echo "== 1. implementation validation: CBS vs independent joint-space search (k=2) =="
for m in maze-32-32-2 empty-32-32 random-32-32-10 random-32-32-20 room-32-32-4 \
         warehouse-10-20-10-2-1 warehouse-20-40-10-2-1 den312d; do
  java -cp "$CP" mapf.experiment.JointAStarVerifier \
    "benchmarks/maps/$m.map" "benchmarks/scen/scen-even/$m-even-1.scen" 12
done

echo "== 2. w=1.0 optimal-mode equivalence check =="
java -cp "$CP" mapf.experiment.ExperimentRunner \
  --maps random-32-32-10 --scen even --scens 2 --agents 4,6,8,10 \
  --algos cbs,ecbs,eecbs --w 1.0 --timeout-ms 30000 --out results/check_w1.csv

echo "== 3. main benchmark grid =="
java -Xmx4g -cp "$CP" mapf.experiment.ExperimentRunner \
  --maps empty-32-32,random-32-32-20,maze-32-32-4,room-32-32-4,warehouse-10-20-10-2-1 \
  --scen even --scens 5 \
  --agents 2,4,6,8,10,12,15,20,25,30 \
  --algos cbs,ecbs,eecbs,pp,carbs --w 1.2 --timeout-ms 60000 --out results/main.csv

echo "== 4. analysis =="
python3 analysis/summarize.py --csv results/main.csv --outdir results/analysis
echo "Done."
