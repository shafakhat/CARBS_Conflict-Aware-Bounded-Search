# CARBS — Conflict-Aware Bounded Search for Multi-Agent Path Finding

CARBS is a bounded-suboptimal solver for Multi-Agent Path Finding (MAPF),
built on the Conflict-Based Search (CBS) family. It achieves adaptive,
"agile" search behavior through mechanisms that provably preserve the
constraint-tree invariants required for a bounded-suboptimality guarantee:

- **MDD-based conflict prioritization** — cardinal conflicts (both agents'
  path sets pass through a single cell at the conflict time) are resolved
  first, then semi-cardinal, then earliest (cf. Boyarski et al. 2015;
  Li et al. 2019).
- **Conflict bypassing** — non-cardinal conflicts whose constraint can be
  absorbed by an equal-cost replan are merged into the parent node without
  splitting the search tree (cf. Barer et al. 2014).
- **Bounded explicit-estimation focal search** — the high level expands nodes
  within a w-bounded cost window using sound lower bounds (the ECBS/EECBS
  mechanism), with an agility tie-break preferring nodes whose agents are
  closer to their goals.
- **True-distance low level** — space-time A* with BFS distance-to-goal
  heuristics and an explicit-estimation admission bound.

**Guarantee.** With w >= 1, CARBS returns only conflict-free solutions with
sum-of-costs <= w * C* (C* = optimal sum-of-costs), or reports timeout; it
never returns an invalid solution. Every solution in every experiment was
re-validated by an independent post-hoc validator.

## Repository layout

```
src/               Java source (framework + solvers)
  mapf/core/       map, scenario, instance, validator, distance cache
  mapf/solver/     CBS, ECBS, EECBS, PP baselines; WP-CBS design point; CARBS
  mapf/experiment/ ExperimentRunner, JointAStarVerifier (optimality cross-check)
analysis/          summarize.py - tables, figures, integrity checks
scripts/           benchmark download + full reproduction pipeline
results/           per-instance raw data (CSV) for the reported experiments
lib/               JGraphT 1.5.2 + JHeaps 0.14 (runtime dependencies)
```

## Algorithms (CLI names)

| name | description |
|---|---|
| `cbs` | optimal CBS (Sharon et al. 2015), vanilla reference |
| `ecbs` | ECBS focal search (Barer et al. 2014) |
| `eecbs` | EECBS explicit-estimation focal search (Li et al. 2021) |
| `pp` | Prioritized Planning (Silver 2005; no backtracking) |
| `wp`, `wpr`, `dcbs` | waypoint/strategy design points from the design-rationale study |
| `carbs` | **CARBS (this work)** |
| `carbs-nb`, `carbs-nm` | CARBS ablations (no bypass / no MDD selection) |

Conventions (identical for every solver): parking model (agents rest at their
goals and block others), sum-of-arrival-times objective, BFS true-distance
heuristic, official MovingAI scenario files with the standard first-k-rows
instance convention, per-instance wall-clock timing with in-search deadline
checks.

## Correctness assurance

1. **Independent validator** — every solution of every solver is re-checked
   (start/goal, move legality, obstacles, pairwise vertex/edge collisions over
   the full horizon, SOC recomputation). Invalid outputs are recorded as
   `INVALID` and never counted.
2. **Joint-space optimality cross-check** — for 2-agent instances, an exact
   SOC-optimal search over the joint state space (no code shared with CBS)
   verifies CBS optimality: 132/132 agreement across 11 maps.
3. **w=1.0 equivalence** — ECBS(w=1.0) and EECBS(w=1.0) must reproduce CBS
   optimal costs; checked automatically (results/check_w1.csv).
4. **Bound check** — analysis verifies solvers' SOC <= w * CBS-optimal on all
   comparable instances (0 violations in the released data).

## Reproducing

```bash
sudo apt-get install -y default-jdk libjgrapht-java   # or place jars in lib/
./scripts/download_benchmarks.sh                      # official MovingAI MAPF bench
./scripts/run_all.sh                                  # build + validate + run + analyze
```

Results in `results/` were produced on a 2-vCPU/2 GB JVM sandbox, 20 s timeout,
single thread; `analysis/summarize.py` regenerates all tables and figures from
the per-instance CSVs.

## Data files

- `results/scaling_main.csv` — scaling grid: 5 maps x k in
  {10,20,30,40,50,75,100} x 2 scenarios x {CBS, ECBS, EECBS, PP, CARBS}
- `results/design_study.csv` — design-rationale study incl. waypoint/strategy
  design points
- `results/check_w1.csv` — w=1.0 optimal-equivalence check

## License

MIT (see LICENSE).

## Key references

- Sharon, Stern, Felner, Sturtevant. Conflict-Based Search for Optimal
  Multi-Agent Pathfinding. AIJ 219 (2015).
- Barer, Sharon, Stern, Felner. Suboptimal Variants of the Conflict-Based
  Search Algorithm. ECAI 2014 / AIJ 2019.
- Li, Ruml, Koenig. EECBS: A Bounded-Suboptimal Search for Multi-Agent Path
  Finding. AAAI 2021 / JAIR.
- Li, Chen, Harabor, Stuckey, Koenig. MAPF-LNS2. AAAI 2022.
- Stern et al. Multi-Agent Pathfinding: Definitions, Variants, and Benchmarks.
  SoCS 2019 (benchmark suite).

## Data
All per-instance result data behind the paper's numbers is in
`results/` (see `results/MANIFEST.md`). Rebuild: `javac`, then run
`scripts/run_all.sh` per map (see scripts/), or use the released CSVs directly.
