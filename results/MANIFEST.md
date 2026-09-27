# Per-instance result data (all numbers in the paper trace here)

Frozen-build full re-run, 14 Sep 2026. Source-code md5 checksums:
`source_checksums_FROZEN.txt` (30 files, re-verified before and after).

| File | Rows | What it is |
|---|---|---|
| scaling_main.csv | 350 | Scaling grid: 5 maps x 2 even scenarios x k in {10,20,30,40,50,75,100} x {CARBS, ECBS, EECBS, PP, CBS}; w=1.2, 20 s, single thread |
| design_study.csv | 588 | Design study (84 instances x 7 solvers incl. WP-CBS, WP-CBS-R, D-CBS) |
| check_w1.csv | 24 | w=1.0 equivalence check (ECBS/EECBS vs CBS optima) |
| ablation_flags_resolved.csv | 48 | CARBS / CARBS-NB / CARBS-NM ablations on the 16 hardest corridor cells (labels recovered by row order) |
| wsweep_1.1.csv / _1.5 / _2.0 | 16 each | CARBS bound sweep on the 16 corridor cells (Table: 6/7/11/15 of 16) |
| lns_*.csv | 9 | CARBS side of the MAPF-LNS2 comparison (30 s both) |

Columns: success = validated solve; status in {SOLVED, TIMEOUT, NO_SOLUTION, ...};
soc = sum of costs; runtime_ms; hl/ll = high-/low-level counters.
Every solution was re-validated post hoc (0 invalid anywhere).
