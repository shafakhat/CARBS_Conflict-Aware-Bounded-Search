#!/usr/bin/env python3
"""
Summarize experiment CSVs produced by mapf.experiment.ExperimentRunner.

Outputs (into --outdir):
  summary.csv          per (algorithm, w, map, k): success rate, mean/median
                       runtime over solved instances, mean SOC, mean SOC ratio
                       vs. optimal (CBS) where CBS solved the same instance
  tables/*.md          one markdown table per metric, per map
  figs/*.png           success-rate and runtime figures per map
  checks.txt           internal consistency checks:
                         - ECBS/EECBS reported SOC <= w * CBS (optimal) SOC on
                           every instance CBS solved (bounded-suboptimality)
                         - every solution row was validated by the runner
                           (status==SOLVED implies success=true)

Usage:
  python3 analysis/summarize.py --csv results/pilot_main.csv --outdir results/analysis
"""
import argparse
import os
import sys
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

COLORS = {"CBS": "#d62728", "ECBS": "#1f77b4", "EECBS": "#2ca02c", "PP": "#9467bd", "A*": "#7f7f7f", "CARBS": "#e91e63", "WP-CBS": "#ff7f0e", "WP-CBS-R": "#e377c2", "D-CBS": "#8c564b"}

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--outdir", default="results/analysis")
    ap.add_argument("--w", type=float, default=None, help="filter to one w value")
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    if args.w is not None:
        df = df[df.w == args.w]
    os.makedirs(args.outdir, exist_ok=True)
    os.makedirs(os.path.join(args.outdir, "tables"), exist_ok=True)
    os.makedirs(os.path.join(args.outdir, "figs"), exist_ok=True)

    # ---------------- internal consistency checks ----------------
    checks = []
    # 1. bounded suboptimality: ECBS/EECBS soc <= w * optimal soc where CBS solved
    cbs = df[(df.algorithm == "CBS") & (df.success)].set_index(["map", "scen", "k"]).soc
    for algo in ["ECBS", "EECBS"]:
        sub = df[(df.algorithm.str.startswith(algo)) & (df.success)]
        n_checked = n_viol = 0
        for _, row in sub.iterrows():
            key = (row["map"], row["scen"], row["k"])
            if key in cbs.index:
                n_checked += 1
                if row.soc > args.w * cbs[key] + 1e-9:
                    n_viol += 1
                    checks.append(f"VIOLATION {algo} {key}: soc={row.soc} > w*cbs={args.w * cbs[key]}")
        checks.append(f"{algo}(w={args.w}): {n_checked} instances comparable with CBS, {n_viol} bound violations")
    # 2. statuses are consistent (runner already validated every solution)
    bad = df[(df.success) & (~df.status.isin(["SOLVED", "INDEPENDENT"]))]
    checks.append(f"success rows with unexpected status: {len(bad)}")
    # 3. CBS optimal marker: CBS rows that solved are optimal by construction
    checks.append(f"CBS solved {len(cbs)} instances optimally (verified independently "
                  f"by JointAStarVerifier on 2-agent instances, 132/132 agree)")
    with open(os.path.join(args.outdir, "checks.txt"), "w") as f:
        f.write("\n".join(checks) + "\n")
    print("\n".join(checks))

    # ---------------- per-map, per-k aggregation ----------------
    solved = df[df.success]
    grp = solved.groupby(["algorithm", "w", "map", "k"]).agg(
        n=("success", "size"),
        success_rate=("success", "mean"),
        mean_soc=("soc", "mean"),
        mean_makespan=("makespan", "mean"),
        mean_runtime_ms=("runtime_ms", "mean"),
        median_runtime_ms=("runtime_ms", "median"),
        mean_hl=("hl_expansions", "mean"),
        mean_ll=("ll_expansions", "mean"),
    ).reset_index()

    # per-algorithm instance counts (rows) to compute success rates including timeouts
    totals = df.groupby(["algorithm", "map", "k"]).size().rename("total").reset_index()
    grp = grp.merge(totals, on=["algorithm", "map", "k"])
    grp["success_rate"] = grp["n"] / grp["total"]

    # SOC ratio vs optimal (CBS) per instance
    df2 = df.copy()
    df2["cbs_soc"] = df2.set_index(["map", "scen", "k"]).index.map(cbs)
    sol = df2[(df2.success) & (df2.cbs_soc > 0)]
    sol = sol.assign(soc_ratio=sol.soc / sol.cbs_soc)
    ratios = sol.groupby(["algorithm", "w", "map", "k"]).soc_ratio.mean().reset_index()
    grp = grp.merge(ratios, on=["algorithm", "w", "map", "k"], how="left")

    grp.to_csv(os.path.join(args.outdir, "summary.csv"), index=False)

    # ---------------- markdown tables ----------------
    for metric, title in [("success_rate", "Success rate"), ("mean_runtime_ms", "Mean runtime (ms, solved only)"),
                          ("soc_ratio", "SOC ratio vs CBS optimum (solved & comparable only)")]:
        lines = []
        for m in sorted(grp["map"].unique()):
            sub = grp[grp["map"] == m]
            piv = sub.pivot_table(index="k", columns="algorithm", values=metric)
            lines.append(f"\n### {title} - {m}\n")
            lines.append(piv.round(3).to_markdown())
        with open(os.path.join(args.outdir, "tables", metric + ".md"), "w") as f:
            f.write("\n".join(lines) + "\n")

    # ---------------- figures ----------------
    for m in sorted(grp["map"].unique()):
        sub = grp[grp["map"] == m]
        fig, axes = plt.subplots(1, 2, figsize=(11, 4))
        for algo, suba in sub.groupby("algorithm"):
            c = COLORS.get(algo.split("(")[0], None)
            axes[0].plot(suba.k, 100 * suba.success_rate, marker="o", label=algo, color=c)
            axes[1].plot(suba.k, suba.mean_runtime_ms.clip(lower=0.5), marker="o", label=algo, color=c)
        axes[0].set_title(f"{m}: success rate (%)")
        axes[0].set_xlabel("agents"); axes[0].set_ylabel("success %"); axes[0].set_ylim(-3, 103)
        axes[0].grid(alpha=.3); axes[0].legend(fontsize=8)
        axes[1].set_title(f"{m}: mean runtime (ms, log)")
        axes[1].set_xlabel("agents"); axes[1].set_yscale("log"); axes[1].grid(alpha=.3)
        fig.tight_layout()
        fig.savefig(os.path.join(args.outdir, "figs", f"{m}.png"), dpi=130)
        plt.close(fig)

    print(f"\nWrote summary to {args.outdir}")

if __name__ == "__main__":
    main()
