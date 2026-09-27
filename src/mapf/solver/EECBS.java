package mapf.solver;

import mapf.core.DistanceCache;

/**
 * EECBS - Explicit Estimation Conflict-Based Search (Li, Ruml, Koenig,
 * "EECBS: A Bounded-Suboptimal Search for Multi-Agent Path Finding",
 * NeurIPS-SoCS 2021; JAIR 2024).
 *
 * Same two-level focal architecture as the ECBS baseline but with the
 * explicit-estimation low level: the admission bound is
 * w * max(inherited node lower bound, current low-level lower bound),
 * and each high-level node's lower bound is updated with the bound returned
 * by its low-level search. This explicit estimation is EECBS's central
 * contribution and is what we implement; auxiliary speedups (MDD-based
 * conflict prioritization, disjoint splitting, bypassing, rectangle/target
 * reasoning) are not implemented and the manuscript documents this.
 */
public final class EECBS extends FocalCBS {
    public EECBS(DistanceCache distances, double w) {
        super(distances, w, "EECBS");
    }
    @Override protected SpaceTimeAStar.Mode lowLevelMode() {
        return SpaceTimeAStar.Mode.EXPLICIT;
    }
}
