package mapf.solver;

import mapf.core.DistanceCache;

/**
 * ECBS - Enhanced Conflict-Based Search (Barer, Sharon, Stern, Felner,
 * "Suboptimal Variants of the Conflict-Based Search Algorithm for the Optimal
 * Multi-Agent Pathfinding Problem", ECAI 2014; corrected lower-bound treatment
 * in the 2019 journal extension).
 *
 * Two-level focal search. Low level: Barer-style focal search - states are
 * admitted with f <= w * (low-level lower bound), secondary criterion =
 * collisions with the other agents' current paths. High level: nodes with
 * cost <= w * min lb are popped by fewest conflicts.
 */
public final class ECBS extends FocalCBS {
    public ECBS(DistanceCache distances, double w) {
        super(distances, w, "ECBS");
    }
    @Override protected SpaceTimeAStar.Mode lowLevelMode() {
        return SpaceTimeAStar.Mode.FOCAL;
    }
}
