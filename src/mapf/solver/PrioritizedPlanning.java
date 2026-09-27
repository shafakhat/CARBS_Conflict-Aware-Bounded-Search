package mapf.solver;

import mapf.core.Agent;
import mapf.core.AgentConstraints;
import mapf.core.GridMap;
import mapf.core.Instance;
import mapf.core.Metrics;
import mapf.core.Path;
import mapf.core.DistanceCache;
import mapf.core.Result;
import mapf.core.Solver;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Prioritized Planning (cooperative A*; Silver 2005; Wang & Botea 2011 style
 * baseline widely used in the MAPF literature). Agents are planned one at a
 * time in a fixed priority order (benchmark scenario order); each agent must
 * avoid the earlier agents: their occupied (cell, time) pairs, their swap
 * moves, and - after arrival - their goal cell forever (agents park at their
 * goals). No backtracking: if some agent cannot be planned, the whole
 * instance fails for this ordering.
 */
public final class PrioritizedPlanning implements Solver {

    private final DistanceCache distances;

    public PrioritizedPlanning(DistanceCache distances) {
        this.distances = distances;
    }

    @Override public String name() { return "PP"; }

    @Override public Result solve(Instance inst, long deadlineNanos, Metrics metrics) {
        final long t0 = System.nanoTime();
        GridMap map = inst.map;
        int k = inst.numAgents();

        List<Path> planned = new ArrayList<>(k);
        // occupied vertices: key = cell<<20|t, plus permanent goal parking: cell<<1|1
        Set<Long> vertexOcc = new HashSet<>();
        Set<Integer> parkedGoals = new HashSet<>();
        Set<Long> edgeOcc = new HashSet<>();
        final int[] maxArrival = new int[1];

        for (Agent a : inst.agents) {
            final Set<Long> vOcc = vertexOcc;
            final Set<Integer> park = parkedGoals;
            final Set<Long> eOcc = edgeOcc;
            SpaceTimeAStar.Obstacles obstacles = new SpaceTimeAStar.Obstacles() {
                @Override public boolean vertexBlocked(int cell, int t) {
                    return park.contains(cell) || vOcc.contains(((long) cell << 20) | t);
                }
                @Override public boolean edgeBlocked(int from, int to, int t) {
                    // earlier agents' moves are stored as (theirFrom -> theirTo);
                    // a swap collision for THIS agent (from -> to at t) occurs iff
                    // some earlier agent makes the opposite move (to -> from) at t
                    return eOcc.contains(mapf.core.AgentConstraints.packEdge(to, from, t));
                }
                @Override public int latestRestrictionTime() { return maxArrival[0]; }
            };

            SpaceTimeAStar.LLResult ll = SpaceTimeAStar.search(
                    map, a, obstacles, distances.distancesTo(a.goal),
                    SpaceTimeAStar.Mode.OPTIMAL, 1.0, 0, null, a.id, deadlineNanos, metrics);
            metrics.lowLevelSearches++;
            if (ll.path == null) {
                String why = ll.lowerBound == -1 ? "TIMEOUT" : "NO_SOLUTION";
                return Result.failure(why, System.nanoTime() - t0, metrics, w());
            }
            Path p = ll.path;
            planned.add(p);
            for (int t = 0; t < p.cells.length; t++) {
                vertexOcc.add(((long) p.cells[t] << 20) | t);
                if (t > 0) edgeOcc.add(AgentConstraints.packEdge(p.cells[t - 1], p.cells[t], t));
            }
            parkedGoals.add(a.goal);
            maxArrival[0] = Math.max(maxArrival[0], p.arrivalTime());
        }

        Path[] paths = planned.toArray(new Path[0]);
        int soc = mapf.core.Validator.sumOfCosts(paths);
        int makespan = mapf.core.Validator.makespan(paths);
        return new Result(true, "SOLVED", paths, soc, makespan,
                System.nanoTime() - t0, metrics, w());
    }

    private static double w() { return 1.0; }
}
