package mapf.solver;

import mapf.core.Agent;
import mapf.core.AgentConstraints;
import mapf.core.Instance;
import mapf.core.Metrics;
import mapf.core.Path;
import mapf.core.DistanceCache;
import mapf.core.Result;
import mapf.core.Solver;

/**
 * Independent A*: plans every agent individually on the static grid,
 * ignoring all other agents (reference lower bound / sanity baseline,
 * reported as such in the manuscript). The joint "solution" almost always
 * contains collisions for k > 1, so the runner records it with
 * status = INDEPENDENT (never counted as a success).
 */
public final class IndependentAStar implements Solver {

    private final DistanceCache distances;

    public IndependentAStar(DistanceCache distances) {
        this.distances = distances;
    }

    @Override public String name() { return "A*"; }

    @Override public Result solve(Instance inst, long deadlineNanos, Metrics metrics) {
        final long t0 = System.nanoTime();
        AgentConstraints none = new AgentConstraints();
        Path[] paths = new Path[inst.numAgents()];
        for (Agent a : inst.agents) {
            SpaceTimeAStar.LLResult ll = SpaceTimeAStar.search(
                    inst.map, a, SpaceTimeAStar.fromConstraints(none),
                    distances.distancesTo(a.goal), SpaceTimeAStar.Mode.OPTIMAL,
                    1.0, 0, null, a.id, deadlineNanos, metrics);
            metrics.lowLevelSearches++;
            if (ll.path == null)
                return Result.failure(ll.lowerBound == -1 ? "TIMEOUT" : "NO_SOLUTION",
                        System.nanoTime() - t0, metrics, 1.0);
            paths[a.id] = ll.path;
        }
        int soc = mapf.core.Validator.sumOfCosts(paths);
        int makespan = mapf.core.Validator.makespan(paths);
        return new Result(true, "INDEPENDENT", paths, soc, makespan,
                System.nanoTime() - t0, metrics, 1.0);
    }
}
