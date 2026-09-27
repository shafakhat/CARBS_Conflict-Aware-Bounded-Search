package mapf.solver;

import mapf.core.Agent;
import mapf.core.AgentConstraints;
import mapf.core.Conflict;
import mapf.core.Instance;
import mapf.core.Metrics;
import mapf.core.Path;
import mapf.core.DistanceCache;
import mapf.core.Result;
import mapf.core.Solver;

import java.util.PriorityQueue;

/**
 * Conflict-Based Search - the optimal algorithm of
 * Sharon, Stern, Felner, Sturtevant, "Conflict-Based Search for Optimal
 * Multi-Agent Pathfinding", Artificial Intelligence 219 (2015).
 *
 * High level: best-first search on the constraint tree, ordered by
 * sum-of-costs (ties: fewer conflicts, then insertion order). Every expanded
 * node's first conflict (earliest time; vertex before edge at equal time) is
 * resolved by branching on the two colliding agents, each child adding one
 * vertex or edge constraint and replanning that agent.
 *
 * No improvements (bypassing, conflict prioritization, disjoint splitting,
 * mutex/rectangle reasoning) are implemented - this is vanilla CBS exactly as
 * published, so that measured behaviour reflects the 2015 algorithm itself.
 */
public final class CBS implements Solver {

    private final DistanceCache distances;
    private final double w;

    public CBS(DistanceCache distances) {
        this(distances, 1.0);
    }

    CBS(DistanceCache distances, double w) {
        this.distances = distances;
        this.w = w;
    }

    @Override public String name() { return w == 1.0 ? "CBS" : "CBS(w=" + w + ")"; }

    @Override public Result solve(Instance inst, long deadlineNanos, Metrics metrics) {
        final long t0 = System.nanoTime();
        int k = inst.numAgents();

        // ---- root: plan each agent optimally with no constraints ----
        CTNode root = new CTNode(k);
        for (Agent a : inst.agents) {
            SpaceTimeAStar.LLResult ll = SpaceTimeAStar.search(
                    inst.map, a, SpaceTimeAStar.fromConstraints(root.constraintsOf(a.id)),
                    distances.distancesTo(a.goal), SpaceTimeAStar.Mode.OPTIMAL,
                    1.0, 0, null, a.id, deadlineNanos, metrics);
            metrics.lowLevelSearches++;
            if (ll.path == null) {
                String why = ll.lowerBound == -1 ? "TIMEOUT" : "NO_SOLUTION";
                return Result.failure(why, System.nanoTime() - t0, metrics, w);
            }
            root.paths[a.id] = ll.path;
        }
        root.recomputeCost();
        root.numConflicts = ConflictDetection.countAll(root.paths);
        root.firstConflict = ConflictDetection.findFirst(root.paths);

        PriorityQueue<CTNode> open = new PriorityQueue<>((x, y) -> {
            if (x.cost != y.cost) return Integer.compare(x.cost, y.cost);
            if (x.numConflicts != y.numConflicts) return Integer.compare(x.numConflicts, y.numConflicts);
            return 0; // FIFO on ties via PriorityQueue stability for equal compare
        });
        open.add(root);

        while (!open.isEmpty()) {
            if ((metrics.highLevelExpansions & 0x1F) == 0 && System.nanoTime() > deadlineNanos)
                return Result.failure("TIMEOUT", System.nanoTime() - t0, metrics, w);

            CTNode node = open.poll();
            metrics.highLevelExpansions++;

            if (node.firstConflict == null) {           // conflict-free: optimal
                long rt = System.nanoTime() - t0;
                return new Result(true, "SOLVED", node.paths, node.cost,
                        mapf.core.Validator.makespan(node.paths), rt, metrics, w);
            }

            Conflict c = node.firstConflict;
            for (int side = 0; side < 2; side++) {
                int agent = side == 0 ? c.agent1 : c.agent2;
                CTNode child = node.child();
                AgentConstraints cons = child.constraintsOf(agent).copy();
                if (c.vertex) cons.addVertex(c.cell, c.time);
                else {
                    int from = side == 0 ? c.cell1From : c.cell2From;
                    int to = side == 0 ? c.cell1To : c.cell2To;
                    cons.addEdge(from, to, c.time);
                }
                child.setConstraints(agent, cons);

                Agent a = inst.agents.get(agent);
                SpaceTimeAStar.LLResult ll = SpaceTimeAStar.search(
                        inst.map, a, SpaceTimeAStar.fromConstraints(cons),
                        distances.distancesTo(a.goal), SpaceTimeAStar.Mode.OPTIMAL,
                        1.0, 0, null, agent, deadlineNanos, metrics);
                metrics.lowLevelSearches++;
                if (ll.path == null) {
                    if (ll.lowerBound == -1)
                        return Result.failure("TIMEOUT", System.nanoTime() - t0, metrics, w);
                    continue;                                // pruned: no path under constraints
                }
                child.paths[agent] = ll.path;
                child.recomputeCost();
                child.numConflicts = ConflictDetection.countAll(child.paths);
                child.firstConflict = ConflictDetection.findFirst(child.paths);
                open.add(child);
            }
        }
        return Result.failure("NO_SOLUTION", System.nanoTime() - t0, metrics, w);
    }
}
