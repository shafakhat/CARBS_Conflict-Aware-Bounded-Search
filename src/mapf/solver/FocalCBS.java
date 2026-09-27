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

import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;

/**
 * Shared high level of ECBS and EECBS: two-level focal search with explicit
 * lower-bound estimation.
 *
 * OPEN is ordered by the node lower bound lb(N). FOCAL contains nodes with
 * cost(N) <= w * min lb over OPEN and is popped by fewest conflicts. Each
 * node's lb is lb = max(lb(parent), lb returned by its last low-level search),
 * a sound explicit estimate: both quantities are lower bounds on the optimal
 * cost achievable in the node's subtree.
 *
 * When a conflict-free node is popped, cost <= w * minLb <= w * C*, which
 * yields the w-bounded-suboptimality guarantee for the sum-of-costs objective.
 * This is the corrected bound bookkeeping of Barer et al. (2019) and the core
 * mechanism of Li, Ruml, Koenig, "EECBS: Bounded-Suboptimal Search for
 * Multi-Agent Path Finding" (2021).
 *
 * Deliberately NOT implemented (kept out of both baselines so measured
 * behaviour is attributable to the core algorithms alone, and documented as
 * such in the manuscript): MDD-based conflict prioritization, disjoint
 * splitting, bypassing, rectangle/target/mirror reasoning, mutex propagation.
 */
public abstract class FocalCBS implements Solver {

    protected final DistanceCache distances;
    protected final double w;
    private final String label;

    protected FocalCBS(DistanceCache distances, double w, String label) {
        this.distances = distances;
        this.w = w;
        this.label = label;
    }

    /** FOCAL (Barer) vs EXPLICIT (EECBS) low-level admission rule. */
    protected abstract SpaceTimeAStar.Mode lowLevelMode();

    @Override public final String name() { return label + "(w=" + w + ")"; }

    @Override public final Result solve(Instance inst, long deadlineNanos, Metrics metrics) {
        final long t0 = System.nanoTime();
        int k = inst.numAgents();

        // ---- root: optimal individual paths define the initial lower bound ----
        CTNode root = new CTNode(k);
        int rootLb = 0;
        for (Agent a : inst.agents) {
            SpaceTimeAStar.LLResult ll = SpaceTimeAStar.search(
                    inst.map, a, SpaceTimeAStar.fromConstraints(root.constraintsOf(a.id)),
                    distances.distancesTo(a.goal), SpaceTimeAStar.Mode.OPTIMAL,
                    1.0, 0, null, a.id, deadlineNanos, metrics);
            metrics.lowLevelSearches++;
            if (ll.path == null) {
                return Result.failure(ll.lowerBound == -1 ? "TIMEOUT" : "NO_SOLUTION",
                        System.nanoTime() - t0, metrics, w);
            }
            root.paths[a.id] = ll.path;
            rootLb += ll.path.arrivalTime();   // SOC lower bound = sum of individual optima
        }
        root.lb = rootLb;
        root.recomputeCost();
        root.numConflicts = ConflictDetection.countAll(root.paths);
        root.firstConflict = ConflictDetection.findFirst(root.paths);

        PriorityQueue<CTNode> open = new PriorityQueue<>((x, y) -> Integer.compare(x.lb, y.lb));
        // FOCAL, maintained lazily: stale entries (expanded, or no longer within
        // the bound) are discarded when polled.
        PriorityQueue<CTNode> focal = new PriorityQueue<>((x, y) -> {
            if (x.numConflicts != y.numConflicts) return Integer.compare(x.numConflicts, y.numConflicts);
            if (x.cost != y.cost) return Integer.compare(x.cost, y.cost);
            return Integer.compare(x.lb, y.lb);
        });
        java.util.HashSet<CTNode> inOpen = new java.util.HashSet<>();
        CTNode unboundBest = null;   // cheapest conflict-free node found out-of-bound
        open.add(root);
        inOpen.add(root);
        if (root.cost <= w * root.lb) focal.add(root);

        while (!open.isEmpty()) {
            if ((metrics.highLevelExpansions & 0x1F) == 0 && System.nanoTime() > deadlineNanos)
                return Result.failure("TIMEOUT", System.nanoTime() - t0, metrics, w);

            final int minLb = open.peek().lb;      // explicit estimate of C*

            CTNode node = null;
            boolean nodeBoundOk = false;
            while (!focal.isEmpty()) {
                CTNode cand = focal.poll();
                if (inOpen.contains(cand) && cand.cost <= w * minLb) { node = cand; nodeBoundOk = true; break; }
            }
            if (node == null) {
                // focal empty: rebuild from open (bound may have risen)
                boolean added = false;
                List<CTNode> valid = new ArrayList<>();
                for (CTNode n : open) {
                    if (n.cost <= w * minLb) { valid.add(n); added = true; }
                }
                if (!added) {
                    // fall back to plain best-first on lb (bound tightening)
                    node = open.poll();
                    inOpen.remove(node);
                } else {
                    focal.addAll(valid);
                    continue;
                }
            } else {
                open.remove(node);
                inOpen.remove(node);
            }
            metrics.highLevelExpansions++;

            if (node.firstConflict == null) {
                // conflict-free; bounded ONLY if popped from FOCAL (cost <= w*minLb).
                // The fallback (min-lb) pop path is marked via nodeBoundOk.
                if (nodeBoundOk) {
                    long rt = System.nanoTime() - t0;
                    return new Result(true, "SOLVED", node.paths, node.cost,
                            mapf.core.Validator.makespan(node.paths), rt, metrics, w);
                } else {
                    if (unboundBest == null || node.cost < unboundBest.cost) unboundBest = node;
                }
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
                Path[] others = child.paths;
                // per-agent inherited lower bound (sound): the agent's unconstrained optimum
                int inheritedLb = distances.distancesTo(a.goal)[a.start];
                SpaceTimeAStar.LLResult ll = SpaceTimeAStar.search(
                        inst.map, a, SpaceTimeAStar.fromConstraints(cons),
                        distances.distancesTo(a.goal), lowLevelMode(),
                        w, inheritedLb, others, agent, deadlineNanos, metrics);
                metrics.lowLevelSearches++;
                if (ll.path == null) {
                    if (ll.lowerBound == -1)
                        return Result.failure("TIMEOUT", System.nanoTime() - t0, metrics, w);
                    continue;                          // pruned
                }
                int oldLen = child.paths[agent] == null ? 0 : child.paths[agent].arrivalTime();
                child.paths[agent] = ll.path;
                child.recomputeCost();
                // sound SOC bound: only this agent's term changes, and the new LL
                // bound is a sound lower bound on its optimal cost under the child's
                // constraints:  lb(child) = lb(parent) - oldLen + llBound
                child.lb = child.lb - oldLen + Math.max(ll.lowerBound, 0);
                child.numConflicts = ConflictDetection.countAll(child.paths);
                child.firstConflict = ConflictDetection.findFirst(child.paths);
                open.add(child);
                inOpen.add(child);
                focal.add(child);
            }
        }
        if (unboundBest != null) {
            // search exhausted; only an out-of-bound conflict-free solution was found.
            // Reported with an explicit status so it is never counted as w-bounded.
            return new Result(true, "SOLVED_UNBOUND", unboundBest.paths, unboundBest.cost,
                    mapf.core.Validator.makespan(unboundBest.paths),
                    System.nanoTime() - t0, metrics, w);
        }
        return Result.failure("NO_SOLUTION", System.nanoTime() - t0, metrics, w);
    }
}
