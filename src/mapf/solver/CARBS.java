package mapf.solver;

import mapf.core.Agent;
import mapf.core.AgentConstraints;
import mapf.core.Conflict;
import mapf.core.DistanceCache;
import mapf.core.Instance;
import mapf.core.Metrics;
import mapf.core.Path;
import mapf.core.Result;
import mapf.core.Solver;
import mapf.core.Validator;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;

/**
 * CARBS: Conflict-Aware Bounded Search.
 *
 * The design follows a failure-driven rationale: naive "agility-layer"
 * extensions of CBS (multi-strategy branching, waypoint planning endpoints)
 * break the constraint-tree invariants; CARBS achieves the same adaptive
 * goals through invariant-preserving mechanisms. Every design choice below
 * responds to a diagnosed failure mode of such naive extensions:
 *
 *  Diagnosed failure                    -> CARBS design response
 *  ---------------------------------------------------------------------------
 *  3-5 children per node (strategies    -> TWO-child standard branching only;
 *  A+B+C) explode the constraint tree      "spatial rerouting" is realized as
 *  and break the compatible-node          BYPASSING (reroute without splitting)
 *  invariant
 *  Constraint-blind low level caused    -> standard deferred goal test
 *  livelock
 *  Obstacle-blind 1/3-coordinate        -> waypoints are not planning endpoints;
 *  waypoints caused root failures        agility acts through conflict SELECTION
 *  First-conflict selection wastes      -> MDD-based conflict prioritization:
 *  effort on avoidable conflicts          CARDINAL conflicts first (both MDD
 *                                         widths = 1 at the conflict time), then
 *                                         semi-cardinal, then earliest
 *                                         (Boyarski et al. 2015; Cohen et al.)
 *  Non-cardinal conflicts split the     -> conflict BYPASSING (Barer et al. 2014):
 *  tree unnecessarily                    a child whose new path has EQUAL cost
 *                                        and no more conflicts is merged back
 *                                        into the parent (no split)
 *  Cost-ordered pop has no              -> bounded explicit-estimation FOCAL
 *  suboptimality control                 (ECBS/EECBS mechanism, w, sound lower
 *                                        bounds); agility tie-break prefers nodes
 *                                        whose agents are closer to their goals
 *  Weak Manhattan low level             -> true-distance (BFS) heuristic; the low
 *                                        level is bounded focal search within w
 *
 * THEOREM (bound, w >= 1): CARBS returns only conflict-free solutions with
 * SOC <= w * C*, or reports timeout; it never returns an invalid solution.
 * Proof: as in ECBS/EECBS. OPEN is ordered by sound lower bounds
 * (child.lb = parent.lb - oldLen + sound LL bound); FOCAL admits nodes with
 * cost <= w * minLb over OPEN; standard 2-child branching preserves a
 * w-bounded node compatible with an optimal solution; bypassing never increases
 * any node's cost. MDD conflict selection changes only WHICH node is expanded,
 * never which solutions remain reachable; the agility tie-break is ordering only.
 */
public final class CARBS implements Solver {

    private final DistanceCache distances;
    private final double w;
    private final boolean useBypass;
    private final boolean useMddSelection;

    /** Full algorithm: MDD conflict prioritization + bypassing + bounded focal. */
    public CARBS(DistanceCache distances, double w) {
        this(distances, w, true, true);
    }

    /** Ablation constructor (for the paper's component analysis). */
    public CARBS(DistanceCache distances, double w, boolean useBypass, boolean useMddSelection) {
        this.distances = distances;
        this.w = w;
        this.useBypass = useBypass;
        this.useMddSelection = useMddSelection;
    }

    @Override public String name() { return "CARBS(w=" + w + ")"; }

    // ---------------- high-level node ----------------

    private static final class Node {
        final Map<Integer, AgentConstraints> cons;
        Path[] paths;
        int cost;
        int numConflicts;
        int lb;
        long seq;
        Node(int k) { cons = new HashMap<>(); paths = new Path[k]; }
    }

    private long seqCounter;
    private java.util.HashMap<Long, MDD> mddCache = new java.util.HashMap<>();

    @Override public Result solve(Instance inst, long deadlineNanos, Metrics metrics) {
        final long t0 = System.nanoTime();
        seqCounter = 0;
        mddCache.clear();
        final int k = inst.numAgents();

        // ---- root: optimal individual paths; SOC lower bound = sum of optima ----
        Node root = new Node(k);
        int rootLb = 0;
        for (Agent a : inst.agents) {
            SpaceTimeAStar.LLResult ll = SpaceTimeAStar.search(
                    inst.map, a,
                    SpaceTimeAStar.fromConstraints(root.cons.computeIfAbsent(a.id, x -> new AgentConstraints())),
                    distances.distancesTo(a.goal), SpaceTimeAStar.Mode.OPTIMAL,
                    1.0, 0, null, a.id, deadlineNanos, metrics);
            metrics.lowLevelSearches++;
            if (ll.path == null)
                return Result.failure(ll.lowerBound == -1 ? "TIMEOUT" : "NO_SOLUTION",
                        System.nanoTime() - t0, metrics, w);
            root.paths[a.id] = ll.path;
            rootLb += ll.path.arrivalTime();
        }
        root.lb = rootLb;
        root.cost = Validator.sumOfCosts(root.paths);
        root.numConflicts = ConflictDetection.countAll(root.paths);
        root.seq = seqCounter++;

        if (root.numConflicts == 0)
            return solved(root, t0, metrics);

        PriorityQueue<Node> open = new PriorityQueue<>((x, y) -> {
            if (x.lb != y.lb) return Integer.compare(x.lb, y.lb);
            return Long.compare(x.seq, y.seq);
        });
        PriorityQueue<Node> focal = new PriorityQueue<>((x, y) -> {
            if (x.numConflicts != y.numConflicts) return Integer.compare(x.numConflicts, y.numConflicts);
            int ax = agilityTieBreak(inst, x), ay = agilityTieBreak(inst, y);
            if (ax != ay) return Integer.compare(ax, ay);
            if (x.cost != y.cost) return Integer.compare(x.cost, y.cost);
            return Long.compare(x.seq, y.seq);
        });
        HashSet<Node> inOpen = new HashSet<>();
        Node unboundBest = null;

        open.add(root); inOpen.add(root); focal.add(root);

        while (!open.isEmpty()) {
            if ((metrics.highLevelExpansions & 0x1F) == 0 && System.nanoTime() > deadlineNanos)
                return finish(unboundBest, t0, metrics);

            final int minLb = open.peek().lb;
            Node node;
            boolean bounded;
            Node head = focal.poll();
            while (head != null && !(inOpen.contains(head) && head.cost <= w * minLb)) head = focal.poll();
            if (head != null) {
                node = head; bounded = true;
                open.remove(node); inOpen.remove(node);
            } else {
                node = open.poll(); bounded = false;     // bound-tightening fallback
                inOpen.remove(node);
            }
            metrics.highLevelExpansions++;

            if (node.numConflicts == 0) {
                if (bounded) return solved(node, t0, metrics);
                if (unboundBest == null || node.cost < unboundBest.cost) unboundBest = node;
                continue;
            }

            // ---- conflict selection (MDD prioritization) ----
            List<Conflict> all = ConflictDetection.findAll(node.paths);
            Conflict c = selectConflict(inst, node, all);

            // ---- children + bypass ----
            boolean bypassed = false;
            List<Node> children = new ArrayList<>(2);
            for (int side = 0; side < 2; side++) {
                int agent = side == 0 ? c.agent1 : c.agent2;
                Node child = copyOf(node);
                AgentConstraints cons = child.cons.computeIfAbsent(agent, x -> new AgentConstraints()).copy();
                if (c.vertex) cons.addVertex(c.cell, c.time);
                else {
                    int from = side == 0 ? c.cell1From : c.cell2From;
                    int to = side == 0 ? c.cell1To : c.cell2To;
                    cons.addEdge(from, to, c.time);
                }
                child.cons.put(agent, cons);

                Agent a = inst.agents.get(agent);
                int inheritedLb = distances.distancesTo(a.goal)[a.start];
                SpaceTimeAStar.LLResult ll = SpaceTimeAStar.search(
                        inst.map, a, SpaceTimeAStar.fromConstraints(cons),
                        distances.distancesTo(a.goal), SpaceTimeAStar.Mode.EXPLICIT,
                        w, inheritedLb, node.paths, agent, deadlineNanos, metrics);
                metrics.lowLevelSearches++;
                if (ll.path == null) {
                    if (ll.lowerBound == -1)
                        return Result.failure("TIMEOUT", System.nanoTime() - t0, metrics, w);
                    continue;                              // child pruned
                }

                // ---- BYPASS: equal-cost child with no more conflicts merges into parent ----
                if (useBypass && !isCardinal(inst, node, c)
                        && ll.path.arrivalTime() == node.paths[agent].arrivalTime()) {
                    Path[] trial = node.paths.clone();
                    trial[agent] = ll.path;
                    int newConf = node.numConflicts - conflictsAgainst(node.paths, agent)
                                + conflictsAgainst(trial, agent);
                    if (newConf <= node.numConflicts) {
                        node.paths = trial;
                        node.cons.put(agent, cons);        // merge constraint into parent
                        node.numConflicts = newConf;
                        // cost unchanged by the equal-cost condition
                        if (node.numConflicts == 0) {
                            if (bounded) return solved(node, t0, metrics);
                            if (unboundBest == null || node.cost < unboundBest.cost) unboundBest = node;
                        } else {
                            open.add(node); inOpen.add(node); focal.add(node);
                        }
                        bypassed = true;
                        break;
                    }
                }

                int oldLen = child.paths[agent] == null ? 0 : child.paths[agent].arrivalTime();
                child.paths[agent] = ll.path;
                child.cost = child.cost - oldLen + ll.path.arrivalTime();
                child.lb = node.lb - oldLen + Math.max(ll.lowerBound, 0);
                child.numConflicts = node.numConflicts - conflictsAgainst(node.paths, agent)
                                   + conflictsAgainst(child.paths, agent);
                child.seq = seqCounter++;
                children.add(child);
            }
            if (bypassed) continue;

            for (Node child : children) {
                open.add(child); inOpen.add(child); focal.add(child);
            }
        }

        return finish(unboundBest, t0, metrics);
    }

    private Result solved(Node n, long t0, Metrics metrics) {
        return new Result(true, "SOLVED", n.paths, n.cost,
                Validator.makespan(n.paths), System.nanoTime() - t0, metrics, w);
    }

    private Result finish(Node unboundBest, long t0, Metrics metrics) {
        if (unboundBest != null)
            return new Result(true, "SOLVED_UNBOUND", unboundBest.paths, unboundBest.cost,
                    Validator.makespan(unboundBest.paths), System.nanoTime() - t0, metrics, w);
        return Result.failure("NO_SOLUTION", System.nanoTime() - t0, metrics, w);
    }

    // ---------------- helpers ----------------

    private static Node copyOf(Node n) {
        Node c = new Node(n.paths.length);
        c.cons.putAll(n.cons);
        c.paths = n.paths.clone();
        c.cost = n.cost;
        c.numConflicts = n.numConflicts;
        c.lb = n.lb;
        return c;
    }

    /** Agility tie-break: total remaining distance-to-goal over all agents (smaller first). */
    private int agilityTieBreak(Instance inst, Node n) {
        if (NOAGILITY) return 0;
        int sum = 0;
        for (int i = 0; i < n.paths.length; i++) {
            Path p = n.paths[i];
            int goal = inst.agents.get(i).goal;
            sum += Math.max(0, distances.distancesTo(goal)[p.posAt(Integer.MAX_VALUE)]);
        }
        return sum;
    }

    private boolean isCardinal(Instance inst, Node node, Conflict c) {
        if (!useMddSelection) return false;
        MDD m1 = mddOf(inst, node, c.agent1);
        MDD m2 = mddOf(inst, node, c.agent2);
        return m1.width(c.time) == 1 && m2.width(c.time) == 1;
    }

    private static final boolean NOTARGET = System.getProperty("mapf.carbs.notarget") != null;
    private static final boolean NOAGILITY = System.getProperty("mapf.carbs.noagility") != null;

    /**
     * Choose the most constrained conflict: cardinal > target > semi-cardinal
     * > earliest time. A TARGET conflict is one where an agent sits parked on
     * its goal at the conflict step: no bound-preserving waiting can remove
     * it, so it must be branched (cf. Boyarski et al. 2015 target conflicts).
     * Selection changes only WHICH conflict is examined first.
     */
    private Conflict selectConflict(Instance inst, Node node, List<Conflict> all) {
        if (!useMddSelection || all.isEmpty()) return all.get(0);
        Conflict best = null;
        int bestRank = -1, bestTime = Integer.MAX_VALUE;
        for (Conflict c : all) {
            MDD m1 = mddOf(inst, node, c.agent1);
            MDD m2 = mddOf(inst, node, c.agent2);
            int w1 = m1.width(c.time), w2 = m2.width(c.time);
            int rank;
            if (w1 == 1 && w2 == 1) rank = 3;                       // cardinal
            else if (!NOTARGET && isTarget(inst, node, c)) rank = 2; // target
            else rank = (w1 == 1 || w2 == 1) ? 1 : 0;               // semi / none
            if (rank > bestRank || (rank == bestRank && c.time < bestTime)) {
                best = c; bestRank = rank; bestTime = c.time;
                if (rank == 3) break;
            }
        }
        return best == null ? all.get(0) : best;
    }

    /** True if either agent is parked on its goal at the conflict step. */
    private static boolean isTarget(Instance inst, Node node, Conflict c) {
        return node.paths[c.agent1].posAt(c.time) == inst.agents.get(c.agent1).goal
            || node.paths[c.agent2].posAt(c.time) == inst.agents.get(c.agent2).goal;
    }

    /** Total pairwise conflicts of agent i's path against all other stored paths. */
    private static int conflictsAgainst(Path[] paths, int i) {
        int s = 0;
        for (int j = 0; j < paths.length; j++)
            if (j != i) s += ConflictDetection.countPair(paths[i], paths[j]);
        return s;
    }

    private MDD mddOf(Instance inst, Node node, int agent) {
        Agent a = inst.agents.get(agent);
        int c = node.paths[agent].arrivalTime();
        long key = (((long) agent) << 32) | (c & 0xFFFFFFFFL);
        MDD m = mddCache.get(key);
        if (m == null) {
            m = MDD.build(inst.map, a, c, distances.distancesTo(a.goal));
            mddCache.put(key, m);
        }
        return m;
    }
}
