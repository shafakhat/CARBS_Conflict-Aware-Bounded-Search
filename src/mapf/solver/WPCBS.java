package mapf.solver;

import mapf.core.Agent;
import mapf.core.AgentConstraints;
import mapf.core.Validator;
import mapf.core.Conflict;
import mapf.core.DistanceCache;
import mapf.core.GridMap;
import mapf.core.Instance;
import mapf.core.Metrics;
import mapf.core.Path;
import mapf.core.Result;
import mapf.core.Solver;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;

/**
 * WPCBS - a waypoint/strategy-layer CBS design point (internal designation).
 *
 * Design point used in the design-rationale study: CBS extended with (a)
 * waypoint-based goal decomposition used as planning endpoints, (b) three
 * conflict-resolution strategies (temporal constraint, spatial ban,
 * priority-based), and (c) an agility tie-break. This variant is analyzed as a
 * cautionary design point: the extra strategies break constraint-tree
 * invariants (see the manuscript's design-rationale section).
 *
 * What is ported, and from which source:
 *  - CT skeleton, per-agent constraint sets, first-conflict branching, cost-ordered
 *    open list, anytime best tracking (fewest conflicts, then cost)  [2].
 *  - Phase 2 high-level evaluation f(N) = cost(N) + w * h(N), h = sum over agents of
 *    true remaining distance (admissible)  [1].
 *  - Phase 1 goal decomposition: when Manhattan(start, goal) > 3, intermediate
 *    sub-goal at ~1/3 of the distance, refined by a 5x5 neighbourhood search
 *    minimizing remaining distance  [1].
 *  - Phase 5 sub-goal advancement: subsequent sub-goals every 2 steps toward the
 *    final goal (same 5x5 refinement)  [1].
 *  - Phase 6 agile A*: position-time states, 4 moves + wait, heuristic
 *    = 1.1 * Manhattan, neighbour prioritization by grid-centre distance  [1].
 *  - Phase 4 conflict resolution, three strategies applied as parallel branches  [1]:
 *      A temporal shifting: constraint + 2-step delay (t and t+1),
 *      B spatial rerouting: ban the conflicting cell from time t onward,
 *      C priority-based: the agent closer to its goal keeps its path; the other
 *        replans avoiding the winner's trajectory (parking model).
 *
 * Documented divergences from the prototype (required for correctness; the
 * validator rejects the prototype's behaviour):
 *  - The prototype's low level returns a path on first goal arrival even when a
 *    later constraint applies to the parked agent; such paths re-create the very
 *    conflict being resolved and the prototype then loops on them until timeout.
 *    The port defers the goal test until no future constraint applies to the
 *    goal cell (the standard CBS low-level goal test).
 *  - A light visited set of (constrained agent, constraint, resulting path cost)
 *    prevents exact duplicate children.
 *
 * NOTE for the evaluation: this design point carries no proven suboptimality
 * bound; it is evaluated EMPIRICALLY - measured SOC ratios vs. the CBS optimum
 * are reported by analysis/summarize.py. The w parameter is used only inside
 * the high-level evaluation f = cost + w*h; it does NOT confer a bound.
 */
public final class WPCBS implements Solver {

    /** Per-agent constraint set: CBS pointwise constraints + spatial bans (Strategy B). */
    static final class Cons {
        final Set<Long> vertex = new HashSet<>();     // (cell<<20)|t
        final Set<Long> edge = new HashSet<>();       // packEdge(from,to,t)
        final Map<Integer, Integer> banFrom = new HashMap<>(); // cell -> banned from time t onward

        boolean blocked(int cell, int t) {
            if (vertex.contains(((long) cell << 20) | t)) return true;
            Integer bf = banFrom.get(cell);
            return bf != null && t >= bf;
        }
        boolean blockedEdge(int from, int to, int t) {
            return edge.contains(AgentConstraints.packEdge(from, to, t));
        }
        boolean isEmpty() { return vertex.isEmpty() && edge.isEmpty() && banFrom.isEmpty(); }
        int size() { return vertex.size() + edge.size() + banFrom.size(); }
        Cons copy() {
            Cons c = new Cons();
            c.vertex.addAll(vertex);
            c.edge.addAll(edge);
            c.banFrom.putAll(banFrom);
            return c;
        }
    }

    private final DistanceCache distances;
    private final double w;

    /** Algorithm variants cross-checked against the two sources:
     *  FULL   - the article's agility features (Phases 1-6, three strategies).
     *  ROBUST - FULL + two repairs (reachability-checked waypoints, direct-goal fallback).
     *  PROTO  - the direct-planning variant (D-CBS): direct-path root,
     *           plain CBS branching, anytime best tracking, no waypoints/strategies.
     */
    public enum Mode { FULL, ROBUST, PROTO }
    private final Mode mode;

    /** Article semantics (Phases 1-6). */
    public WPCBS(DistanceCache distances, double w) {
        this(distances, w, Mode.FULL);
    }
    /** true -> ROBUST variant, false -> FULL. */
    public WPCBS(DistanceCache distances, double w, boolean robust) {
        this(distances, w, robust ? Mode.ROBUST : Mode.FULL);
    }
    public WPCBS(DistanceCache distances, double w, Mode mode) {
        this.distances = distances;
        this.w = w;
        this.mode = mode;
    }

    @Override public String name() {
        switch (mode) {
            case ROBUST: return "WP-CBS-R(w=" + w + ")";
            case PROTO:  return "D-CBS(w=" + w + ")";
            default:     return "WP-CBS(w=" + w + ")";
        }
    }
    private boolean robust() { return mode != Mode.FULL; }

    // ==================================================================
    // High level
    // ==================================================================

    private static final class Node {
        final Map<Integer, Cons> cons;     // agent -> constraints (copy-on-write)
        Path[] paths;
        int cost;
        int numConflicts;
        Conflict firstConflict;
        double f;                          // cost + w * h
        Node(int k) { cons = new HashMap<>(); paths = new Path[k]; }
    }

    @Override public Result solve(Instance inst, long deadlineNanos, Metrics metrics) {
        final long t0 = System.nanoTime();
        final GridMap map = inst.map;
        final int k = inst.numAgents();

        // ---- root with goal-decomposed agile paths (Phase 1) ----
        Node root = new Node(k);
        for (Agent a : inst.agents) {
            Path p = agilePath(inst, a, root.cons.computeIfAbsent(a.id, x -> new Cons()),
                    deadlineNanos, metrics);
            metrics.lowLevelSearches++;
            if (p == null)
                return Result.failure(System.nanoTime() > deadlineNanos ? "TIMEOUT" : "NO_SOLUTION",
                        System.nanoTime() - t0, metrics, w);
            root.paths[a.id] = p;
        }
        root.cost = Validator.sumOfCosts(root.paths);
        root.numConflicts = ConflictDetection.countAll(root.paths);
        root.firstConflict = ConflictDetection.findFirst(root.paths);

        // Phase 3 + anytime best tracking [2]
        if (root.firstConflict == null)
            return solved(root, t0, metrics);

        PriorityQueue<Node> open = new PriorityQueue<>((x, y) -> {
            int c = Double.compare(x.f, y.f);
            if (c != 0) return c;
            return Integer.compare(x.numConflicts, y.numConflicts);
        });
        root.f = root.cost + w * jointH(inst, root.paths);
        open.add(root);
        Node best = root;                                   // anytime best [2]
        Set<String> visited = new HashSet<>();              // duplicate-state guard [Phase 7]

        while (!open.isEmpty()) {
            if ((metrics.highLevelExpansions & 0x1F) == 0 && System.nanoTime() > deadlineNanos)
                break;
            Node cur = open.poll();
            metrics.highLevelExpansions++;

            if (cur.firstConflict == null)
                return solved(cur, t0, metrics);

            // anytime best update [2]
            if (cur.numConflicts < best.numConflicts
                    || (cur.numConflicts == best.numConflicts && cur.cost < best.cost))
                best = cur;

            Conflict c = cur.firstConflict;
            for (Node child : generateChildren(inst, cur, c, deadlineNanos, metrics)) {
                String sig = childSignature(child);
                if (visited.add(sig)) open.add(child);
            }
        }

        // timeout / exhaustion: return best-effort only if genuinely conflict-free [2]
        if (best.firstConflict == null) return solved(best, t0, metrics);
        return Result.failure("TIMEOUT", System.nanoTime() - t0, metrics, w);
    }

    private Result solved(Node n, long t0, Metrics metrics) {
        return new Result(true, "SOLVED", n.paths, n.cost,
                Validator.makespan(n.paths), System.nanoTime() - t0, metrics, w);
    }

    /** Phase 2 evaluation: f = cost + w * sum of true distances-to-go (admissible h). */
    private double jointH(Instance inst, Path[] paths) {
        double h = 0;
        for (int i = 0; i < paths.length; i++) {
            int goal = inst.agents.get(i).goal;
            h += Math.max(0, distances.distancesTo(goal)[paths[i].posAt(Integer.MAX_VALUE)]);
        }
        return h;
    }

    private String childSignature(Node n) {
        StringBuilder sb = new StringBuilder(n.cost).append('|');
        for (Map.Entry<Integer, Cons> e : n.cons.entrySet()) {
            Cons c = e.getValue();
            sb.append(e.getKey()).append(':')
              .append(c.vertex.hashCode()).append(',')
              .append(c.edge.hashCode()).append(',')
              .append(c.banFrom.hashCode()).append(';');
        }
        return sb.toString();
    }

    // ==================================================================
    // Phase 4: three resolution strategies as parallel branches
    // ==================================================================

    private List<Node> generateChildren(Instance inst, Node parent, Conflict c,
                                        long deadlineNanos, Metrics metrics) {
        List<Node> children = new ArrayList<>();
        int horizon = 4096;

        if (c.vertex) {
            // Strategy A (temporal shifting): ONE standard vertex constraint per child.
            // NOTE (cross-check fix): adding a 2-step double constraint (t and t+1)
            // here breaks CT completeness - the resolution "agent waits at v for both
            // steps" would be excluded by BOTH children, so solutions could be missed.
            // The article's "2-step delays" therefore belongs to the low level's wait
            // handling, not to doubled constraints.
            for (int agent : new int[]{c.agent1, c.agent2}) {
                Node ch = childWith(inst, parent, agent, (cons) ->
                        cons.vertex.add(((long) c.cell << 20) | c.time), deadlineNanos, metrics);
                if (ch != null) children.add(ch);
            }
            // Strategy B (spatial rerouting): ban the cell from t onward, one child per agent
            if (mode != Mode.PROTO) {
                for (int agent : new int[]{c.agent1, c.agent2}) {
                    Node ch = childWith(inst, parent, agent, (cons) ->
                            cons.banFrom.put(c.cell, c.time), deadlineNanos, metrics);
                    if (ch != null) children.add(ch);
                }
            }
        } else {
            // Strategy A for edge conflicts: standard single edge constraint
            for (int side = 0; side < 2; side++) {
                int agent = side == 0 ? c.agent1 : c.agent2;
                int from = side == 0 ? c.cell1From : c.cell2From;
                int to = side == 0 ? c.cell1To : c.cell2To;
                int f = from, t = to;
                Node ch = childWith(inst, parent, agent, (cons) ->
                        cons.edge.add(AgentConstraints.packEdge(f, t, c.time)), deadlineNanos, metrics);
                if (ch != null) children.add(ch);
            }
            // Strategy B for edge conflicts: ban the landing cell for each agent
            for (int side = 0; side < 2; side++) {
                int agent = side == 0 ? c.agent1 : c.agent2;
                int to = side == 0 ? c.cell1To : c.cell2To;
                Node ch = childWith(inst, parent, agent, (cons) ->
                        cons.banFrom.put(to, c.time), deadlineNanos, metrics);
                if (ch != null) children.add(ch);
            }
        }

        // Strategy C (priority-based) [1]: agent closer to its goal wins;
        // the other replans avoiding the winner's trajectory (parking model).
        int d1 = distances.distancesTo(inst.agents.get(c.agent1).goal)
                    [parent.paths[c.agent1].posAt(0)];
        int d2 = distances.distancesTo(inst.agents.get(c.agent2).goal)
                    [parent.paths[c.agent2].posAt(0)];
        int winner = d1 <= d2 ? c.agent1 : c.agent2;
        int loser = winner == c.agent1 ? c.agent2 : c.agent1;
        Path winnerPath = parent.paths[winner];
        {
            Node ch = copyNode(parent);
            Cons cons = ch.cons.computeIfAbsent(loser, x -> new Cons()).copy();
            // dynamic obstacles: winner's (cell,t) occupancy, swaps, and parked goal
            Set<Long> winV = new HashSet<>();
            Set<Long> winE = new HashSet<>();
            for (int t = 0; t <= winnerPath.arrivalTime(); t++) {
                winV.add(((long) winnerPath.cells[t] << 20) | t);
                if (t > 0) winE.add(AgentConstraints.packEdge(winnerPath.cells[t - 1], winnerPath.cells[t], t));
            }
            ch.cons.put(loser, cons); // ensure map holds the copy we extend
            Path replanned = agilePathWithDynamic(inst, inst.agents.get(loser), cons,
                    winnerPath, winV, winE, deadlineNanos, metrics);
            metrics.lowLevelSearches++;
            if (replanned != null) {
                int oldLen = ch.paths[loser] == null ? 0 : ch.paths[loser].arrivalTime();
                ch.paths[loser] = replanned;
                ch.cost = ch.cost - oldLen + replanned.arrivalTime();
                ch.numConflicts = ConflictDetection.countAll(ch.paths);
                ch.firstConflict = ConflictDetection.findFirst(ch.paths);
                ch.f = ch.cost + w * jointH(inst, ch.paths);
                children.add(ch);
            }
        }
        return children;
    }

    interface ConsEdit { void apply(Cons cons); }

    /** Create a child that adds constraints to one agent and replans it. */
    private Node childWith(Instance inst, Node parent, int agent, ConsEdit edit,
                           long deadlineNanos, Metrics metrics) {
        Node ch = copyNode(parent);
        Cons cons = ch.cons.computeIfAbsent(agent, x -> new Cons()).copy();
        edit.apply(cons);
        ch.cons.put(agent, cons);

        Path p = agilePath(inst, inst.agents.get(agent), cons, deadlineNanos, metrics);
        metrics.lowLevelSearches++;
        if (p == null) return null;
        int oldLen = ch.paths[agent] == null ? 0 : ch.paths[agent].arrivalTime();
        ch.paths[agent] = p;
        ch.cost = ch.cost - oldLen + p.arrivalTime();
        ch.numConflicts = ConflictDetection.countAll(ch.paths);
        ch.firstConflict = ConflictDetection.findFirst(ch.paths);
        ch.f = ch.cost + w * jointH(inst, ch.paths);
        return ch;
    }

    private Node copyNode(Node parent) {
        Node n = new Node(parent.paths.length);
        for (Map.Entry<Integer, Cons> e : parent.cons.entrySet()) n.cons.put(e.getKey(), e.getValue());
        n.paths = parent.paths.clone();
        n.cost = parent.cost;
        n.numConflicts = parent.numConflicts;
        return n;
    }

    // ==================================================================
    // Phase 1 + 5: goal decomposition & sub-goal advancement
    // Phase 6: agile A* (1.1 * Manhattan, centre-prioritized neighbours)
    // ==================================================================

    /** Waypoint chain from start to goal per Phases 1/5; plan legs sequentially. */
    private Path agilePath(Instance inst, Agent a, Cons cons, long deadlineNanos, Metrics metrics) {
        return agilePathImpl(inst, a, cons, null, null, null, deadlineNanos, metrics);
    }

    private Path agilePathWithDynamic(Instance inst, Agent a, Cons cons, Path winnerPath,
                                      Set<Long> winV, Set<Long> winE,
                                      long deadlineNanos, Metrics metrics) {
        return agilePathImpl(inst, a, cons, winnerPath, winV, winE, deadlineNanos, metrics);
    }

    private Path agilePathImpl(Instance inst, Agent a, Cons cons, Path dynPath,
                               Set<Long> dynV, Set<Long> dynE,
                               long deadlineNanos, Metrics metrics) {
        GridMap map = inst.map;
        List<int[]> waypoints = waypointChain(map, a);

        int curCell = a.start;
        int curTime = 0;
        ArrayList<Integer> cells = new ArrayList<>();
        cells.add(a.start);

        for (int wi = 0; wi < waypoints.size(); wi++) {
            int[] wp = waypoints.get(wi);
            int[] leg = legSearch(map, a, curCell, curTime, wp, cons, dynPath, dynV, dynE,
                    deadlineNanos, metrics);
            if (leg == null && robust() && wi < waypoints.size() - 1) {
                // repair 2: fall back to direct planning to the final goal
                wp = waypoints.get(waypoints.size() - 1);
                leg = legSearch(map, a, curCell, curTime, wp, cons, dynPath, dynV, dynE,
                        deadlineNanos, metrics);
                if (leg != null) wi = waypoints.size();   // done after this leg
            }
            if (leg == null) return null;
            for (int i = 1; i < leg.length; i++) cells.add(leg[i]);
            curCell = leg[leg.length - 1];
            curTime = curTime + leg.length - 1;
        }
        int[] arr = new int[cells.size()];
        for (int i = 0; i < arr.length; i++) arr[i] = cells.get(i);
        return new Path(a.id, arr);
    }

    /**
     * Waypoints: Phase 1 - if Manhattan(start, goal) > 3 the first sub-goal sits at
     * ~1/3 of the way, refined in its 5x5 neighbourhood to minimize distance to the
     * final goal. Phase 5 - subsequent sub-goals advance 2 steps toward the final
     * goal (same 5x5 refinement). Final waypoint = the goal itself.
     */
    private List<int[]> waypointChain(GridMap map, Agent a) {
        List<int[]> wps = new ArrayList<>();
        if (mode == Mode.PROTO) { wps.add(new int[]{map.xOf(a.goal), map.yOf(a.goal)}); return wps; }
        int sx = map.xOf(a.start), sy = map.yOf(a.start);
        int gx = map.xOf(a.goal), gy = map.yOf(a.goal);
        int md = Math.abs(gx - sx) + Math.abs(gy - sy);
        if (md <= 3) {
            wps.add(new int[]{gx, gy});
            return wps;
        }
        // repair 1: a coordinate-space waypoint may sit behind a wall; keep
        // only waypoints in the start's connected component (one BFS decides for all)
        short[] reach = robust() ? map.bfsDistances(a.start) : null;
        // first sub-goal at ~1/3 of the way
        double frac = 1.0 / 3.0;
        int cx = sx + (int) Math.round(frac * (gx - sx));
        int cy = sy + (int) Math.round(frac * (gy - sy));
        int[] wp = refine(map, cx, cy, gx, gy);
        if (!robust() || reach[map.idx(wp[0], wp[1])] >= 0) wps.add(wp);
        // Phase 5: advance 2 steps toward the goal until within reach of it
        int[] cur = wps.isEmpty() ? new int[]{sx, sy} : wp;
        while (Math.abs(gx - cur[0]) + Math.abs(gy - cur[1]) > 2) {
            int nx = cur[0] + Integer.compare(gx, cur[0]) * Math.min(2, Math.abs(gx - cur[0]));
            int ny = cur[1];
            if (nx == cur[0]) {
                ny = cur[1] + Integer.compare(gy, cur[1]) * 2;
            }
            int[] next = refine(map, nx, ny, gx, gy);
            if (next[0] == cur[0] && next[1] == cur[1]) break;  // no progress
            if (!robust() || reach[map.idx(next[0], next[1])] >= 0) wps.add(next);
            cur = next;
        }
        wps.add(new int[]{gx, gy});
        return wps;
    }

    /** 5x5 neighbourhood refinement: free cell minimizing distance to the final goal. */
    private int[] refine(GridMap map, int cx, int cy, int gx, int gy) {
        int[] best = new int[]{cx, cy};
        int bestD = map.isObstacle(cx, cy) ? Integer.MAX_VALUE : Math.abs(gx - cx) + Math.abs(gy - cy);
        for (int dx = -2; dx <= 2; dx++) {
            for (int dy = -2; dy <= 2; dy++) {
                int x = cx + dx, y = cy + dy;
                if (map.isObstacle(x, y)) continue;
                int d = Math.abs(gx - x) + Math.abs(gy - y);
                if (d < bestD) { bestD = d; best = new int[]{x, y}; }
            }
        }
        return best;
    }

    /**
     * One leg of the agile A* (Phase 6): position-time search from (fromCell, fromTime)
     * to the waypoint, heuristic 1.1 * Manhattan, neighbours iterated in grid-centre
     * distance order, per-agent constraints + optional dynamic obstacles (Strategy C).
     * Goal test deferred while future constraints apply to the waypoint (documented
     * divergence that fixes the prototype's non-termination).
     */
    private int[] legSearch(GridMap map, Agent a, int fromCell, int fromTime, int[] waypoint,
                            Cons cons, Path dynPath, Set<Long> dynV, Set<Long> dynE,
                            long deadlineNanos, Metrics metrics) {
        int goalCell = map.idx(waypoint[0], waypoint[1]);
        // a spatial ban on the waypoint itself makes this leg infeasible (the agent
        // parks at its path end, so it can never outwait the ban)
        for (int bf : cons.banFrom.keySet())
            if (bf == goalCell) return null;
        int cx = map.width / 2, cy = map.height / 2;      // grid centre for neighbour ordering
        boolean useDyn = dynPath != null;

        // latest time any constraint could still affect this agent
        int latest = cons.isEmpty() ? (useDyn ? dynPath.arrivalTime() : 0) : latestOf(cons, useDyn ? dynPath.arrivalTime() : 0);

        // time horizon cap (ABSOLUTE time): exact BFS distance from the leg start to
        // the waypoint + a map-scaled slack for constraint detours
        int manh = Math.abs(waypoint[0] - map.xOf(fromCell)) + Math.abs(waypoint[1] - map.yOf(fromCell));
        short[] dWp = map.bfsDistances(goalCell);
        int direct = dWp[fromCell] < 0 ? Integer.MAX_VALUE : dWp[fromCell];
        if (direct == Integer.MAX_VALUE) return null;         // waypoint unreachable
        int slack = (map.width + map.height) / 2 + 16;
        int base = latest >= Integer.MAX_VALUE / 4 ? fromTime : Math.max(latest, fromTime);
        int tCap = Math.max(base, fromTime + direct + slack);

        final double hm = (mode == Mode.PROTO) ? 1.0 : 1.1;   // prototype: plain Manhattan
        PriorityQueue<double[]> open = new PriorityQueue<>((x, y) -> Double.compare(x[0], y[0]));
        Map<Long, Integer> bestG = new HashMap<>();
        Map<Long, Long> parent = new HashMap<>();
        long startKey = key(fromCell, fromTime);
        double h0 = hm * (Math.abs(waypoint[0] - map.xOf(fromCell)) + Math.abs(waypoint[1] - map.yOf(fromCell)));
        open.add(new double[]{h0, fromCell, fromTime});
        bestG.put(startKey, fromTime);

        while (!open.isEmpty()) {
            if ((metrics.lowLevelExpansions & 0xFF) == 0 && System.nanoTime() > deadlineNanos) return null;
            double[] cur = open.poll();
            int cell = (int) cur[1];
            int t = (int) cur[2];
            long k = key(cell, t);
            Integer bg = bestG.get(k);
            if (bg == null || bg != t) continue;
            metrics.lowLevelExpansions++;

            boolean constrainedLater = cell == goalCell && t < latest;
            if (cell == goalCell && !constrainedLater) {
                // reconstruct
                int len = t - fromTime + 1;
                int[] out = new int[len];
                long kk = k;
                for (int i = len - 1; i >= 0; i--) {
                    out[i] = (int) (kk >> 20);
                    kk = parent.getOrDefault(kk, -1L);
                    if (kk == -1L && i > 0) return null;
                }
                return out;
            }

            // neighbours ordered by grid-centre distance (Phase 6 neighbour prioritization)
            int[][] dirs = new int[][]{{0,0},{1,0},{0,1},{-1,0},{0,-1}};
            int[][] order = new int[5][];
            int n = 0;
            for (int[] d : dirs) {
                int x = map.xOf(cell) + d[0], y = map.yOf(cell) + d[1];
                if (map.isObstacle(x, y)) continue;
                order[n++] = new int[]{x, y, Math.abs(x - cx) + Math.abs(y - cy)};
            }
            if (mode != Mode.PROTO)   // prototype iterates neighbours in fixed order
                java.util.Arrays.sort(order, 0, n, (p, q) -> Integer.compare(p[2], q[2]));

            for (int i = 0; i < n; i++) {
                int np = map.idx(order[i][0], order[i][1]);
                int nt = t + 1;
                if (nt > tCap) continue;
                long nk = key(np, nt);
                if (bestG.containsKey(nk) && bestG.get(nk) <= nt) continue;
                if (cons.blocked(np, nt)) continue;
                if (order[i][0] != map.xOf(cell) || order[i][1] != map.yOf(cell)) {
                    if (cons.blockedEdge(cell, np, nt)) continue;
                }
                if (useDyn) {
                    if (dynV.contains(((long) np << 20) | nt)) continue;
                    int darr = dynPath.arrivalTime();
                    if (nt > darr && np == dynPath.cells[darr]) continue;   // parked winner
                    if (nt > 0 && dynV.contains(((long) cell << 20) | nt) && np == dynPath.posAt(nt - 1)) continue;
                    if (dynE.contains(AgentConstraints.packEdge(cell, np, nt))) continue;
                }
                double h = hm * (Math.abs(waypoint[0] - order[i][0]) + Math.abs(waypoint[1] - order[i][1]));
                bestG.put(nk, nt);
                parent.put(nk, k);
                open.add(new double[]{(nt - fromTime) + h, np, nt});
            }
        }
        return null;
    }

    private int latestOf(Cons cons, int dynArrival) {
        int latest = dynArrival;
        for (long v : cons.vertex) latest = Math.max(latest, (int) (v & ((1L << 20) - 1)));
        for (long e : cons.edge) latest = Math.max(latest, (int) (e & ((1L << 20) - 1)));
        // NOTE: spatial bans (Strategy B) on other cells do not delay parking at an
        // unbanned goal; a ban ON the goal cell makes the leg infeasible and is
        // rejected upfront in legSearch.
        return latest;
    }

    private static long key(int cell, int t) { return ((long) cell << 20) | t; }
}
