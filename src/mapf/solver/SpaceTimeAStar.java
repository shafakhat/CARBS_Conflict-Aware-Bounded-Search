package mapf.solver;

import mapf.core.Agent;
import mapf.core.AgentConstraints;
import mapf.core.GridMap;
import mapf.core.Metrics;
import mapf.core.Path;

import java.util.ArrayDeque;
import java.util.HashSet;
import java.util.PriorityQueue;

/**
 * Low-level space-time search for a single agent under constraints.
 *
 * State = (cell, time). Moves: 4-connected + wait.
 * Heuristic: BFS true distance to the goal (admissible).
 *
 * Three modes, mirroring the CBS-family algorithms as published:
 *
 *  OPTIMAL  Plain space-time A* (Sharon et al. 2015): pops states in f-order,
 *           returns the first goal state -> provably optimal path under the
 *           agent's constraints. Used by CBS.
 *
 *  FOCAL    Two-heap focal search (Barer et al. 2014). OPEN ordered by f;
 *           FOCAL = unexpanded states with f <= w * lb where lb is the
 *           low-level lower bound; FOCAL is popped by the secondary criterion
 *           = number of collisions with the other agents' current paths.
 *           Used by ECBS.
 *
 *  EXPLICIT Explicit-estimation search (Li et al. 2021). Identical structure,
 *           but lb = max(inherited lower bound, min f in OPEN): the bound is
 *           estimated explicitly and grows monotonically during the search.
 *           Used by EECBS.
 *
 * Soundness of the lower-bound bookkeeping (this is what makes the high-level
 * w-boundedness guarantee valid):
 *  - OPEN always holds EVERY generated-but-unexpanded state (states admitted
 *    to FOCAL are referenced, never removed from OPEN).
 *  - lb := min f in OPEN (floored by the inherited bound in EXPLICIT mode).
 *    Because h is admissible and goal tests are deferred to expansion, OPEN
 *    always contains a state of some optimal solution with f <= C*, so
 *    min f in OPEN <= C*: the returned bound is never inflated.
 *  - The admitted goal state satisfies cost <= w * lb(at pop time)
 *    <= w * lb(at termination): the returned path is w-bounded w.r.t. the
 *    returned lower bound.
 *
 * Returned lowerBound = -1 signals timeout; 0 with a null path means the
 * search space was exhausted (no path under the constraints).
 */
public final class SpaceTimeAStar {

    public enum Mode { OPTIMAL, FOCAL, EXPLICIT }

    /** Obstacle view: constraints (CBS family) or dynamic occupancy (prioritized planning). */
    public interface Obstacles {
        boolean vertexBlocked(int cell, int t);
        boolean edgeBlocked(int from, int to, int t);
        /** No restriction applies after this time (used for the goal test). */
        int latestRestrictionTime();
    }

    /** Adapter for CBS/ECBS/EECBS per-agent constraint sets. */
    public static Obstacles fromConstraints(AgentConstraints cons) {
        return new Obstacles() {
            @Override public boolean vertexBlocked(int cell, int t) { return cons.forbidsVertex(cell, t); }
            @Override public boolean edgeBlocked(int from, int to, int t) { return cons.forbidsEdge(from, to, t); }
            @Override public int latestRestrictionTime() { return cons.latestConstraintTime(); }
        };
    }

    public static final class LLResult {
        public final Path path;      // null if none found / timeout
        public final int lowerBound; // sound lower bound at termination; -1 => timeout
        public final long expansions;
        LLResult(Path path, int lowerBound, long expansions) {
            this.path = path; this.lowerBound = lowerBound; this.expansions = expansions;
        }
    }

    static final int[] DX = {0, 1, 0, -1, 0};
    static final int[] DY = {0, 0, 1, 0, -1};
    private static final long TIME_BITS = 20;
    private static final long T_MAX = (1L << TIME_BITS) - 1;

    private static final class Node implements Comparable<Node> {
        final int pos, t, h, f, sec;
        final Node parent;
        boolean inFocal;      // deduplication of FOCAL references
        Node(int pos, int t, int h, int sec, Node parent) {
            this.pos = pos; this.t = t; this.h = h; this.f = t + h;
            this.sec = sec; this.parent = parent;
        }
        @Override public int compareTo(Node o) {
            if (f != o.f) return Integer.compare(f, o.f);
            return Integer.compare(o.t, t);   // prefer larger g on f-ties
        }
    }

    private static final class FocalRef implements Comparable<FocalRef> {
        final Node node;
        FocalRef(Node node) { this.node = node; }
        @Override public int compareTo(FocalRef o) {
            if (node.sec != o.node.sec) return Integer.compare(node.sec, o.node.sec);
            if (node.f != o.node.f) return Integer.compare(node.f, o.node.f);
            return Integer.compare(o.node.t, node.t);
        }
    }

    private static long key(int pos, int t) { return ((long) pos << TIME_BITS) | t; }

    /**
     * @param otherPaths current paths of the other agents (null -> collision
     *                   counting disabled; the secondary criterion stays 0).
     * @param inheritedLb the parent node's lower bound (EXPLICIT mode only).
     */
    public static LLResult search(GridMap map, Agent agent, Obstacles obstacles,
                                  short[] distToGoal, Mode mode, double w, int inheritedLb,
                                  Path[] otherPaths, int myIndex, long deadlineNanos,
                                  Metrics metrics) {
        final long expansions0 = metrics.lowLevelExpansions;
        // benchmark-only (default OFF): zero the heuristic -> uniform-cost search,
        // i.e. pure Dijkstra-style SSSP behaviour on the time-expanded grid
        final boolean NOH = System.getProperty("mapf.ll.noh") != null;
        long expansions = 0;
        if (distToGoal[agent.start] < 0) return new LLResult(null, 0, 0);

        final boolean bounded = (mode != Mode.OPTIMAL);
        final Path[] others = otherPaths;
        final int latest = obstacles.latestRestrictionTime();

        PriorityQueue<Node> open = new PriorityQueue<>();
        PriorityQueue<FocalRef> focal = new PriorityQueue<>();
        HashSet<Long> closed = new HashSet<>();
        double lastBound = Double.NEGATIVE_INFINITY;   // admission watermark

        Node root = new Node(agent.start, 0, distToGoal[agent.start], 0, null);
        open.add(root);

        while (true) {
            if ((expansions & 0xFF) == 0 && System.nanoTime() > deadlineNanos) {
                metrics.lowLevelExpansions = expansions0 + expansions;
                return new LLResult(null, -1, expansions);
            }

            // ---- refresh: drop expanded heads, compute the sound bound ----
            while (!open.isEmpty() && closed.contains(key(open.peek().pos, open.peek().t)))
                open.poll();
            if (open.isEmpty()) break;                 // frontier exhausted
            int fMin = open.peek().f;                  // sound lower bound (never inflated)
            int lb = (mode == Mode.EXPLICIT) ? Math.max(inheritedLb, fMin) : fMin;
            double bound = bounded ? w * lb : Double.POSITIVE_INFINITY;

            if (bounded && bound > lastBound) {
                // The bound is monotonically non-decreasing (consistent h),
                // so a full admission pass is needed ONLY when it grows.
                // Otherwise every generated node was admitted at creation.
                // admit the f-sorted prefix of OPEN into FOCAL (refs, not moves).
                // The first non-admitted head implies all later heads are also
                // unadmitted, so we buffer + re-add and stop at the first new one.
                ArrayDeque<Node> buffer = new ArrayDeque<>();
                boolean stopAdmission = false;
                while (!open.isEmpty() && open.peek().f <= bound) {
                    Node head = open.poll();
                    buffer.add(head);
                    if (stopAdmission) continue;
                    if (head.inFocal) continue;
                    head.inFocal = true;
                    focal.add(new FocalRef(head));
                    // after admitting the first new node, continue draining
                    // the remaining prefix (they are <= bound by loop condition)
                }
                for (Node n : buffer) open.add(n);
            }
            lastBound = bound;

            // ---- pop the state to expand ----
            Node cur;
            if (!bounded) {
                cur = open.poll();
            } else {
                FocalRef ref;
                do {
                    ref = focal.poll();
                    if (ref == null) { break; }
                } while (closed.contains(key(ref.node.pos, ref.node.t)));
                if (ref == null) continue;   // focal drained; refresh will admit more
                cur = ref.node;
            }

            long curKey = key(cur.pos, cur.t);
            if (closed.contains(curKey)) continue;
            closed.add(curKey);
            expansions++;

            // Goal test: at the goal, past the last restriction on this agent.
            if (cur.pos == agent.goal && cur.t >= latest) {
                metrics.lowLevelExpansions = expansions0 + expansions;
                return new LLResult(reconstruct(agent.id, cur), fMin, expansions);
            }

            for (int dir = 0; dir < 5; dir++) {
                int nx = map.xOf(cur.pos) + DX[dir];
                int ny = map.yOf(cur.pos) + DY[dir];
                if (map.isObstacle(nx, ny)) continue;
                int np = map.idx(nx, ny);
                int nt = cur.t + 1;
                if (nt > T_MAX) continue;
                long nkey = key(np, nt);
                if (closed.contains(nkey)) continue;
                if (obstacles.vertexBlocked(np, nt)) continue;
                // swap check applies to every real move (dirs 1..4); dir 0 is WAIT
                if (dir != 0 && obstacles.edgeBlocked(cur.pos, np, nt)) continue;

                int nh = NOH ? 0 : distToGoal[np];
                if (distToGoal[np] < 0) continue;     // goal unreachable from np

                int sec = cur.sec;
                if (others != null) sec += collisionsAt(np, nt, cur.pos, others, myIndex);

                Node child = new Node(np, nt, nh, sec, cur);
                open.add(child);
                if (bounded && nh + nt <= lastBound && !child.inFocal) {
                    child.inFocal = true;
                    focal.add(new FocalRef(child));
                }
            }
        }

        metrics.lowLevelExpansions = expansions0 + expansions;
        return new LLResult(null, 0, expansions);      // exhausted: no solution
    }

    private static Path reconstruct(int agentId, Node goalNode) {
        int[] cells = new int[goalNode.t + 1];
        for (Node n = goalNode; n != null; n = n.parent) cells[n.t] = n.pos;
        return new Path(agentId, cells);
    }

    /** Collisions incurred by occupying np at time nt, arriving from cp at nt-1. */
    private static int collisionsAt(int np, int nt, int cp, Path[] others, int myIndex) {
        int c = 0;
        for (int i = 0; i < others.length; i++) {
            if (i == myIndex || others[i] == null) continue;
            Path p = others[i];
            if (p.posAt(nt) == np) c++;
            if (nt > 0 && p.posAt(nt) == cp && p.posAt(nt - 1) == np) c++;
        }
        return c;
    }

    private SpaceTimeAStar() {}
}
