package mapf.experiment;

import mapf.core.Agent;
import mapf.core.DistanceCache;
import mapf.core.GridMap;
import mapf.core.Instance;
import mapf.core.Metrics;
import mapf.core.ScenarioReader;
import mapf.solver.CBS;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.PriorityQueue;
import java.util.Set;

/**
 * Independent optimality verifier for 2-agent instances.
 *
 * Exact Dijkstra-style search over the JOINT state space under the same
 * convention as the CBS implementation (the parking model of Sharon et al.
 * 2015: an agent that has reached its goal remains there and other agents
 * cannot pass through it).
 *
 * States are (s1, s2, committed1, committed2, t1, t2, t): a committed agent
 * rests forever at its goal (its arrival time is fixed); uncommitted agents
 * may move or wait - including moving away from their goal - and may commit
 * only while standing on their goal. The search minimizes SOC = t1 + t2 with
 * the admissible heuristic h = sum of true distances of uncommitted agents
 * (consistent for every transition type, so the first expanded terminal state
 * is SOC-optimal).
 *
 * Shares NOTHING with the CBS search code (only the map loader), so agreement
 * between this solver and CBS is strong independent evidence that CBS is
 * optimal. This is the implementation-validation methodology reported in the
 * manuscript.
 */
public final class JointAStarVerifier {

    private static final int[] DX = {0, 1, 0, -1, 0};
    private static final int[] DY = {0, 0, 1, 0, -1};

    private static final class State {
        final int s1, s2, c1, c2, t1, t2, t;
        State(int s1, int s2, int c1, int c2, int t1, int t2, int t) {
            this.s1 = s1; this.s2 = s2; this.c1 = c1; this.c2 = c2;
            this.t1 = t1; this.t2 = t2; this.t = t;
        }
        @Override public boolean equals(Object o) {
            if (!(o instanceof State)) return false;
            State s = (State) o;
            return s1 == s.s1 && s2 == s.s2 && c1 == s.c1 && c2 == s.c2
                    && t1 == s.t1 && t2 == s.t2;
        }
        @Override public int hashCode() { return Objects.hash(s1, s2, c1, c2, t1, t2); }
        /** pessimistic SOC: uncommitted agents counted at current time */
        int g() { return (c1 == 1 ? t1 : t) + (c2 == 1 ? t2 : t); }
    }

    private static long hOf(State s, short[] d1, short[] d2) {
        return (s.c1 == 1 ? 0 : d1[s.s1]) + (s.c2 == 1 ? 0 : d2[s.s2]);
    }

    /** Joint-space SOC-optimal search for exactly 2 agents; returns optimal SOC or -1/-2. */
    public static int jointAStar(GridMap map, Agent a1, Agent a2) {
        final short[] d1 = map.bfsDistances(a1.goal);
        final short[] d2 = map.bfsDistances(a2.goal);
        if (d1[a1.start] < 0 || d2[a2.start] < 0) return -1;

        PriorityQueue<long[]> open = new PriorityQueue<>((x, y) -> Long.compare(x[0], y[0]));
        Map<State, Integer> bestF = new HashMap<>();

        State s0 = new State(a1.start, a2.start, 0, 0, -1, -1, 0);
        open.add(new long[]{s0.g() + hOf(s0, d1, d2), s0.s1, s0.s2, 0, 0, -1, -1, 0});
        bestF.put(s0, (int) (s0.g() + hOf(s0, d1, d2)));
        int expanded = 0;

        while (!open.isEmpty()) {
            long[] cur = open.poll();
            State s = new State((int) cur[1], (int) cur[2], (int) cur[3], (int) cur[4],
                    (int) cur[5], (int) cur[6], (int) cur[7]);
            if (bestF.getOrDefault(s, Integer.MAX_VALUE) != (int) cur[0]) continue;  // stale
            expanded++;
            if (expanded > 40_000_000) return -2;          // safety cap
            if (s.c1 == 1 && s.c2 == 1) return s.t1 + s.t2;

            List<State> succ = new ArrayList<>();
            if (s.c1 == 0 && s.c2 == 0) {
                // commit one or both agents (only while standing on the goal)
                if (s.s1 == a1.goal && s.s2 == a2.goal) {
                    succ.add(new State(s.s1, s.s2, 1, 1, s.t, s.t, s.t));
                } else if (s.s1 == a1.goal) {
                    succ.add(new State(s.s1, s.s2, 1, 0, s.t, -1, s.t));
                } else if (s.s2 == a2.goal) {
                    succ.add(new State(s.s1, s.s2, 0, 1, -1, s.t, s.t));
                }
                // simultaneous joint move of both uncommitted agents
                int x1 = map.xOf(s.s1), y1 = map.yOf(s.s1);
                int x2 = map.xOf(s.s2), y2 = map.yOf(s.s2);
                for (int m1 = 0; m1 < 5; m1++) {
                    int n1 = nb(map, x1, y1, m1);
                    if (n1 < 0) continue;
                    for (int m2 = 0; m2 < 5; m2++) {
                        int n2 = nb(map, x2, y2, m2);
                        if (n2 < 0) continue;
                        if (n1 == n2) continue;                       // vertex collision
                        if (n1 == s.s2 && n2 == s.s1) continue;       // swap collision
                        succ.add(new State(n1, n2, 0, 0, -1, -1, s.t + 1));
                    }
                }
            } else if (s.c1 == 1 && s.c2 == 0) {
                // agent 1 parked at s.s1 forever; agent 2 moves alone in time
                if (s.s2 == a2.goal) {
                    succ.add(new State(s.s1, s.s2, 1, 1, s.t1, s.t, s.t));
                }
                int x2 = map.xOf(s.s2), y2 = map.yOf(s.s2);
                for (int m2 = 0; m2 < 5; m2++) {
                    int n2 = nb(map, x2, y2, m2);
                    if (n2 < 0 || n2 == s.s1) continue;               // parked agent blocks
                    succ.add(new State(s.s1, n2, 1, 0, s.t1, -1, s.t + 1));
                }
            } else if (s.c1 == 0 && s.c2 == 1) {
                if (s.s1 == a1.goal) {
                    succ.add(new State(s.s1, s.s2, 1, 1, s.t, s.t2, s.t));
                }
                int x1 = map.xOf(s.s1), y1 = map.yOf(s.s1);
                for (int m1 = 0; m1 < 5; m1++) {
                    int n1 = nb(map, x1, y1, m1);
                    if (n1 < 0 || n1 == s.s2) continue;
                    succ.add(new State(n1, s.s2, 0, 1, -1, s.t2, s.t + 1));
                }
            }

            for (State ns : succ) {
                if (ns.t > 4096) continue;
                long f = ns.g() + hOf(ns, d1, d2);
                Integer bf = bestF.get(ns);
                if (bf != null && bf <= f) continue;
                bestF.put(ns, (int) f);
                open.add(new long[]{f, ns.s1, ns.s2, ns.c1, ns.c2, ns.t1, ns.t2, ns.t});
            }
        }
        return -1;
    }

    private static int nb(GridMap map, int x, int y, int m) {
        int nx = x + DX[m], ny = y + DY[m];
        return map.isObstacle(nx, ny) ? -1 : map.idx(nx, ny);
    }

    public static void main(String[] args) throws Exception {
        Path mapFile = Paths.get(args[0]);
        String mapName = mapFile.getFileName().toString().replace(".map", "");
        Path scenFile = Paths.get(args[1]);
        int numTests = Integer.parseInt(args[2]);

        GridMap map = GridMap.load(mapFile, mapName);
        List<ScenarioReader.Row> rows = ScenarioReader.read(scenFile);
        CBS cbs = new CBS(new DistanceCache(map, 8));

        int tested = 0, agreed = 0;
        // test disjoint PAIRS of agents: rows [0,1], [2,3], [4,5], ...
        for (int base = 0; tested < numTests && (base + 1) < rows.size(); base += 2) {
            {
                List<Agent> agents = new ArrayList<>();
                Set<Integer> used = new HashSet<>();
                boolean ok = true;
                for (int i = base; i <= base + 1; i++) {
                    ScenarioReader.Row r = rows.get(i);
                    int s = map.idx(r.sx, r.sy), g = map.idx(r.gx, r.gy);
                    if (map.isObstacle(s) || map.isObstacle(g)
                            || !used.add(s * 2) || !used.add(g * 2 + 1)) { ok = false; break; }
                    agents.add(new Agent(agents.size(), s, g));
                }
                if (!ok || agents.size() != 2) { System.err.println("SKIP pair@" + base + " (bad rows)"); continue; }
                Instance inst = new Instance(map, agents, mapName,
                        scenFile.getFileName().toString(), "verify");

                long t0 = System.nanoTime();
                Metrics m = new Metrics();
                mapf.core.Result res = cbs.solve(inst, t0 + 30_000_000_000L, m);
                if (!res.success) { System.err.println("SKIP pair@" + base + " (CBS " + res.status + ")"); continue; }

                int jointCost = jointAStar(map, agents.get(0), agents.get(1));
                tested++;
                boolean match = jointCost == res.soc;
                if (match) agreed++;
                System.out.printf("pair@%d %s cbs=%d jointA*=%d %s%n", base,
                        scenFile.getFileName(), res.soc, jointCost, match ? "AGREE" : "MISMATCH ***");
            }
        }
        System.out.printf("Verification on %s: %d/%d instances agree%n",
                mapName, agreed, tested);
        if (agreed != tested) System.exit(1);
    }
}
