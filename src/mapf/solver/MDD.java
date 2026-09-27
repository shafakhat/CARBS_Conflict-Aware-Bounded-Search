package mapf.solver;

import mapf.core.Agent;
import mapf.core.GridMap;

import java.util.ArrayList;
import java.util.List;

/**
 * Multi-Value Decision Diagram (MDD) for one agent: the set of (cell, t) states
 * that lie on SOME path of exact cost C (arrival time C) from start to goal.
 *
 * Construction: forward time-layered BFS pruned by the true distance-to-goal,
 * i.e. (v, t) is in the MDD iff v is reachable from the start in exactly t steps
 * AND distToGoal(v) <= C - t.
 *
 * Use in CARBS: a conflict at time t is CARDINAL for an agent whose MDD level t
 * has width 1 (every path of cost C passes through that single cell) - resolving
 * cardinal conflicts is mandatory, so they must be prioritized (Boyarski et al.
 * 2015; Cohen et al. 2019). Level widths are the soundness-critical quantity.
 */
public final class MDD {
    public final List<List<Integer>> levels;   // levels.get(t) = cells of the MDD at time t
    public final int cost;

    private MDD(List<List<Integer>> levels, int cost) {
        this.levels = levels;
        this.cost = cost;
    }

    public static MDD build(GridMap map, Agent a, int cost, short[] distToGoal) {
        if (distToGoal[a.start] < 0 || distToGoal[a.start] > cost) return new MDD(new ArrayList<>(), cost);
        List<List<Integer>> levels = new ArrayList<>(cost + 1);
        List<Integer> cur = new ArrayList<>(1);
        cur.add(a.start);
        levels.add(cur);
        for (int t = 1; t <= cost; t++) {
            List<Integer> next = new ArrayList<>();
            for (int v : levels.get(t - 1)) {
                int x = map.xOf(v), y = map.yOf(v);
                for (int d = 0; d < 5; d++) {
                    int nx = x + DX[d], ny = y + DY[d];
                    if (map.isObstacle(nx, ny)) continue;
                    int u = map.idx(nx, ny);
                    if (distToGoal[u] > cost - t) continue;      // cannot finish in time
                    if (!next.contains(u)) next.add(u);
                }
            }
            levels.add(next);
        }
        return new MDD(levels, cost);
    }

    public int width(int t) {
        if (t < 0 || t >= levels.size()) return 0;
        return levels.get(t).size();
    }

    public boolean contains(int t, int cell) {
        if (t < 0 || t >= levels.size()) return false;
        return levels.get(t).contains(cell);
    }

    private static final int[] DX = {0, 1, 0, -1, 0};
    private static final int[] DY = {0, 0, 1, 0, -1};
}
