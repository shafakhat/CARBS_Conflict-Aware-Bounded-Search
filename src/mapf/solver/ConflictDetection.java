package mapf.solver;

import mapf.core.Agent;
import mapf.core.Conflict;
import mapf.core.GridMap;
import mapf.core.Path;

/**
 * Collision detection over a candidate joint solution.
 * Finds the earliest collision (vertex first, then edge at the same time step)
 * and counts the total number of collisions. Pure functions, no state.
 */
public final class ConflictDetection {

    /** Earliest collision in the joint paths, or null if conflict-free. */
    public static Conflict findFirst(Path[] paths) {
        int makespan = 0;
        for (Path p : paths) makespan = Math.max(makespan, p.arrivalTime());
        for (int t = 0; t <= makespan; t++) {
            for (int i = 0; i < paths.length; i++) {
                for (int j = i + 1; j < paths.length; j++) {
                    int pi = paths[i].posAt(t), pj = paths[j].posAt(t);
                    if (pi == pj) return Conflict.vertex(i, j, pi, t);
                    if (t > 0 && paths[i].posAt(t) == paths[j].posAt(t - 1)
                             && paths[j].posAt(t) == paths[i].posAt(t - 1))
                        return Conflict.edge(i, j, t,
                                paths[i].posAt(t - 1), paths[i].posAt(t),
                                paths[j].posAt(t - 1), paths[j].posAt(t));
                }
            }
        }
        return null;
    }

    /** All conflicts in the joint paths, ordered by time (vertex before edge at equal t). */
    public static java.util.List<Conflict> findAll(Path[] paths) {
        java.util.List<Conflict> out = new java.util.ArrayList<>();
        int makespan = 0;
        for (Path p : paths) makespan = Math.max(makespan, p.arrivalTime());
        for (int t = 0; t <= makespan; t++) {
            for (int i = 0; i < paths.length; i++) {
                for (int j = i + 1; j < paths.length; j++) {
                    if (paths[i].posAt(t) == paths[j].posAt(t))
                        out.add(Conflict.vertex(i, j, paths[i].posAt(t), t));
                    if (t > 0 && paths[i].posAt(t) == paths[j].posAt(t - 1)
                             && paths[j].posAt(t) == paths[i].posAt(t - 1))
                        out.add(Conflict.edge(i, j, t,
                                paths[i].posAt(t - 1), paths[i].posAt(t),
                                paths[j].posAt(t - 1), paths[j].posAt(t)));
                }
            }
        }
        return out;
    }

    /** Total number of pairwise collisions in the joint paths (secondary criterion). */
    public static int countAll(Path[] paths) {
        int count = 0;
        int makespan = 0;
        for (Path p : paths) makespan = Math.max(makespan, p.arrivalTime());
        for (int t = 0; t <= makespan; t++) {
            for (int i = 0; i < paths.length; i++) {
                for (int j = i + 1; j < paths.length; j++) {
                    if (paths[i].posAt(t) == paths[j].posAt(t)) count++;
                    if (t > 0 && paths[i].posAt(t) == paths[j].posAt(t - 1)
                             && paths[j].posAt(t) == paths[i].posAt(t - 1)) count++;
                }
            }
        }
        return count;
    }

    /**
     * Number of vertex and edge conflicts between exactly two paths over the
     * joint horizon. Summing countPair over all pairs equals countAll.
     */
    public static int countPair(Path p, Path q) {
        int hi = Math.max(p.arrivalTime(), q.arrivalTime());
        int count = 0;
        for (int t = 0; t <= hi; t++) {
            if (p.posAt(t) == q.posAt(t)) count++;
            if (t > 0 && p.posAt(t) == q.posAt(t - 1) && q.posAt(t) == p.posAt(t - 1)) count++;
        }
        return count;
    }

    private ConflictDetection() {}
}
