package mapf.core;

import java.util.Arrays;
import java.util.List;

/**
 * Independent post-hoc validator: checks that a joint solution is genuinely
 * collision-free and legal. EVERY result reported by the experiment runner
 * passes through this validator - nothing is trusted from the solvers.
 * This is the single most important credibility feature of the framework.
 */
public final class Validator {

    public static final class ValidationException extends RuntimeException {
        public ValidationException(String msg) { super(msg); }
    }

    /** Throws ValidationException if the joint solution is invalid; returns (soc, makespan). */
    public static int[] validate(Instance inst, Path[] paths) {
        GridMap map = inst.map;
        List<Agent> agents = inst.agents;
        if (paths == null || paths.length != agents.size())
            throw new ValidationException("solution does not cover all agents");

        int makespan = 0;
        for (Path p : paths) makespan = Math.max(makespan, p.arrivalTime());

        for (int i = 0; i < agents.size(); i++) {
            Path p = paths[i];
            Agent a = agents.get(i);
            if (p.cells[0] != a.start)
                throw new ValidationException("agent " + i + " does not start at its start cell");
            if (p.posAt(makespan) != a.goal && p.cells[p.cells.length - 1] != a.goal)
                throw new ValidationException("agent " + i + " does not end at its goal cell");
            for (int t = 0; t < p.cells.length; t++) {
                if (map.isObstacle(p.cells[t]))
                    throw new ValidationException("agent " + i + " inside obstacle at t=" + t);
                if (t > 0) {
                    int from = p.cells[t - 1], to = p.cells[t];
                    if (from != to) {
                        int dx = Math.abs(map.xOf(from) - map.xOf(to));
                        int dy = Math.abs(map.yOf(from) - map.yOf(to));
                        if (dx + dy != 1)
                            throw new ValidationException("agent " + i + " illegal move at t=" + t);
                    }
                }
            }
            // every step between arrival and makespan must stay parked at the goal
            for (int t = p.arrivalTime(); t <= makespan; t++)
                if (p.posAt(t) != a.goal)
                    throw new ValidationException("agent " + i + " not parked at goal at t=" + t);
        }

        // pairwise collision check over 0..makespan (independent of solver internals)
        for (int i = 0; i < agents.size(); i++) {
            for (int j = i + 1; j < agents.size(); j++) {
                Path pi = paths[i], pj = paths[j];
                int lastT = Math.max(pi.arrivalTime(), pj.arrivalTime());
                int horizon = Math.max(lastT, makespan);
                for (int t = 0; t <= horizon; t++) {
                    if (pi.posAt(t) == pj.posAt(t))
                        throw new ValidationException("vertex collision agents " + i + "," + j + " cell "
                                + pi.posAt(t) + " t=" + t);
                    if (t > 0 && pi.posAt(t) == pj.posAt(t - 1) && pj.posAt(t) == pi.posAt(t - 1))
                        throw new ValidationException("edge collision agents " + i + "," + j + " t=" + t);
                }
            }
        }

        int soc = 0;
        for (Path p : paths) soc += p.arrivalTime();
        return new int[]{soc, makespan};
    }

    /** Validate only per-agent path legality (no collision checks) - used for the
     *  independent-A* reference baseline, whose joint output is not collision-free. */
    public static int[] validatePaths(Instance inst, Path[] paths) {
        GridMap map = inst.map;
        List<Agent> agents = inst.agents;
        if (paths == null || paths.length != agents.size())
            throw new ValidationException("solution does not cover all agents");
        for (int i = 0; i < agents.size(); i++) {
            Path p = paths[i];
            Agent a = agents.get(i);
            if (p.cells[0] != a.start)
                throw new ValidationException("agent " + i + " does not start at its start cell");
            if (p.cells[p.cells.length - 1] != a.goal)
                throw new ValidationException("agent " + i + " does not end at its goal cell");
            for (int t = 0; t < p.cells.length; t++) {
                if (map.isObstacle(p.cells[t]))
                    throw new ValidationException("agent " + i + " inside obstacle at t=" + t);
                if (t > 0) {
                    int from = p.cells[t - 1], to = p.cells[t];
                    if (from != to) {
                        int dx = Math.abs(map.xOf(from) - map.xOf(to));
                        int dy = Math.abs(map.yOf(from) - map.yOf(to));
                        if (dx + dy != 1)
                            throw new ValidationException("agent " + i + " illegal move at t=" + t);
                    }
                }
            }
        }
        int soc = 0;
        for (Path p : paths) soc += p.arrivalTime();
        return new int[]{soc, makespan(paths)};
    }

    /** True if the two agent lists are identical (same starts and goals in same order). */
    public static boolean sameAgents(List<Agent> a, List<Agent> b) {
        if (a.size() != b.size()) return false;
        for (int i = 0; i < a.size(); i++)
            if (a.get(i).start != b.get(i).start || a.get(i).goal != b.get(i).goal) return false;
        return true;
    }

    public static int sumOfCosts(Path[] paths) {
        int soc = 0; for (Path p : paths) soc += p.arrivalTime(); return soc;
    }

    public static int makespan(Path[] paths) {
        int m = 0; for (Path p : paths) m = Math.max(m, p.arrivalTime()); return m;
    }

    private Validator() {}
}
