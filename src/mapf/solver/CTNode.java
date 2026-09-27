package mapf.solver;

import mapf.core.Agent;
import mapf.core.AgentConstraints;
import mapf.core.Conflict;
import mapf.core.Instance;
import mapf.core.Path;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

/**
 * Constraint-tree node shared by CBS / ECBS / EECBS high levels.
 * Constraint maps use copy-on-write structural sharing: branching clones only
 * the branched agent's constraint set.
 */
public final class CTNode {
    /** agent id -> its constraint set (never null). */
    public final Map<Integer, AgentConstraints> constraints;
    /** agent id -> current path (null at root before planning). */
    public Path[] paths;
    /** sum of arrival times. */
    public int cost;
    /** number of pairwise collisions in the current joint paths (secondary criterion). */
    public int numConflicts;
    /** explicit lower-bound estimate for the node (focal high level). */
    public int lb;
    /** earliest collision in the current joint paths (null if none). */
    public Conflict firstConflict;

    public CTNode(int numAgents) {
        this.constraints = new HashMap<>();
        this.paths = new Path[numAgents];
        this.cost = 0;
        this.lb = 0;
    }

    private CTNode(Map<Integer, AgentConstraints> constraints, Path[] paths,
                   int cost, int numConflicts, int lb) {
        this.constraints = constraints;
        this.paths = paths;
        this.cost = cost;
        this.numConflicts = numConflicts;
        this.lb = lb;
    }

    /** Copy for branching: shares unmodified constraint sets, copies on write. */
    public CTNode child() {
        return new CTNode(new HashMap<>(constraints),
                Arrays.copyOf(paths, paths.length), cost, numConflicts, lb);
    }

    public AgentConstraints constraintsOf(int agent) {
        return constraints.computeIfAbsent(agent, a -> new AgentConstraints());
    }

    /** Replace the constraint set of one agent (after copying this node). */
    public void setConstraints(int agent, AgentConstraints cons) {
        constraints.put(agent, cons);
    }

    public void recomputeCost() {
        int c = 0;
        for (Path p : paths) if (p != null) c += p.arrivalTime();
        cost = c;
    }
}
