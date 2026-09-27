package mapf.core;

/**
 * A collision between two agents in a candidate joint solution:
 * either a vertex collision (same cell, same time) or an edge collision
 * (two agents traverse the same edge in opposite directions between t-1 and t).
 */
public final class Conflict {
    public final int agent1, agent2;
    public final int time;
    public final boolean vertex;
    public final int cell;          // vertex collision cell
    public final int cell1From, cell1To; // edge collision: agent1 moves cell1From -> cell1To
    public final int cell2From, cell2To;

    private Conflict(int a1, int a2, int time, boolean vertex, int cell,
                     int c1f, int c1t, int c2f, int c2t) {
        this.agent1 = a1; this.agent2 = a2; this.time = time; this.vertex = vertex;
        this.cell = cell;
        this.cell1From = c1f; this.cell1To = c1t;
        this.cell2From = c2f; this.cell2To = c2t;
    }

    public static Conflict vertex(int a1, int a2, int cell, int t) {
        return new Conflict(a1, a2, t, true, cell, -1, -1, -1, -1);
    }
    public static Conflict edge(int a1, int a2, int t, int c1f, int c1t, int c2f, int c2t) {
        return new Conflict(a1, a2, t, false, -1, c1f, c1t, c2f, c2t);
    }

    @Override public String toString() {
        return vertex
                ? "VertexConflict(a" + agent1 + ",a" + agent2 + ",cell=" + cell + ",t=" + time + ")"
                : "EdgeConflict(a" + agent1 + ",a" + agent2 + ",t=" + time + ")";
    }
}
