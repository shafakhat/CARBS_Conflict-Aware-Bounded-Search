package mapf.core;

import java.util.HashSet;
import java.util.Set;

/**
 * Vertex/edge constraints for ONE agent, used by the low-level space-time search.
 *
 * Vertex constraint (v, t): agent may not occupy cell v at time t.
 * Edge constraint  (u, v, t): agent may not move u -> v between t-1 and t.
 *
 * Packing: vertex key = (v << 20) | t        (v < 2^43, t < 2^20)
 *          edge key   = (u << 38) | (v << 19) | t   (u,v < 2^19 cells, t < 2^19)
 * which comfortably covers every benchmark map (max 512x512 = 2^18 cells).
 */
public final class AgentConstraints {
    private static final int TIME_BITS = 20;
    private static final long TIME_MASK = (1L << TIME_BITS) - 1;
    private static final int CELL_BITS = 19;
    private static final long CELL_MASK = (1L << CELL_BITS) - 1;

    private final Set<Long> vertex = new HashSet<>();
    private final Set<Long> edge = new HashSet<>();
    private int latestTime = -1;

    public static long packVertex(int cell, int t) {
        assert cell < (1 << 30);
        return ((long) cell << TIME_BITS) | t;
    }
    public static long packEdge(int from, int to, int t) {
        assert from < (1 << CELL_BITS) && to < (1 << CELL_BITS);
        return (((long) from) << (CELL_BITS + TIME_BITS)) | (((long) to) << TIME_BITS) | t;
    }

    public void addVertex(int cell, int t) {
        vertex.add(packVertex(cell, t));
        if (t > latestTime) latestTime = t;
    }
    public void addEdge(int from, int to, int t) {
        edge.add(packEdge(from, to, t));
        if (t > latestTime) latestTime = t;
    }

    public boolean forbidsVertex(int cell, int t) { return vertex.contains(packVertex(cell, t)); }
    public boolean forbidsEdge(int from, int to, int t) { return edge.contains(packEdge(from, to, t)); }
    public boolean isEmpty() { return vertex.isEmpty() && edge.isEmpty(); }
    /** After this time no constraint applies, so an agent at its goal may rest. */
    public int latestConstraintTime() { return latestTime; }
    public int size() { return vertex.size() + edge.size(); }

    /** Deep copy (used when branching the constraint tree). */
    public AgentConstraints copy() {
        AgentConstraints c = new AgentConstraints();
        c.vertex.addAll(this.vertex);
        c.edge.addAll(this.edge);
        c.latestTime = this.latestTime;
        return c;
    }
}
