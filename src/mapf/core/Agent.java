package mapf.core;

import java.util.Objects;

/** One MAPF agent: packed start and goal cell ids. */
public final class Agent {
    public final int id;
    public final int start;
    public final int goal;

    public Agent(int id, int start, int goal) {
        this.id = id;
        this.start = start;
        this.goal = goal;
    }

    @Override public boolean equals(Object o) {
        if (!(o instanceof Agent)) return false;
        Agent a = (Agent) o;
        return id == a.id && start == a.start && goal == a.goal;
    }
    @Override public int hashCode() { return Objects.hash(id, start, goal); }
    @Override public String toString() { return "A" + id + "[" + start + "->" + goal + "]"; }
}
