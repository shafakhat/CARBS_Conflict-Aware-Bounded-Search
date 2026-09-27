package mapf.core;

/**
 * A single-agent path over time. cells[t] = position at time step t for
 * 0 <= t <= arrivalTime. After arrival the agent stays at its goal forever.
 *
 * Convention (standard in the MAPF literature): the cost of a path is its
 * arrival time (number of moves + waits performed before first reaching the
 * goal). Waiting at the goal after arrival is not charged.
 */
public final class Path {
    public final int agent;
    public final int[] cells;

    public Path(int agent, int[] cells) {
        if (cells == null || cells.length == 0) throw new IllegalArgumentException("empty path");
        this.agent = agent;
        this.cells = cells;
    }

    /** Arrival time == path cost under the sum-of-costs convention. */
    public int arrivalTime() { return cells.length - 1; }

    /** Position at time t; the agent parks at its goal after arrival. */
    public int posAt(int t) { return t >= cells.length ? cells[cells.length - 1] : cells[t]; }

    @Override public String toString() { return "Path(a" + agent + ", len=" + arrivalTime() + ")"; }
}
