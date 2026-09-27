package mapf.core;

/**
 * Interface every solver implements. New solvers integrate by implementing
 * this interface; all baselines remain exactly the algorithms from the
 * original papers.
 */
public interface Solver {
    /** Unique, paper-facing name, e.g. "CBS", "ECBS(w=1.5)". */
    String name();

    /**
     * Solve the instance. Implementations must poll {@code deadlineNanos}
     * (System.nanoTime()) regularly and abort with a TIMEOUT result.
     * Implementations must return paths that pass {@link Validator#validate}.
     */
    Result solve(Instance instance, long deadlineNanos, Metrics metrics);
}
