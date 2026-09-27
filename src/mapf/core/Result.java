package mapf.core;

/**
 * Result of one solver run on one instance.
 * status: SOLVED | TIMEOUT | NO_SOLUTION | ERROR
 */
public final class Result {
    public final boolean success;
    public final String status;
    public final Path[] paths;      // null unless success
    public final int soc;           // sum of arrival times (valid iff success)
    public final int makespan;      // max arrival time   (valid iff success)
    public final long runtimeNanos; // measured by the runner
    public final Metrics metrics;
    public final double w;          // suboptimality bound used (1.0 for optimal solvers)

    public Result(boolean success, String status, Path[] paths, int soc, int makespan,
                  long runtimeNanos, Metrics metrics, double w) {
        this.success = success;
        this.status = status;
        this.paths = paths;
        this.soc = soc;
        this.makespan = makespan;
        this.runtimeNanos = runtimeNanos;
        this.metrics = metrics;
        this.w = w;
    }

    public static Result failure(String status, long runtimeNanos, Metrics metrics, double w) {
        return new Result(false, status, null, -1, -1, runtimeNanos, metrics, w);
    }
}
