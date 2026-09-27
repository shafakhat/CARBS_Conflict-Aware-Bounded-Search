package mapf.core;

import java.util.Map;

/** Statistics accumulated by a solver during a single run. */
public final class Metrics {
    public long highLevelExpansions;   // constraint-tree nodes expanded (CBS family)
    public long lowLevelExpansions;    // space-time states expanded across all low-level searches
    public long lowLevelSearches;      // number of individual low-level searches performed
    public void reset() { highLevelExpansions = 0; lowLevelExpansions = 0; lowLevelSearches = 0; }
    public String toString() {
        return "hl=" + highLevelExpansions + " ll=" + lowLevelExpansions + " searches=" + lowLevelSearches;
    }
    public static String csvHeader() { return "hl_expansions,ll_expansions,ll_searches"; }
    public String csv() { return highLevelExpansions + "," + lowLevelExpansions + "," + lowLevelSearches; }
}
