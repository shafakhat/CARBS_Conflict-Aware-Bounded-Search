package mapf.core;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Bounded LRU cache of BFS true-distance tables keyed by goal cell.
 * Keeps memory bounded on maps with many distinct goals (e.g. 1000-agent
 * city scenarios).
 */
public final class DistanceCache {
    private final GridMap map;
    private final LinkedHashMap<Integer, short[]> cache;

    public DistanceCache(GridMap map, int maxEntries) {
        this.map = map;
        this.cache = new LinkedHashMap<Integer, short[]>(64, 0.75f, true) {
            @Override protected boolean removeEldestEntry(Map.Entry<Integer, short[]> eldest) {
                return size() > maxEntries;
            }
        };
    }

    /** Distance-to-goal table for the given goal cell (BFS from goal). */
    public synchronized short[] distancesTo(int goal) {
        short[] d = cache.get(goal);
        if (d == null) {
            d = map.bfsDistances(goal);
            cache.put(goal, d);
        }
        return d;
    }
}
