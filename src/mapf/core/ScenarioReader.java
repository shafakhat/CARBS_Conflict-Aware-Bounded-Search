package mapf.core;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Reader for the official MovingAI MAPF benchmark ".scen" files.
 *
 * Format:
 *   version 1
 *   bucket<TAB>map<TAB>width<TAB>height<TAB>startX<TAB>startY<TAB>goalX<TAB>goalY<TAB>optimalCost
 *   ...
 *
 * One file = one scenario. The k agents of an instance are the rows with
 * bucket < k (the standard convention; rows are sorted by optimal single-agent
 * path length within each file). This replaces the flawed practice of
 * generating random start/goal pairs, which produces unreachable or trivially
 * unsolvable instances.
 */
public final class ScenarioReader {

    public static final class Row {
        public final int bucket, sx, sy, gx, gy;
        Row(int bucket, int sx, int sy, int gx, int gy) {
            this.bucket = bucket; this.sx = sx; this.sy = sy; this.gx = gx; this.gy = gy;
        }
    }

    public static List<Row> read(Path scenFile) throws IOException {
        List<Row> rows = new ArrayList<>();
        try (BufferedReader r = Files.newBufferedReader(scenFile, StandardCharsets.UTF_8)) {
            String line = r.readLine();               // "version 1"
            if (line == null || !line.startsWith("version"))
                throw new IOException("Not a scen file: " + scenFile);
            while ((line = r.readLine()) != null) {
                if (line.isBlank()) continue;
                String[] p = line.trim().split("\t");
                rows.add(new Row(Integer.parseInt(p[0]), Integer.parseInt(p[4]),
                        Integer.parseInt(p[5]), Integer.parseInt(p[6]), Integer.parseInt(p[7])));
            }
        }
        return rows;
    }

    /**
     * Build the instance for the first k rows of one scenario file - the
     * standard MAPF-benchmark convention (cf. Li et al., MAPF-LNS/EECBS:
     * "for a given number of agents k, the first k rows of the scen file
     * are used"). Sanity checks: starts unique among starts, goals unique
     * among goals, all cells free. Cross start-goal sharing between agents
     * and degenerate start==goal agents are legal and accepted.
     * Returns null if the scenario cannot host k agents.
     */
    public static Instance buildInstance(GridMap map, String mapName, Path scenFile, int k) throws IOException {
        List<Row> rows = read(scenFile);
        if (rows.size() < k) return null;
        Set<Integer> starts = new HashSet<>();
        Set<Integer> goals = new HashSet<>();
        List<Agent> agents = new ArrayList<>(k);
        for (int i = 0; i < k; i++) {
            Row row = rows.get(i);
            int s = map.idx(row.sx, row.sy);
            int g = map.idx(row.gx, row.gy);
            if (map.isObstacle(s) || map.isObstacle(g)) return null;
            if (!starts.add(s) || !goals.add(g)) return null;  // duplicate start/goal
            agents.add(new Agent(i, s, g));
        }
        String scenName = scenFile.getFileName().toString();
        String label = mapName + "/" + scenName + "/k=" + k;
        return new Instance(map, agents, mapName, scenName, label);
    }
}
