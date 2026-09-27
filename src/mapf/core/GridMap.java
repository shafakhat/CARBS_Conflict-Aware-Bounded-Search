package mapf.core;

import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.SimpleGraph;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Deque;

/**
 * 4-connected grid map in the standard MovingAI ".map" format.
 *
 * Cells are addressed by a single packed int id = y * width + x.
 * Traversable characters: '.', 'G'. Obstacles: '@', 'O', 'T', 'W' (standard MovingAI set).
 *
 * Note on JGraphT: JGraphT 1.5.2 is used for utility/analysis operations
 * (connectivity checking, graph statistics) exactly as in the original toolchain.
 * The search hot path uses primitive arrays for performance - this is standard
 * practice and is documented openly in the manuscript.
 */
public final class GridMap {
    public final int width;
    public final int height;
    private final boolean[] obstacle;
    public final String name;

    private GridMap(int width, int height, boolean[] obstacle, String name) {
        this.width = width;
        this.height = height;
        this.obstacle = obstacle;
        this.name = name;
    }

    public static GridMap load(Path file, String name) throws IOException {
        try (BufferedReader r = Files.newBufferedReader(file, StandardCharsets.UTF_8)) {
            String typeLine = r.readLine();          // "type octile"
            if (typeLine == null || !typeLine.startsWith("type"))
                throw new IOException("Not a MovingAI map file: " + file);
            int height = Integer.parseInt(r.readLine().trim().split("\\s+")[1]);
            int width = Integer.parseInt(r.readLine().trim().split("\\s+")[1]);
            r.readLine();                             // "map"
            boolean[] obs = new boolean[width * height];
            for (int y = 0; y < height; y++) {
                String row = r.readLine();
                if (row == null) throw new IOException("Truncated map file: " + file);
                for (int x = 0; x < width && x < row.length(); x++) {
                    char c = row.charAt(x);
                    obs[y * width + x] = (c == '@' || c == 'O' || c == 'T' || c == 'W');
                }
            }
            return new GridMap(width, height, obs, name);
        }
    }

    public int idx(int x, int y) { return y * width + x; }
    public int xOf(int id) { return id % width; }
    public int yOf(int id) { return id / width; }
    public boolean isObstacle(int id) { return obstacle[id]; }
    public boolean isObstacle(int x, int y) {
        return x < 0 || x >= width || y < 0 || y >= height || obstacle[y * width + x];
    }
    public int freeCells() { int n = 0; for (boolean o : obstacle) if (!o) n++; return n; }

    /**
     * BFS 4-connected shortest distance from src to every free cell.
     * Undirected grid, so dist[v] also equals distance from v to src:
     * used with src = agent goal as the true-distance heuristic (admissible,
     * stronger than Manhattan). Unreachable cells = -1.
     */
    public short[] bfsDistances(int src) {
        short[] dist = new short[width * height];
        Arrays.fill(dist, (short) -1);
        if (obstacle[src]) return dist;
        Deque<Integer> queue = new ArrayDeque<>();
        dist[src] = 0;
        queue.add(src);
        while (!queue.isEmpty()) {
            int cur = queue.poll();
            int d = dist[cur];
            int x = xOf(cur), y = yOf(cur);
            if (x > 0 && !obstacle[cur - 1] && dist[cur - 1] < 0) { dist[cur - 1] = (short) (d + 1); queue.add(cur - 1); }
            if (x < width - 1 && !obstacle[cur + 1] && dist[cur + 1] < 0) { dist[cur + 1] = (short) (d + 1); queue.add(cur + 1); }
            if (y > 0 && !obstacle[cur - width] && dist[cur - width] < 0) { dist[cur - width] = (short) (d + 1); queue.add(cur - width); }
            if (y < height - 1 && !obstacle[cur + width] && dist[cur + width] < 0) { dist[cur + width] = (short) (d + 1); queue.add(cur + width); }
        }
        return dist;
    }

    /** Build a JGraphT graph of the free cells (utility/analysis use only). */
    public Graph<Integer, DefaultEdge> toJGraphT() {
        Graph<Integer, DefaultEdge> g = new SimpleGraph<>(DefaultEdge.class);
        for (int id = 0; id < width * height; id++) if (!obstacle[id]) g.addVertex(id);
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                if (isObstacle(x, y)) continue;
                if (x > 0 && !isObstacle(x - 1, y)) g.addEdge(idx(x, y), idx(x - 1, y));
                if (y > 0 && !isObstacle(x, y - 1)) g.addEdge(idx(x, y), idx(x, y - 1));
            }
        }
        return g;
    }

    @Override public String toString() { return name + "(" + width + "x" + height + ")"; }
}
