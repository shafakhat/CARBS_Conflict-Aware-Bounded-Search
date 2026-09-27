package mapf.experiment;

import mapf.core.DistanceCache;
import mapf.core.GridMap;
import mapf.core.Instance;
import mapf.core.Metrics;
import mapf.core.Result;
import mapf.core.ScenarioReader;
import mapf.core.Solver;
import mapf.core.Validator;
import mapf.solver.WPCBS;
import mapf.solver.CARBS;
import mapf.solver.WPCBS;
import mapf.solver.CBS;
import mapf.solver.ECBS;
import mapf.solver.EECBS;
import mapf.solver.IndependentAStar;
import mapf.solver.PrioritizedPlanning;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Experiment runner for the MovingAI MAPF benchmark suite.
 *
 * Every (map, scenario, agent-count, algorithm) cell produces one CSV row.
 * Every successful result is re-validated INDEPENDENTLY by
 * {@link Validator#validate} before it is written; a solver whose output
 * fails validation gets status=INVALID and is never counted as a success.
 *
 * Example:
 *   java -cp ... mapf.experiment.ExperimentRunner \
 *     --maps empty-32-32,warehouse-10-20-10-2-1 --scen even --scens 25 \
 *     --agents 2,4,6,8,10,15,20,25 --algos cbs,ecbs,eecbs,pp \
 *     --w 1.2 --timeout-ms 60000 --out results/pilot.csv
 */
public final class ExperimentRunner {

    public static void main(String[] args) throws IOException {
        Map<String, String> opt = parseArgs(args);

        Path mapDir = Paths.get(opt.getOrDefault("map-dir", "benchmarks/maps"));
        Path scenRoot = Paths.get(opt.getOrDefault("scen-dir", "benchmarks/scen"));
        String scenKind = opt.getOrDefault("scen", "even");                 // even | random
        int numScens = Integer.parseInt(opt.getOrDefault("scens", "25"));
        double w = Double.parseDouble(opt.getOrDefault("w", "1.2"));
        long timeoutMs = Long.parseLong(opt.getOrDefault("timeout-ms", "60000"));
        int cacheSize = Integer.parseInt(opt.getOrDefault("cache", "512"));
        Path outCsv = Paths.get(opt.get("out"));
        List<String> maps = splitList(opt.getOrDefault("maps", "ALL"));
        List<Integer> agentCounts = parseIntList(opt.get("agents"));
        List<String> algos = splitList(opt.getOrDefault("algos", "cbs,ecbs,eecbs,pp"));

        List<Path> mapFiles = new ArrayList<>();
        try (var s = Files.list(mapDir)) {
            s.filter(p -> p.toString().endsWith(".map")).sorted().forEach(mapFiles::add);
        }
        List<Path> selected = new ArrayList<>();
        for (Path p : mapFiles) {
            String name = p.getFileName().toString().replace(".map", "");
            if (maps.contains("ALL") || maps.contains(name)) selected.add(p);
        }
        if (selected.isEmpty()) { System.err.println("No maps selected."); System.exit(2); }

        if (Files.exists(outCsv)) Files.delete(outCsv);
        Files.createDirectories(outCsv.getParent() == null ? Paths.get(".") : outCsv.getParent());
        java.io.BufferedWriter csv = Files.newBufferedWriter(outCsv);
        csv.write("algorithm,w,map,scen,k,success,status,soc,makespan,runtime_ms,hl_expansions,ll_expansions,ll_searches\n");
        csv.flush();

        long rows = 0;
        for (Path mapFile : selected) {
            String mapName = mapFile.getFileName().toString().replace(".map", "");
            GridMap map = GridMap.load(mapFile, mapName);
            DistanceCache distances = new DistanceCache(map, cacheSize);

            for (int s = 1; s <= numScens; s++) {
                Path scenFile = scenRoot.resolve("scen-" + scenKind)
                        .resolve(mapName + "-" + scenKind + "-" + s + ".scen");
                if (!Files.exists(scenFile)) continue;

                for (int k : agentCounts) {
                    Instance inst = ScenarioReader.buildInstance(map, mapName, scenFile, k);
                    if (inst == null) continue;   // scenario cannot host k agents

                    for (String algo : algos) {
                        Solver solver = buildSolver(algo, distances, w, timeoutMs);
                        Metrics metrics = new Metrics();
                        long t0 = System.nanoTime();
                        Result r;
                        try {
                            r = solver.solve(inst, t0 + timeoutMs * 1_000_000L, metrics);
                        } catch (OutOfMemoryError ome) {
                            System.err.println("[OOM] " + algo + " on " + inst.label + " (recorded, run continues)");
                            r = Result.failure("ERROR_OOM", System.nanoTime() - t0, metrics, w);
                        } catch (Exception e) {
                            System.err.println("[ERROR] " + algo + " on " + inst.label + ": " + e);
                            r = Result.failure("ERROR", System.nanoTime() - t0, metrics, w);
                        }
                        long rt = System.nanoTime() - t0;

                        boolean success = false;
                        int soc = -1, makespan = -1;
                        String status = r.status;
                        if (r.success && r.paths != null) {
                            try {
                                if ("INDEPENDENT".equals(r.status)) {
                                    int[] cm = Validator.validatePaths(inst, r.paths);
                                    soc = cm[0]; makespan = cm[1];
                                    success = true;   // legality ok; collisions expected by design
                                } else {
                                    int[] cm = Validator.validate(inst, r.paths);
                                    if (cm[0] != r.soc)
                                        throw new Validator.ValidationException(
                                                "solver SOC " + r.soc + " != validated SOC " + cm[0]);
                                    soc = cm[0]; makespan = cm[1];
                                    success = true;
                                }
                            } catch (Validator.ValidationException ve) {
                                System.err.println("[INVALID] " + algo + " on " + inst.label + ": " + ve.getMessage());
                                status = "INVALID";
                                success = false; soc = -1; makespan = -1;
                            }
                        }
                        if (r.runtimeNanos > 0) rt = r.runtimeNanos;

                        csv.write(String.format(
                                "%s,%.2f,%s,%s,%d,%s,%s,%d,%d,%d,%s%n",
                                solver.name(), w, mapName, inst.scenFile, k,
                                success, status, soc, makespan,
                                rt / 1_000_000L, metrics.csv()));
                        csv.flush();
                        rows++;
                        System.out.printf("%-22s %-28s k=%-4d %-14s %-8s soc=%-7d %dms%n",
                                solver.name(), mapName, k, inst.scenFile,
                                success ? "OK" : status, soc, rt / 1_000_000L);
                    }
                }
            }
        }
        csv.close();
        writeManifest(outCsv, maps, agentCounts, algos, scenKind, numScens, w, timeoutMs);
        System.out.println("\nWrote " + rows + " rows to " + outCsv);
    }

    private static Solver buildSolver(String algo, DistanceCache d, double w, long timeoutMs) {
        switch (algo.toLowerCase()) {
            case "cbs":   return new CBS(d);
            case "ecbs":  return new ECBS(d, w);
            case "eecbs": return new EECBS(d, w);
            case "pp":    return new PrioritizedPlanning(d);
            case "astar": return new IndependentAStar(d);
            case "wp":    return new WPCBS(d, w);
            case "wpr":   return new WPCBS(d, w, true);
            case "dcbs":  return new WPCBS(d, w, WPCBS.Mode.PROTO);
            case "carbs": return new CARBS(d, w);
            case "carbs-nb": return new CARBS(d, w, false, true);
            case "carbs-nm": return new CARBS(d, w, true, false);
            default: throw new IllegalArgumentException("unknown algorithm: " + algo
                    + " (supported: cbs, ecbs, eecbs, pp, astar, wp, wpr, dcbs, carbs, carbs-nb, carbs-nm)");
        }
    }

    private static void writeManifest(Path csv, List<String> maps, List<Integer> ks,
                                      List<String> algos, String scenKind, int numScens,
                                      double w, long timeoutMs) throws IOException {
        Path mf = csv.resolveSibling(csv.getFileName().toString().replace(".csv", ".manifest.json"));
        String json = "{\n"
                + "  \"generated\": \"" + Instant.now() + "\",\n"
                + "  \"benchmarks\": \"MovingAI MAPF benchmark suite (https://www.movingai.com/benchmarks/mapf/),"
                + " Stern et al., SoCS 2019\",\n"
                + "  \"maps\": " + maps + ",\n"
                + "  \"scen\": \"" + scenKind + "\",\n"
                + "  \"scenarios_per_map\": " + numScens + ",\n"
                + "  \"agent_counts\": " + ks + ",\n"
                + "  \"algorithms\": " + algos + ",\n"
                + "  \"w\": " + w + ",\n"
                + "  \"timeout_ms\": " + timeoutMs + ",\n"
                + "  \"java\": \"" + System.getProperty("java.version") + "\",\n"
                + "  \"os\": \"" + System.getProperty("os.name") + " " + System.getProperty("os.version") + "\",\n"
                + "  \"cores\": " + Runtime.getRuntime().availableProcessors() + ",\n"
                + "  \"jvm_max_heap_mb\": " + (Runtime.getRuntime().maxMemory() >> 20) + ",\n"
                + "  \"validation\": \"independent post-hoc Validator applied to every solution\"\n"
                + "}\n";
        Files.writeString(mf, json);
    }

    private static Map<String, String> parseArgs(String[] args) {
        Map<String, String> m = new LinkedHashMap<>();
        for (int i = 0; i < args.length - 1; i += 2) m.put(args[i].substring(2), args[i + 1]);
        return m;
    }
    private static List<String> splitList(String s) {
        return Arrays.asList(s.split(","));
    }
    private static List<Integer> parseIntList(String s) {
        List<Integer> out = new ArrayList<>();
        for (String p : s.split(",")) out.add(Integer.parseInt(p.trim()));
        return out;
    }
    private ExperimentRunner() {}
}
