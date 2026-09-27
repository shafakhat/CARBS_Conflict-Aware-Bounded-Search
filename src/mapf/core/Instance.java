package mapf.core;

import java.util.List;

/**
 * A MAPF instance: a grid map plus agents with start/goal cells,
 * together with provenance metadata (which benchmark map and scenario
 * file the agents were taken from).
 */
public final class Instance {
    public final GridMap map;
    public final List<Agent> agents;
    public final String mapName;
    public final String scenFile;   // official scenario file the agents come from
    public final String label;      // e.g. "warehouse-10-20-10-2-1/even-3/k=25"

    public Instance(GridMap map, List<Agent> agents, String mapName, String scenFile, String label) {
        this.map = map;
        this.agents = agents;
        this.mapName = mapName;
        this.scenFile = scenFile;
        this.label = label;
    }

    public int numAgents() { return agents.size(); }
}
