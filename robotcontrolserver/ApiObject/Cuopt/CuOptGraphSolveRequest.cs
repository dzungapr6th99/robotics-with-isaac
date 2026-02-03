using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;

namespace ApiObject.Cuopt
{
    public sealed class CuOptGraphSolveRequest
    {
        [JsonPropertyName("cost_waypoint_graph_data")]
        public CostWaypointGraphData CostWaypointGraphData { get; set; }

        [JsonPropertyName("fleet_data")]
        public FleetData FleetData { get; set; }

        [JsonPropertyName("task_data")]
        public TaskData TaskData { get; set; }

        [JsonPropertyName("solver_config")]
        public SolverConfig SolverConfig { get; set; }
    }
    public sealed class CostWaypointGraphData
    {
        // key là "0", "1", ... (string)
        [JsonPropertyName("waypoint_graph")]
        public Dictionary<string, WaypointGraph> WaypointGraph { get; set; }
    }
    public sealed class WaypointGraph
    {
        [JsonPropertyName("offsets")]
        public List<int> Offsets { get; set; }

        [JsonPropertyName("edges")]
        public List<int> Edges { get; set; }

        [JsonPropertyName("weights")]
        public List<double> Weights { get; set; }
    }
}
