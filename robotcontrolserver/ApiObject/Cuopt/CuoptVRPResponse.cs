namespace ApiObject.Cuopt
{
    using System.Collections.Generic;
    using System.Text.Json;
    using System.Text.Json.Serialization;

    public class CuoptVRPResponse
    {
        [JsonPropertyName("notes")]
        public List<string>? Notes { get; set; }

        [JsonPropertyName("reqId")]
        public string? ReqId { get; set; }

        [JsonPropertyName("response")]
        public CuoptResponseBody? Response { get; set; }

        [JsonPropertyName("warnings")]
        public List<string>? Warnings { get; set; }
    }

    public class CuoptResponseBody
    {
        [JsonPropertyName("perf_times")]
        public PerfTimes? PerfTimes { get; set; }

        [JsonPropertyName("solver_response")]
        public SolverResponse? SolverResponse { get; set; }

        // NEW: infeasible response shape
        [JsonPropertyName("solver_infeasible_response")]
        public SolverInfeasibleResponse? SolverInfeasibleResponse { get; set; }

        [JsonPropertyName("total_solve_time")]
        public double? TotalSolveTime { get; set; }
    }

    public class PerfTimes
    {
        // Accept arbitrary properties under perf_times
        [JsonExtensionData]
        public Dictionary<string, JsonElement>? AdditionalData { get; set; }
    }



    public class SolverInfeasibleResponse
    {
        [JsonPropertyName("status")]
        public int? Status { get; set; }

        [JsonPropertyName("num_vehicles")]
        public int? NumVehicles { get; set; }

        [JsonPropertyName("solution_cost")]
        public double? SolutionCost { get; set; }

        [JsonPropertyName("objective_values")]
        public ObjectiveValues? ObjectiveValues { get; set; }   // direct, no wrapper

        [JsonPropertyName("vehicle_data")]
        public Dictionary<string, VehicleRouteData>? VehicleData { get; set; } // direct, no wrapper

        [JsonPropertyName("initial_solutions")]
        public List<JsonElement>? InitialSolutions { get; set; }

        [JsonPropertyName("dropped_tasks")]
        public DroppedTasks? DroppedTasks { get; set; }
    }
    public class SolverResponse
    {
        [JsonPropertyName("dropped_tasks")]
        public DroppedTasks? DroppedTasks { get; set; }

        [JsonPropertyName("initial_solutions")]
        public List<JsonElement>? InitialSolutions { get; set; }

        [JsonPropertyName("msg")]
        public string? Message { get; set; }

        [JsonPropertyName("num_vehicles")]
        public int? NumVehicles { get; set; }

        [JsonPropertyName("objective_values")]
        public ObjectiveValuesWrapper? ObjectiveValues { get; set; }

        [JsonPropertyName("solution_cost")]
        public double? SolutionCost { get; set; }

        [JsonPropertyName("status")]
        public int? Status { get; set; }

        [JsonPropertyName("vehicle_data")]
        public VehicleDataWrapper? VehicleData { get; set; }
    }

    public class DroppedTasks
    {
        [JsonPropertyName("task_id")]
        public List<string>? TaskIds { get; set; }

        [JsonPropertyName("task_index")]
        public List<int>? TaskIndexes { get; set; }
    }

    public class ObjectiveValuesWrapper
    {
        [JsonPropertyName("objective_values")]
        public ObjectiveValues? ObjectiveValues { get; set; }
    }

    public class ObjectiveValues
    {
        [JsonPropertyName("cost")]
        public double? Cost { get; set; }

        [JsonPropertyName("prize")]
        public double? Prize { get; set; }

        [JsonPropertyName("travel_time")]
        public double? TravelTime { get; set; }
    }

    public class VehicleDataWrapper
    {
        [JsonPropertyName("vehicle_data")]
        public Dictionary<string, VehicleRouteData>? VehicleData { get; set; }
    }

    public class VehicleRouteData
    {
        [JsonPropertyName("arrival_stamp")]
        public List<double>? ArrivalStamp { get; set; }   

        [JsonPropertyName("route")]
        public List<int>? Route { get; set; }

        [JsonPropertyName("task_id")]
        public List<string>? TaskIds { get; set; }

        [JsonPropertyName("type")]
        public List<string>? Type { get; set; }
    }
}
