namespace ApiObject.Cuopt
{
    using System.Text.Json.Serialization;

    public class CuoptVRPRequest
    {
        [JsonPropertyName("cost_matrix_data")]
        public MatrixData? CostMatrixData { get; set; }

        [JsonPropertyName("fleet_data")]
        public FleetData? FleetData { get; set; }

        [JsonPropertyName("solver_config")]
        public SolverConfig? SolverConfig { get; set; }

        [JsonPropertyName("task_data")]
        public TaskData? TaskData { get; set; }

        [JsonPropertyName("travel_time_matrix_data")]
        public MatrixData? TravelTimeMatrixData { get; set; }
    }

    public class MatrixData
    {
        [JsonPropertyName("data")]
        public Dictionary<string, List<List<double>>>? Data { get; set; }
    }

    public class FleetData
    {
        [JsonPropertyName("capacities")]
        public List<List<int>>? Capacities { get; set; }

        [JsonPropertyName("drop_return_trips")]
        public List<bool>? DropReturnTrips { get; set; }

        [JsonPropertyName("min_vehicles")]
        public int? MinVehicles { get; set; }

        [JsonPropertyName("skip_first_trips")]
        public List<bool>? SkipFirstTrips { get; set; }

        [JsonPropertyName("vehicle_break_durations")]
        public List<List<int>>? VehicleBreakDurations { get; set; }

        [JsonPropertyName("vehicle_break_locations")]
        public List<int>? VehicleBreakLocations { get; set; }

        [JsonPropertyName("vehicle_break_time_windows")]
        public List<List<List<int>>>? VehicleBreakTimeWindows { get; set; }

        [JsonPropertyName("vehicle_fixed_costs")]
        public List<int>? VehicleFixedCosts { get; set; }

        [JsonPropertyName("vehicle_ids")]
        public List<string>? VehicleIds { get; set; }

        [JsonPropertyName("vehicle_locations")]
        public List<List<int>>? VehicleLocations { get; set; }

        [JsonPropertyName("vehicle_max_costs")]
        public List<int>? VehicleMaxCosts { get; set; }

        [JsonPropertyName("vehicle_max_times")]
        public List<int>? VehicleMaxTimes { get; set; }

        [JsonPropertyName("vehicle_order_match")]
        public List<VehicleOrderMatch>? VehicleOrderMatch { get; set; }

        [JsonPropertyName("vehicle_time_windows")]
        public List<List<int>>? VehicleTimeWindows { get; set; }

        [JsonPropertyName("vehicle_types")]
        public List<int>? VehicleTypes { get; set; }
    }

    public class VehicleOrderMatch
    {
        [JsonPropertyName("order_ids")]
        public List<int>? OrderIds { get; set; }

        [JsonPropertyName("vehicle_id")]
        public int VehicleId { get; set; }
    }

    public class TaskData
    {
        [JsonPropertyName("demand")]
        public List<List<int>>? Demand { get; set; }

        [JsonPropertyName("order_vehicle_match")]
        public List<OrderVehicleMatch>? OrderVehicleMatch { get; set; }

        [JsonPropertyName("service_times")]
        public List<int>? ServiceTimes { get; set; }

        [JsonPropertyName("task_ids")]
        public List<string>? TaskIds { get; set; }

        [JsonPropertyName("task_locations")]
        public List<int>? TaskLocations { get; set; }

        [JsonPropertyName("task_time_windows")]
        public List<List<int>>? TaskTimeWindows { get; set; }
    }

    public class OrderVehicleMatch
    {
        [JsonPropertyName("order_id")]
        public int OrderId { get; set; }

        [JsonPropertyName("vehicle_ids")]
        public List<int>? VehicleIds { get; set; }
    }

    public class SolverConfig
    {
        [JsonPropertyName("error_logging")]
        public bool? ErrorLogging { get; set; }

        [JsonPropertyName("objectives")]
        public Objectives? Objectives { get; set; }

        [JsonPropertyName("time_limit")]
        public int? TimeLimit { get; set; }

        [JsonPropertyName("verbose_mode")]
        public bool? VerboseMode { get; set; }
    }

    public class Objectives
    {
        [JsonPropertyName("cost")]
        public int? Cost { get; set; }

        [JsonPropertyName("prize")]
        public int? Prize { get; set; }

        [JsonPropertyName("travel_time")]
        public int? TravelTime { get; set; }

        [JsonPropertyName("variance_route_service_time")]
        public int? VarianceRouteServiceTime { get; set; }

        [JsonPropertyName("variance_route_size")]
        public int? VarianceRouteSize { get; set; }

        [JsonPropertyName("vehicle_fixed_cost")]
        public int? VehicleFixedCost { get; set; }
    }

    public class CuOptVRPSubmitResponse
    {
        [JsonPropertyName("reqId")]
        public string? ReqId { get; set; }
    }
}
