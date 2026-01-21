using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;
using ApiObject.Cuopt;
using CuOptClientService.Common;
using DbObject;
using ShareMemoryData;
using VDA5050Message;

namespace CuOptClientService;

public class CuOptClient
{
    private readonly HttpClient _http;
    private readonly Uri _baseUri;
    private readonly JsonSerializerOptions _json;

    public CuOptClient(string baseUrl, HttpMessageHandler? handler = null)
    {
        _baseUri = new Uri(baseUrl.TrimEnd('/') + "/");
        _http = handler is null ? new HttpClient() : new HttpClient(handler);
        _http.BaseAddress = _baseUri;
        _http.Timeout = TimeSpan.FromSeconds(30);

        _json = new JsonSerializerOptions
        {
            PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
            DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull
        };
    }

    /// <summary>
    /// Solve routing for the given robots and tasks, returning VDA5050 orders keyed by vehicle id.
    /// </summary>
    public async Task<Dictionary<string, List<Order>>> SolveAsync(IEnumerable<RobotStatus> robots, IEnumerable<RobotTask> tasks, IEnumerable<Point> points, IEnumerable<Route> routes,        CancellationToken ct = default)
    {
        var robotList = robots?.ToList() ?? new List<RobotStatus>();
        var taskList = tasks?.ToList() ?? new List<RobotTask>();
        var pointList = points?.ToList() ?? new List<Point>();
        var routeList = routes?.ToList() ?? new List<Route>();

        if (robotList.Count == 0 || taskList.Count == 0)
        {
            return new Dictionary<string, List<Order>>();
        }

        var request = BuildRequest(robotList, taskList, pointList, routeList);

        var reqId = await SubmitCuOptRequest(request, ct).ConfigureAwait(false);
        var response = await PollCuOptResponse(reqId, ct).ConfigureAwait(false);

        return CuOpt2VDA.Convert(response);
    }

    private CuoptVRPRequest BuildRequest(List<RobotStatus> robots, List<RobotTask> tasks, List<Point> points, List<Route> routes)
    {
        var costMatrix = DTO2CuOpt.CreateCostmapMatrix(points, routes);
        if (costMatrix.Count == 0)
        {
            costMatrix.Add(new List<int> { 0 });
        }

        var vehicleIds = robots
            .Select(r => string.IsNullOrWhiteSpace(r.SerialNumber) ? Guid.NewGuid().ToString() : r.SerialNumber)
            .ToList();
        var capacities = robots
            .Select(r => Math.Max(1, r.RemainCapacity > 0 ? r.RemainCapacity : (r.MaxCapacity > 0 ? r.MaxCapacity : 1)))
            .Select(c => new List<int> { c })
            .ToList();
        var vehicleLocations = robots.Select(_ => new List<int> { 0, 0 }).ToList(); // default depot index 0
        var vehicleTimeWindows = robots.Select(_ => new List<int> { 0, 10_000 }).ToList();
        var vehicleMaxTimes = robots.Select(_ => 10_000).ToList();
        var vehicleMaxCosts = robots.Select(_ => 10_000).ToList();

        var taskIds = new List<string>();
        var taskLocations = new List<int>();
        var demands = new List<List<int>>();
        var serviceTimes = new List<int>();
        var orderVehicleMatch = new List<OrderVehicleMatch>();

        // map point ids to matrix index
        var indexByPointId = new Dictionary<int, int>();
        for (var i = 0; i < points.Count; i++)
        {
            if (points[i].Id.HasValue && !indexByPointId.ContainsKey(points[i].Id.Value))
            {
                indexByPointId.Add(points[i].Id.Value, i);
            }
        }

        for (var i = 0; i < tasks.Count; i++)
        {
            var task = tasks[i];
            int locationId = task.DeliveryToPointId ?? task.DeliveryFromPointId ?? points.FirstOrDefault()?.Id ?? 0;
            var matrixIndex = indexByPointId.TryGetValue(locationId, out var idx) ? idx : 0;

            var id = $"task-delivery-{task.RobotTypeId ?? 0}-{Guid.NewGuid()}";
            taskIds.Add(id);
            taskLocations.Add(matrixIndex);
            demands.Add(new List<int> { Math.Max(1, task.Capacity ?? 1) });
            serviceTimes.Add(0);

            if (task.RobotTypeId.HasValue)
            {
                // constrain to matching vehicle types when Status encodes robot type (e.g., "type-<id>")
                var matchingVehicles = robots
                    .Select((r, vi) => new { Vehicle = r, Index = vi })
                    .Where(x => x.Vehicle.Status.Equals("type-" + task.RobotTypeId.Value, StringComparison.OrdinalIgnoreCase))
                    .Select(x => x.Index)
                    .ToList();

                if (matchingVehicles.Count > 0)
                {
                    orderVehicleMatch.Add(new OrderVehicleMatch
                    {
                        OrderId = i,
                        VehicleIds = matchingVehicles
                    });
                }
            }
        }

        return new CuoptVRPRequest
        {
            CostMatrixData = new MatrixData
            {
                Data = new Dictionary<string, List<List<int>>>
                {
                    { "1", costMatrix }
                }
            },
            TravelTimeMatrixData = new MatrixData
            {
                Data = new Dictionary<string, List<List<int>>>
                {
                    { "1", costMatrix }
                }
            },
            FleetData = new FleetData
            {
                VehicleIds = vehicleIds,
                VehicleLocations = vehicleLocations,
                Capacities = capacities,
                VehicleTimeWindows = vehicleTimeWindows,
                VehicleMaxTimes = vehicleMaxTimes,
                VehicleMaxCosts = vehicleMaxCosts,
                DropReturnTrips = vehicleIds.Select(_ => true).ToList(),
                SkipFirstTrips = vehicleIds.Select(_ => false).ToList()
            },
            TaskData = new TaskData
            {
                TaskIds = taskIds,
                TaskLocations = taskLocations,
                Demand = demands,
                ServiceTimes = serviceTimes,
                OrderVehicleMatch = orderVehicleMatch
            },
            SolverConfig = new SolverConfig
            {
                ErrorLogging = true,
                TimeLimit = 10,
                Objectives = new Objectives { Cost = 1, TravelTime = 1 }
            }
        };
    }

    private async Task<string> SubmitCuOptRequest(CuoptVRPRequest request, CancellationToken ct = default)
    {
        var url = new Uri("cuopt/request", UriKind.Relative);
        using var msg = new HttpRequestMessage(HttpMethod.Post, url);
        msg.Headers.Accept.ParseAdd("application/json");

        var json = JsonSerializer.Serialize(request, _json);
        msg.Content = new StringContent(json, Encoding.UTF8, "application/json");

        using var resp = await _http.SendAsync(msg, HttpCompletionOption.ResponseHeadersRead, ct).ConfigureAwait(false);
        var body = await resp.Content.ReadAsStringAsync(ct).ConfigureAwait(false);

        if (!resp.IsSuccessStatusCode)
            throw new HttpRequestException($"cuOpt submit failed HTTP={(int)resp.StatusCode} {resp.StatusCode}. Body={Trunc(body)}");

        CuOptVRPSubmitResponse? parsed;
        try
        {
            parsed = JsonSerializer.Deserialize<CuOptVRPSubmitResponse>(body, _json);
        }
        catch (Exception ex)
        {
            throw new InvalidOperationException($"Submit returned non-JSON. Body={Trunc(body)}", ex);
        }

        if (parsed?.ReqId is null || parsed.ReqId.Length < 10)
            throw new InvalidOperationException($"Submit JSON missing reqId. Body={Trunc(body)}");

        return parsed.ReqId.Trim();
    }

    private async Task<CuoptVRPResponse> PollCuOptResponse(string reqId, CancellationToken ct)
    {
        var url = new Uri($"cuopt/solution/{reqId}", UriKind.Relative);
        const int maxAttempts = 30;
        for (var attempt = 0; attempt < maxAttempts; attempt++)
        {
            using var resp = await _http.GetAsync(url, ct).ConfigureAwait(false);
            var body = await resp.Content.ReadAsStringAsync(ct).ConfigureAwait(false);

            if (!resp.IsSuccessStatusCode)
                throw new HttpRequestException($"cuOpt response failed HTTP={(int)resp.StatusCode} {resp.StatusCode}. Body={Trunc(body)}");

            CuoptVRPResponse? parsed = null;
            try
            {
                parsed = JsonSerializer.Deserialize<CuoptVRPResponse>(body, _json);
            }
            catch
            {
                // ignore and retry below
            }

            if (parsed?.Response?.SolverResponse != null)
                return parsed;

            await Task.Delay(TimeSpan.FromSeconds(1), ct).ConfigureAwait(false);
        }

        throw new TimeoutException($"Timed out waiting for cuOpt response reqId={reqId}");
    }

    private static string TryDecode(byte[] raw)
    {
        // Best-effort decoding
        string text;

        // UTF-8 (strip BOM if present)
        text = Encoding.UTF8.GetString(raw);
        text = text.TrimStart('\uFEFF');

        // If that yields mostly garbage, try UTF-16
        if (LooksGarbled(text))
        {
            try { text = Encoding.Unicode.GetString(raw); } catch { /*ignore*/ }
        }

        // If still garbled, fall back Latin-1 (preserve bytes)
        if (LooksGarbled(text))
            text = Encoding.Latin1.GetString(raw);

        return text;
    }

    private static bool LooksGarbled(string s)
    {
        if (string.IsNullOrEmpty(s)) return true;
        int letters = 0, weird = 0;
        foreach (var ch in s)
        {
            if (char.IsLetter(ch)) letters++;
            else if (char.IsControl(ch) && ch != '\r' && ch != '\n' && ch != '\t') weird++;
        }
        // heuristic
        return letters == 0 && weird > 0;
    }

    private static string Trunc(string? s, int max = 900)
    {
        if (string.IsNullOrEmpty(s)) return "";
        return s.Length <= max ? s : s.Substring(0, max) + "...";
    }
}
