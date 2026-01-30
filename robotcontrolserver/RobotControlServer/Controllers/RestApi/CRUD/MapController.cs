using BusinessLayer.Interfaces;
using DbObject;
using RobotControlServer.Validators;
using Microsoft.AspNetCore.Mvc;
using ShareMemoryData;
using ApiObject.CRUD;
using ApiObject;
using CommonLib;
using System.Text.Json;
using System.Xml.Linq;

namespace RobotControlServer.Controllers.RestApi.CRUD
{
    [Route("api1/map/")]
    public class MapController : CRUDBaseController<Map>
    {
        private readonly IMapBL _mapBL;
        public MapController(IMapBL mapBL, BaseCRUDValidator<Map>? validator = null) : base(mapBL, validator)
        {
            _mapBL = mapBL;
        }

        public override Task<IActionResult> Create([FromForm] CreateRequest<Map> request)
        {
            return base.Create(request);
        }
        [HttpGet("latest")]
        public async Task<IActionResult> GetLatest()
        {
            int code = 0;
            string message = string.Empty;
            Map? latest = await Task.Run(() => _mapBL.GetLatest(out code, out message));
            if (latest != null)
            {
                return Ok(latest);
            }
            return BadRequest(new { Code = code, Message = message });
        }

        public class ValidateRequest
        {
            public int MapId { get; set; }
        }

        [HttpPost("validate")]
        public async Task<IActionResult> ValidateMap([FromBody] ValidateRequest request)
        {
            int code = 0;
            string message = string.Empty;
            List<string> details = new();
            bool valid = await Task.Run(() => _mapBL.ValidateMap(request.MapId, out code, out message, out details));
            return Ok(new { Code = code, Message = message, Valid = valid, Details = details });
        }

        public class AssignRequest
        {
            public int MapId { get; set; }
            public List<int> RobotIds { get; set; } = new List<int>();
        }

        [HttpPost("assign")]
        public async Task<IActionResult> Assign([FromBody] AssignRequest request)
        {
            int code = 0;
            string message = string.Empty;
            List<string> details = new();
            bool result = await Task.Run(() => _mapBL.AssignMapToRobots(request.MapId, request.RobotIds, out code, out message, out details));
            if (result)
            {
                return Ok(new { Code = code, Message = message, Details = details });
            }
            return BadRequest(new { Code = code, Message = message, Details = details });
        }

        public class DownloadRequest
        {
            public int MapId { get; set; }
            public List<int> RobotIds { get; set; } = new List<int>();
        }

        [HttpPost("download")]
        public async Task<IActionResult> Download([FromBody] DownloadRequest request)
        {
            int code = 0;
            string message = string.Empty;
            List<string> details = new();
            bool result = await Task.Run(() => _mapBL.DownloadMap(request.MapId, request.RobotIds, out code, out message, out details));
            if (result)
            {
                return Ok(new { Code = code, Message = message, Details = details });
            }
            return BadRequest(new { Code = code, Message = message, Details = details });
        }

        [HttpGet("download/{token}")]
        public IActionResult DownloadByToken(string token)
        {
            if (!Guid.TryParse(token, out var guidToken))
            {
                return BadRequest("Invalid token");
            }
            if (MapDownloadTokenStore.TryConsume(guidToken, out var mapToken))
            {
                return Ok(new
                {
                    MapId = mapToken.MapId,
                    Url = mapToken.MapUrl,
                    ExpireAt = mapToken.ExpireAt
                });
            }
            return BadRequest("Token expired or not found");
        }



        [HttpPost("import-matrix")]
        public async Task<IActionResult> ImportMatrixFromFile([FromForm] ImportMatrixRequest request)
        {
            Map? map = _baseBL.GetById(request.MapId, out int returnCode, out string returnMessage);
            if (map == null)
            {
                return BadRequest("Can not found map in mapId");
            }
            using (var stream = request.JsonFile.OpenReadStream())
            {
                // Deserialize the stream directly into the C# object
                string jsonString = await CommonFunc.ReadJsonStringAsync(request.JsonFile);
                var result = ImportMatrix.LoadFromJson(jsonString);
                //var result = JsonSerializer.Deserialize<ImportMatrix>(jsonString);
                if (result == null || result.Edges == null || result.Nodes == null || (result.Edges.Count == 0 && result.Nodes.Count == 0))
                {
                    return BadRequest(new ImportMatrixResponse()
                    {
                        ReturnCode = -999,
                        ReturnMessage = "File json is invalid"
                    });
                }
                else
                {
                    bool import = _mapBL.ImportMatrix(map, result, out returnMessage);
                    if (import)
                    {
                        return Ok(new ImportMatrixResponse()
                        {
                            ReturnCode = 1,
                            ReturnMessage = "success"
                        });
                    }
                    else
                    {
                        return BadRequest(new ImportMatrixResponse()
                        {
                            ReturnCode = -999,
                            ReturnMessage = returnMessage
                        });
                    }
                }
            }

        }
    }
}
