using BusinessLayer.Interfaces;
using DbObject;
using RobotControlServer.Validators;
using Microsoft.AspNetCore.Mvc;
using ShareMemoryData;

namespace RobotControlServer.Controllers.RestApi.CRUD
{
    [Route("api1/map/")]
    public class MapController : CRUDBaseController<Map>
    {
        public MapController(IBaseBL<Map> baseBL, BaseCRUDValidator<Map>? validator = null) : base(baseBL, validator)
        {

        }

        [HttpGet("latest")]
        public async Task<IActionResult> GetLatest()
        {
            if (_baseBL is not IMapBL mapBL)
            {
                return BadRequest("Map business layer is not available");
            }
            int code = 0;
            string message = string.Empty;
            Map? latest = await Task.Run(() => mapBL.GetLatest(out code, out message));
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
            if (_baseBL is not IMapBL mapBL)
            {
                return BadRequest("Map business layer is not available");
            }
            int code = 0;
            string message = string.Empty;
            List<string> details = new();
            bool valid = await Task.Run(() => mapBL.ValidateMap(request.MapId, out code, out message, out details));
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
            if (_baseBL is not IMapBL mapBL)
            {
                return BadRequest("Map business layer is not available");
            }
            int code = 0;
            string message = string.Empty;
            List<string> details = new();
            bool result = await Task.Run(() => mapBL.AssignMapToRobots(request.MapId, request.RobotIds, out code, out message, out details));
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
            if (_baseBL is not IMapBL mapBL)
            {
                return BadRequest("Map business layer is not available");
            }
            int code = 0;
            string message = string.Empty;
            List<string> details = new();
            bool result = await Task.Run(() => mapBL.DownloadMap(request.MapId, request.RobotIds, out code, out message, out details));
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
    }
}
