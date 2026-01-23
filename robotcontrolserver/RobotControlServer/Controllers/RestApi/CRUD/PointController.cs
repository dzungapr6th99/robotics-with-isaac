using BusinessLayer.Interfaces;
using DbObject;
using Microsoft.AspNetCore.Mvc;
using RobotControlServer.Validators;

namespace RobotControlServer.Controllers.RestApi.CRUD
{
     [Route("api1/point/")]
    public class PointController : CRUDBaseController<Point>
    {
        public PointController(IBaseBL<Point> baseBL, BaseCRUDValidator<Point>? validator = null) : base(baseBL, validator)
        {

        }

        public class AlignRequest
        {
            public List<Point> Points { get; set; } = new List<Point>();
        }

        [HttpPost("align")]
        public async Task<IActionResult> Align([FromBody] AlignRequest request)
        {
            if (_baseBL is not IPointBL pointBL)
            {
                return BadRequest("Point business layer is not available");
            }
            int code = 0;
            string message = string.Empty;
            List<string> details = new();
            bool result = await Task.Run(() => pointBL.AlignPoints(request.Points, out code, out message, out details));
            if (result)
            {
                return Ok(new { Code = code, Message = message, Details = details });
            }
            return BadRequest(new { Code = code, Message = message, Details = details });
        }
    }
}
