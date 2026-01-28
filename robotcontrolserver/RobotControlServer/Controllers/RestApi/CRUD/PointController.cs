using BusinessLayer.Interfaces;
using DbObject;
using Microsoft.AspNetCore.Mvc;
using RobotControlServer.Validators;

namespace RobotControlServer.Controllers.RestApi.CRUD
{
     [Route("api1/point/")]
    public class PointController : CRUDBaseController<Point>
    {
        private readonly IPointBL _pointBL;
        public PointController(IPointBL pointBL, BaseCRUDValidator<Point>? validator = null) : base(pointBL, validator)
        {
            _pointBL = pointBL;
        }

        public class AlignRequest
        {
            public List<Point> Points { get; set; } = new List<Point>();
        }

        [HttpPost("align")]
        public async Task<IActionResult> Align([FromBody] AlignRequest request)
        {
            int code = 0;
            string message = string.Empty;
            List<string> details = new();
            bool result = await Task.Run(() => _pointBL.AlignPoints(request.Points, out code, out message, out details));
            if (result)
            {
                return Ok(new { Code = code, Message = message, Details = details });
            }
            return BadRequest(new { Code = code, Message = message, Details = details });
        }
    }
}
