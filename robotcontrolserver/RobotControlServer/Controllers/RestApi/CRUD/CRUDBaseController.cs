using ApiObject.CRUD;
using BusinessLayer.Interfaces;
using CommonLib;
using DbObject;
using Microsoft.AspNetCore.Mvc;
using RobotControlServer.Validators;

namespace RobotControlServer.Controllers.RestApi.CRUD
{ 
    [Route("api1/base/")]
    public class CRUDBaseController<T> :ControllerBase where T : BaseDbObject
    {
        protected readonly IBaseBL<T> _baseBL;
        public readonly BaseCRUDValidator<T>? _validator;
        public CRUDBaseController(IBaseBL<T> baseBL, BaseCRUDValidator<T>? validator = null)
        {
            _baseBL = baseBL;
            _validator = validator;
            if (_validator != null)
            {
                _validator.InitRules();
            }
        }

        public virtual string FunctionId { get; } = string.Empty;

        [HttpPost("create")]
        public virtual async Task<IActionResult> Create([FromBody] CreateRequest<T> request)
        {
            if (request.Data == null)
            {
                return BadRequest("Data is null");
            }    
            if (_validator != null)
            {
                var validate = _validator.Validate(request.Data);
                if (!validate.IsValid)
                {
                    return BadRequest(new CreateResponse()
                    {
                        Code = [-996],
                        Message = validate.Errors.Select(x => x.ErrorMessage)?.ToList() ?? new List<string>()
                    });
                }
            }

            CreateResponse response = new CreateResponse()
            {
                RequestId = request.RequestId
            };
            List<int> returnCode = new List<int>();
            List<string> returnMessage = new List<string>();
            Task<int> task = Task.Run(() => { return _baseBL.Insert(request.Data, out returnCode, out returnMessage); });

            await task;
            response.Message.AddRange(returnMessage);
            response.Code.AddRange(returnCode);
            response.RecordId = task.Result;
            if (task.Result >=0)
            {
                return Ok(response);
            }
            else
            {
                return BadRequest(response);
            }
        }

        [HttpPut("update")]
        public virtual async Task<IActionResult> Update([FromBody] UpdateRequest<T> request)
        {
            if (_validator != null)
            {
                var validate = _validator.Validate(request.Data);
                if (!validate.IsValid)
                {
                    return BadRequest(new UpdateResponse()
                    {
                        Code = [-996],
                        Message = validate.Errors.Select(x => x.ErrorMessage)?.ToList() ?? new List<string>()
                    });
                }
            }
            UpdateResponse response = new UpdateResponse()
            {
                RequestId = request.RequestId
            };
            string updateBy = Request.Headers["UserId"].ToString();
            List<int> returnCode = new List<int>();
            List<string> returnMessage = new List<string>();
            Task<int> task = Task.Run(() => { return _baseBL.Update(request.Data, out returnCode, out returnMessage); });
            await task;
            response.Code.AddRange(returnCode);
            response.Message.AddRange(returnMessage);
            if (task.Result >= 1)
            {
                return Ok(response);
            }
            else
            {
                return BadRequest(response);
            }
        }
        [HttpGet("query")]
        public virtual async Task<IActionResult> Query([FromQuery] int[] ids)
        {
            int returnCode = ConstData.ReturnCode.SUCCESS;
            string returnMessage = ConstData.ReturnMessage.SUCCESS;
            Task<List<T>> task = Task.Run(() => { return _baseBL.GetByIds(ids.ToList(), out returnCode, out returnMessage); });

            List<T> result = await task;
            if (result != null && result.Count > 0)
            {
                return Ok(result);
            }
            else
            {
                return BadRequest(new QueryResponse<T>()
                {
                    Code = returnCode,
                    Message = returnMessage
                });
            }

        }
        [HttpDelete("delete")]
        public virtual async Task<IActionResult> Delete([FromBody] DeleteRequest<T> request)
        {
            DeleteResponse response = new DeleteResponse()
            {
                RequestId = request.RequestId,
            };
            List<int> returnCode = new List<int>();
            List<string> returnMessage = new List<string>();
            Task<int> task = Task.Run(() => { return _baseBL.Delete(request.Data, out returnCode, out returnMessage); });

            await task;
            response.Message.AddRange(returnMessage);
            response.Code.AddRange(returnCode);
            if (task.Result >= 1)
            {
                return Ok(response);
            }
            else
            {
                return BadRequest(response);
            }
        }

    }
}
