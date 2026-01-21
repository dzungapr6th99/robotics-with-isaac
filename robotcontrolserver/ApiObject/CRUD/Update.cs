using DbObject;
using Microsoft.AspNetCore.Http;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ApiObject.CRUD
{
    public class UpdateRequest<T> where T : BaseDbObject
    {
        public string? RequestId { get; set; }
        public T? Data { get; set; }
        public List<IFormFile>? AttachFiles { get; set; }

    }
    public class UpdateResponse
    {
        public string? RequestId { get; set; }
        public List<string> Message { get; set; } = new List<string> { };
        public List<int> Code { get; set; } = new List<int> { };


    }
}
