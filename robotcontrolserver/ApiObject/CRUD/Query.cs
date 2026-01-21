using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ApiObject.CRUD
{
    public class QueryRequest
    {
        public string? RequestId { get; set; }
        public List<object> Ids { get; set; } = new List<object>();

    }
    public class QueryResponse<T> where T : class
    {
        public string? RequestId { get; set; }
        public string? Message { get; set; }
        public int Code { get; set; }
        public List<T>? Datas { get; set; }

    }
}
