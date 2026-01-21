using DbObject;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ApiObject.CRUD
{
    public class DeleteRequest<T> where T : BaseDbObject
    {
        public string? RequestId { get; set; }
        public T? Data { get; set; }

    }
    public class DeleteResponse
    {
        public string? RequestId { get; set; }
        public List<string> Message { get; set; } = new List<string>();
        public List<int> Code { get; set; } = new List<int>();


    }
}
