using Microsoft.AspNetCore.Http;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ApiObject
{
    public class ImportMatrixRequest
    {
        public int MapId { get; set; }
        public IFormFile? JsonFile { get; set; }
    }

    public class ImportMatrixResponse
    {
        public int ReturnCode { get; set; }
        public string ReturnMessage { get; set; }
    }
    public class ImportMatrix
    {
        public List<ImportNode> Nodes;
        public List<ImportEdge> Edges;
    }

    public class ImportNode
    {
        public int Id { get; set; }
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
    }

    public class ImportEdge
    {
        public int From { get; set; }
        public int To { get; set; }
    }
}
