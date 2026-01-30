using Microsoft.AspNetCore.Http;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;
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
    public sealed class ImportMatrix
    {
        //[JsonPropertyName("nodes")]
        public List<ImportNode> Nodes { get; set; } = new List<ImportNode>();
        //[JsonPropertyName("edges")]
        public List<ImportEdge> Edges { get; set; } = new List<ImportEdge>();
        public static ImportMatrix LoadFromJson(string json)
        {
            var options = new JsonSerializerOptions
            {
                PropertyNameCaseInsensitive = true,
                AllowTrailingCommas = true,
                ReadCommentHandling = JsonCommentHandling.Skip
            };

            var obj = JsonSerializer.Deserialize<ImportMatrix>(json, options);
            if (obj == null) throw new InvalidOperationException("Failed to parse JSON into WaypointGraphExport.");
            return obj;
        }
    }



    public sealed class ImportNode
    {
        //[JsonPropertyName("id")]
        public int Id { get; set; }
        //[JsonPropertyName("x")]
        public double X { get; set; }
        //[JsonPropertyName("y")]
        public double Y { get; set; }
        //[JsonPropertyName("z")]
        public double Z { get; set; }
    }

    public sealed class ImportEdge
    {
        //[JsonPropertyName("from")]
        public int From { get; set; }
        //[JsonPropertyName("to")]
        public int To { get; set; }
    }
}
