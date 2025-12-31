using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class Error
    {
        public string ErrorType { get; set; }
        public List<ErrorReference>? ErrorReferences { get; set; }
        public string? ErrorDescription { get; set; }
        public string? ErrorHint { get; set; }

        [JsonConverter(typeof(JsonStringEnumConverter))]
        public ErrorLevel ErrorLevel { get; set; }
    }

    public class ErrorReference
    {
        public string ReferenceKey { get; set; }
        public string ReferenceValue { get; set; }
    }

    public enum ErrorLevel
    {
        WARNING,
        FATAL
    }
}
