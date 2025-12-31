using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class Error: VDA5050MessageBase
    {
        public string ErrorType { get; set; }
        public List<ErrorReference>? ErrorReferences { get; set; }
        public string? ErrorDescription { get; set; }
        public string? ErrorHint { get; set; }

        [JsonConverter(typeof(JsonStringEnumConverter))]
        public ErrorLevel ErrorLevel { get; set; }

        public override void CreateWrapper() { }
        public override void GetDataWrapper(IntPtr prt) { }
    }

    public class ErrorReference: VDA5050MessageBase
    {
        public string ReferenceKey { get; set; }
        public string ReferenceValue { get; set; }

        public override void CreateWrapper() { }
        public override void GetDataWrapper(IntPtr prt) { }
    }

    public enum ErrorLevel
    {
        WARNING,
        FATAL
    }
}
