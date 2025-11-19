using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class SafetyState
    {
        [JsonConverter(typeof(JsonStringEnumConverter))]
        public EStop EStop { get; set; }

        public bool FieldViolation { get; set; }
    }

    public enum EStop
    {
        AUTOACK,
        MANUAL,
        REMOTE,
        NONE
    }
}
