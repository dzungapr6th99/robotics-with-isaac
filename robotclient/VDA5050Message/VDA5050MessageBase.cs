using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;
using System.Text.Json.Serialization;

namespace VDA5050Message
{
    public class VDA5050MessageBase
    {
        internal IntPtr? _wrapperPtr { get; set; }

        [JsonIgnore(Condition = JsonIgnoreCondition.Always)]
        public virtual int HeaderId { get; set; }

        [JsonIgnore(Condition = JsonIgnoreCondition.Always)]
        public virtual DateTime Timestamp { get; set; }

        [JsonIgnore(Condition = JsonIgnoreCondition.Always)]
        public virtual string Version { get; set; }

        [JsonIgnore(Condition = JsonIgnoreCondition.Always)]
        public virtual string Manufacturer { get; set; }

        [JsonIgnore(Condition = JsonIgnoreCondition.Always)]
        public virtual string SerialNumber { get; set; }

        internal static string? PtrToString(IntPtr ptr) => ptr == IntPtr.Zero ? null : Marshal.PtrToStringAnsi(ptr);

        public virtual void CreateWrapper()
        {

        }

        public virtual void GetDataWrapper(IntPtr prt)
        {

        }

        public IntPtr? GetWrapperPtr()
        {
            if (_wrapperPtr != null && _wrapperPtr.HasValue)
            {
                return _wrapperPtr;
            }
            else
            {
                CreateWrapper();
                return _wrapperPtr;
            }

        }
    }
}
