using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;

namespace VDA5050Message
{
    public class VDA5050MessageBase
    {
        internal IntPtr? _wrapperPtr{ get; set; }

        internal static string? PtrToString(IntPtr ptr) => ptr == IntPtr.Zero ? null : Marshal.PtrToStringAnsi(ptr);

        public virtual void CreateWrapper()
        {

        }

        public virtual void GetDataWrapper(IntPtr prt)
        {

        }
    }
}
