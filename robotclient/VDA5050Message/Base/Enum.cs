using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{

    public enum BlockingType
    {
        NONE,
        SOFT,
        HARD
    }
    public enum MapStatus
    {
        ENABLED,
        DISABLED
    }

    public enum OperatingMode
    {
        AUTOMATIC,
        SEMIAUTOMATIC,
        MANUAL,
        SERVICE,
        TEACHIN
    }
}
