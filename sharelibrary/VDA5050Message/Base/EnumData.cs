using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public static class EnumData
    {
        public static class ActionType
        {
            public const string INIT_POSITION = "initPosition";
            public const string DOWNLOAD_MAP = "downloadMap";
            public const string ENABLE_MAP = "enableMap";
            public const string DELETE_MAP = "deleteMap";
            public const string CANCEL_ORDER = "cancelOrder";
        }
    }
}
