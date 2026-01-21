using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CommonLib
{
    public static class ConstData
    {
        public static class Mqtt
        {
            public static class Topic
            {
                public const string ORDER = "order";
                public const string INSTANTACTIONS = "instantActions";
                public const string STATE = "state";
                public const string VISUALIZATION = "visualization";
                public const string CONNECTION = "connection";
                public const string FACTSHEET = "factsheet";
            }
        }

        public static class NavigationResult
        {
            public static int UNKOWN = 0;
            public static int ACCEPTED = 1;
            public static int EXECUTING= 2;
            public static int CANCELING = 3;
            public static int SUCCEEDED = 4;
            public static int CANCELLED = 5;
            public static int ABORTED = 6;
        }
        public static class ReturnCode
        {
            public static int SUCCESS = 1;
            public static int SERVICE_GET_ERROR = -999;
        }
        public static class ReturnMessage
        {
            public static string SUCCESS = "Success";
            public static string SERVICE_GET_ERROR = "Service get error";
            public static string ERROR_WHEN_INSERT_DATA = "Error when insert data";
            public static string ERROR_WHEN_SEARCH_DATA = "Error when search data";
            public static string ERROR_WHEN_UPDATE_DATA = "Error when update data";
            public static string ERROR_WHEN_DELETE_DATA = "Error when delete data";
        }
    }
}
