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
            public const int UNKOWN = 0;
            public const int ACCEPTED = 1;
            public const int EXECUTING= 2;
            public const int CANCELING = 3;
            public const int SUCCEEDED = 4;
            public const int CANCELLED = 5;
            public const int ABORTED = 6;
        }
        public static class ReturnCode
        {
            public const int SUCCESS = 1;
            public const int SERVICE_GET_ERROR = -999;
            public static class Point
            {
                public const int MAP_NOT_EXIST_IN_DATABASE = -100_000_001;
                public const int MUST_NOT_MOVE_POINT_TO_OTHER_MAP = -100_000_002;
            }
            public static class Route
            {
                public const int MAP_NOT_EXIST_IN_DATABASE = -100_001_003;
                public const int FROM_POINT_NOT_EXIST_IN_DATABASE = -100_001_004;
                public const int TO_POINT_NOT_EXIST_IN_DATABASE = -100_001_005;
                public const int POINT_NOT_IN_MAP = -100_001_006;
            }

            public static class Robot
            {
                public const int SERIAL_NUMBER_DUPLICATED = -100_002_001;
            }
        }
        public static class ReturnMessage
        {
            public const string SUCCESS = "Success";
            public const string SERVICE_GET_ERROR = "Service get error";
            public const string ERROR_WHEN_INSERT_DATA = "Error when insert data";
            public const string ERROR_WHEN_SEARCH_DATA = "Error when search data";
            public const string ERROR_WHEN_UPDATE_DATA = "Error when update data";
            public const string ERROR_WHEN_DELETE_DATA = "Error when delete data";
            public static class Point
            {
                public const string MAP_NOT_EXIST_IN_DATABASE = "Map is not exist in database";
                public const string MUST_NOT_MOVE_POINT_TO_OTHER_MAP = "Must not move point to other map";
            }

            public static class Route
            {
                public const string MAP_NOT_EXIST_IN_DATABASE = "Map is not exist in database";
                public const string FROM_POINT_NOT_EXIST_IN_DATABASE = "FromPoint is not exist in database";
                public const string TO_POINT_NOT_EXIST_IN_DATABASE = "ToPoint is not exist in database";
                public const string POINT_NOT_IN_MAP = "FromPoint and ToPoint must belong to the selected map";
            }

            public static class Robot
            {
                public const string SERIAL_NUMBER_DUPLICATED = "Serial number already exists in database";
            }
        }
    }
}
