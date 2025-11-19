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

    }
}
