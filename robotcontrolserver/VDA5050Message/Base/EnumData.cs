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
            /// <summary>
            /// Activates the pause mode. |instant: yes| node: no| edge: no
            /// </summary>
            public const string START_PAUSE = "startPause";
            /// <summary>
            /// Deactivate the pause mode. |instant: yes| node: no| edge: no
            /// </summary>
            public const string STOP_PAUSE = "stopPause";
            /// <summary>
            /// Activates the charging process. |instant: yes| node: yes| edge: no
            /// </summary>
            public const string START_CHARGING = "startCharging";
            /// <summary>
            /// Deactivates the charging process to send a new order. 
            /// </summary>
            public const string STOP_CHARGING = "stopCharging";
            /// <summary>
            /// Resets (overrides) the pose of the AGV with the given parameter.| instant: yes| node: no| edge: no
            /// </summary>
            public const string INIT_POSITION = "initPosition";

            /// <summary>
            /// Enable a previously downloaded map 
            ///explicitly to be used in orders without 
            ///initializing a new position.|  instant: yes| node: no| edge: no
            /// </summary>
            public const string ENABLE_MAP = "enableMap";
            /// <summary>
            /// Trigger the download of a new map. 
            ///Active during the download. Errors 
            ///reported in vehicle state. Finished after 
            ///verifying the successful download, 
            ///preparing the map for use and setting 
            ///the map in the state. | instant: yes| node: no| edge: no
            /// </summary>
            public const string DOWNLOAD_MAP = "downloadMap";
            /// <summary>
            /// Trigger the removal of a map from the 
            ///vehicle memory.| instant: yes| node: no| edge: no
            /// </summary>
            public const string DELETE_MAP = "deleteMap";
            /// <summary>
            /// Requests the AGV to send a new state 
            ///report. | instant: yes| node: no| edge: no
            /// </summary>
            public const string STATE_REQUEST = "stateRequest";
            /// <summary>
            /// Requests the AGV to generate and 
            ///store a log report. |instant: yes|node: no| edge: no
            /// </summary>
            public const string LOG_REPORT = "logReport";
            /// <summary>
            /// Request the AGV to pick a load. |instant: no|node: yes| edge: yes
            /// </summary>
            public const string PICK = "pick";
            /// <summary>
            /// Request the AGV to drop a load.|instant: no| node: yes| edge: yes
            /// </summary>
            public const string DROP = "drop";
            /// <summary>
            /// AGV detects object (e.g., load, charging 
            ///spot, free parking position). |instant: no| node: yes| edge: yes
            /// </summary>
            public const string DETECT_OBJECT = "detectObject";

            /// <summary>
            /// On a node, AGV will position exactly on a target.| instant: no | node: yes | edge: yes
            /// </summary>
            public const string FINE_POSITIONING = "finePositioning";
            /// <summary>
            /// AGV has to wait for a trigger on the AGV (e.g., button press, manual loading). | instant: no | node: yes | edge: no
            /// </summary>
            public const string WAIT_FOR_TRIGGER = "waitForTrigger";
            /// <summary>
            /// AGV stops as soon as possible. 
            ///This could be immediately or on the 
            ///next node.|instant: yes|node: no|edge: no
            /// </summary>
            public const string CANCEL_ORDER = "cancelOrder";
            /// <summary>
            /// Requests the AGV to send a factsheet | instant: yes | node: no | edge: no
            /// </summary>
            public const string FACTSHEET_REQUEST = "factsheetRequest";
        }


    }
}
