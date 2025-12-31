using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class Action : VDA5050MessageBase
    {

        private const string Lib = "libVDAWrapper.so";
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Action_Create();

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Action_Destroy(IntPtr actionWrapper);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern void Action_ActionType(IntPtr wrapper, string? actionType);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern void Action_SetActionDescription(IntPtr wrapper, string? actionDescription);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern void Action_SetActionId(IntPtr wrapper, string? actionId);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern void Action_SetBlockingType(IntPtr wrapper, string? blockingType);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Action_ClearActionParameters(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Action_AddActionParameter(IntPtr wrapper, IntPtr parameter);

        public string ActionType { get; set; }

        public string ActionId { get; set; }

        public string? ActionDescription { get; set; }
        [JsonConverter(typeof(JsonStringEnumConverter))]
        public BlockingType BlockingType { get; set; }

        public List<ActionParameter>? ActionParameters { get; set; }

        public override void CreateWrapper()
        {
            if (_wrapperPtr.HasValue)
            {
                Action_Destroy(_wrapperPtr.Value);
                _wrapperPtr = null;
            }

            var prt = Action_Create();
            _wrapperPtr = prt;

            Action_ActionType(prt, ActionType);
            Action_SetActionId(prt, ActionId);
            Action_SetActionDescription(prt, ActionDescription);
            Action_SetBlockingType(prt, BlockingType.ToString());

            Action_ClearActionParameters(prt);
            if (ActionParameters == null)
            {
                return;
            }
            foreach (var parameter in ActionParameters)
            {
                parameter.CreateWrapper();
                if (parameter._wrapperPtr.HasValue)
                {
                    Action_AddActionParameter(prt, parameter._wrapperPtr.Value);
                }
            }
        }

        public override void GetDataWrapper(IntPtr prt)
        {
            // Action is used only as a nested "set-only" structure inside Order/InstantActions.
        }

        ~Action()
        {
            if (_wrapperPtr.HasValue)
            {
                Action_Destroy(_wrapperPtr.Value);
                _wrapperPtr = null;
            }
        }
    }
    public class ActionParameter : VDA5050MessageBase

    {
        private const string Lib = "libVDAWrapper.so";
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ActionParameter_Create();

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ActionParameter_Destroy(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern void ActionParameter_SetKey(IntPtr wrapper, string? key);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern void ActionParameter_SetValue(IntPtr wrapper, string? value);
        [JsonPropertyName("key")]
        public string Key { get; set; }

        [JsonPropertyName("value")]
        public object Value { get; set; }
        public override void CreateWrapper()
        {
            if (_wrapperPtr.HasValue)
            {
                ActionParameter_Destroy(_wrapperPtr.Value);
                _wrapperPtr = null;
            }

            var prt = ActionParameter_Create();
            _wrapperPtr = prt;

            ActionParameter_SetKey(prt, Key);
            var value = Value switch
            {
                null => "",
                string s => s,
                _ => JsonSerializer.Serialize(Value)
            };
            ActionParameter_SetValue(prt, value);
        }

        public override void GetDataWrapper(IntPtr prt)
        {
            // ActionParameter is a nested "set-only" structure (inside Action).
        }

        ~ActionParameter()
        {
            if (_wrapperPtr.HasValue)
            {
                ActionParameter_Destroy(_wrapperPtr.Value);
                _wrapperPtr = null;
            }
        }
    }

    public class ActionState : VDA5050MessageBase
    {
        private const string Lib = "libVDAWrapper.so";

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ActionState_GetActionId(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ActionState_GetActionType(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ActionState_GetActionDescription(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ActionState_GetActionStatus(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ActionState_GetResultDescription(IntPtr wrapper);

        public string ActionId { get; set; }
        public string? ActionType { get; set; }
        public string? ActionDescription { get; set; }

        [JsonConverter(typeof(JsonStringEnumConverter))]
        public ActionStatus ActionStatus { get; set; }

        public string? ResultDescription { get; set; }

        public override void CreateWrapper()
        {
            // ActionState is a nested "get-only" structure (read from State wrapper).
        }

        public override void GetDataWrapper(IntPtr prt)
        {
            ActionId = VDA5050MessageBase.PtrToString(ActionState_GetActionId(prt)) ?? "";
            ActionType = VDA5050MessageBase.PtrToString(ActionState_GetActionType(prt));
            ActionDescription = VDA5050MessageBase.PtrToString(ActionState_GetActionDescription(prt));
            ResultDescription = VDA5050MessageBase.PtrToString(ActionState_GetResultDescription(prt));

            var status = VDA5050MessageBase.PtrToString(ActionState_GetActionStatus(prt));
            if (Enum.TryParse<ActionStatus>(status, true, out var parsed))
            {
                ActionStatus = parsed;
            }
        }
    }

    public enum ActionStatus
    {
        WAITING,
        INITIALIZING,
        RUNNING,
        PAUSED,
        FINISHED,
        FAILED
    }
}
