using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text.Json;
using System.Text.Json.Serialization;
using VDA5050Message.Base;
using ActionWrapper = VDA5050Message.Base.Action;
using ActionParameterWrapper = VDA5050Message.Base.ActionParameter;

namespace VDA5050Message
{
    public class InstantActions : VDA5050MessageBase
    {
        private const string Lib = "libVDAWrapper.so";

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr InstantActions_Create();

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void InstantActions_Destroy(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void InstantActions_SetHeaderId(IntPtr wrapper, int headerId);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern void InstantActions_SetTimeStamp(IntPtr wrapper, string? timestamp);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern void InstantActions_SetVersion(IntPtr wrapper, string? version);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern void InstantActions_SetManufacture(IntPtr wrapper, string? manufacturer);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern void InstantActions_SetSerialNumber(IntPtr wrapper, string? serialNumber);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void InstantActions_ClearActions(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void InstantActions_AddAction(IntPtr wrapper, IntPtr action);

        [JsonIgnore(Condition = JsonIgnoreCondition.Never)]
        public override int HeaderId {get { return base.HeaderId; } set { base.HeaderId = value; } }

        [JsonIgnore(Condition = JsonIgnoreCondition.Never)]
        public override DateTime Timestamp {get { return base.Timestamp; } set { base.Timestamp = value; } }

        [JsonIgnore(Condition = JsonIgnoreCondition.Never)]
        public override string Version {get { return base.Version; } set { base.Version = value; } }

        [JsonIgnore(Condition = JsonIgnoreCondition.Never)]
        public override string Manufacturer {get { return base.Manufacturer; } set { base.Manufacturer = value; } }


        [JsonIgnore(Condition = JsonIgnoreCondition.Never)]
        public override string SerialNumber {get { return base.SerialNumber; } set { base.SerialNumber = value; } }
        public List<InstantAction> Actions { get; set; } = new();

        public override void CreateWrapper()
        {
            if (_wrapperPtr.HasValue)
            {
                InstantActions_Destroy(_wrapperPtr.Value);
                _wrapperPtr = null;
            }

            var prt = InstantActions_Create();
            _wrapperPtr = prt;

            InstantActions_SetHeaderId(prt, HeaderId);
            InstantActions_SetTimeStamp(prt, Timestamp.ToString("o"));
            InstantActions_SetVersion(prt, Version);
            InstantActions_SetManufacture(prt, Manufacturer);
            InstantActions_SetSerialNumber(prt, SerialNumber);

            InstantActions_ClearActions(prt);
            foreach (var action in Actions)
            {
                var actionPtr = ActionWrapper.Action_Create();
                try
                {
                    ActionWrapper.Action_ActionType(actionPtr, action.ActionType);
                    ActionWrapper.Action_SetActionId(actionPtr, action.ActionId);
                    ActionWrapper.Action_SetActionDescription(actionPtr, action.ActionDescription);
                    ActionWrapper.Action_SetBlockingType(actionPtr, action.BlockingType.ToString());

                    ActionWrapper.Action_ClearActionParameters(actionPtr);
                    if (action.ActionParameters != null)
                    {
                        foreach (var parameter in action.ActionParameters)
                        {
                            var parameterPtr = ActionParameterWrapper.ActionParameter_Create();
                            try
                            {
                                ActionParameterWrapper.ActionParameter_SetKey(parameterPtr, parameter.Key);
                                var value = parameter.Value switch
                                {
                                    null => "",
                                    string s => s,
                                    _ => JsonSerializer.Serialize(parameter.Value)
                                };
                                ActionParameterWrapper.ActionParameter_SetValue(parameterPtr, value);
                                ActionWrapper.Action_AddActionParameter(actionPtr, parameterPtr);
                            }
                            finally
                            {
                                ActionParameterWrapper.ActionParameter_Destroy(parameterPtr);
                            }
                        }
                    }

                    InstantActions_AddAction(prt, actionPtr);
                }
                finally
                {
                    ActionWrapper.Action_Destroy(actionPtr);
                }
            }
        }

        public override void GetDataWrapper(IntPtr prt)
        {
            // InstantActions is a "set-only" message (robot receives), so C# does not read it from a C++ wrapper.
        }

        ~InstantActions()
        {
            if (_wrapperPtr.HasValue)
            {
                InstantActions_Destroy(_wrapperPtr.Value);
                _wrapperPtr = null;
            }
        }
    }

    public class InstantAction
    {
        public string ActionId { get; set; }

        public string ActionType { get; set; }

        public string? ActionDescription { get; set; }

        [JsonConverter(typeof(JsonStringEnumConverter))]
        public BlockingType BlockingType { get; set; }

        public List<ActionParameter>? ActionParameters { get; set; }
    }
}
