using ConfigApp;
using System.Runtime.InteropServices;
using System.Text;
using VDA5050Message;

namespace RosNodeWrapper
{
    public class VDARosClient
    {
        private IntPtr _nodeVDA;

        private bool _isRunningNode = false;
        
        #region Wrapper function
        internal const string _libVDAClient = "libVDAMissionClient.so";
        internal const string _libVDAMsg = "libVDAMessage.so";
        [DllImport(_libVDAClient)]
        public static extern void InitEnviroment();
        [DllImport(_libVDAClient, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr CreateVDANode(string robotNamespace);

        [DllImport(_libVDAClient)]
        private static extern void SpinNode();

        [DllImport(_libVDAClient)]
        private static extern IntPtr GetCurrentVisualization();

        [DllImport(_libVDAClient)]
        private static extern IntPtr GetCurrentRobotState();

        [DllImport(_libVDAClient)]
        private static extern bool RunThroughPoses(IntPtr nodePtr, double[] xs, double[] ys, int n, double theta_final_rad, byte[] order_id);
        
        [DllImport(_libVDAClient)]
        private static extern bool ExecuteOrder(IntPtr nodePtr, IntPtr orderPtr);

        [DllImport(_libVDAClient)]
        private static extern bool ExecuteInstantActions(IntPtr nodePtr, IntPtr instantActionsPtr);

        #endregion
        public VDARosClient()
        {

            _nodeVDA = CreateVDANode(ConfigData.RosNamespace);
        }


        public void ExecuteOrder(Order order)
        {

            IntPtr? orderWrapperPtr =  order.GetWrapperPtr();
            if (orderWrapperPtr != null && orderWrapperPtr.HasValue)
            {
                ExecuteOrder(_nodeVDA, (IntPtr)orderWrapperPtr);
            }
        }

        public void ExecuteInstantActions(InstantActions instantActions)
        {

            IntPtr? instantActionsWrapperPtr = instantActions.GetWrapperPtr();
            if (instantActionsWrapperPtr != null && instantActionsWrapperPtr.HasValue)
            {
                ExecuteInstantActions(_nodeVDA, (IntPtr)instantActionsWrapperPtr);
            }
        }

        private void threadPublishRobotData()
        {
            
        }
    }
}
