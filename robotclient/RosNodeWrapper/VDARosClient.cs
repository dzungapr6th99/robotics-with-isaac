using ConfigApp;
using Disruptor;
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
        private static extern IntPtr CreateVDAMissionClient(string robotNamespace);

        [DllImport(_libVDAClient)]
        private static extern void SpinNode(IntPtr nodePtr);

        [DllImport(_libVDAClient)]
        private static extern IntPtr GetVisualization();

        [DllImport(_libVDAClient)]
        private static extern IntPtr GetAGVState();

        [DllImport(_libVDAClient)]
        private static extern bool RunThroughPoses(IntPtr nodePtr, double[] xs, double[] ys, int n, double theta_final_rad, byte[] order_id);

        [DllImport(_libVDAClient)]
        private static extern bool ExecuteOrder(IntPtr nodePtr, IntPtr orderPtr);

        [DllImport(_libVDAClient)]
        private static extern bool ExecuteInstantActions(IntPtr nodePtr, IntPtr instantActionsPtr);

        private List<IEventHandler<VDA5050MessageBase>> _listMessageQueueService;

        private Thread? _threadSpinNode;
        #endregion
        public VDARosClient()
        {
            _listMessageQueueService = new List<IEventHandler<VDA5050MessageBase>>();
            _nodeVDA = CreateVDAMissionClient(ConfigData.RosNamespace);
        }

        public void StartSpinNode()
        {
            if (_threadSpinNode == null || !_threadSpinNode.IsAlive)
            {
                _threadSpinNode = new Thread(threadSpinNodeVDA);
                _threadSpinNode.IsBackground = true;
                _threadSpinNode.Start();
            }
        }

        public void ExecuteOrder(Order order)
        {

            IntPtr? orderWrapperPtr = order.GetWrapperPtr();
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

        public void SubscribeData(IEventHandler<VDA5050MessageBase> queueMsgService)
        {
            _listMessageQueueService.Add(queueMsgService);
        }

        public void UnsubscribeData(IEventHandler<VDA5050MessageBase> queueMsgService)
        {
            _listMessageQueueService.Remove(queueMsgService);
        }

        private void threadSpinNodeVDA()
        {
            while (_isRunningNode)
            {
                SpinNode(_nodeVDA);
                IntPtr ptrVisualization = GetVisualization();
                IntPtr ptrState = GetAGVState();
                Visualization visualization = new Visualization();
                visualization.GetDataWrapper(ptrVisualization);
                State state = new State();
                state.GetDataWrapper(ptrState);

            }
        }
    }
}