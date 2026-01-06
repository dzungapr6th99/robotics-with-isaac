using CommonLib;
using ConfigApp;
using Disruptor;
using LocalMemmory;
using RosNodeWrapper.Interfaces;
using System.Runtime.InteropServices;
using System.Text;
using VDA5050Message;
using VDA5050Message.Base;
using static CommonLib.ConstData.Mqtt;
namespace RosNodeWrapper
{
    public class VDARosClient : IVDARosClient
    {
        private IntPtr _nodeVDA;

        private bool _isRunningNode = false;

        #region Wrapper function
        internal const string _libVDAClient = "libVDAMissionClient.so";
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

        private List<RingBuffer<DataContainer>> _listRingBuffer;
        private Thread? _threadSpinNode;
        #endregion
        public VDARosClient()
        {
            _listRingBuffer = new List<RingBuffer<DataContainer>>();
            InitEnviroment();
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

        public void SubscribeData(RingBuffer<DataContainer> queueMsgService)
        {
            _listRingBuffer.Add(queueMsgService);
        }

        public void UnsubscribeData(RingBuffer<DataContainer> queueMsgService)
        {
            _listRingBuffer.Remove(queueMsgService);
        }

        private void threadSpinNodeVDA()
        {
            while (_isRunningNode)
            {
                try
                {
                    SpinNode(_nodeVDA);
                    IntPtr ptrVisualization = GetVisualization();
                    IntPtr ptrState = GetAGVState();
                    Visualization visualization = new Visualization();
                    visualization.GetDataWrapper(ptrVisualization);
                    State state = new State();
                    state.GetDataWrapper(ptrState);
                    foreach (var msgQueueItem in _listRingBuffer)
                    {
                        EnqueueToRingBuffer(state, EnumData.TopicName.STATE, msgQueueItem);
                        EnqueueToRingBuffer(visualization, EnumData.TopicName.VISUALIZATION, msgQueueItem);
                    }
                }
                catch (Exception ex)
                {
                    CommonLog.log.Error(ex);
                }
                Thread.Sleep(100);
            }
        }

        private bool EnqueueToRingBuffer(VDA5050MessageBase message, string topic, RingBuffer<DataContainer> ringBuffer)
        {
            if (ringBuffer.GetRemainingCapacity() > 0)
            {
                long sequences = ringBuffer.Next();
                try
                {
                    ringBuffer[sequences].Message = message;
                    ringBuffer[sequences].Topic = topic;
                }
                finally
                {
                    ringBuffer.Publish(sequences);
                }
            }

            return true;
        }
    }
}