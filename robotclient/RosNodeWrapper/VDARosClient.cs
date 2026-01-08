using CommonLib;
using ConfigApp;
using Disruptor;
using LocalMemmory;
using RosNodeWrapper.Interfaces;
using System.Diagnostics;
using System.IO;
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

#if DEBUG
        private static readonly object _debugEnvInitLock = new();
        private static bool _debugEnvInitialized;
#endif

        #region Wrapper function
        internal const string _libVDAClient = "libVDAMissionClient.so";
        [DllImport(_libVDAClient)]
        private static extern void InitEnviroment();
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
            try
            {
                CommonLog.log.Info("Initializing VDA ROS Client");
                _listRingBuffer = new List<RingBuffer<DataContainer>>();
#if DEBUG
                EnsureDebugRosEnvironment();
#endif
                //InitEnviroment();
                //_nodeVDA = CreateVDAMissionClient(ConfigData.RosNamespace);
                CommonLog.log.Info("VDARosClient initialized successfully");
            }
            catch (Exception ex)
            {
                CommonLog.log.Error(ex);
                throw;
            }
        }

#if DEBUG
        private static void EnsureDebugRosEnvironment()
        {
            lock (_debugEnvInitLock)
            {
                if (_debugEnvInitialized)
                {
                    return;
                }

                _debugEnvInitialized = true;

                var extraLdPaths = new[]
                {
                    "/opt/ros/humble/lib",
                    "/home/it-team/Desktop/RobotIsaac/RobotClientIsaac/roswrapperpackage/build/VDAMissionClient",
                    "/home/it-team/Desktop/RobotIsaac/RobotClientIsaac/roswrapperpackage/build/vda5050_msgs",
                };

                try
                {
                    PrependPathEnv("LD_LIBRARY_PATH", extraLdPaths);

                    var rosSetup = "/opt/ros/humble/setup.bash";
                    var wrapperSetup = "/home/it-team/Desktop/RobotIsaac/RobotClientIsaac/roswrapperpackage/install/setup.bash";
                    if (!File.Exists(rosSetup) || !File.Exists(wrapperSetup))
                    {
                        CommonLog.log.Warn($"ROS setup files not found; skip debug env bootstrap. ros='{rosSetup}' wrapper='{wrapperSetup}'");
                        return;
                    }

                    ApplyBashSourcedEnvironment($"source \"{rosSetup}\" && source \"{wrapperSetup}\" && env -0");
                }
                catch (Exception ex)
                {
                    CommonLog.log.Error(ex);
                }
            }
        }

        private static void PrependPathEnv(string name, IEnumerable<string> prependPaths)
        {
            var current = Environment.GetEnvironmentVariable(name) ?? string.Empty;
            var parts = new List<string>();
            var seen = new HashSet<string>(StringComparer.Ordinal);

            foreach (var path in prependPaths)
            {
                if (string.IsNullOrWhiteSpace(path))
                {
                    continue;
                }

                if (seen.Add(path))
                {
                    parts.Add(path);
                }
            }

            foreach (var existing in current.Split(':', StringSplitOptions.RemoveEmptyEntries))
            {
                if (seen.Add(existing))
                {
                    parts.Add(existing);
                }
            }

            Environment.SetEnvironmentVariable(name, string.Join(':', parts));
        }

        private static void ApplyBashSourcedEnvironment(string bashCommand)
        {
            var startInfo = new ProcessStartInfo
            {
                FileName = "/bin/bash",
                ArgumentList = { "-lc", bashCommand },
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                UseShellExecute = false,
                StandardOutputEncoding = Encoding.UTF8,
                StandardErrorEncoding = Encoding.UTF8,
            };

            using var process = Process.Start(startInfo);
            if (process == null)
            {
                CommonLog.log.Warn("Failed to start bash to source ROS environment.");
                return;
            }

            var stdout = process.StandardOutput.ReadToEnd();
            var stderr = process.StandardError.ReadToEnd();
            process.WaitForExit();

            if (process.ExitCode != 0)
            {
                CommonLog.log.Warn($"ROS debug env bootstrap failed (exit {process.ExitCode}): {stderr}");
                return;
            }

            if (!string.IsNullOrWhiteSpace(stderr))
            {
                CommonLog.log.Warn($"ROS debug env bootstrap stderr: {stderr}");
            }

            foreach (var entry in stdout.Split('\0', StringSplitOptions.RemoveEmptyEntries))
            {
                var idx = entry.IndexOf('=');
                if (idx <= 0)
                {
                    continue;
                }

                var key = entry.Substring(0, idx);
                var value = entry.Substring(idx + 1);

                if (!IsRosRelatedEnvKey(key))
                {
                    continue;
                }

                Environment.SetEnvironmentVariable(key, value);
            }
        }

        private static bool IsRosRelatedEnvKey(string key)
        {
            return key.StartsWith("ROS_", StringComparison.Ordinal)
                || key.StartsWith("AMENT_", StringComparison.Ordinal)
                || key.StartsWith("COLCON_", StringComparison.Ordinal)
                || key.StartsWith("RMW_", StringComparison.Ordinal)
                || key.StartsWith("RCUTILS_", StringComparison.Ordinal)
                || key.StartsWith("FASTRTPS_", StringComparison.Ordinal)
                || key.StartsWith("CYCLONEDDS_", StringComparison.Ordinal)
                || key.StartsWith("DDS_", StringComparison.Ordinal)
                || key == "CMAKE_PREFIX_PATH"
                || key == "LD_LIBRARY_PATH"
                || key == "PATH"
                || key == "PYTHONPATH";
        }
#endif

        public void StartSpinNode()
        {
            if (!_isRunningNode && (_threadSpinNode == null || !_threadSpinNode.IsAlive))
            {
                _isRunningNode = true;
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
            CommonLog.log.Info("Starting VDA ROS Node spin thread.");
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
