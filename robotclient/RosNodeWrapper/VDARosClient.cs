using ConfigApp;
using System.Runtime.InteropServices;
using System.Text;
using VDA5050Message;

namespace RosNodeWrapper
{
    public class VDARosClient
    {
        private IntPtr _nodeVDA;
        [DllImport("libVDAMissionClient.so")]
        public static extern void InitEnviroment();
        [DllImport("libVDAMissionClient.so", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr CreateVDANode(string robotNamespace);


        [DllImport("libVDAMissionClient.so")]
        private static extern bool RunThroughPoses(IntPtr nodePtr, double[] xs, double[] ys, int n, double theta_final_rad, byte[] order_id);
        public VDARosClient()
        {
            _nodeVDA = CreateVDANode(ConfigData.RosNamespace);
        }


        public void ExecuteOrder(Order order)
        {
            List<double> x = new List<double>();
            List<double> y = new List<double>();
            foreach (var node in order.Nodes)
            {
                if (node.NodePosition != null)
                {
                    x.Add(node.NodePosition.X);
                    y.Add(node.NodePosition.Y);
                }
            }
            double theta = order.Nodes.Last().NodePosition?.Theta ?? 0;
            byte[] orderId = Encoding.ASCII.GetBytes(order.OrderId);
            RunThroughPoses(_nodeVDA, x.ToArray(), y.ToArray(), x.Count(), theta ,orderId);
        }
    }
}
