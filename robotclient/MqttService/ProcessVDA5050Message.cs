using RosNodeWrapper;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VDA5050Message;

namespace MqttService
{
    internal class ProcessVDA5050Message
    {
        private VDARosClient _vdaRosClient;
        public ProcessVDA5050Message()
        {
            _vdaRosClient = new VDARosClient();
        }
        public void ProcessOrder(Order? order, string clientId)
        {
            if (order == null)
            {
                return;
            }
            _vdaRosClient.ExecuteOrder(order);

            //Handle order

        }

        public void ProcessConnection(Connection connection, string clientId)
        {
            if (connection == null)
            {
                return;
            }


            //handle connection
        }
    }
}
