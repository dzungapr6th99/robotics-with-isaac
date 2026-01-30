using CommonLib;
using RosNodeWrapper;
using RosNodeWrapper.Interfaces;
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
        private IVDARosClient _vdaRosClient;
        
        public ProcessVDA5050Message(IVDARosClient vdaRosClient)
        {
            _vdaRosClient = vdaRosClient;
        }
        public void ProcessOrder(Order? order, string clientId)
        {
            if (order == null)
            {
                return;
            }
            CommonLog.log.Info("Start Execute order {0}", order.OrderId);
            _vdaRosClient.ExecuteOrder(order);
        }

        public void ProcessConnection(Connection? connection, string clientId)
        {
            if (connection == null)
            {
                return;
            }

            //handle connection
        }

        public void PocessIntantActions(InstantActions? instanceActions)
        {
            if (instanceActions == null)
            {
                return;
            }
            _vdaRosClient.ExecuteInstantActions(instanceActions);
        }
    }
}
