using Disruptor;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VDA5050Message;

namespace RosNodeWrapper.Interfaces
{
    public interface IVDARosClient
    {
        public void StartSpinNode();
        public void ExecuteOrder(Order order);
        public void ExecuteInstantActions(InstantActions instantActions);
        public void SubscribeData(RingBuffer<DataContainer> queueMsgService);
        public void UnsubscribeData(RingBuffer<DataContainer> queueMsgService);
    }
}
