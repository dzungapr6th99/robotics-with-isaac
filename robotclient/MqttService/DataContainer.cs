using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VDA5050Message;

namespace MqttService
{
    public class DataContainer
    {
        public string Topic;
        public VDA5050MessageBase Message;

        public DataContainer()
        {
            Topic = string.Empty;
            Message = new VDA5050MessageBase();
        }
    }
}
