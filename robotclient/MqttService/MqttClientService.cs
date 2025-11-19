using System.Text.Json;
using MQTTnet;
using CommonLib;
using System.Text;
using VDA5050Message;
using System.Net;
using ConfigApp;
using Disruptor;
using MqttService.Interfaces;
using LocalMemmory;
namespace MqttService
{
    public class MqttClientService : IEventHandler<DataContainer>, IMqttClientService
    {
        private readonly MqttClientFactory _mqttClientFactory;
        private readonly IMqttClient _mqttClient;
        private readonly MqttClientOptions mqttOptions;
        protected readonly JsonSerializerOptions _jsonSerializerOptions;
        private Thread _threadReceiveMessage;
        private ProcessVDA5050Message _processVDAMsg;
        public MqttClientService()
        {
            _mqttClientFactory = new MqttClientFactory();
            _mqttClient = _mqttClient = _mqttClientFactory.CreateMqttClient();
            _jsonSerializerOptions = Common.CamelCaseSerialization;
            IPEndPoint endpoints = IPEndPoint.Parse(ConfigData.MqttConfig.Endpoint);

            var topics = new[]
            {
                $"{ShareMemoryData.RobotSettings.InterfaceName}/{ShareMemoryData.RobotSettings.MajorVersion}/{ShareMemoryData.RobotSettings.Manufacturer}/{ShareMemoryData.RobotSettings.SerialNumber}/{ConstData.Mqtt.Topic.ORDER}",
                $"{ShareMemoryData.RobotSettings.InterfaceName}/{ShareMemoryData.RobotSettings.MajorVersion}/{ShareMemoryData.RobotSettings.Manufacturer}/{ShareMemoryData.RobotSettings.SerialNumber}/{ConstData.Mqtt.Topic.INSTANTACTIONS}",
            };
            mqttOptions = new MqttClientOptionsBuilder().WithClientId(ConfigData.MqttConfig.ClientId).WithEndPoint(endpoints).WithWillTopic("").Build();
            foreach (var topic in topics)
            {
                _mqttClient.SubscribeAsync(topic).RunSynchronously();
                CommonLog.log.Info($"Subscribed to: {topic}");
            }
        }


        public void StartReceiveMessage()
        {
            _threadReceiveMessage = new Thread(ThreadReceiveMessage);
            _threadReceiveMessage.IsBackground = true;
            _threadReceiveMessage.Start();
        }

        private void ThreadReceiveMessage()
        {
            _threadReceiveMessage = new Thread(() =>
            {
                _mqttClient.ApplicationMessageReceivedAsync += e =>
                {
                    string message = Encoding.ASCII.GetString(e.ApplicationMessage.Payload);
                    ProcessMessage(e.ApplicationMessage.Topic, e.ClientId, message);
                    return Task.CompletedTask;
                };
            });
            _threadReceiveMessage.IsBackground = true;
            _threadReceiveMessage.Start();
        }


        protected virtual void ProcessMessage(string topic, string clientId, string payload)
        {
            switch (topic)
            {
                case ConstData.Mqtt.Topic.ORDER:
                    Order? order = JsonSerializer.Deserialize<Order>(payload, _jsonSerializerOptions);
                    _processVDAMsg.ProcessOrder(order, clientId);
                    break;
                case ConstData.Mqtt.Topic.CONNECTION:
                    Connection? connection = JsonSerializer.Deserialize<Connection>(payload, _jsonSerializerOptions);
                    _processVDAMsg.ProcessConnection(connection, clientId);
                    break;
            }
        }

        public void OnEvent(DataContainer data, long sequence, bool endOfBatch)
        {
            string msg = string.Empty;
            switch (data.Topic)
            {
                case ConstData.Mqtt.Topic.VISUALIZATION:
                    msg = JsonSerializer.Serialize((Visualization)data.Message, _jsonSerializerOptions);
                    SendMessage(data.Topic, msg);
                    break;
                case ConstData.Mqtt.Topic.STATE:
                    msg = JsonSerializer.Serialize((State)data.Message, _jsonSerializerOptions);
                    SendMessage(data.Topic, msg);
                    break;

                    break;


            }
        }

        public async Task<bool> SendMessage(string topic, string msgJson)
        {
            var applicationMessage = new MqttApplicationMessageBuilder().WithTopic(topic).WithPayload(msgJson).Build();
            var sent = await _mqttClient.PublishAsync(applicationMessage);

            if (sent.IsSuccess)
            {
                return true;
            }
            else
            {
                return false;
            }

        }
    }
}
