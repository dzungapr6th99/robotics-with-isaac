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
using RosNodeWrapper;
using RosNodeWrapper.Interfaces;
using Disruptor.Dsl;

namespace MqttService
{
    public class MqttClientService : IEventHandler<DataContainer>, IMqttClientService
    {
        private readonly MqttClientFactory _mqttClientFactory;
        private readonly IMqttClient _mqttClient;
        private MqttClientOptions? _mqttOptions;
        protected readonly JsonSerializerOptions _jsonSerializerOptions;
        private Thread? _threadReceiveMessage;
        private ProcessVDA5050Message _processVDAMsg;
        private string? _topicRobotLevel;
        private readonly IVDARosClient _vdaRosClient;
        private Disruptor<DataContainer> _disruptor;
        private int _reconnectTimes = 0;
        private RingBuffer<DataContainer> _ringBuffer;
        private CancellationToken _cancellationToken;
        private Thread _threadKeepConnect;
        private bool _isConnect = false;
        private int _headerIdFactsheets = 0;
        private int _headerIdVisualization = 0;
        private int _headerIdState = 0;
        public MqttClientService(IVDARosClient vdaRosClient)
        {
            _vdaRosClient = vdaRosClient;
            _mqttClientFactory = new MqttClientFactory();
            _mqttClient = _mqttClient = _mqttClientFactory.CreateMqttClient();
            _jsonSerializerOptions = Common.CamelCaseSerialization;
            _processVDAMsg = new ProcessVDA5050Message(_vdaRosClient);
            _disruptor = new Disruptor<DataContainer>(() => new DataContainer(), 1024, TaskScheduler.Default, ProducerType.Single, new BlockingWaitStrategy());
            _ringBuffer = _disruptor.RingBuffer;
            _disruptor.HandleEventsWith(this);
            _vdaRosClient.SubscribeData(_ringBuffer);
            _disruptor.Start();
        }

        public async Task ConnectToBroker(CancellationToken cancellationToken = default)
        {
            IPEndPoint endpoints = IPEndPoint.Parse($"{ShareMemoryData.RobotConfiguration.IP}:{ShareMemoryData.RobotConfiguration.Port}");
            _mqttOptions = new MqttClientOptionsBuilder().WithClientId(ConfigData.MqttClientId).WithEndPoint(endpoints).WithWillTopic("").Build();
            _topicRobotLevel = $"{ShareMemoryData.RobotConfiguration.InterfaceName}/{ShareMemoryData.RobotConfiguration.MajorVersion}/{ShareMemoryData.RobotConfiguration.Manufacturer}/{ShareMemoryData.RobotConfiguration.SerialNumber}";

            _cancellationToken = cancellationToken;
            await _mqttClient.ConnectAsync(_mqttOptions, _cancellationToken);

            await _mqttClient.SubscribeAsync($"{_topicRobotLevel}/#");

            var connection = new Connection()
            {
                HeaderId = _reconnectTimes++,
                ConnectionState = ConnectionState.ONLINE,
                SerialNumber = ShareMemoryData.RobotConfiguration.SerialNumber,
                Timestamp = DateTime.Now,
                Manufacturer = ShareMemoryData.RobotConfiguration.Manufacturer,
                Version = ShareMemoryData.RobotConfiguration.MajorVersion,
            };
            string connMsg = JsonSerializer.Serialize(connection, _jsonSerializerOptions);
            SendMessage(ConstData.Mqtt.Topic.CONNECTION, connMsg);
            _isConnect = true;
            _threadKeepConnect = new Thread(ThreadKeepConnect);
            _threadKeepConnect.IsBackground = true;
            _threadKeepConnect.Start();
            _threadReceiveMessage = new Thread(ThreadReceiveMessage);
            _threadReceiveMessage.IsBackground = true;
            _threadReceiveMessage.Start();
            CommonLog.log.Info("Connect with Mqtt Server Success");

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
                    List<string> topicLevel = e.ApplicationMessage.Topic.Split('/').ToList();


                    string topic = topicLevel.Last();
                    ProcessMessage(topic, e.ClientId, message);
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
                case ConstData.Mqtt.Topic.INSTANTACTIONS:
                    InstantActions? instantActions = JsonSerializer.Deserialize<InstantActions>(payload, _jsonSerializerOptions);
                    _processVDAMsg.PocessIntantActions(instantActions);
                    break;
            }
        }

        public void OnEvent(DataContainer data, long sequence, bool endOfBatch)
        {
            string msg = string.Empty;
            VDA5050MessageBase messageBase = data.Message;
            messageBase.Timestamp = DateTime.Now;
            messageBase.Version = ShareMemoryData.RobotConfiguration.MajorVersion;  
            messageBase.Manufacturer = ShareMemoryData.RobotConfiguration.Manufacturer;
            messageBase.SerialNumber = ShareMemoryData.RobotConfiguration.SerialNumber; 

            switch (data.Topic)
            {
                case ConstData.Mqtt.Topic.VISUALIZATION:
                    messageBase.HeaderId = ++_headerIdVisualization;
                    msg = JsonSerializer.Serialize((Visualization)messageBase, _jsonSerializerOptions);
                    SendMessage(data.Topic, msg);
                    break;
                case ConstData.Mqtt.Topic.STATE:
                    messageBase.HeaderId = ++_headerIdState;
                    msg = JsonSerializer.Serialize((State)messageBase, _jsonSerializerOptions);
                    SendMessage(data.Topic, msg);
                    break;
                case ConstData.Mqtt.Topic.FACTSHEET:
                    messageBase.HeaderId = ++_headerIdFactsheets;
                    msg = JsonSerializer.Serialize((Factsheet)messageBase, _jsonSerializerOptions);
                    SendMessage(data.Topic, msg);
                    break;


            }
        }

        public async Task<bool> SendMessage(string topic, string msgJson)
        {
            try
            {
                if (_mqttClient.IsConnected)
                {
                    var applicationMessage = new MqttApplicationMessageBuilder().WithTopic($"{_topicRobotLevel}/{topic}").WithPayload(msgJson).Build();
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
                else
                {
                    return false;
                }
            }
            catch (Exception ex)
            {
                CommonLog.log.Error(ex, "Error while sending MQTT message {0} with error {1}", topic, ex.Message);
                return false;
            }

        }
        private void ThreadKeepConnect()
        {
            while (_isConnect)
            {
                try
                {
                    if (_mqttClient.IsConnected)
                    {
                        Thread.Sleep(1000);
                    }
                    else
                    {
                        _mqttClient.ReconnectAsync(_cancellationToken).Wait();

                        var connection = new Connection()
                        {
                            HeaderId = _reconnectTimes++,
                            ConnectionState = ConnectionState.ONLINE,
                            SerialNumber = ShareMemoryData.RobotConfiguration.SerialNumber,
                            Timestamp = DateTime.Now,
                            Manufacturer = ShareMemoryData.RobotConfiguration.Manufacturer,
                            Version = ShareMemoryData.RobotConfiguration.MajorVersion,
                        };
                        string connMsg = JsonSerializer.Serialize(connection, _jsonSerializerOptions);
                        SendMessage(ConstData.Mqtt.Topic.CONNECTION, connMsg);
                    }

                }
                catch (Exception ex)
                {
                    CommonLog.log.Error(ex);
                    Thread.Sleep(1000);
                }
            }
        }

    }
}
