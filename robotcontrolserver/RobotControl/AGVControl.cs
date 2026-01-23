using CommonLib;
using VDA5050Message;
using MQTTnet;
using Disruptor;
using RobotControl.Interfaces;
using DataAccess.Interfaces;
using DbObject;
using DataAccess.Interface;
using System.Net;
using ConfigApp;
using System.Text.Json;
using ShareMemoryData;
using System.Text;
using Microsoft.AspNetCore.Http.Features;
namespace RobotControl
{
    public class AgvControl : IAgvControl
    {
        private readonly MqttClientFactory _mqttClientFactory;
        private IMqttClient _mqttClient;
        private MqttClientOptions _mqttClientOptions;
        private CancellationToken _cancellationToken;
        private readonly IBaseDA<Robot> _robotBaseDA;
        private readonly IDbManagement _dbManagement;
        private JsonSerializerOptions _jsonSerializerOptions;
        private Thread? _threadReceiveMessage, _threadKeepConnect;
        private int _reconnectTimes = 0;
        private bool _isConnect = false;

        public AgvControl(IBaseDA<Robot> robotBaseDA, IDbManagement dbManagement)
        {
            _robotBaseDA = robotBaseDA;
            _dbManagement = dbManagement;
            _mqttClientFactory = new MqttClientFactory();
            _mqttClientOptions = new MqttClientOptions();
            _jsonSerializerOptions = Common.CamelCaseSerialization;
            _mqttClient = _mqttClientFactory.CreateMqttClient();
            IPEndPoint endpoints = IPEndPoint.Parse(ConfigData.MqttClientConfig.Endpoint);
            _mqttClientOptions = new MqttClientOptionsBuilder()
                                        .WithClientId(ConfigData.MqttClientConfig.AgvControl)
                                        .WithEndPoint(endpoints)
                                        .WithWillTopic("")
                                        .Build();
            CommonLog.log.Info("Setup Mqtt broker for AgvControl done");

        }

        public async Task ConnectToServer(CancellationToken cancellationToken)
        {
            try
            {
                CommonLog.log.Info("Start connect with Mqtt Server");
                _cancellationToken = cancellationToken;
                _isConnect = true;
                await _mqttClient.ConnectAsync(_mqttClientOptions, _cancellationToken);
                await _mqttClient.SubscribeAsync("#");

            }
            catch (Exception ex)
            {
                CommonLog.log.Error(ex);
            }
            finally
            {
                _threadReceiveMessage = new Thread(threadReceiveMessage);
                _threadReceiveMessage.IsBackground = true;
                _threadReceiveMessage.Start();
                _threadKeepConnect = new Thread(threadKeepConnect);
                _threadKeepConnect.IsBackground = true;
                _threadKeepConnect.Start();
            }
        }


        #region Thread
        private void threadKeepConnect()
        {
            CommonLog.log.Info("Start Thread keep connect");
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

                        CommonLog.log.Debug("Reconnect time = 0, Agv Control can not connect to Broker. Reconnect session");
                        _mqttClient.ConnectAsync(_mqttClientOptions, _cancellationToken).Wait();
                        _mqttClient.SubscribeAsync("#").Wait();
                        if (!(_threadReceiveMessage != null && _threadReceiveMessage.IsAlive))
                        {
                            _threadReceiveMessage = new Thread(threadReceiveMessage);
                            _threadReceiveMessage.IsBackground = true;
                            _threadReceiveMessage.Start();
                        }
                        var reconnectMessage = new Connection()
                        {
                            HeaderId = _reconnectTimes++,
                            ConnectionState = ConnectionState.ONLINE,
                            SerialNumber = ConfigData.MqttClientConfig.AgvControl,
                            Timestamp = DateTime.Now,
                            Manufacturer = ConfigData.MqttClientConfig.Manufacturer,
                            Version = ConfigData.Version,
                        };
                        SendConnection(reconnectMessage).Wait();

                        Thread.Sleep(100);
                    }

                }
                catch (Exception ex)
                {
                    CommonLog.log.Error(ex);
                    Thread.Sleep(1000);
                }
            }
        }

        private void threadReceiveMessage()
        {

            _mqttClient.ApplicationMessageReceivedAsync += e =>
            {
                string message = Encoding.ASCII.GetString(e.ApplicationMessage.Payload);
                string allTopic = e.ApplicationMessage.Topic;
                List<string> levelTopic = allTopic.Split('/').ToList();
                string interfaceName = "";
                string version = "";
                string manufacturer = "";
                string serialNumber = "";
                string topic = "";
                if (levelTopic.Count >= 5)
                {
                    interfaceName = levelTopic[0];
                    version = levelTopic[1];
                    manufacturer = levelTopic[2];
                    serialNumber = levelTopic[3];
                    topic = levelTopic[4];
                    processMessage(e.ApplicationMessage.Topic, message, version, interfaceName, manufacturer, serialNumber);
                }
                return Task.CompletedTask;
            };
        }
        #endregion

        #region send message
        public async Task<bool> SendConnection(Connection connection)
        {
            CommonLog.log.Info("Start send connection");
            string topic = BuildTopic(ConstData.Mqtt.Topic.CONNECTION,
                                      ConfigData.MqttClientConfig.InterfaceName,
                                      ConfigData.MqttClientConfig.MajorVersion,
                                      ConfigData.MqttClientConfig.Manufacturer,
                                      ConfigData.MqttClientConfig.AgvControl);
            string jsonString = JsonSerializer.Serialize(connection, _jsonSerializerOptions);
            var applicationMessage = new MqttApplicationMessageBuilder().WithTopic(topic).WithPayload(jsonString).Build();

            var sent = await _mqttClient.PublishAsync(applicationMessage);
            if (!sent.IsSuccess)
            {
                CommonLog.log.Warn("Send connection message fail");
                return false;
            }
            return true;
        }

        public async Task<bool> SendOrder(Order order, RobotStatus robot)
        {
            string topic = BuildTopic(ConstData.Mqtt.Topic.ORDER, robot.InterfaceName, robot.MajorVersion, robot.Manufacturer, robot.SerialNumber);
            string jsonString = JsonSerializer.Serialize(order, _jsonSerializerOptions);
            var applicationMessage = new MqttApplicationMessageBuilder().WithTopic(topic).WithPayload(jsonString).Build();

            var sent = await _mqttClient.PublishAsync(applicationMessage);
            if (!sent.IsSuccess)
            {
                CommonLog.log.Warn("Send connection message fail");
                return false;
            }
            return true;
        }


        public async Task<bool> SendInstantActions(InstantActions instantActions, RobotStatus robot)
        {
            string topic = BuildTopic(ConstData.Mqtt.Topic.INSTANTACTIONS, robot.InterfaceName, robot.MajorVersion, robot.Manufacturer, robot.SerialNumber);
            string jsonString = JsonSerializer.Serialize(instantActions, _jsonSerializerOptions);
            var applicationMessage = new MqttApplicationMessageBuilder().WithTopic(topic).WithPayload(jsonString).Build();

            var sent = await _mqttClient.PublishAsync(applicationMessage);
            if (!sent.IsSuccess)
            {
                CommonLog.log.Warn("Send connection message fail");
                return false;
            }
            return true;
        }



        private string BuildTopic(string topic, string interfaceName, string majorVersion, string manufacturer, string serialNumber)
        {
            return $"{interfaceName}/{majorVersion}/{manufacturer}/{serialNumber}/{topic}";
        }


        #endregion


        #region Process received Message
        private void processMessage(string topic, string message, string interfaceName, string majorVersion, string manufacturer, string serialNumber)
        {
            switch (topic)
            {
                case ConstData.Mqtt.Topic.STATE:
                    State? state = JsonSerializer.Deserialize<State>(message, _jsonSerializerOptions);
                    processState(state);
                    break;
                case ConstData.Mqtt.Topic.CONNECTION:
                    Connection? connection = JsonSerializer.Deserialize<Connection>(message);
                    processConnection(connection);
                    break;
            }
        }


        private void processState(State? state)
        {
            if (state != null)
            {
                LocalMemory.UpdateRobotStatus(state);

            }
        }

        private void processConnection(Connection? connection)
        {
            
        }
        #endregion
    }
}
