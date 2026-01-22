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
namespace RobotControl
{
    public class AgvControl : IAgvControl
    {
        private readonly MqttClientFactory _mqttClientFactory;
        private IMqttClient _mqttClient;
        private MqttClientOptions _mqttClientOptions;
        private CancellationToken _cancelationToken;
        private readonly IBaseDA<Robot> _robotBaseDA;
        private readonly IDbManagement _dbManagement;
        private JsonSerializerOptions _jsonSerializerOptions;
        private Thread _threadReceiveMessage, _threadKeepConnect;

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
            _mqttClientOptions = new MqttClientOptionsBuilder().WithClientId(ConfigData.MqttClientConfig.AgvControl).WithEndPoint(endpoints).WithWillTopic("").Build();
            CommonLog.log.Info("Setup Mqtt broker for AgvControl done");

        }

        public async Task ConnectToServer(CancellationToken cancellationToken)
        {
            try
            {
                CommonLog.log.Info("Start connect with Mqtt Server");
                _cancelationToken = cancellationToken;
                _isConnect = true;
                await _mqttClient.ConnectAsync(_mqttClientOptions, _cancelationToken);
                await _mqttClient.SubscribeAsync("#");

            }
            catch(Exception ex)
            {
                CommonLog.log.Error(ex);
            }
            finally
            {
                _threadReceiveMessage = new Thread(ThreadReceiveMessage);
                _threadReceiveMessage.IsBackground = true;
                _threadReceiveMessage.Start();
                _threadKeepConnect = new Thread(ThreadKeepConnect);
                _threadKeepConnect.IsBackground = true;
                _threadKeepConnect.Start();
            }
        }


        #region Thread
                private void ThreadKeepConnect()
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
                        if (_reconnectTimes == 0)
                        {
                            CommonLog.log.Debug("Reconnect time = 0, Agv Control can not connect to Broker. Reconnect session");
                            _mqttClient.ConnectAsync(_mqttOptions, _cancellationToken).Wait();
                            _mqttClient.SubscribeAsync("#").Wait();
                            Interlocked.Increment(ref _reconnectTimes);
                            if (!_threadReceiveMessage.IsAlive)
                            {
                                _threadReceiveMessage = new Thread(ThreadReceiveMessage);
                                _threadReceiveMessage.IsBackground = true;
                                _threadReceiveMessage.Start();
                            }
                            var reconnectMessage = new Connection()
                            {
                                HeaderId = _reconnectTimes,
                                ConnectionState = ConnectionState.ONLINE,
                                SerialNumber = ConfigData.MqttClientConfig.AgvControl,
                                Timestamp = DateTime.Now,
                                Manufacturer = ConfigData.Manufacturer,
                                Version = ConfigData.Version,
                            };
                            SendConnection(reconnectMessage, ConfigData.MqttClientConfig.AgvControl, ExtractMajorVersion(reconnectMessage.Version), ConfigData.Manufacturer).Wait();
                        }
                        else
                        {
                            CommonLog.log.Debug("Reconnect time # 0, Agv Control can not connect to Broker. Reconnect session");
                            _mqttClient.ReconnectAsync(_cancelationToken).Wait();
                            _mqttClient.SubscribeAsync("#").Wait();
                            if (!_threadReceiveMessage.IsAlive)
                            {
                                _threadReceiveMessage = new Thread(ThreadReceiveMessage);
                                _threadReceiveMessage.IsBackground = true;
                                _threadReceiveMessage.Start();
                            }
                            Interlocked.Increment(ref _reconnectTimes);
                            var reconnectMessage = new Connection()
                            {
                                HeaderId = _reconnectTimes,
                                ConnectionState = ConnectionState.ONLINE,
                                SerialNumber = ConfigData.MqttClientConfig.AgvControl,
                                Timestamp = DateTime.Now,
                                Manufacturer = ConfigData.Manufacturer,
                                Version = ConfigData.Version,
                            };
                            SendConnection(reconnectMessage, ConfigData.MqttClientConfig.AgvControl, ExtractMajorVersion(reconnectMessage.Version), ConfigData.Manufacturer).Wait();
                        }
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

        #endregion
    }
}
