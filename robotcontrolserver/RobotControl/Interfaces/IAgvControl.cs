using System;
using ShareMemoryData;
using VDA5050Message;

namespace RobotControl.Interfaces;

public interface IAgvControl
{

    Task ConnectToServer(CancellationToken cancellationToken);
    Task<bool> SendConnection(Connection connection);
    Task<bool> SendOrder(Order order, RobotStatus robot);
    Task<bool> SendInstantActions(InstantActions instantActions, RobotStatus robot);
}
