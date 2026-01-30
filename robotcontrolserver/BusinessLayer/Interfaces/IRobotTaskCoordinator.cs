using System;
using DbObject;

namespace BusinessLayer.Interfaces
{
    public interface IRobotTaskCoordinator
    {
        public void Enqueue(RobotTask task);
    }
}
