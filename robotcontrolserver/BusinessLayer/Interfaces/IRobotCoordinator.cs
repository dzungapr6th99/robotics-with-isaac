using System;
using DbObject;

namespace BusinessLayer.Interfaces
{
    public interface IRobotCoordinator
    {
        public void Enqueue(RobotTask task);
    }
}
