using DbObject;
using System.Collections.Generic;

namespace BusinessLayer.Interfaces
{
    public interface IPointBL : IBaseBL<Point>
    {
        bool AlignPoints(List<Point> points, out int returnCode, out string returnMessage, out List<string> details);
    }
}
