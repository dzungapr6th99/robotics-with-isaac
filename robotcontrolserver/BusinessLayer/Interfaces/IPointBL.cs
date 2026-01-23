using DbObject;
using System.Collections.Generic;

namespace BusinessLayer.Interfaces
{
    public interface IPointBL
    {
        bool AlignPoints(List<Point> points, out int returnCode, out string returnMessage, out List<string> details);
    }
}
