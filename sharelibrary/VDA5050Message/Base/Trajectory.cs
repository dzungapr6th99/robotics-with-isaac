using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class Trajectory
    {
        public int Degree { get; set; }

        public List<double> KnotVector { get; set; }

        public List<ControlPoint> ControlPoints { get; set; }
    }
}
