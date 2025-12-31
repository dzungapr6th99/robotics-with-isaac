using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{

    public class Edge
    {
        public string EdgeId { get; set; }

        public int SequenceId { get; set; }

        public string? EdgeDescription { get; set; }

        public bool Released { get; set; }

        public string StartNodeId { get; set; }

        public string EndNodeId { get; set; }

        public double? MaxSpeed { get; set; }

        public double? MaxHeight { get; set; }
        public double? MinHeight { get; set; }

        public double? Orientation { get; set; }

        public string? OrientationType { get; set; }

        public string? Direction { get; set; }

        public bool? RotationAllowed { get; set; }

        public double? MaxRotationSpeed { get; set; }

        public double? Length { get; set; }

        public Trajectory? Trajectory { get; set; }

        public Corridor? Corridor { get; set; }

        public List<Action> Actions { get; set; } = new();
    }

    public class EdgeState
    {
        public string EdgeId { get; set; }
        public int SequenceId { get; set; }
        public string? EdgeDescription { get; set; }
        public bool Released { get; set; }
        public Trajectory? Trajectory { get; set; }
    }

}
