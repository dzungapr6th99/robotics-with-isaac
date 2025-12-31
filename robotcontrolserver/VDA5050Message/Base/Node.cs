using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class Node
    {
        public string NodeId { get; set; }

        public int SequenceId { get; set; }

        public string? NodeDescription { get; set; }

        public bool Released { get; set; }

        public NodePosition? NodePosition { get; set; }

        public List<Action> Actions { get; set; } = new();
    }


    public class NodePosition
    {
        public double X { get; set; }

        public double Y { get; set; }

        public double? Theta { get; set; }

        public double? AllowedDeviationXY { get; set; }

        public double? AllowedDeviationTheta { get; set; }

        public string MapId { get; set; }

        public string? MapDescription { get; set; }
    }


    public class NodeState
    {
        public string NodeId { get; set; }
        public int SequenceId { get; set; }
        public string? NodeDescription { get; set; }
        public bool Released { get; set; }
        public Node? NodePosition { get; set; }
    }

}
