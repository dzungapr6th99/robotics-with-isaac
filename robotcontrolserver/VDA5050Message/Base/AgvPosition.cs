using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class AgvPosition
    {
        public double X { get; set; }

        public double Y { get; set; }

        public double Theta { get; set; }

        public string MapId { get; set; }

        public string? MapDescription { get; set; }

        public bool PositionInitialized { get; set; }

        public double? LocalizationScore { get; set; }

        public double? DeviationRange { get; set; }
    }
}
