using DbObject.Attributes;
using DbObject.Common;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DbObject
{
    [DbTable(Name = DatabaseEnum.TableName.RobotType)]
    public class RobotType
    {
        [DbField(IsKey = true)]
        public int Id { get; set; }
        public string ModelName { get; set; }
        public int MaxCapacity { get; set; } = 1;

    }
}
