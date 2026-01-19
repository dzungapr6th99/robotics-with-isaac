using DbObject.Attributes;
using DbObject.Common;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DbObject
{
    [DbTable(Name = DatabaseEnum.TableName.RobotTask)]
    public class RobotTask
    {
        [DbField(IsKey = true)]
        public int? Id { get; set; }
        public string? Name { get; set; }    
        public string? Description { get; set; } 
        public int? RobotTypeId { get; set; }
        [DbField(IgnoreInsert = true, IgnoreUpdate = true)]
        public RobotType? RobotType { get; set; }

    }
}
