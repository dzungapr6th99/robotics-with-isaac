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
    public class RobotTask : BaseDbObject
    {
        public string? Name { get; set; }
        [DbField(IsKey = true)]
        public int? MapId { get; set; } 
        public string? Description { get; set; }
        [DbField(IsKey = true)]
        public int? RobotTypeId { get; set; }
        [DbField(IsKey = true)]
        public int? RobotTaskTemplateId{get;set;}
        public int? DeliveryFromPointId { get; set; }
        public int? DeliveryToPointId { get; set; }
        public int? Capacity { get; set; }
        [DbField(IgnoreInsert = true, IgnoreUpdate = true)]
        public Point? DeliveryFrom { get; set; }
        [DbField(IgnoreInsert = true, IgnoreUpdate = true)]
        public Point? DeliveryTo { get; set; }
        [DbField(IgnoreInsert = true, IgnoreUpdate = true)]
        public RobotType? RobotType { get; set; }

    }
}
