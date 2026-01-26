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
    public class RobotTaskTemplate : BaseDbObject
    {
        [DbField(IsKey = true)]
        public int? MapId { get; set; }
        [DbField(IsKey = true)]
        public int? FromPointId { get; set; }
        [DbField(IsKey = true)]
        public int? ToPointId { get; set; }
        [DbField(IsKey = true)]
        public int? RobotTypeId { get; set; }
        public string? Name { get; set; }
        public string? Description { get; set; }

        [DbField(IgnoreInsert = true, IgnoreUpdate = true)]
        public Point? FromPoint { get; set; }
        [DbField(IgnoreInsert = true, IgnoreUpdate = true)]
        public Point? ToPoint { get; set; }
    }
}
