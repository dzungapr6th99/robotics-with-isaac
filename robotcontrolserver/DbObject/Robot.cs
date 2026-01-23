using System;
using DbObject.Attributes;
using DbObject.Common;

namespace DbObject
{
    [DbTable(Name = DatabaseEnum.TableName.Robot)]
    public class Robot : BaseDbObject
    {
        public string? InterfaceName { get; set; }
        public string? MajorVersion { get; set; }
        public string? Manufacturer { get; set; }
        public string? SerialNumber { get; set; }
        public int? RobotTypeId { get; set; }

        [DbField(IgnoreInsert = true, IgnoreUpdate = true)]
        public RobotType? RobotType { get; set; }
    }
}
