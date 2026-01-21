using System;
using DbObject.Attributes;
using DbObject.Common;

namespace DbObject
{
    [DbTable(Name = DatabaseEnum.TableName.Robot)]
    public class Robot
    {
        public int? Id { get; set; }
        public string? InterfaceName { get; set; }
        public string? Manufacturer { get; set; }
        public string? SerialNumber { get; set; }
        public int? RobotTypeId { get; set; }

        [DbField(IgnoreInsert = true, IgnoreUpdate = true)]
        public RobotType? RobotType { get; set; }
    }
}
