using System;
using DbObject.Attributes;
using DbObject.Common;

namespace DbObject
{
    [DbTable(Name = DatabaseEnum.TableName.Map)]
    public class Map
    {
        [DbField(IsKey = true)]
        public int? Id { get; set; }

        public string? Name { get; set; }

        public string? MinioUrl { get; set; }

        [DbField(IgnoreInsert = true, IgnoreUpdate = true)]
        public List<Point>? Points { get; set; }

        [DbField(IgnoreInsert = true, IgnoreUpdate = true)]
        public List<Route>? Routes { get; set; }
    }
}
