using DbObject.Attributes;
namespace DbObject
{
    [DbTable(Name = "POINT")]
    public class Point
    {
        [DbField(IsKey = true)]
        public int? Id { get; set; }
        [DbField(IsKey = true)]
        public int? MapId { get; set; }

        public string? Name { get; set; }
        public double? X { get; set; }
        public double? Y { get; set; }
        public int PointTypeId { get; set; }
    }
}
