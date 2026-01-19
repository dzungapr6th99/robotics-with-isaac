using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DbObject.Attributes
{
    [AttributeUsage(AttributeTargets.Class, AllowMultiple = false, Inherited = false)]
    public class DbTableAttribute : System.Attribute
    {
        public string Name { get; init; } = string.Empty;
        public string ViewName { get; init; } = string.Empty;

    }

    [AttributeUsage(AttributeTargets.Property, AllowMultiple = false, Inherited = false)]
    public class DbFieldAttribute : System.Attribute
    {
        public string Name { get; init; } = string.Empty;
        public bool IsKey { get; init; } = false;
        public bool IgnoreInsert { get; init; } = false;
        public bool IgnoreUpdate { get; init; } = false;

        public bool IsDetailTable { get; init; } = false;
        public bool Ignore { get; init; } = false;
        public bool IsClob { get; init; } = false;

    }
}
