using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using DeepCopy;
namespace DbObject.Extension
{
    public static class ObjectExtension
    {
        public static T Clone<T>(this T source) where T : class
        {
            if (Object.ReferenceEquals(source, null))
            {
                return default(T);
            }
            return DeepCopier.Copy(source);
        }    


        public static object? GetPropertyValue(this object obj, string propertyName) 
        {
            if (obj == null || string.IsNullOrEmpty(propertyName))
            {
                return null;
            }    
            PropertyInfo? propertyInfo = obj.GetType().GetProperty(propertyName, BindingFlags.IgnoreCase | BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
            if (propertyInfo == null || !propertyInfo.CanRead)
            {
                return null;
            }
            return propertyInfo.GetValue(obj);
        }
    }
}