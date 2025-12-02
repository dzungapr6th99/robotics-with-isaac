using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xml.Serialization;

namespace CommonLib
{
    public static class CommonFunc
    {
        public static void SaveToXmlFile<T>(T obj, string filePath)
        {
            var serializer = new XmlSerializer(typeof(T));

            // Create directory if needed
            var dir = Path.GetDirectoryName(filePath);
            if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir))
            {
                Directory.CreateDirectory(dir);
            }

            using (var stream = new FileStream(filePath, FileMode.Create, FileAccess.Write))
            {
                serializer.Serialize(stream, obj);
            }
        }

        public static T? LoadFromXmlFile<T>(string filePath)
        {
            if (!File.Exists(filePath))
                throw new FileNotFoundException("XML file not found", filePath);

            var serializer = new XmlSerializer(typeof(T));

            using (var stream = new FileStream(filePath, FileMode.Open, FileAccess.Read))
            {
                return (T)serializer.Deserialize(stream)!;
            }
        }
    }
}
