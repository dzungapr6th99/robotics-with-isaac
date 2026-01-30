using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.AspNetCore.Mvc;
using System.Xml.Serialization;
using Microsoft.AspNetCore.Http;

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
                return default(T);

            var serializer = new XmlSerializer(typeof(T));

            using (var stream = new FileStream(filePath, FileMode.Open, FileAccess.Read))
            {
                return (T)serializer.Deserialize(stream)!;
            }
        }

        public static async Task<string> ReadJsonStringAsync(IFormFile file)
        {
            if (file == null || file.Length == 0)
                throw new ArgumentException("Empty file");

            await using var stream = file.OpenReadStream();
            using var reader = new StreamReader(stream);

            return await reader.ReadToEndAsync();
        }
    }
}
