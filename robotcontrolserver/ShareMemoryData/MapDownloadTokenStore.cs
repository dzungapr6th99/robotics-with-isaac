using System.Collections.Concurrent;

namespace ShareMemoryData
{
    public static class MapDownloadTokenStore
    {
        private static readonly ConcurrentDictionary<Guid, MapDownloadToken> _tokens = new();
        private static readonly TimeSpan _defaultExpiry = TimeSpan.FromMinutes(10);

        public static MapDownloadToken Create(int mapId, string mapUrl, int? robotId = null, TimeSpan? expiry = null)
        {
            var token = Guid.NewGuid();
            var item = new MapDownloadToken
            {
                Token = token,
                MapId = mapId,
                MapUrl = mapUrl,
                RobotId = robotId,
                ExpireAt = DateTime.UtcNow.Add(expiry ?? _defaultExpiry)
            };
            _tokens[token] = item;
            return item;
        }

        public static bool TryConsume(Guid token, out MapDownloadToken mapToken)
        {
            mapToken = null;
            if (_tokens.TryGetValue(token, out var saved))
            {
                if (saved.ExpireAt > DateTime.UtcNow)
                {
                    mapToken = saved;
                    _tokens.TryRemove(token, out _);
                    return true;
                }
                _tokens.TryRemove(token, out _);
            }
            return false;
        }
    }

    public class MapDownloadToken
    {
        public Guid Token { get; set; }
        public int MapId { get; set; }
        public string MapUrl { get; set; } = string.Empty;
        public int? RobotId { get; set; }
        public DateTime ExpireAt { get; set; }
    }
}
