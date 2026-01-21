using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;
using ApiObject.Cuopt;

namespace CuOptClientService
{
    public class CuOptClient
    {
        private readonly HttpClient _http;
        private readonly Uri _baseUri;
        private readonly JsonSerializerOptions _json;
        public CuOptClient(string baseUrl, HttpMessageHandler? handler = null)
        {
            _baseUri = new Uri(baseUrl.TrimEnd('/') + "/");
            _http = handler is null ? new HttpClient() : new HttpClient(handler);
            _http.BaseAddress = _baseUri;
            _http.Timeout = TimeSpan.FromSeconds(30);

            _json = new JsonSerializerOptions
            {
                PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
                DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull
            };
        }

        



        private async Task<string> SubmitCuOptRequest(CuoptVRPRequest request, CancellationToken ct = default)
        {
            var url = new Uri("cuopt/request", UriKind.Relative);
            using var msg = new HttpRequestMessage(HttpMethod.Post, url);
            msg.Headers.Accept.ParseAdd("application/json");

            var json = JsonSerializer.Serialize(request, _json);
            msg.Content = new StringContent(json, Encoding.UTF8, "application/json");

            using var resp = await _http.SendAsync(msg, HttpCompletionOption.ResponseHeadersRead, ct).ConfigureAwait(false);
            var body = await resp.Content.ReadAsStringAsync(ct).ConfigureAwait(false);

            if (!resp.IsSuccessStatusCode)
                throw new HttpRequestException($"cuOpt submit failed HTTP={(int)resp.StatusCode} {resp.StatusCode}. Body={Trunc(body)}");

            CuOptVRPSubmitResponse? parsed;
            try
            {
                parsed = JsonSerializer.Deserialize<CuOptVRPSubmitResponse>(body, _json);
            }
            catch (Exception ex)
            {
                throw new InvalidOperationException($"Submit returned non-JSON. Body={Trunc(body)}", ex);
            }

            if (parsed?.ReqId is null || parsed.ReqId.Length < 10)
                throw new InvalidOperationException($"Submit JSON missing reqId. Body={Trunc(body)}");

            return parsed.ReqId.Trim();
        }


        private static string TryDecode(byte[] raw)
        {
            // Best-effort decoding
            string text;

            // UTF-8 (strip BOM if present)
            text = Encoding.UTF8.GetString(raw);
            text = text.TrimStart('\uFEFF');

            // If that yields mostly garbage, try UTF-16
            if (LooksGarbled(text))
            {
                try { text = Encoding.Unicode.GetString(raw); } catch { /*ignore*/ }
            }

            // If still garbled, fall back Latin-1 (preserve bytes)
            if (LooksGarbled(text))
                text = Encoding.Latin1.GetString(raw);

            return text;
        }
        private static bool LooksGarbled(string s)
        {
            if (string.IsNullOrEmpty(s)) return true;
            int letters = 0, weird = 0;
            foreach (var ch in s)
            {
                if (char.IsLetter(ch)) letters++;
                else if (char.IsControl(ch) && ch != '\r' && ch != '\n' && ch != '\t') weird++;
            }
            // heuristic
            return letters == 0 && weird > 0;
        }

        private static string Trunc(string? s, int max = 900)
        {
            if (string.IsNullOrEmpty(s)) return "";
            return s.Length <= max ? s : s.Substring(0, max) + "...";
        }

    }
}
