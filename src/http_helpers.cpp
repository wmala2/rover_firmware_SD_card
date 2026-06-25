#include "http_helpers.h"
#include <stdlib.h>

void http_send_text(WiFiClient &client, const char* body) {
  char hdr[128];
  size_t blen = strlen(body);
  int n = snprintf(hdr, sizeof(hdr),
                   "HTTP/1.1 200 OK\r\n"
                   "Content-Type: text/plain\r\n"
                   "Content-Length: %u\r\n"
                   "Connection: close\r\n\r\n",
                   (unsigned)blen);
  client.write((const uint8_t*)hdr, (size_t)n);
  client.write((const uint8_t*)body, blen);
}
void http_send_404(WiFiClient &client) {
  static const char* body = "not found\n";
  char hdr[128];
  int n = snprintf(hdr, sizeof(hdr),
                   "HTTP/1.1 404 Not Found\r\n"
                   "Content-Type: text/plain\r\n"
                   "Content-Length: %u\r\n"
                   "Connection: close\r\n\r\n",
                   (unsigned)strlen(body));
  client.write((const uint8_t*)hdr, (size_t)n);
  client.write((const uint8_t*)body, strlen(body));
}

bool http_read_request(WiFiClient &client, String &method, String &path, String &query) {
  const uint32_t T0 = millis();
  String req;
  while (client.connected() && (millis() - T0) < 1500) {
    while (client.available()) {
      char c = (char)client.read();
      req += c;
      if (req.length() > 1024) break;
      if (req.endsWith("\r\n\r\n")) goto headers_done;
    }
    if (req.length() > 1024) break;
    delay(1);
  }
headers_done:
  int lineEnd = req.indexOf("\r\n");
  if (lineEnd < 0) return false;
  String line = req.substring(0, lineEnd);

  int sp1 = line.indexOf(' ');
  int sp2 = line.indexOf(' ', sp1 + 1);
  if (sp1 < 0 || sp2 < 0) return false;

  method = line.substring(0, sp1);
  String full = line.substring(sp1 + 1, sp2);

  if (full.startsWith("http://") || full.startsWith("https://")) {
    int pos = full.indexOf('/', full.indexOf("://") + 3);
    full = (pos >= 0) ? full.substring(pos) : String("/");
  }

  int qpos = full.indexOf('?');
  if (qpos >= 0) { path = full.substring(0, qpos); query = full.substring(qpos + 1); }
  else { path = full; query = ""; }
  return true;
}

void normalize_path(String &path) {
  path.trim();
  while (path.length() > 1 && path.endsWith("/")) path.remove(path.length() - 1);
  if (path.length() == 0 || path.charAt(0) != '/') path = "/" + path;
}
bool route_match(String path, const char* routeLiteral) {
  String route = String(routeLiteral);
  auto strip = [](String &s){
    s.trim();
    while (s.length() > 1 && s.endsWith("/")) s.remove(s.length() - 1);
    if (s.length() == 0 || s.charAt(0) != '/') s = "/" + s;
  };
  strip(path);
  strip(route);
  return path.equals(route);
}

bool query_get_float_kv(const String& query, const char* key, float& out) {
  String k = String(key) + "=";
  int i = query.indexOf(k);
  if (i < 0) return false;
  i += k.length();
  int j = query.indexOf('&', i);
  String val = (j < 0) ? query.substring(i) : query.substring(i, j);
  val.trim();
  out = val.toFloat();
  return true;
}

bool query_get_uint16_kv(const String& query, const char* key, uint16_t& out) {
  String k = String(key) + "=";
  int i = query.indexOf(k);
  if (i < 0) return false;
  i += k.length();
  int j = query.indexOf('&', i);
  String val = (j < 0) ? query.substring(i) : query.substring(i, j);
  val.trim();
  const char* p = val.c_str();
  char* endp = nullptr;
  long v = strtol(p, &endp, 10);
  if (endp == p) return false;          // no digits parsed
  if (v < 0 || v > 65535) return false; // out of range
  out = static_cast<uint16_t>(v);
  return true;
}
