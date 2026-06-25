#pragma once
#include <Arduino.h>
#include <WiFi.h>

void http_send_text(WiFiClient &client, const char* body);
void http_send_404(WiFiClient &client);

// Request parsing
bool http_read_request(WiFiClient &client, String &method, String &path, String &query);

// Path helpers
void normalize_path(String &path);
bool route_match(String path, const char* routeLiteral);

// Query helpers
bool query_get_float_kv(const String& query, const char* key, float& out);
bool query_get_uint16_kv(const String& query, const char* key, uint16_t& out);  // NEW
