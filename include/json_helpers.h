#pragma once
#include <Arduino.h>

const char* findJsonValueStart(const char* json, const char* key);
bool parseJsonFloat(const char* json, const char* key, float& outVal);
bool parseJsonInt(const char* json, const char* key, long& outVal);
bool parseJsonString(const char* json, const char* key, char* out, size_t n);
