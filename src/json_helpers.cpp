#include "json_helpers.h"
#include <string.h>
#include <stdlib.h>

const char* findJsonValueStart(const char* json, const char* key) {
  String pat = String("\"") + key + "\"";
  const char* p = strstr(json, pat.c_str());
  if (!p) return nullptr;
  p = strchr(p, ':'); if (!p) return nullptr;
  do { ++p; } while (*p==' '||*p=='\t'||*p=='\r'||*p=='\n');
  return p;
}
bool parseJsonFloat(const char* json, const char* key, float& outVal) {
  const char* p = findJsonValueStart(json, key); if (!p) return false;
  if (*p == '\"') ++p;
  char* endp=nullptr; double v = strtod(p,&endp);
  if (endp==p) return false; outVal=(float)v; return true;
}
bool parseJsonInt(const char* json, const char* key, long& outVal) {
  const char* p = findJsonValueStart(json, key); if (!p) return false;
  if (*p == '\"') ++p;
  char* endp=nullptr; long v = strtol(p,&endp,10);
  if (endp==p) return false; outVal=v; return true;
}
bool parseJsonString(const char* json, const char* key, char* out, size_t n) {
  if (!out||!n) return false; out[0]='\0';
  const char* p = findJsonValueStart(json, key); if (!p) return false;
  if (*p!='\"'){ size_t i=0; while(*p&&*p!=','&&*p!='}'&&i+1<n) out[i++]=*p++; out[i]='\0'; return i>0; }
  ++p; size_t i=0; while(*p&&*p!='\"'&&i+1<n) out[i++]=*p++; out[i]='\0'; return (*p=='\"');
}
