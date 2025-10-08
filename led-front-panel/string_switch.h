// string_switch.h
#pragma once
#include <Arduino.h>
#include <string.h>

// Convert to const char* for both C-strings and Arduino String
static inline const char* to_cstr(const char* s) {
  return s ? s : "";
}
static inline const char* to_cstr(const String& s) {
  return s.c_str();
}

// ASCII case-insensitive equality (portable)
static inline bool eq_ci_ascii(const char* a, const char* b) {
  unsigned char ca, cb;
  while (*a && *b) {
    ca = static_cast<unsigned char>(*a++);
    cb = static_cast<unsigned char>(*b++);
    if (ca >= 'A' && ca <= 'Z') ca += 32;
    if (cb >= 'A' && cb <= 'Z') cb += 32;
    if (ca != cb) return false;
  }
  return *a == *b;
}

// Case-sensitive equality (optional)
static inline bool eq_cs(const char* a, const char* b) {
  return strcmp(a, b) == 0;
}

// Macros: emulate switch/case/default through if/else if/else.
// Usage:
//   STR_SWITCH(x)
//     STR_CASE("A") { ... }
//     STR_CASE("B") { ... }
//     STR_DEFAULT { ... }
//   STR_SWITCH_END;
#define STR_SWITCH(s) \
  do { \
    const char* __sw_s = to_cstr(s); \
    bool __sw_matched = false; \
    if (false) {} /* anchor */

#define STR_CASE(lit) \
  else if (!__sw_matched && eq_ci_ascii(__sw_s, (lit)) && (__sw_matched = true))

#define STR_CASE_CS(lit) \
  else if (!__sw_matched && eq_cs(__sw_s, (lit)) && (__sw_matched = true))

#define STR_DEFAULT else

#define STR_SWITCH_END \
  } \
  while (0)
