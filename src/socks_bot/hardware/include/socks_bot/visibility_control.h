#ifndef SOCKS_BOT__VISIBILITY_CONTROL_H_
#define SOCKS_BOT__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SOCKS_BOT_EXPORT __attribute__ ((dllexport))
    #define SOCKS_BOT_IMPORT __attribute__ ((dllimport))
  #else
    #define SOCKS_BOT_EXPORT __declspec(dllexport)
    #define SOCKS_BOT_IMPORT __declspec(dllimport)
  #endif
  #ifdef SOCKS_BOT_BUILDING_LIBRARY
    #define SOCKS_BOT_PUBLIC SOCKS_BOT_EXPORT
  #else
    #define SOCKS_BOT_PUBLIC SOCKS_BOT_IMPORT
  #endif
  #define SOCKS_BOT_PUBLIC_TYPE SOCKS_BOT_PUBLIC
  #define SOCKS_BOT_LOCAL
#else
  #define SOCKS_BOT_EXPORT __attribute__ ((visibility("default")))
  #define SOCKS_BOT_IMPORT
  #if __GNUC__ >= 4
    #define SOCKS_BOT_PUBLIC __attribute__ ((visibility("default")))
    #define SOCKS_BOT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SOCKS_BOT_PUBLIC
    #define SOCKS_BOT_LOCAL
  #endif
  #define SOCKS_BOT_PUBLIC_TYPE
#endif

#endif  // SOCKS_BOT__VISIBILITY_CONTROL_H_
