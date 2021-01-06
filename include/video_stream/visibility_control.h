#ifndef VIDEO_STREAM__VISIBILITY_CONTROL_H_
#define VIDEO_STREAM__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define VIDEO_STREAM_EXPORT __attribute__ ((dllexport))
    #define VIDEO_STREAM_IMPORT __attribute__ ((dllimport))
  #else
    #define VIDEO_STREAM_EXPORT __declspec(dllexport)
    #define VIDEO_STREAM_IMPORT __declspec(dllimport)
  #endif
  #ifdef VIDEO_STREAM_BUILDING_LIBRARY
    #define VIDEO_STREAM_PUBLIC VIDEO_STREAM_EXPORT
  #else
    #define VIDEO_STREAM_PUBLIC VIDEO_STREAM_IMPORT
  #endif
  #define VIDEO_STREAM_PUBLIC_TYPE VIDEO_STREAM_PUBLIC
  #define VIDEO_STREAM_LOCAL
#else
  #define VIDEO_STREAM_EXPORT __attribute__ ((visibility("default")))
  #define VIDEO_STREAM_IMPORT
  #if __GNUC__ >= 4
    #define VIDEO_STREAM_PUBLIC __attribute__ ((visibility("default")))
    #define VIDEO_STREAM_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define VIDEO_STREAM_PUBLIC
    #define VIDEO_STREAM_LOCAL
  #endif
  #define VIDEO_STREAM_PUBLIC_TYPE
#endif

#endif  // VIDEO_STREAM__VISIBILITY_CONTROL_H_
