// teleop_ik/visibility_control.hpp
// ament_cmake 慣例の export マクロ boilerplate.

#ifndef TELEOP_IK__VISIBILITY_CONTROL_HPP_
#define TELEOP_IK__VISIBILITY_CONTROL_HPP_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TELEOP_IK_EXPORT __attribute__((dllexport))
    #define TELEOP_IK_IMPORT __attribute__((dllimport))
  #else
    #define TELEOP_IK_EXPORT __declspec(dllexport)
    #define TELEOP_IK_IMPORT __declspec(dllimport)
  #endif
  #ifdef TELEOP_IK_BUILDING_DLL
    #define TELEOP_IK_PUBLIC TELEOP_IK_EXPORT
  #else
    #define TELEOP_IK_PUBLIC TELEOP_IK_IMPORT
  #endif
  #define TELEOP_IK_PUBLIC_TYPE TELEOP_IK_PUBLIC
  #define TELEOP_IK_LOCAL
#else
  #define TELEOP_IK_EXPORT __attribute__((visibility("default")))
  #define TELEOP_IK_IMPORT
  #if __GNUC__ >= 4
    #define TELEOP_IK_PUBLIC __attribute__((visibility("default")))
    #define TELEOP_IK_LOCAL __attribute__((visibility("hidden")))
  #else
    #define TELEOP_IK_PUBLIC
    #define TELEOP_IK_LOCAL
  #endif
  #define TELEOP_IK_PUBLIC_TYPE
#endif

#endif  // TELEOP_IK__VISIBILITY_CONTROL_HPP_
