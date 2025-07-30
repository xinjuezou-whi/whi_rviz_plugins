/******************************************************************
visibility control for whi_rviz_plugins

Features:
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2025-07-30: Initial version
2025-xx-xx: xxx
******************************************************************/
#ifndef WHI_RVIZ_PLUGINS__VISIBILITY_CONTROL_HPP_
#define WHI_RVIZ_PLUGINS__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define WHI_RVIZ_PLUGINS_EXPORT __attribute__ ((dllexport))
    #define WHI_RVIZ_PLUGINS_IMPORT __attribute__ ((dllimport))
  #else
    #define WHI_RVIZ_PLUGINS_EXPORT __declspec(dllexport)
    #define WHI_RVIZ_PLUGINS_IMPORT __declspec(dllimport)
  #endif
  #ifdef WHI_RVIZ_PLUGINS_BUILDING_LIBRARY
    #define WHI_RVIZ_PLUGINS_PUBLIC WHI_RVIZ_PLUGINS_EXPORT
  #else
    #define WHI_RVIZ_PLUGINS_PUBLIC WHI_RVIZ_PLUGINS_IMPORT
  #endif
  #define WHI_RVIZ_PLUGINS_PUBLIC_TYPE WHI_RVIZ_PLUGINS_PUBLIC
  #define WHI_RVIZ_PLUGINS_LOCAL
#else
  #define WHI_RVIZ_PLUGINS_EXPORT __attribute__ ((visibility("default")))
  #define WHI_RVIZ_PLUGINS_IMPORT
  #if __GNUC__ >= 4
    #define WHI_RVIZ_PLUGINS_PUBLIC __attribute__ ((visibility("default")))
    #define WHI_RVIZ_PLUGINS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define WHI_RVIZ_PLUGINS_PUBLIC
    #define WHI_RVIZ_PLUGINS_LOCAL
  #endif
  #define WHI_RVIZ_PLUGINS_PUBLIC_TYPE
#endif

#endif  // WHI_RVIZ_PLUGINS__VISIBILITY_CONTROL_HPP_
