#define STR_VALUE(arg) #arg
#define DEFINE_TO_STRING(name) STR_VALUE(name)

#ifndef ___HARDWARE_MAJOR_VERSION___
  #define ___HARDWARE_MAJOR_VERSION___ 2
#endif
#ifndef ___HARDWARE_MINOR_VERSION___
  #define ___HARDWARE_MINOR_VERSION___ 1
#endif
#ifndef ___HARDWARE_VERSION___
  #define ___HARDWARE_VERSION___ \
   DEFINE_TO_STRING(___HARDWARE_MAJOR_VERSION___.___HARDWARE_MINOR_VERSION___)
#endif
#ifndef ___SOFTWARE_VERSION___
  #define ___SOFTWARE_VERSION___ "0.1.0"
#endif

