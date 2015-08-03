#undef DLLEXPORT_drakeCollision
#if defined(WIN32) || defined(WIN64)
  #if defined(drakeCollision_EXPORTS)
    #define DLLEXPORT_drakeCollision __declspec( dllexport )
  #else
    #define DLLEXPORT_drakeCollision __declspec( dllimport )
  #endif
#else
    #define DLLEXPORT_drakeCollision
#endif
