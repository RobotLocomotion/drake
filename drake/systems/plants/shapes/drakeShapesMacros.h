#undef DLLEXPORT_drakeShapes
#if defined(WIN32) || defined(WIN64)
  #if defined(drakeShapes_EXPORTS)
    #define DLLEXPORT_drakeShapes __declspec( dllexport )
  #else
    #define DLLEXPORT_drakeShapes __declspec( dllimport )
  #endif
#else
    #define DLLEXPORT_drakeShapes
#endif
