#ifndef __DLLEXPORT_drakeCollision_H__
#define __DLLEXPORT_drakeCollision_H__

#if defined(WIN32) || defined(WIN64)
  #if defined(drakeCollision_EXPORTS)
    #define DLLEXPORT_drakeCollision __declspec( dllexport )
  #else
    #define DLLEXPORT_drakeCollision __declspec( dllimport )
  #endif
#else
    #define DLLEXPORT_drakeCollision
#endif

#endif
