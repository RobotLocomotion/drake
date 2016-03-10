/*****************************************************************************/
/* Import.h                                                                  */
/*                                                                           */
/* DESCRIPTION                                                               */
/*   Macro declarations for importing functions from a comppak library.      */
/*   If you are *building* a Windows DLL, you must define USE_EXPORT.        */
/*   Nothing special need be defined to *use* a Windows DLL.                 */
/*****************************************************************************/

#ifndef IMPORT_H
#define IMPORT_H

#if defined(_WIN32)
#  include <windows.h>
#  if defined(_PTH_STATIC_LIB_)
#    define FUN_DECL(type) type
#  else
#    if defined(USE_EXPORT)
#      define FUN_DECL(type) __declspec(dllexport) type WINAPI
#    else
#      define FUN_DECL(type) __declspec(dllimport) type WINAPI
#    endif
#  endif
#  define CB_FPTR WINAPI *
#  define CB_FUNC(type) type WINAPI
#  define SPD_SLEEP Sleep(1000)
#else
#  include <unistd.h>
#  if __GNUC__ >= 4
#    define FUN_DECL(type) __attribute__((visibility("default"))) type
#  else
#    define FUN_DECL(type) type
#  endif /* __GNUC__ >= 4 */
#  define CB_FPTR *
#  define CB_FUNC(type) type
#  define SPD_SLEEP sleep(1)
#endif

#endif
