
// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkAutoInit.h>                 // vtkCommonCore
#include <vtkOpenGLRenderer.h>           // vtkRenderingOpenGL2

// This enables VTK's OpenGL2 infrastructure.
VTK_MODULE_INIT(vtkRenderingOpenGL2)
