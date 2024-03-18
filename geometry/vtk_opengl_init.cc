
// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkAutoInit.h>        // vtkCommonCore
#include <vtkOpenGLRenderer.h>  // vtkRenderingOpenGL2

// There are some VTK operations that depend on VTK's OpenGL module being
// initialized. In addition to the obvious RenderEngineVtk, the less obvious
// MakeConvexHull() also depends (VTK parses glTF files directly into OpenGL
// constructs).
//
// If a translation unit uses VTK *and* requires OpenGL to be initialized, it
// should declare //geometry:vtk_opengl_init as a dependency.

VTK_MODULE_INIT(vtkRenderingOpenGL2)
