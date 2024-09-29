
// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkAutoInit.h>        // vtkCommonCore
#include <vtkOpenGLRenderer.h>  // vtkRenderingOpenGL2

// There are some VTK operations that depend on VTK's OpenGL module being
// initialized. In addition to the obvious RenderEngineVtk, there are less
// obvious uses, e.g., parsing glTF files (VTK parses glTF files directly into
// OpenGL constructs).
//
// If a translation unit uses VTK *and* requires OpenGL to be initialized, it
// must declare deps on @drake//tools/workspace/vtk_internal:vtk_opengl_init.

VTK_MODULE_INIT(vtkRenderingOpenGL2)
