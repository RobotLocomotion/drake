// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkAutoInit.h>  // vtkCommonCore

// This file is only for VTK debugging programs (e.g., vtkProbeOpenGLVersion).
// Drake code must call drake::internal::VtkOpenGlInit() instead, because this
// creates a global constructor.
VTK_MODULE_INIT(vtkRenderingOpenGL2)
