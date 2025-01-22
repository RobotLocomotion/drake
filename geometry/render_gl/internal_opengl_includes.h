/* @file
 Provides a one-stop shop for pulling in the OpenGl symbols with the required
 extensions. By design, this includes only the baseline OpenGl headers and
 handles their ordering. If code requires more elaborate libraries, it is
 responsible for pulling them in directly.
 */

#pragma once

#include <vtkglad/include/glad/gl.h>
