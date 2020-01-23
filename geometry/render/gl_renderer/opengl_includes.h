/// @file
///
/// Provides a one-stop shop for pulling in the OpenGl symbols in with the
/// required extensions. Handles the ordering and the required macros in one
/// place so dependent headers don't have to worry about it.

#pragma once

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>
#undef GL_GLEXT_PROTOTYPES
