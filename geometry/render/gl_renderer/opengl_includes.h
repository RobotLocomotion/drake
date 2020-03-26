/** @file
 Provides a one-stop shop for pulling in the OpenGl symbols in with the
 required extensions. Note that this distinguishes 'generic' OpenGL headers
 from those that might use a specific implementation, whether it be for support
 on some particular OS or hardware. This handles the ordering and the required
 macros in one place so dependent headers don't have to worry about it.
 Currently this is only enabled on Ubuntu in Drake.
 */

#pragma once

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>
#undef GL_GLEXT_PROTOTYPES
