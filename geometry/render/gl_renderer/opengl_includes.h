/** @file
 Provides a one-stop shop for pulling in the OpenGl symbols in with the
 required extensions. Note that this distinguishes 'generic' OpenGl headers
 from those that are OS-based. This handles the ordering and the required
 macros in one place so dependent headers don't have to worry about it.
 */

#pragma once

#define GL_GLEXT_PROTOTYPES
#include <GL/glew.h>
#undef GL_GLEXT_PROTOTYPES
