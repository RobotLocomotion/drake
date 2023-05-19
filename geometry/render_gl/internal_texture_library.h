#pragma once

#include <map>
#include <mutex>
#include <optional>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/render_gl/internal_opengl_includes.h"

namespace drake {
namespace geometry {
namespace render_gl {
namespace internal {

/* Stores a set of OpenGl textures objects, keyed by their in-filesystem name.
 Its purpose is to guarantee that an image in the file system is only stored
 once in an OpenGl context. As such, a %TextureLibrary instance is associated
 with an instance of OpenGlContext by RenderEngineGl.

 By design, a single texture library is shared by a "family" of RenderEngineGl
 instances -- the original and all instances *cloned* from it. The library
 is threadsafe for adding new textures to the OpenGl context such that if one
 instance loads a texture, the others will benefit from it instead of blindly
 duplicating the texture in memory. This works because the OpenGlContext is
 also shared across a family of RenderEngineGl instances. */
class TextureLibrary {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TextureLibrary);

  TextureLibrary() = default;

  /* Returns the unique OpenGl identifier for the texture with the given name.
   A new OpenGl identifier will be created if one doesn't already exist.
   Errors in loading the texture are processed *silently*. This will attempt
   to load the image if a texture hasn't already been added with the given
   `file_name`.

   @param file_name  The absolute path to the file containing the texture data.
   @returns A valid OpenGl identifier if the texture has been successfully
            added to the library, std::nullopt if not.
   @pre The OpenGl context associated with this library has been bound. */
  std::optional<GLuint> GetTextureId(const std::string& file_name);

 private:
  /* The mapping from requested texture file name to its OpenGl id (as existing
   in context_).  */
  std::map<std::string, GLuint> textures_;

  // TODO(SeanCurtis-TRI): The asserted thread safety has not yet been tested.
  //  When we create the multithread_safety_test.cc for render_gl, add a test
  //  that confirms thread safety.
  /* Mutex to control access to textures_. */
  std::mutex mutex_;
};

}  // namespace internal
}  // namespace render_gl
}  // namespace geometry
}  // namespace drake
