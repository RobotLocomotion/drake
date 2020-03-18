#pragma once

#include <limits>

#include "drake/geometry/render/gl_renderer/opengl_includes.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {

/** For a fixed OpenGL context, defines the definition of a mesh geometry. The
 geometry is defined by the handles to various objects in the OpenGL context.
 If the context is changed or otherwise invalidated, these handles will no
 longer be valid.  */
struct OpenGlGeometry {
  GLuint vertex_array{kInvalid};
  GLuint vertex_buffer{kInvalid};
  GLuint index_buffer{kInvalid};
  int index_buffer_size{0};

  /** Reports true if `this` has been defined with meaningful values.  */
  bool is_defined() const {
    return vertex_array != kInvalid && vertex_buffer != kInvalid &&
           index_buffer != kInvalid;
  }

  /** Throws an exception with the given `message` if `this` hasn't been
   populated with meaningful values.  */
  void throw_if_undefined(const char* message) {
    if (!is_defined()) throw std::logic_error(message);
  }

  /** The value of an object that should be considered invalid.  */
  static constexpr GLuint kInvalid = std::numeric_limits<GLuint>::max();
};

/** An instance of a geometry in the renderer - the geometry definition and the
 pose of that geometry in the world frame.  */
struct OpenGlInstance {
  OpenGlInstance(const OpenGlGeometry& g_in,
                 const math::RigidTransformd& pose_in,
                 const Vector3<double>& scale_in)
      : geometry(g_in), X_WG(pose_in), scale(scale_in) {}
  OpenGlGeometry geometry;
  math::RigidTransformd X_WG;
  // The scale factors providing the basis for non-unit geometry.
  Vector3<double> scale;
};

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
