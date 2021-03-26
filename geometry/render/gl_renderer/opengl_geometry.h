#pragma once

#include <array>
#include <limits>
#include <utility>

#include "drake/geometry/render/gl_renderer/gl_common.h"
#include "drake/geometry/render/gl_renderer/opengl_includes.h"
#include "drake/geometry/render/gl_renderer/shader_program_data.h"
#include "drake/geometry/render/render_label.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {

/* For a fixed OpenGL context, defines the definition of a mesh geometry. The
 geometry is defined by the handles to various objects in the OpenGL context.
 If the context is changed or otherwise invalidated, these handles will no
 longer be valid.

 The code that constructs instances is completely responsible for guaranteeing
 that the array and buffer values are valid in the OpenGl context and that the
 index buffer size is likewise sized correctly.  */
struct OpenGlGeometry {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(OpenGlGeometry)

  /* Default constructor; the resultant instance is considered "undefined".  */
  OpenGlGeometry() = default;

  /* Constructs an %OpenGlGeometry from the given "object names" OpenGl objects.
   (See e.g., https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/glGenFramebuffers.xhtml
   for an example of where such an "object name" would come from.)

   @param vertex_array_in       The handle to the OpenGl vertex array object
                                containing the mesh's data.
   @param vertex_buffer_in      The handle to the OpenGl vertex buffer
                                containing mesh per-vertex data.
   @param index_buffer_in       The handle to the OpenGl index buffer defining a
                                set of triangles.
   @param index_buffer_size_in  The number of indices in the index buffer.
   @pre `index_buffer_size_in >= 0`.  */
  OpenGlGeometry(GLuint vertex_array_in, GLuint vertex_buffer_in,
                 GLuint index_buffer_in, int index_buffer_size_in,
                 bool has_tex_coord_in)
      : vertex_array{vertex_array_in},
        vertex_buffer{vertex_buffer_in},
        index_buffer{index_buffer_in},
        index_buffer_size{index_buffer_size_in},
        has_tex_coord{has_tex_coord_in} {
    if (index_buffer_size < 0) {
      throw std::logic_error("Index buffer size must be non-negative");
    }
  }

  /* Reports true if `this` has been defined with "meaningful" values. In this
   case, "meaningful" is limited to "not default initialized". It can't know
   if the values are actually object identifiers in the current OpenGl context.
   */
  bool is_defined() const {
    return vertex_array != kInvalid && vertex_buffer != kInvalid &&
           index_buffer != kInvalid;
  }

  /* Throws an exception with the given `message` if `this` hasn't been
   populated with meaningful values.
   @see if_defined().  */
  void throw_if_undefined(const char* message) const {
    if (!is_defined()) throw std::logic_error(message);
  }

  GLuint vertex_array{kInvalid};
  GLuint vertex_buffer{kInvalid};
  GLuint index_buffer{kInvalid};
  int index_buffer_size{0};
  /* True indicates that this has texture coordinates to support texture
   maps. See MeshData::has_tex_coord for detail.  */
  bool has_tex_coord{};

  /* The value of an object (array, buffer) that should be considered invalid.
   */
  static constexpr GLuint kInvalid = std::numeric_limits<GLuint>::max();
};

/* An instance of a geometry in the renderer - the underlying OpenGl geometry
 definition in Frame G, its pose in the world frame W, and scale factors. The
 scale factors are not required to be uniform. They _can_ be negative, but that
 is not recommended; in addition to mirroring the geometry it will also turn
 the geometry "inside out".

 When rendering, the visual geometry will be scaled around G's origin and
 subsequently posed relative to W.  */
struct OpenGlInstance {
  /* Constructs an instance from a geometry definition, a pose, a scale factor
   and the instance's shader data for depth and label shaders.
   @pre `g_in.is_defined()` reports `true`.
   @pre The shader program data has valid shader ids.  */
  OpenGlInstance(const OpenGlGeometry& g_in,
                 const math::RigidTransformd& pose_in,
                 const Vector3<double>& scale_in, ShaderProgramData color_data,
                 ShaderProgramData depth_data, ShaderProgramData label_data)
      : geometry(g_in), X_WG(pose_in), scale(scale_in) {
    DRAKE_DEMAND(color_data.shader_id().is_valid());
    DRAKE_DEMAND(depth_data.shader_id().is_valid());
    DRAKE_DEMAND(label_data.shader_id().is_valid());
    shader_data[RenderType::kColor] = std::move(color_data);
    shader_data[RenderType::kDepth] = std::move(depth_data);
    shader_data[RenderType::kLabel] = std::move(label_data);
    DRAKE_DEMAND(geometry.is_defined());
  }

  // TODO(SeanCurtis-TRI) Chanage these quantities to be float-valued so they
  //  can go directly into the shader without casting.
  OpenGlGeometry geometry;
  math::RigidTransformd X_WG;
  Vector3<double> scale;
  std::array<ShaderProgramData, RenderType::kTypeCount> shader_data;
};

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
