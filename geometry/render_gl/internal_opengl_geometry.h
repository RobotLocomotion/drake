#pragma once

#include <array>
#include <limits>
#include <utility>

#include "drake/geometry/render/render_label.h"
#include "drake/geometry/render_gl/internal_opengl_includes.h"
#include "drake/geometry/render_gl/internal_shader_program_data.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace render_gl {
namespace internal {

// TODO(SeanCurtis-TRI): Consider moving this up to RenderEngine; it's useful
//  for multiple RenderEngine types.
/* Rendering types available. Used to index into render-type-dependent data
 structures. Because it serves as an index, we use kTypeCount to declare the
 *number* of index values available (relying on C++'s default behavior of
 assigning sequential values in enumerations).  */
enum RenderType { kColor = 0, kLabel, kDepth, kTypeCount };

/* Describes the data associated with a particular vertex attribute (for
 defining an OpenGlGeometry instance). The data is ultimately stored in an
 associated vertex buffer object (VBO). This struct merely communicates how to
 identify and interpret the bytes in that buffer object. */
struct VertexAttrib {
  /* The index for this attribute as defined by the shaders. To match the
   shaders this value must be 0 for position, 1 for normals, and 2 for uvs. */
  int attribute_index{};
  /* Number of values per element; e.g., 3 values per position, etc. */
  int components_per_element{};
  /* Offset into the corresponding buffer at which this data starts. */
  int byte_offset{};
  /* The byte distance between subsequent elements. Zero is a valid value;
   OpenGL will assume compact representation and use a stride equal to
   `components_per_element * sizeof("value_type")`. */
  int stride{};
  /* The numeric type of the single values (e.g., GL_INT, GL_FLOAT, etc.). */
  int numeric_type{};
};

/* The collection of vertex attributes for a single mesh. As with VertexAttrib
 there should be a single vertex buffer object (VBO) associated with a
 VertexSpec instance. Note, if a particular VertexAttrib is undefined, its
 `components_per_element` will be zero. */
struct VertexSpec {
  VertexAttrib positions;
  VertexAttrib normals;
  VertexAttrib uvs;
};

/* @name Frames, Geometries, and Instances

 There are a number of frames relating to how geometries are handled in
 RenderEngineGl. What they are and how they relate can be confusing. This
 discussion defines the frames, their relationships, and looks at the provenance
 of the values in those relationships (with particular focus on what values are
 stored in Drake entities and how they are used).

   W: Drake's world frame.
   G: Drake's geometry frame (this is what gets posed by SceneGraph). In this
      file it is the instance's frame.
   M: An OpenGl "model" frame. This is a conceptual frame relating the parts of
      a prop. It is neither the instance frame nor the frame that OpenGlGeometry
      vertex positions or normals are measured and expressed in. For example,
      the underlying model data can be scaled (as with Mesh or Convex shapes) to
      create the effective geometry instance. Also, the instance can be built of
      multiple parts (the constituent pieces of a "prop") and those parts can
      themselves be transformed relative to each other to build the model.
   N: The frame in which the vertex positions and normals of an OpenGlGeometry
      are measured and expressed.

 Relationships between these frames:

   While Drake mostly focuses on rigid transforms between frames (X_AB) and
   relative orientations (R_AB), rendering needs to describe additional
   relationships. So, in addition to the X_ and R_ prefixes, we also use the
   P_, S_, T_, and N_ prefixes. All transforms noted here concatenate as 4x4
   homogeneous matrices. In implementation, they may be replaced with an
   alternative, functionally equivalent representation.

     P_: A homogenous matrix representing translation. The same as a
         RigidTransform consisting of only a translation.
     S_: A *scale* matrix. A diagonal matrix with Sx, Sy, and Sz on the
         diagonal.
     T_: An affine transform which can include translation, rotation, and
         anisotropic scale. T_AB = P_AB * R_AB * S_AB.
     N_: The transform to apply to a mesh's normals to re-express them in
         another frame. Given T_AB = P_AB * R_AB * S_AB, we can derive the
         corresponding normal transform: N_AB = R_AB * S⁻¹_AB.

   Immutable relationships defined when a Shape is registered with the engine:

      S_GM: The scaling applied to the underlying "model" data to create the
            requested Drake-specified geometry. This can be the explicitly
            declared scale factor associated with a Mesh or Convex. It can also
            be scale factors applied by Drake to map a canonical shape to a
            particular size. For example, we store the mesh of a unit sphere,
            but apply an arbitrary scale to model an ellipsoid; this scale
            matrix contains that transformation.
      S_MN: For some sources (e.g., glTF files), an OpenGlGeometry will have its
            vertex data expressed in a different frame from that of the file (or
            model). This is the scale between those two frames.
      T_MN: The pose (not necessarily RigidTransform) of the vertex position
            data associated with a single OpenGlGeometry instance (sometimes
            referred to as a "part" or "node") in the conceptual frame of the
            model.
      T_GN: The transform for measuring and expressing the vertex position data
            associated with an OpenGlGeometry in the instance's frame. Defined
            as T_GN = S_GM * T_MN
      N_GN: The transform for expressing the vertex normal data associated with
            an OpenGlGeometry in the instance's frame.

   Relationships that depend on the pose of the instance (as defined by
   SceneGraph):

      X_WG: The pose (RigidTransform) of the Drake geometry frame in Drake's
            world frame. This is provided by SceneGraph and provided to
            RenderEngineGl via UpdateVisualPose() and updates frequently.
      T_WN: T_WG = X_WG * S_GN.
      N_WN: N_WN = R_WG * N_GN.
*/

/* For a fixed OpenGL context, defines the definition of a mesh geometry. The
 geometry is defined by the handles to various objects in the OpenGL context.
 If the context is changed or otherwise invalidated, these handles will no
 longer be valid.

 The code that constructs instances is completely responsible for guaranteeing
 that the array and buffer values are valid in the OpenGl context and that the
 index buffer size is likewise sized correctly.

 Each instance stores the the transforms necessary to express its vertex data
 in the source model's frame. For primitives and obj files, these transforms
 will be the identity. For meshes from other sources (e.g., glTF files), these
 transforms may be non-identity.  */
struct OpenGlGeometry {
  /* Reports true if `this` has been defined with "meaningful" values. In this
   case, "meaningful" is limited to "not default initialized". It can't know
   if the values are actually object identifiers in the current OpenGl context.
   */
  bool is_defined() const {
    return vertex_array != kInvalid && vertex_buffer != kInvalid &&
           index_buffer != kInvalid && v_count >= 0 && index_count >= 0;
  }

  /* Throws an exception with the given `message` if `this` hasn't been
   populated with meaningful values.
   @see if_defined().  */
  void throw_if_undefined(const char* message) const {
    if (!is_defined()) throw std::logic_error(message);
  }

  // TODO(SeanCurtis-TRI): This can't really be a struct; there are invariants
  // that need to be maintained: vertex_array depends on vertex_buffer,
  // and index_count needs to be the actual number of indices stored in
  // index_buffer.
  GLuint vertex_array{kInvalid};
  GLuint vertex_buffer{kInvalid};
  GLuint index_buffer{kInvalid};

  // The number of vertices encoded in `vertex_buffer`.
  int v_count{};

  // Parameters for glDrawElements(). See
  // https://registry.khronos.org/OpenGL-Refpages/gl4/html/glDrawElements.xhtml
  int index_count{-1};
  // The numerical representation of the indices (e.g., u char, u short, u int).
  GLenum type{};
  // How the indices should be interpreted (e.g., triangles, triangle strip,
  // points, etc.)
  GLenum mode{};

  /* The transform mapping vertex position to the model frame intrinsic to the
   geometry definition. */
  Eigen::Matrix4f T_MN{Eigen::Matrix4f::Identity()};
  /* The transform mapping vertex normals to the model frame intrinsic to the
   geometry definition. */
  Eigen::Matrix3f N_MN{Eigen::Matrix3f::Identity()};

  /* Stores the description of the geometry's vertex data so that the arrays
   can be recreated upon cloning -- clones share the underlying buffer objects
   but vertex arrays are *not* shared. */
  VertexSpec spec;

  /* The value of an object (array, buffer) that should be considered invalid.
   */
  static constexpr GLuint kInvalid = std::numeric_limits<GLuint>::max();
};

/* An instance of a geometry in the renderer - a reference to the underlying
 OpenGl geometry definition in Frame G, its pose in the world frame W, and scale
 factors. The scale factors are not required to be uniform. They _can_ be
 negative, but that is not recommended; in addition to mirroring the geometry it
 will also turn the geometry "inside out".

 When rendering, the visual geometry will be scaled around G's origin and
 subsequently posed relative to W.

 With each instance, we store the necessary immutable transforms and, when
 updating the pose of the geometry (X_WG), update the final transforms that
 depend on the instantaneous pose and immutable transforms (see below). */
struct OpenGlInstance {
  /* Constructs an instance from a geometry definition, transforms for the
   geometry's vertex position and normals, and the instance's shader data for
   depth and label shaders.

   @param g_in        The index of the geometry `this` instantiates.
   @param scale       The scale to apply to the underlying model geometry to
                      create the drake geometry: S_GM.
   @param geo         The geometry this is an instance of.
   @param color_data  The shader data this instance uses for color images.
   @param depth_data  The shader data this instance uses for depth images.
   @param label_data  The shader data this instance uses for label images.

   @pre g_in indexes into the geometry referenced by `geo`.
   @pre The shader program data has valid shader ids.  */
  OpenGlInstance(int g_in, const Eigen::Vector3f& scale,
                 const OpenGlGeometry& geo, ShaderProgramData color_data,
                 ShaderProgramData depth_data, ShaderProgramData label_data)
      : geometry(g_in) {
    const Eigen::DiagonalMatrix<float, 4> S_GM(
        Eigen::Vector4f(scale.x(), scale.y(), scale.z(), 1.0));
    T_GN = S_GM * geo.T_MN;
    const Eigen::DiagonalMatrix<float, 3> N_GM(
        Eigen::Vector3f(1.0 / scale.x(), 1.0 / scale.y(), 1.0 / scale.z()));
    N_GN = N_GM * geo.N_MN;
    DRAKE_DEMAND(color_data.shader_id().is_valid());
    DRAKE_DEMAND(depth_data.shader_id().is_valid());
    DRAKE_DEMAND(label_data.shader_id().is_valid());
    shader_data[RenderType::kColor] = std::move(color_data);
    shader_data[RenderType::kDepth] = std::move(depth_data);
    shader_data[RenderType::kLabel] = std::move(label_data);
    DRAKE_DEMAND(geometry >= 0);
  }

  /* This is the index to the OpenGlGeometry stored by RenderEngineGl. */
  int geometry{};

  /* The immutable transform mapping vertex position to the instance frame.
   Initialized in constructor. */
  Eigen::Matrix4f T_GN;
  /* The pose-dependent transform mapping vertex position to Drake World.
   RenderEngineGl::DoUpdateVisualPose() is responsible for updating this.  */
  Eigen::Matrix4f T_WN{Eigen::Matrix4f::Identity()};

  /* The immutable transform mapping vertex normals to the instance frame.
   Initialized in constructor. */
  Eigen::Matrix3f N_GN;
  /* The pose-dependent transform mapping vertex normals to Drake World.
   RenderEngineGl::DoUpdateVisualPose() is responsible for updating this.  */
  Eigen::Matrix3f N_WN{Eigen::Matrix3f::Identity()};

  std::array<ShaderProgramData, RenderType::kTypeCount> shader_data;
};

}  // namespace internal
}  // namespace render_gl
}  // namespace geometry
}  // namespace drake
