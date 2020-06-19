#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/render/camera_properties.h"
#include "drake/geometry/render/gl_renderer/buffer_dim.h"
#include "drake/geometry/render/gl_renderer/opengl_context.h"
#include "drake/geometry/render/gl_renderer/opengl_geometry.h"
#include "drake/geometry/render/gl_renderer/shader_program.h"
#include "drake/geometry/render/gl_renderer/shape_meshes.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace render {

/** See documentation of MakeRenderEngineGl().  */
class RenderEngineGl final : public RenderEngine {
 public:
  /** \name Does not allow public copy, move, or assignment  */
  //@{
#ifdef DRAKE_DOXYGEN_CXX
  // Note: the copy constructor operator is actually protected to serve as the
  // basis for implementing the DoClone() method.
  RenderEngineGl(const RenderEngineGl&) = delete;
#endif
  RenderEngineGl& operator=(const RenderEngineGl&) = delete;
  RenderEngineGl(RenderEngineGl&&) = delete;
  RenderEngineGl& operator=(RenderEngineGl&&) = delete;
  //@}}

  RenderEngineGl();

  ~RenderEngineGl() final;

  /** @see RenderEngine::UpdateViewpoint().  */
  void UpdateViewpoint(const math::RigidTransformd& X_WR) final;

  /** @see RenderEngine::RenderColorImage().  */
  void RenderColorImage(
      const CameraProperties& camera, bool show_window,
      systems::sensors::ImageRgba8U* color_image_out) const final;

  /** @see RenderEngine::RenderDepthImage().  */
  void RenderDepthImage(
      const DepthCameraProperties& camera,
      systems::sensors::ImageDepth32F* depth_image_out) const final;

  /** @see RenderEngine::RenderLabelImage().  */
  void RenderLabelImage(
      const CameraProperties& camera, bool show_window,
      systems::sensors::ImageLabel16I* label_image_out) const final;

  /** @name    Shape reification  */
  //@{
  using RenderEngine::ImplementGeometry;
  void ImplementGeometry(const Sphere& sphere, void* user_data) final;
  void ImplementGeometry(const Cylinder& cylinder, void* user_data) final;
  void ImplementGeometry(const HalfSpace& half_space, void* user_data) final;
  void ImplementGeometry(const Box& box, void* user_data) final;
  void ImplementGeometry(const Mesh& mesh, void* user_data) final;
  void ImplementGeometry(const Convex& convex, void* user_data) final;
  //@}

 private:
  // @see RenderEngine::DoRegisterVisual().
  bool DoRegisterVisual(GeometryId id, const Shape& shape,
                        const PerceptionProperties& properties,
                        const math::RigidTransformd& X_WG) final;

  // @see RenderEngine::DoUpdateVisualPose().
  void DoUpdateVisualPose(GeometryId id,
                          const math::RigidTransformd& X_WG) final;

  // @see RenderEngine::DoRemoveGeometry().
  bool DoRemoveGeometry(GeometryId id) final;

  // see RenderEngine::DoClone().
  std::unique_ptr<RenderEngine> DoClone() const final;

  // Copy constructor used for cloning.
  RenderEngineGl(const RenderEngineGl& other) = default;

  // Render inverse depth of the object at a specific pose in the camera frame.
  void RenderAt(const Eigen::Matrix4f& X_CM) const;

  // Provide triangle mesh definitions of the various canonical geometries
  // supported by this renderer: sphere, cylinder, half space, box, and mesh.
  // These update the stored OpenGlGeometry members of this class. They are
  // *not* threadsafe.
  internal::OpenGlGeometry GetSphere();
  internal::OpenGlGeometry GetCylinder();
  internal::OpenGlGeometry GetHalfSpace();
  internal::OpenGlGeometry GetBox();
  internal::OpenGlGeometry GetMesh(const std::string& filename);

  // Creates a *new* render target for the given camera. This creates OpenGL
  // objects (render buffer, frame_buffer, and texture). It should only be
  // called if there is not already a cached render target for the camera's
  // reported image size (w, h) in render_targets_.
  static internal::RenderTarget CreateRenderTarget(
      const DepthCameraProperties& camera);

  // Configure the model view and projection matrices.
  void SetGlProjectionMatrix(const DepthCameraProperties& camera) const;
  void SetGlModelViewMatrix(const Eigen::Matrix4f& X_CM) const;

  // Configure the OpenGL properties dependent on the camera properties. This
  // updates the cache of RenderTargets (creating one for the camera if one
  // does not already exist). As such, it is _not_ threadsafe.
  internal::RenderTarget SetCameraProperties(
      const DepthCameraProperties& camera) const;

  // Creates an OpenGlGeometry from the mesh defined by the given vertices and
  // triangle indices.
  static internal::OpenGlGeometry CreateGlGeometry(
      const internal::VertexBuffer& vertices,
      const internal::IndexBuffer& indices);

  // The cached value transformation between camera and world frame.
  math::RigidTransformd X_CW_;

  // All clones of this context share the same underlying OpenGlContext. They
  // share geometry and frame buffer objects. The following structs are either
  // shared, or copy safe w.r.t. the shared context.
  std::shared_ptr<internal::OpenGlContext> opengl_context_;

  // All of the objects below here *depend* on the OpenGL context. Right now,
  // I'm having each instance of the renderer share these OpenGl objects and
  // the context. If I'm going to do that, I may be better off making them
  // part of the Context rather than part of the renderer.
  std::shared_ptr<internal::ShaderProgram> shader_program_;

  // One OpenGlGeometry per primitive type. They represent a canonical, "unit"
  // version of the primitive type. Each instance scales and poses the
  // corresponding primitive to create arbitrarily sized geometries.
  internal::OpenGlGeometry sphere_;
  internal::OpenGlGeometry cylinder_;
  internal::OpenGlGeometry half_space_;
  internal::OpenGlGeometry box_;

  // Mapping from obj filename to the mesh loaded into an OpenGlGeometry.
  std::shared_ptr<std::unordered_map<std::string, internal::OpenGlGeometry>>
      meshes_;

  // This is a cache of reusable RenderTargets. There is a unique render target
  // for each unique render image size (BufferDim). It is mutable so that it
  // can be updated in what would otherwise be a const action of updating the
  // OpenGL state for the camera.
  //
  // Note: copies of this render engine share the same frame buffer objects.
  // This is *not* threadsafe!
  mutable std::shared_ptr<
      std::unordered_map<internal::BufferDim, internal::RenderTarget>>
      render_targets_;

  // Mapping from GeometryId to the visual data associated with that geometry.
  // When copying the render engine, this data is copied verbatim allowing the
  // copied render engine access to the same OpenGL objects in the OpenGL
  // context. However, each independent copy is allowed to independently
  // modify their copy of visuals_ (adding and removing geometries).
  std::unordered_map<GeometryId, internal::OpenGlInstance> visuals_;
};

}  // namespace render
}  // namespace geometry
}  // namespace drake
