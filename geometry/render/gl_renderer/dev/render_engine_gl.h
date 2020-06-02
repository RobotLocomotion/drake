#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/render/camera_properties.h"
#include "drake/geometry/render/gl_renderer/dev/buffer_dim.h"
#include "drake/geometry/render/gl_renderer/dev/opengl_geometry.h"
#include "drake/geometry/render/gl_renderer/dev/shader_program.h"
#include "drake/geometry/render/gl_renderer/opengl_context.h"
#include "drake/geometry/render/gl_renderer/shape_meshes.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {

/** See documentation of MakeRenderEngineGl().  */
class RenderEngineGl final : public RenderEngine {
 public:
  /** \name Does not allow public copy, move, or assignment  */
  //@{
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

  // Obtain the depth image of rendered from a specific object pose. This is
  // slow because it reads the buffer back from the GPU.
  void GetDepthImage(systems::sensors::ImageDepth32F* depth_image_out,
                     const RenderTarget& target) const;

  // Provide triangle mesh definitions of the various geometries supported by
  // this renderer: sphere, cylinder, half space, box, and mesh.
  OpenGlGeometry GetSphere();
  OpenGlGeometry GetCylinder();
  OpenGlGeometry GetHalfSpace();
  OpenGlGeometry GetBox();
  OpenGlGeometry GetMesh(const std::string& filename);

  // Infrastructure for setting up the frame buffer object.
  RenderTarget SetupFBO(const DepthCameraProperties& camera);

  // Configure the model view and projection matrices.
  void SetGlProjectionMatrix(const DepthCameraProperties& camera);
  void SetGlModelViewMatrix(const Eigen::Matrix4f& X_CM) const;

  // Configure the OpenGL properties dependent on the camera properties.
  RenderTarget SetCameraProperties(const DepthCameraProperties& camera);

  // Configure the vertex array object for a triangle mesh.
  OpenGlGeometry SetupVAO(const VertexBuffer& vertices,
                          const IndexBuffer& indices);

  // The cached value transformation between camera and world frame.
  mutable math::RigidTransformd X_CW_;

  // All clones of this context share the same underlying opengl_context_. They
  // share geometry and frame buffer objects. The following structs are either
  // shared, or copy safe w.r.t. the shared context.
  std::shared_ptr<OpenGlContext> opengl_context_;

  // All of the objects below here *depend* on the OpenGL context. Right now,
  // I'm having each instance of the renderer share these OpenGl objects and
  // the context. If I'm going to do that, I may be better off making them
  // part of the Context rather than part of the renderer.
  std::shared_ptr<ShaderProgram> shader_program_;

  // One OpenGLGeometry per primitive type -- allow for instancing.
  OpenGlGeometry sphere_;
  OpenGlGeometry cylinder_;
  OpenGlGeometry half_space_;
  OpenGlGeometry box_;

  // Mapping from obj filename to the mesh loaded into an OpenGlGeometry.
  std::shared_ptr<std::unordered_map<std::string, OpenGlGeometry>> meshes_;

  // Mapping from width and height to a RenderTarget. Allows for the re-use of
  // frame buffers if they are of the same dimension.
  std::shared_ptr<std::unordered_map<BufferDim, RenderTarget>> frame_buffers_;

  // Mapping from RenderIndex to the visual data associated with that geometry.
  // This is copied so independent renderers can have different *instances* but
  // the instances still refer to the same, shared, underlying geometry.
  std::unordered_map<GeometryId, OpenGlInstance> visuals_;
};

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
