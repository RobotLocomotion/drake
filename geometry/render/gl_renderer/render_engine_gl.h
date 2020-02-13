#pragma once

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/render/camera_properties.h"
#include "drake/geometry/render/gl_renderer/buffer_dim.h"
#include "drake/geometry/render/gl_renderer/load_mesh.h"
#include "drake/geometry/render/gl_renderer/opengl_context.h"
#include "drake/geometry/render/gl_renderer/opengl_geometry.h"
#include "drake/geometry/render/gl_renderer/shader_program.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace render {
namespace gl {

/** See documentation of MakeRenderEngineGl().  */
class RenderEngineGl final : public drake::geometry::render::RenderEngine {
 public:
  /** \name Does not allow public copy, move, or assignment  */
  //@{
  RenderEngineGl& operator=(const RenderEngineGl&) = delete;
  RenderEngineGl(RenderEngineGl&&) = delete;
  RenderEngineGl& operator=(RenderEngineGl&&) = delete;
  //@}}

  RenderEngineGl();

  ~RenderEngineGl() override;

  /** Inherits RenderEngine::UpdateViewpoint().  */
  void UpdateViewpoint(const drake::math::RigidTransformd& X_WR) override;

  /** Inherits RenderEngine::RenderColorImage().  */
  void RenderColorImage(
      const drake::geometry::render::CameraProperties& camera, bool show_window,
      drake::systems::sensors::ImageRgba8U* color_image_out) const override;

  /** Inherits RenderEngine::RenderDepthImage().  */
  void RenderDepthImage(
      const drake::geometry::render::DepthCameraProperties& camera,
      drake::systems::sensors::ImageDepth32F* depth_image_out) const override;

  /** Inherits RenderEngine::RenderLabelImage().  */
  void RenderLabelImage(
      const drake::geometry::render::CameraProperties& camera, bool show_window,
      drake::systems::sensors::ImageLabel16I* label_image_out) const override;

  /** @name    Shape reification  */
  //@{
  using drake::geometry::render::RenderEngine::ImplementGeometry;
  void ImplementGeometry(const drake::geometry::Sphere& sphere,
                         void* user_data) override;
  void ImplementGeometry(const drake::geometry::Cylinder& cylinder,
                         void* user_data) override;
  void ImplementGeometry(const drake::geometry::HalfSpace& half_space,
                         void* user_data) override;
  void ImplementGeometry(const drake::geometry::Box& box,
                         void* user_data) override;
  void ImplementGeometry(const drake::geometry::Mesh& mesh,
                         void* user_data) override;
  void ImplementGeometry(const drake::geometry::Convex& convex,
                         void* user_data) override;
  //@}

 private:
  // @see RenderEngine::DoRegisterVisual().
  bool DoRegisterVisual(drake::geometry::GeometryId id,
                        const drake::geometry::Shape& shape,
                        const drake::geometry::PerceptionProperties& properties,
                        const drake::math::RigidTransformd& X_WG) override;

  // @see RenderEngine::DoUpdateVisualPose().
  void DoUpdateVisualPose(drake::geometry::GeometryId id,
                          const drake::math::RigidTransformd& X_WG) override;

  // @see RenderEngine::DoRemoveGeometry().
  bool DoRemoveGeometry(drake::geometry::GeometryId id) override;

  // see RenderEngine::DoClone().
  std::unique_ptr<drake::geometry::render::RenderEngine> DoClone()
      const override;

  // Copy constructor used for cloning.
  RenderEngineGl(const RenderEngineGl& other) = default;

  // Render inverse depth of the object at a specific pose in the camera frame.
  RenderTarget RenderAt(
      const Eigen::Matrix4f& X_CM,
      const drake::geometry::render::DepthCameraProperties& camera) const;

  // Obtain the depth image of rendered from a specific object pose. This is
  // slow because it reads the buffer back from the GPU.
  void GetDepthImage(drake::systems::sensors::ImageDepth32F* depth_image_out,
                     const RenderTarget& target) const;

  // Provide triangle mesh definitions of the various geometries supported by
  // this renderer: sphere, cylinder, half space, box, and mesh.
  OpenGlGeometry GetSphere();
  OpenGlGeometry GetCylinder();
  OpenGlGeometry GetHalfSpace();
  OpenGlGeometry GetBox();
  OpenGlGeometry GetMesh(const std::string& filename);

  // Infrastructure for setting up the frame buffer object.
  RenderTarget SetupFBO(
      const drake::geometry::render::DepthCameraProperties& camera);

  // Configure the model view and projection matrices.
  void SetGLProjectionMatrix(
      const drake::geometry::render::DepthCameraProperties& camera);
  void SetGLModelViewMatrix(const Eigen::Matrix4f& X_CM) const;

  // Configure the OpenGL properties dependent on the camera properties.
  void SetCameraProperties(
      const drake::geometry::render::DepthCameraProperties& camera);

  // Configure the vertex array object for a triangle mesh.
  OpenGlGeometry SetupVAO(const VertexBuffer& vertices,
                          const IndexBuffer& indices);

  // The cached value transformation between camera and world frame.
  mutable drake::math::RigidTransformd X_CW_;

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

  std::shared_ptr<std::unordered_map<std::string, OpenGlGeometry>> meshes_;

  std::shared_ptr<std::unordered_map<BufferDim, RenderTarget>> frame_buffers_;

  // Mapping from RenderIndex to the visual data associated with that geometry.
  // This is copied so independent renderers can have different *instances* but
  // the instances still refer to the same, shared, underlying geometry.
  std::unordered_map<drake::geometry::GeometryId, OpenGlInstance> visuals_;
};

}  // namespace gl
}  // namespace render
}  // namespace geometry
}  // namespace drake
