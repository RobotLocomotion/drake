#pragma once

#include <memory>
#include <string>
#include <unordered_map>

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
namespace internal {

// Image types available.
// TODO(tehbelinda): Implement color image type.
enum ImageType {
  kColor = 0,
  kLabel = 1,
  kDepth = 2,
};

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

  ~RenderEngineGl() override;

  /** Inherits RenderEngine::UpdateViewpoint().  */
  void UpdateViewpoint(const math::RigidTransformd& X_WR) override;

  /** Inherits RenderEngine::RenderColorImage(). Note that the display window
   triggered by `show_window` is shared with RenderLabelImage, and only the
   last color or label image rendered will be visible in the window.  */
  void RenderColorImage(
      const CameraProperties& camera, bool show_window,
      systems::sensors::ImageRgba8U* color_image_out) const override;

  /** Inherits RenderEngine::RenderDepthImage().  */
  void RenderDepthImage(
      const DepthCameraProperties& camera,
      systems::sensors::ImageDepth32F* depth_image_out) const override;

  /** Inherits RenderEngine::RenderLabelImage(). Note that the display window
   triggered by `show_window` is shared with RenderColorImage, and only the
   last color or label image rendered will be visible in the window. */
  void RenderLabelImage(
      const CameraProperties& camera, bool show_window,
      systems::sensors::ImageLabel16I* label_image_out) const override;

  /** @name    Shape reification  */
  //@{
  using RenderEngine::ImplementGeometry;
  void ImplementGeometry(const Sphere& sphere, void* user_data) override;
  void ImplementGeometry(const Cylinder& cylinder, void* user_data) override;
  void ImplementGeometry(const HalfSpace& half_space, void* user_data) override;
  void ImplementGeometry(const Box& box, void* user_data) override;
  void ImplementGeometry(const Mesh& mesh, void* user_data) override;
  void ImplementGeometry(const Convex& convex, void* user_data) override;
  //@}

 private:
  // @see RenderEngine::DoRegisterVisual().
  bool DoRegisterVisual(GeometryId id,
                        const Shape& shape,
                        const PerceptionProperties& properties,
                        const math::RigidTransformd& X_WG) override;

  // @see RenderEngine::DoUpdateVisualPose().
  void DoUpdateVisualPose(GeometryId id,
                          const math::RigidTransformd& X_WG) override;

  // @see RenderEngine::DoRemoveGeometry().
  bool DoRemoveGeometry(GeometryId id) override;

  // see RenderEngine::DoClone().
  std::unique_ptr<RenderEngine> DoClone() const override;

  // Copy constructor used for cloning.
  RenderEngineGl(const RenderEngineGl& other) = default;

  // Performs the common setup for all shape types.
  void ImplementGeometry(const OpenGlGeometry& geometry, void* user_data,
                         const Vector3<double>& scale);

  // Render the object at a specific pose in the camera frame using the given
  // shader program.
  void RenderAt(std::shared_ptr<ShaderProgram> shader_program,
                const Eigen::Matrix4f& X_CM, const ImageType image_type) const;

  // Obtain the depth image rendered from a specific object pose. This is
  // slow because it reads the buffer back from the GPU.
  void GetDepthImage(systems::sensors::ImageDepth32F* depth_image_out,
                     const RenderTarget& target) const;
  // Obtain the label image rendered from a specific object pose. This is
  // slow because it reads the buffer back from the GPU.
  void GetLabelImage(drake::systems::sensors::ImageLabel16I* label_image_out,
                     const RenderTarget& target) const;

  // Provide triangle mesh definitions of the various geometries supported by
  // this renderer: sphere, cylinder, half space, box, and mesh.
  OpenGlGeometry GetSphere();
  OpenGlGeometry GetCylinder();
  OpenGlGeometry GetHalfSpace();
  OpenGlGeometry GetBox();
  OpenGlGeometry GetMesh(const std::string& filename);

  // Infrastructure for setting up the frame buffer object.
  RenderTarget SetupFBO(const CameraProperties& camera,
                        const ImageType image_type);

  // Configure the model view and projection matrices.
  void SetGlProjectionMatrix(std::shared_ptr<ShaderProgram> shader_program,
                             const CameraProperties& camera);
  void SetGlModelViewMatrix(std::shared_ptr<ShaderProgram> shader_program,
                            const Eigen::Matrix4f& X_CM) const;

  // Configure the shader program for the given image type.
  std::shared_ptr<ShaderProgram> SetShaderProperties(
      const CameraProperties& camera, const ImageType image_type);

  // Configure the OpenGL properties dependent on the camera properties and
  // image type.
  RenderTarget SetCameraProperties(const CameraProperties& camera,
                                   const ImageType image_type);

  // Configure the vertex array object for a triangle mesh.
  OpenGlGeometry SetupVAO(const VertexBuffer& vertices,
                          const IndexBuffer& indices);

  // Updates the display window with the last image rendered.
  // If `show_window` is true:
  //  - if the window is not currently visible, it is made visible
  //  - the windows contents display the most recent image rendered.
  // If `show_window` is false:
  //  - the display window's state is unchanged; hidden remains hidden, and
  //    visible remains visible with unchanged contents.
  // @pre RenderTarget's BufferDim is the same as the size reported by `camera`.
  void UpdateVisibleWindow(const CameraProperties& camera, bool show_window,
                           const RenderTarget& target) const;

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
  // Renders the inverse depth of objects.
  std::shared_ptr<ShaderProgram> depth_shader_program_;
  // Renders the label of objects.
  std::shared_ptr<ShaderProgram> label_shader_program_;

  // One OpenGLGeometry per primitive type -- allow for instancing.
  OpenGlGeometry sphere_;
  OpenGlGeometry cylinder_;
  OpenGlGeometry half_space_;
  OpenGlGeometry box_;

  // Mapping from obj filename to the mesh loaded into an OpenGlGeometry.
  std::shared_ptr<std::unordered_map<std::string, OpenGlGeometry>> meshes_;

  // Mapping from width and height to a RenderTarget. Allows for the re-use of
  // frame buffers if they are of the same dimension.
  std::shared_ptr<std::unordered_map<BufferDim, RenderTarget>>
      depth_frame_buffers_;
  std::shared_ptr<std::unordered_map<BufferDim, RenderTarget>>
      label_frame_buffers_;

  // Mapping from RenderIndex to the visual data associated with that geometry.
  // This is copied so independent renderers can have different *instances* but
  // the instances still refer to the same, shared, underlying geometry.
  std::unordered_map<GeometryId, OpenGlInstance> visuals_;

  // Obnoxious bright orange.
  Eigen::Vector4d default_diffuse_{0.9, 0.45, 0.1, 1.0};
};

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
