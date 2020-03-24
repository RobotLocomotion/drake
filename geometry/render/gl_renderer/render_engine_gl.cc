#include "drake/geometry/render/gl_renderer/render_engine_gl.h"

#include <fmt/format.h>

namespace drake {
namespace geometry {
namespace render {
namespace gl {

using math::RigidTransformd;
using std::make_shared;
using std::string;
using std::unique_ptr;
using std::unordered_map;
using systems::sensors::ColorI;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

namespace {

// Set boundaries for OpenGl clipping planes.
const float kGLZNear = 0.01;
const float kGLZFar = 10.0;
const float kInvZNearMinusZFar = 1. / (kGLZNear - kGLZFar);

// Data to pass through the reification process.
struct RegistrationData {
  const GeometryId id;
  const RigidTransformd& X_WG;
  const PerceptionProperties& properties;
};

}  // namespace

RenderEngineGl::RenderEngineGl()
    : opengl_context_(make_shared<OpenGlContext>()),
      depth_shader_program_(make_shared<ShaderProgram>()),
      label_shader_program_(make_shared<ShaderProgram>()),
      meshes_(make_shared<unordered_map<string, OpenGlGeometry>>()),
      depth_frame_buffers_(
          make_shared<unordered_map<BufferDim, RenderTarget>>()),
      label_frame_buffers_(
          make_shared<unordered_map<BufferDim, RenderTarget>>()),
      visuals_() {
  if (!opengl_context_->is_initialized())
    throw std::runtime_error("OpenGL Context has not been initialized.");

  // Setup shader program.
  const string kDepthVertexShader = R"__(
#version 330

layout(location = 0) in vec3 p_Model;
out float depth;
uniform mat4 model_view_matrix;
uniform mat4 projection_matrix;

void main() {
  vec4 p_Camera = model_view_matrix * vec4(p_Model, 1);
  depth = -p_Camera.z;
  gl_Position = projection_matrix * p_Camera;
})__";
  const string kDepthFragmentShader = R"__(
#version 330

in float depth;
layout(location = 0) out float inverse_depth;
uniform float depth_z_near;
uniform float depth_z_far;

void main() {
  if (depth < depth_z_near)
    // This is ok for OpenGL >= 4.1:
    // https://stackoverflow.com/questions/10435253/glsl-infinity-constant
    inverse_depth = 1.0 / 0.0;
  else if (depth > depth_z_far)
    inverse_depth = 0.0;
  else
    inverse_depth = 1.0 / depth;
})__";
  const std::string kLabelVertexShader = R"__(
#version 330
layout(location = 0) in vec3 p_Model;
uniform mat4 model_view_matrix;
uniform mat4 projection_matrix;
void main() {
  vec4 p_Camera = model_view_matrix * vec4(p_Model, 1);
  gl_Position = projection_matrix * p_Camera;
})__";
  const std::string kLabelFragmentShader = R"__(
#version 330
out vec4 color;
uniform vec4 diffuse;
void main() {
  color = diffuse;
})__";
  label_shader_program_->LoadFromSources(kLabelVertexShader,
                                         kLabelFragmentShader);
  depth_shader_program_->LoadFromSources(kDepthVertexShader,
                                         kDepthFragmentShader);
}

void RenderEngineGl::UpdateViewpoint(const RigidTransformd& X_WR) {
  X_CW_ = X_WR.inverse();
}

void RenderEngineGl::RenderColorImage(const CameraProperties&, bool,
                                      ImageRgba8U*) const {
  throw std::runtime_error("RenderEngineDepthGl cannot render color images");
}

void RenderEngineGl::RenderDepthImage(const DepthCameraProperties& camera,
                                      ImageDepth32F* depth_image_out) const {
  opengl_context_->make_current();

  RenderEngineGl* renderer = const_cast<RenderEngineGl*>(this);
  std::shared_ptr<ShaderProgram> shader_program =
      renderer->SetShaderProperties(camera, ImageType::kDepth);
  RenderTarget target =
      renderer->SetCameraProperties(camera, ImageType::kDepth);

  RenderAt(shader_program, X_CW_.GetAsMatrix4().matrix().cast<float>(),
           ImageType::kDepth);
  GetDepthImage(depth_image_out, target);
}

void RenderEngineGl::RenderLabelImage(const CameraProperties& camera,
                                      bool show_window,
                                      ImageLabel16I* label_image_out) const {
  opengl_context_->make_current();

  RenderEngineGl* renderer = const_cast<RenderEngineGl*>(this);
  std::shared_ptr<ShaderProgram> shader_program =
      renderer->SetShaderProperties(camera, ImageType::kLabel);
  RenderTarget target =
      renderer->SetCameraProperties(camera, ImageType::kLabel);

  RenderAt(shader_program, X_CW_.GetAsMatrix4().matrix().cast<float>(),
           ImageType::kLabel);
  UpdateVisibleWindow(camera, show_window, target);
  GetLabelImage(label_image_out, target);
}

std::shared_ptr<ShaderProgram> RenderEngineGl::SetShaderProperties(
    const CameraProperties& camera, const ImageType image_type) {
  std::shared_ptr<ShaderProgram> shader_program;
  if (image_type == kDepth) {
    shader_program = depth_shader_program_;
    shader_program->Use();
    const DepthCameraProperties& depth_camera =
        static_cast<const DepthCameraProperties&>(camera);
    shader_program->SetUniformValue1f("depth_z_near", depth_camera.z_near);
    shader_program->SetUniformValue1f("depth_z_far", depth_camera.z_far);
    if (depth_camera.z_near < kGLZNear)
      throw std::runtime_error(
          fmt::format("Camera's z_near ({}) is closer than what this renderer "
                      "can handle ({})",
                      depth_camera.z_near, kGLZNear));
    if (depth_camera.z_far > kGLZFar)
      throw std::runtime_error(
          fmt::format("Camera's z_far ({}) is farther than what this renderer "
                      "can handle ({})",
                      depth_camera.z_far, kGLZFar));

  } else if (image_type == kLabel) {
    shader_program = label_shader_program_;
    shader_program->Use();
  }
  SetGLProjectionMatrix(shader_program, camera);
  return shader_program;
}

void RenderEngineGl::SetGLProjectionMatrix(
    std::shared_ptr<ShaderProgram> shader_program,
    const CameraProperties& camera) {
  // https://unspecified.wordpress.com/2012/06/21/calculating-the-gluperspective-matrix-and-other-opengl-matrix-maths/
  // An OpenGL projection matrix maps points in a camera coordinate to a "clip
  // coordinate", in which the projection step maps a 3D point into 2D
  // normalized device coordinate (NDC). Effectively, image corners are mapped
  // into a square from -1 to 1 in NDC. Hence, the clip coordinate is
  // essentially a scaled version of the camera coordinate, where the camera
  // image frustum is scaled into a "square" frustum.
  //
  // Because the xy elements of the image corner [f*w/2, f*h/2] is mapped to
  // [fx*f*w/2, fy*f*h/2] in the "square" clip coordinate by the OpenGL
  // projection matrix P, we have: fx*f*w/2 == fy*f*h/2, i.e. fx*w == fy*h.

  const float fy = 1.0f / static_cast<float>(tan(camera.fov_y * 0.5));
  const float fx = fy * camera.height / camera.width;
  const float A = (kGLZNear + kGLZFar) * kInvZNearMinusZFar;
  const float B = 2.0f * kGLZNear * kGLZFar * kInvZNearMinusZFar;
  Eigen::Matrix4f P;
  // Eigen matrices are col-major, similar to OpenGL.
  // clang-format off
  P << fx, 0.0,  0.0, 0.0,
      0.0, fy,   0.0, 0.0,
      0.0, 0.0,    A,   B,
      0.0, 0.0, -1.0, 0.0;
  // clang-format on
  auto projection_matrix_id =
      shader_program->GetUniformLocation("projection_matrix");
  glUniformMatrix4fv(projection_matrix_id, 1, GL_FALSE, P.data());
}

OpenGlGeometry RenderEngineGl::SetupVAO(const VertexBuffer& vertices,
                                        const IndexBuffer& indices) {
  OpenGlGeometry geometry;
  // Create the VAO.
  glCreateVertexArrays(1, &geometry.vertex_array);

  // Vertex Buffer Object.
  glCreateBuffers(1, &geometry.vertex_buffer);
  glNamedBufferStorage(geometry.vertex_buffer,
                       vertices.size() * sizeof(GLfloat), vertices.data(), 0);
  // Bind with the VAO.
  const int kBindingIndex = 0;  // The binding point.
  glVertexArrayVertexBuffer(geometry.vertex_array, kBindingIndex,
                            geometry.vertex_buffer, 0, 3 * sizeof(GLfloat));

  // Bind the attribute in vertex shader to the VAO at the same binding point.
  const int kLocP_ModelAttrib = 0;  // p_Model's location in vertex shader.
  glVertexArrayAttribFormat(geometry.vertex_array, kLocP_ModelAttrib, 3,
                            GL_FLOAT, GL_FALSE, 0);
  glVertexArrayAttribBinding(geometry.vertex_array, kLocP_ModelAttrib,
                             kBindingIndex);
  glEnableVertexArrayAttrib(geometry.vertex_array, kLocP_ModelAttrib);

  // Index Buffer Object.
  glCreateBuffers(1, &geometry.index_buffer);
  glNamedBufferStorage(geometry.index_buffer, indices.size() * sizeof(GLuint),
                       indices.data(), 0);
  // Bind with the VAO.
  glVertexArrayElementBuffer(geometry.vertex_array, geometry.index_buffer);

  geometry.index_buffer_size = indices.size();

  // Note: We won't need to call the corresponding glDeleteVertexArrays or
  // glDeleteBuffers. The meshes we store are "canonical" meshes. Even if a
  // particular GeometryId is removed, it was only referencing its corresponding
  // canonical mesh. We keep all canonical meshes alive for the lifetime of the
  // Context for convenient reuse.
  return geometry;
}

RenderTarget RenderEngineGl::SetupFBO(const CameraProperties& camera,
                                      const ImageType image_type) {
  // Create a framebuffer object.
  RenderTarget target;
  glCreateFramebuffers(1, &target.frame_buffer);

  // Create the texture object to render to.
  const int kWidth = camera.width;
  const int kHeight = camera.height;
  glGenTextures(1, &target.texture);
  glBindTexture(GL_TEXTURE_2D, target.texture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  if (image_type == ImageType::kDepth) {
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, kWidth, kHeight, 0, GL_RED,
                 GL_FLOAT, 0);
  } else {
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, kWidth, kHeight, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, 0);
  }
  glBindTexture(GL_TEXTURE_2D, 0);

  // Attach the texture to FBO color attachment point.
  glNamedFramebufferTexture(target.frame_buffer, GL_COLOR_ATTACHMENT0,
                            target.texture, 0);

  // Create the renderbuffer object, acting as the z buffer.
  glCreateRenderbuffers(1, &target.render_buffer);
  glNamedRenderbufferStorage(target.render_buffer, GL_DEPTH_COMPONENT, kWidth,
                             kHeight);
  // Attach the renderbuffer to FBO's depth attachment point.
  glNamedFramebufferRenderbuffer(target.frame_buffer, GL_DEPTH_ATTACHMENT,
                                 GL_RENDERBUFFER, target.render_buffer);

  // Check FBO status.
  GLenum status =
      glCheckNamedFramebufferStatus(target.frame_buffer, GL_FRAMEBUFFER);
  if (status != GL_FRAMEBUFFER_COMPLETE) {
    throw std::runtime_error("FBO creation failed.");
  }

  // Specify which buffer to be associated with the fragment shader output.
  GLenum buffers[] = {GL_COLOR_ATTACHMENT0};
  glNamedFramebufferDrawBuffers(target.frame_buffer, 1, buffers);

  return target;
}

void RenderEngineGl::SetGLModelViewMatrix(
    std::shared_ptr<ShaderProgram> shader_program,
    const Eigen::Matrix4f& X_CM) const {
  auto model_view_matrix_id =
      shader_program->GetUniformLocation("model_view_matrix");

  // Our camera frame C wrt the OpenGL's camera frame Cgl.
  static const Eigen::Matrix4f kX_CglC =
      (Eigen::Matrix4f() << 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1)
          .finished();

  Eigen::Matrix4f X_CglM = kX_CglC * X_CM;
  glUniformMatrix4fv(model_view_matrix_id, 1, GL_FALSE, X_CglM.data());
}

RenderTarget RenderEngineGl::SetCameraProperties(const CameraProperties& camera,
                                                 const ImageType image_type) {
  const BufferDim dim{camera.width, camera.height};
  RenderTarget target;
  std::shared_ptr<std::unordered_map<BufferDim, RenderTarget>> frame_buffers =
      image_type == kDepth ? depth_frame_buffers_ : label_frame_buffers_;
  auto iter = frame_buffers->find(dim);
  if (iter == frame_buffers->end()) {
    target = SetupFBO(camera, image_type);
    frame_buffers->insert({dim, target});
  } else {
    target = iter->second;
  }
  glBindFramebuffer(GL_FRAMEBUFFER, target.frame_buffer);
  glViewport(0, 0, camera.width, camera.height);
  return target;
}

void RenderEngineGl::RenderAt(std::shared_ptr<ShaderProgram> shader_program,
                              const Eigen::Matrix4f& X_CW,
                              const ImageType image_type) const {
  glClipControl(GL_UPPER_LEFT, GL_NEGATIVE_ONE_TO_ONE);
  glEnable(GL_DEPTH_TEST);
  if (image_type == kLabel) {
    // Set the default background for label images to be the empty label.
    const auto empty_color =
        RenderEngine::GetColorDFromLabel(RenderLabel::kEmpty);
    glClearColor(empty_color.r, empty_color.g, empty_color.b, 1);
  }
  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

  for (const auto& pair : visuals_) {
    const auto& vis = pair.second;
    glBindVertexArray(vis.geometry.vertex_array);
    if (image_type == kLabel) {
      const auto color = RenderEngine::GetColorDFromLabel(vis.label);
      shader_program->SetUniformValue4f("diffuse", color.r, color.g, color.b,
                                        1.f);
    }
    Eigen::DiagonalMatrix<float, 4, 4> scale(
        Vector4<float>(vis.scale(0), vis.scale(1), vis.scale(2), 1.0));
    // Create the scaled transform (S_CG = X_CW * X_WG * scale) which poses a
    // scaled version of a canonical geometry.
    SetGLModelViewMatrix(shader_program,
                         X_CW * vis.X_WG.GetAsMatrix4().cast<float>() * scale);
    glDrawElements(GL_TRIANGLES, vis.geometry.index_buffer_size,
                   GL_UNSIGNED_INT, 0);
  }
  // Unbind the vertex array back to the default of 0.
  glBindVertexArray(0);
  shader_program->Unuse();
}

void RenderEngineGl::GetDepthImage(ImageDepth32F* depth_image_out,
                                   const RenderTarget& target) const {
  glGetTextureImage(target.texture, 0, GL_RED, GL_FLOAT,
                    depth_image_out->size() * sizeof(GLfloat),
                    depth_image_out->at(0, 0));

  for (int y = 0; y < depth_image_out->height(); ++y) {
    for (int x = 0; x < depth_image_out->width(); ++x) {
      *depth_image_out->at(x, y) = 1.f / *depth_image_out->at(x, y);
    }
  }
}

void RenderEngineGl::GetLabelImage(ImageLabel16I* label_image_out,
                                   const RenderTarget& target) const {
  ImageRgba8U image(label_image_out->width(), label_image_out->height());
  glGetTextureImage(target.texture, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                    image.size() * sizeof(GLubyte), image.at(0, 0));
  ColorI color;
  for (int y = 0; y < image.height(); ++y) {
    for (int x = 0; x < image.width(); ++x) {
      color.r = image.at(x, y)[0];
      color.g = image.at(x, y)[1];
      color.b = image.at(x, y)[2];
      *label_image_out->at(x, y) = RenderEngine::LabelFromColor(color);
    }
  }
}

void RenderEngineGl::UpdateVisibleWindow(const CameraProperties& camera,
                                         bool show_window,
                                         const RenderTarget& target) const {
  // We don't need to check that the camera width and camera height match the
  // target dimensions as this is already handled in the target's creation. The
  // same camera and target are then used together in the invocation of this
  // function. The target itself does not retain any size information as it is
  // retrieved via the BufferDim.

  if (show_window) {
    opengl_context_->resize_window(camera.width, camera.height);
    // Clear the default buffer (0) which will be used for the window.
    const GLint clear_color = 0;
    glClearNamedFramebufferiv(0, GL_COLOR, 0, &clear_color);
    // Use the render target buffer as the read buffer and the default buffer
    // (0) as the draw buffer for displaying in the window. We transfer the full
    // image bounded by (0, 0) and (camera.width, camera.height) from source to
    // destination.
    glBlitNamedFramebuffer(target.frame_buffer, 0,
                           0, 0, camera.width, camera.height,  // Src bounds.
                           0, 0, camera.width, camera.height,  // Dest bounds.
                           GL_COLOR_BUFFER_BIT, GL_NEAREST);
    opengl_context_->display_window();
  }
}

void RenderEngineGl::ImplementGeometry(const Sphere& sphere, void* user_data) {
  OpenGlGeometry geometry = GetSphere();
  const double r = sphere.radius();
  ImplementGeometry(geometry, user_data, Vector3<double>{r, r, r});
}

void RenderEngineGl::ImplementGeometry(const Cylinder& cylinder,
                                       void* user_data) {
  OpenGlGeometry geometry = GetCylinder();
  const double r = cylinder.radius();
  const double l = cylinder.length();
  ImplementGeometry(geometry, user_data, Vector3<double>{r, r, l});
}

void RenderEngineGl::ImplementGeometry(const HalfSpace&, void* user_data) {
  OpenGlGeometry geometry = GetHalfSpace();
  const RegistrationData& data = *static_cast<RegistrationData*>(user_data);
  const RenderLabel label = GetRenderLabelOrThrow(data.properties);
  visuals_.emplace(data.id, OpenGlInstance(geometry, data.X_WG,
                                           Vector3<double>{1, 1, 1}, label));
}

void RenderEngineGl::ImplementGeometry(const Box& box, void* user_data) {
  OpenGlGeometry geometry = GetBox();
  ImplementGeometry(geometry, user_data,
                    Vector3<double>{box.width(), box.depth(), box.height()});
}

void RenderEngineGl::ImplementGeometry(const Mesh& mesh, void* user_data) {
  OpenGlGeometry geometry = GetMesh(mesh.filename());
  ImplementGeometry(geometry, user_data,
                    Vector3<double>{1, 1, 1} * mesh.scale());
}

void RenderEngineGl::ImplementGeometry(const Convex& convex, void* user_data) {
  OpenGlGeometry geometry = GetMesh(convex.filename());
  ImplementGeometry(geometry, user_data,
                    Vector3<double>{1, 1, 1} * convex.scale());
}

void RenderEngineGl::ImplementGeometry(const OpenGlGeometry& geometry,
                                       void* user_data,
                                       const Vector3<double>& scale) {
  const RegistrationData& data = *static_cast<RegistrationData*>(user_data);
  const RenderLabel label = GetRenderLabelOrThrow(data.properties);
  visuals_.emplace(data.id, OpenGlInstance(geometry, data.X_WG, scale, label));
}

bool RenderEngineGl::DoRegisterVisual(GeometryId id, const Shape& shape,
                                      const PerceptionProperties& properties,
                                      const RigidTransformd& X_WG) {
  opengl_context_->make_current();
  RegistrationData data{id, RigidTransformd{X_WG}, properties};
  shape.Reify(this, &data);
  return true;
}

void RenderEngineGl::DoUpdateVisualPose(GeometryId id,
                                        const RigidTransformd& X_WG) {
  visuals_.at(id).X_WG = X_WG;
}

bool RenderEngineGl::DoRemoveGeometry(GeometryId id) {
  auto iter = visuals_.find(id);
  if (iter != visuals_.end()) {
    visuals_.erase(iter);
    return true;
  } else {
    return false;
  }
}

unique_ptr<RenderEngine> RenderEngineGl::DoClone() const {
  return unique_ptr<RenderEngineGl>(new RenderEngineGl(*this));
}

OpenGlGeometry RenderEngineGl::GetSphere() {
  if (!sphere_.is_defined()) {
    const int kLatitudeBands = 50;
    const int kLongitudeBands = 50;

    const int tri_count = 2 * (kLatitudeBands - 1) * kLongitudeBands;
    const int vert_count = (kLatitudeBands - 1) * kLongitudeBands + 2;

    VertexBuffer vertices{vert_count, 3};
    IndexBuffer indices{tri_count, 3};

    vertices.block<1, 3>(0, 0) << 0.f, 0.f, 1.f;

    // We don't take slices of the sphere that are equidistant along the z-axis.
    // Instead, we create slices so that the chord length along the perimeter
    // of the longitudinal circle are equal.
    int v = 1;
    int t = 0;

    GLfloat z = cos(M_PI / kLatitudeBands);
    GLfloat radius = sqrt(1.0 - z * z);
    int line_start = 1;
    for (int i = 0; i < kLongitudeBands; ++i) {
      const GLfloat theta = i * M_PI * 2 / kLongitudeBands;
      const GLfloat cos_theta = cos(theta);
      const GLfloat sin_theta = sin(theta);
      vertices.block<1, 3>(v++, 0) << radius * cos_theta, radius * sin_theta, z;
      indices.block<1, 3>(t++, 0) << 0, line_start + i,
          line_start + (i + 1) % kLongitudeBands;
    }

    // Latitudinal bands
    for (int latitude = 1; latitude < kLatitudeBands - 1; ++latitude) {
      int previous_line_start = line_start;
      line_start += kLongitudeBands;
      z = cos((latitude + 1) * M_PI / kLatitudeBands);
      radius = sqrt(1.0f - z * z);
      for (int i = 0; i < kLongitudeBands; ++i) {
        const GLfloat theta = i * M_PI * 2 / kLongitudeBands;
        const GLfloat cos_theta = cos(theta);
        const GLfloat sin_theta = sin(theta);
        vertices.block<1, 3>(v++, 0) << radius * cos_theta, radius * sin_theta,
            z;
        const int next = (i + 1) % kLongitudeBands;
        indices.block<1, 3>(t++, 0) << previous_line_start + i, line_start + i,
            line_start + next;
        indices.block<1, 3>(t++, 0) << previous_line_start + i,
            line_start + next, previous_line_start + next;
      }
    }

    // South pole fan
    vertices.block<1, 3>(v++, 0) << 0.f, 0.f, -1.f;
    DRAKE_DEMAND(v == vert_count);
    const int south_pole = line_start + kLongitudeBands;
    for (int i = 0; i < kLongitudeBands; ++i) {
      indices.block<1, 3>(t++, 0) << line_start + i, south_pole,
          line_start + (i + 1) % kLongitudeBands;
    }
    DRAKE_DEMAND(t == tri_count);

    sphere_ = SetupVAO(vertices, indices);
  }

  sphere_.throw_if_undefined("Built-in sphere has some invalid objects");

  return sphere_;
}

OpenGlGeometry RenderEngineGl::GetCylinder() {
  if (!cylinder_.is_defined()) {
    const int kLongitudeBands = 50;

    // A fan of triangles on each cap with kLongitudeBands triangles, and then
    // that many rectangles along the barrels (with twice as many triangles).
    const int tri_count = 4 * kLongitudeBands;
    // A pair of vertices for each longitudinal band + 1 for each cap.
    const int vert_count = 2 * kLongitudeBands + 2;

    VertexBuffer vertices{vert_count, 3};
    IndexBuffer indices{tri_count, 3};

    vertices.block<1, 3>(0, 0) << 0.f, 0.f, 0.5f;

    // Both caps are triangle fans around central points (leading to nicer
    // triangles).
    const GLfloat radius = 1.f;
    int line_start = 1;
    // Index of the previous vertices and triangles, respectively.
    int v = 0;
    int t = -1;

    // Top cap.
    GLfloat z = 0.5f;
    for (int i = 0; i < kLongitudeBands; ++i) {
      const auto theta = static_cast<GLfloat>(i * M_PI * 2 / kLongitudeBands);
      const auto cos_theta = static_cast<GLfloat>(cos(theta));
      const auto sin_theta = static_cast<GLfloat>(sin(theta));
      vertices.block<1, 3>(++v, 0) << radius * cos_theta, radius * sin_theta, z;
      indices.block<1, 3>(++t, 0) << 0, line_start + i,
          line_start + (i + 1) % kLongitudeBands;
    }

    // Barrel.
    line_start += kLongitudeBands;
    z = -0.5;
    const int previous_line_start = 1;
    for (int i = 0; i < kLongitudeBands; ++i) {
      const auto theta = static_cast<GLfloat>(i * M_PI * 2 / kLongitudeBands);
      const auto cos_theta = static_cast<GLfloat>(cos(theta));
      const auto sin_theta = static_cast<GLfloat>(sin(theta));
      vertices.block<1, 3>(++v, 0) << radius * cos_theta, radius * sin_theta, z;
      const int next = (i + 1) % kLongitudeBands;
      indices.block<1, 3>(++t, 0) << previous_line_start + i, line_start + i,
          line_start + next;
      indices.block<1, 3>(++t, 0) << previous_line_start + i, line_start + next,
          previous_line_start + next;
    }

    // Bottom cap.
    line_start += kLongitudeBands;
    vertices.block<1, 3>(++v, 0) << 0, 0, z;
    for (int i = 0; i < kLongitudeBands; ++i) {
      indices.block<1, 3>(++t, 0) << v, line_start + (i + 1) % kLongitudeBands,
          line_start + i;
    }

    DRAKE_DEMAND(v == vert_count - 1);
    DRAKE_DEMAND(t == tri_count - 1);

    cylinder_ = SetupVAO(vertices, indices);
  }

  cylinder_.throw_if_undefined("Built-in cylinder has some invalid objects");

  return cylinder_;
}

OpenGlGeometry RenderEngineGl::GetHalfSpace() {
  if (!half_space_.is_defined()) {
    //                 _  y
    //                  /|
    //                 /
    //     3_________________________ 2
    //     /         ^ z            /
    //    /          |_            /  --> x
    //   /           Go           /
    //  /                        /
    // /________________________/
    // 0                         1
    const GLfloat kHalfSize = 100.f;
    VertexBuffer vertices{4, 3};
    vertices << -kHalfSize, -kHalfSize, 0.f, kHalfSize, -kHalfSize, 0.f,
        kHalfSize, kHalfSize, 0.f, -kHalfSize, kHalfSize, 0.f;

    IndexBuffer indices{2, 3};
    indices << 0, 1, 2, 0, 2, 3;
    half_space_ = SetupVAO(vertices, indices);
  }

  half_space_.throw_if_undefined(
      "Built-in half space has some invalid objects");

  return half_space_;
}

OpenGlGeometry RenderEngineGl::GetBox() {
  if (!box_.is_defined()) {
    //     7      6
    //     _____
    //    /|    /|
    //  2/_|__3/ |
    //  |  |   | |
    //  | 4|___|_| 5
    //  | /    | /
    //  |/_____|/
    //  0      1
    VertexBuffer vertices{8, 3};
    vertices << -0.5f, -0.5f, -0.5f, 0.5f, -0.5f, -0.5f, 0.5f, 0.5f, -0.5f,
        -0.5f, 0.5f, -0.5f, -0.5f, -0.5f, 0.5f, 0.5f, -0.5f, 0.5f, 0.5f, 0.5f,
        0.5f, -0.5f, 0.5f, 0.5f;
    IndexBuffer indices{12, 3};
    indices << 0, 1, 2, 0, 2, 3, 1, 5, 6, 1, 6, 2, 2, 6, 7, 2, 7, 3, 3, 7, 4, 3,
        4, 0, 7, 6, 5, 7, 5, 4, 1, 0, 4, 1, 4, 5;
    box_ = SetupVAO(vertices, indices);
  }

  box_.throw_if_undefined("Built-in box has some invalid objects");

  return box_;
}

OpenGlGeometry RenderEngineGl::GetMesh(const string& filename) {
  OpenGlGeometry mesh;
  if (meshes_->count(filename) == 0) {
    auto [vertices, indices] = LoadMeshFromObj(filename);  // NOLINT
    mesh = SetupVAO(vertices, indices);
    meshes_->insert({filename, mesh});
  } else {
    mesh = meshes_->at(filename);
  }

  mesh.throw_if_undefined(
      fmt::format("Error creating object for mesh {}", filename).c_str());

  return mesh;
}

RenderEngineGl::~RenderEngineGl() = default;

}  // namespace gl
}  // namespace render
}  // namespace geometry
}  // namespace drake
