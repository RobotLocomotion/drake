#include "drake/geometry/render/gl_renderer/render_engine_gl.h"

#include <algorithm>
#include <optional>
#include <utility>

#include <fmt/format.h>

namespace drake {
namespace geometry {
namespace render {

using Eigen::Vector3d;
using internal::BufferDim;
using internal::MeshData;
using internal::OpenGlContext;
using internal::OpenGlGeometry;
using internal::OpenGlInstance;
using internal::RenderTarget;
using internal::RenderType;
using internal::ShaderId;
using internal::ShaderProgram;
using internal::ShaderProgramData;
using math::RigidTransformd;
using std::make_shared;
using std::make_unique;
using std::move;
using std::string;
using std::unique_ptr;
using std::unordered_map;
using std::vector;
using systems::sensors::ColorD;
using systems::sensors::ColorI;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;
using systems::sensors::InvalidDepth;

namespace {

// Set default boundaries for OpenGl clipping planes.
const float kGlZNear = 0.01;
const float kGlZFar = 100.0;

// Data to pass through the reification process.
struct RegistrationData {
  const GeometryId id;
  const RigidTransformd& X_WG;
  const PerceptionProperties& properties;
};

/* The built-in shader for objects in depth images. By default, the shader
 supports all geometries.  */
class DefaultDepthShader final : public internal::ShaderProgram {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DefaultDepthShader)

  DefaultDepthShader() : ShaderProgram() {
    LoadFromSources(kVertexShader, kFragmentShader);
    depth_z_near_loc_ = GetUniformLocation("depth_z_near");
    depth_z_far_loc_ = GetUniformLocation("depth_z_far");
  }

  void SetDepthCameraParameters(
      const DepthCameraProperties& camera) const final {
    glUniform1f(depth_z_near_loc_, camera.z_near);
    glUniform1f(depth_z_far_loc_, camera.z_far);
  }

 private:
  std::unique_ptr<ShaderProgram> DoClone() const final {
    return make_unique<DefaultDepthShader>(*this);
  }

  std::optional<ShaderProgramData> DoCreateProgramData(
      const PerceptionProperties&) const final {
    // The depth shader supports all geometries, but requires no data.
    return ShaderProgramData{shader_id(), nullptr};
  }

  // Uniform locations.
  GLint depth_z_near_loc_{};
  GLint depth_z_far_loc_{};

  // The vertex shader computes two pieces of information per vertex: its
  // transformed position and its depth. Both get linearly interpolated across
  // the rasterized triangle's fragments.
  static constexpr char kVertexShader[] = R"""(
#version 330

layout(location = 0) in vec3 p_MV;
out float depth;
uniform mat4 model_view_matrix;
uniform mat4 projection_matrix;

void main() {
  vec4 p_CV = model_view_matrix * vec4(p_MV, 1);
  depth = -p_CV.z;
  gl_Position = projection_matrix * p_CV;
})""";

  // The fragment shader "clamps" the depth of the fragment to the specified
  // sensor range. Values closer than depth_z_near are set to zero, values
  // farther than depth_z_far are pushed to infinity. This "clamped" depth
  // value gets written to the frame buffer.
  static constexpr char kFragmentShader[] = R"""(
#version 330

in float depth;
// Depth is encoded such that values closer than depth_z_near or farther than
// depth_z_far get saturated to zero and infinity, respectively.
layout(location = 0) out float encoded_depth;
uniform float depth_z_near;
uniform float depth_z_far;

void main() {
  // We need a value for infinity; 1 / 0 is only guaranteed to work for
  // OpenGL >= 4.1. We apply the bit encoding of IEEE 32-bit infinity
  // directly.
  // https://stackoverflow.com/questions/10435253/glsl-infinity-constant
  // Note: endianness is not a concern.  OpenGL has client pixel data in client
  // byte ordering.
  // https://www.khronos.org/opengl/wiki/Pixel_Transfer#Endian_issues
  // This literal gets represented with the client's byte order, so the
  // corresponding float will likewise have the right byte order.
  const float pos_infinity = intBitsToFloat(0x7F800000);
  if (depth < depth_z_near)
    encoded_depth = 0;
  else if (depth > depth_z_far)
    encoded_depth = pos_infinity;
  else
    encoded_depth = depth;
})""";
};

/* The built-in shader for objects in label images. The support this shader
 gives for geometry depends on the label encoder function. The shader program
 assumes the encoder will either provide a label or throw based on the given
 perception properties.  */
class DefaultLabelShader final : public internal::ShaderProgram {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DefaultLabelShader)

  /* Constructs the label shader with the given `label_encoder`.

   @param label_encoder  A function that extracts an encoded color (Vector4f)
                         from a set of PerceptionProperties representing the
                         color-encoded RenderLabel. If such a color can't be
                         defined, the function should throw.
  */
  explicit DefaultLabelShader(
      std::function<Vector4<float>(const PerceptionProperties&)> label_encoder)
      : ShaderProgram(), label_encoder_(move(label_encoder)) {
    LoadFromSources(kVertexShader, kFragmentShader);
    encoded_label_loc_ = GetUniformLocation("encoded_label");
  }

  void SetInstanceParameters(const ShaderProgramData& data) const final {
    glUniform4fv(encoded_label_loc_, 1,
                 data.value().get_value<Vector4<float>>().data());
  }

 private:
  std::unique_ptr<ShaderProgram> DoClone() const final {
    return make_unique<DefaultLabelShader>(*this);
  }

  std::optional<ShaderProgramData> DoCreateProgramData(
      const PerceptionProperties& properties) const final {
    return ShaderProgramData{shader_id(),
                             AbstractValue::Make(label_encoder_(properties))};
  }

  std::function<Vector4<float>(const PerceptionProperties&)> label_encoder_;

  GLint encoded_label_loc_{};

  // The vertex shader simply transforms the vertices. Strictly speaking, we
  // could combine modelview and projection matrices into a single transform,
  // but there's no real value in doing so. Leaving it as is maintains
  // compatibility with the depth shader.
  static constexpr char kVertexShader[] = R"""(
#version 330
layout(location = 0) in vec3 p_MV;
uniform mat4 model_view_matrix;
uniform mat4 projection_matrix;
void main() {
  // X_CM = model_view_matrix (although X may not be a *rigid* transform).
  vec4 p_CV = model_view_matrix * vec4(p_MV, 1);
  gl_Position = projection_matrix * p_CV;
})""";

  // For each fragment from a geometry, it simply colors the fragment with the
  // provided label encoded as an RGBA color.
  static constexpr char kFragmentShader[] = R"""(
#version 330
out vec4 color;
uniform vec4 encoded_label;
void main() {
  color = encoded_label;
})""";
};

}  // namespace

RenderEngineGl::RenderEngineGl()
    : RenderEngine(),
      opengl_context_(make_shared<OpenGlContext>()) {
  // Configuration of basic OpenGl state.
  opengl_context_->MakeCurrent();
  glClipControl(GL_UPPER_LEFT, GL_NEGATIVE_ONE_TO_ONE);
  glClearDepth(1.0);
  glEnable(GL_DEPTH_TEST);
  // Depth shaders -- a single shader that accepts all geometry.
  AddShader(make_unique<DefaultDepthShader>(), RenderType::kDepth);

  // Label shaders -- a single shader that accepts all geometry (unless it has
  // an invalid RenderLabel -- see RenderEngine::GetLabelOrThrow).
  // Extracts the label from properties (with error checking) and returns the
  // r,g,b,a color to represent it.
  auto label_encoder = [this](const PerceptionProperties& props) {
    const RenderLabel& label = this->GetRenderLabelOrThrow(props);
    const ColorD color = this->GetColorDFromLabel(label);
    return Vector4<float>(color.r, color.g, color.b, 1.f);
  };
  AddShader(make_unique<DefaultLabelShader>(label_encoder), RenderType::kLabel);
}

RenderEngineGl::~RenderEngineGl() = default;

void RenderEngineGl::UpdateViewpoint(const RigidTransformd& X_WR) {
  X_CW_ = X_WR.inverse();
}

void RenderEngineGl::RenderColorImage(const CameraProperties&, bool,
                                      ImageRgba8U*) const {
  throw std::runtime_error("RenderEngineGl cannot render color images");
}

void RenderEngineGl::RenderDepthImage(const DepthCameraProperties& camera,
                                      ImageDepth32F* depth_image_out) const {
  opengl_context_->MakeCurrent();

  const RenderTarget render_target =
      GetRenderTarget(camera, RenderType::kDepth);

  // We initialize the color buffer to be all "too far" values. This is the
  // pixel value if nothing draws there -- i.e., nothing there implies that
  // whatever *might* be there is "too far" beyond the depth range.
  glClearNamedFramebufferfv(render_target.frame_buffer, GL_COLOR, 0,
                            &InvalidDepth::kTooFar);
  glClear(GL_DEPTH_BUFFER_BIT);

  // TODO(SeanCurtis-TRI): When clipping planes get set by the user, conflict
  //  between depth camera range and clipping range should be an error. For
  //  now, we simply define the clipping planes to tightly (but not exactly)
  //  bound the depth ranges. The tightness gives us the most precision in the
  //  z-buffer in the depth range. The slightly larger (10 cm in front and 10
  //  cm behind) domain will allow us to recognize at least *some* fragments
  //  which rendered outside the depth range.
  const double near_clip = std::max(0.01, camera.z_near - 0.1);
  const double far_clip = camera.z_far + 0.1;
  // Matrix mapping a geometry vertex from its frame G to the device frame D.
  const Eigen::Matrix4f X_DC =
      ComputeGlProjectionMatrix(camera, near_clip, far_clip);

  for (const auto& id_shader_pair : shader_programs_[RenderType::kDepth]) {
    const ShaderProgram& shader_program = *(id_shader_pair.second);
    shader_program.Use();

    shader_program.SetProjectionMatrix(X_DC);
    shader_program.SetDepthCameraParameters(camera);
    RenderAt(shader_program, RenderType::kDepth);

    shader_program.Unuse();
  }

  glGetTextureImage(render_target.value_texture, 0, GL_RED, GL_FLOAT,
                    depth_image_out->size() * sizeof(GLfloat),
                    depth_image_out->at(0, 0));
}

void RenderEngineGl::RenderLabelImage(const CameraProperties& camera,
                                      bool show_window,
                                      ImageLabel16I* label_image_out) const {
  opengl_context_->MakeCurrent();

  const RenderTarget render_target =
      GetRenderTarget(camera, RenderType::kLabel);
  // TODO(SeanCurtis-TRI) Consider converting Rgba to float[4] as a member.
  const ColorD empty_color =
      RenderEngine::GetColorDFromLabel(RenderLabel::kEmpty);
  float clear_color[4] = {static_cast<float>(empty_color.r),
                          static_cast<float>(empty_color.g),
                          static_cast<float>(empty_color.b), 1.f};
  glClearNamedFramebufferfv(render_target.frame_buffer, GL_COLOR, 0,
                            &clear_color[0]);
  glClear(GL_DEPTH_BUFFER_BIT);
  // Matrix mapping a geometry vertex from its frame G to the device frame D.
  const Eigen::Matrix4f X_DC =
      ComputeGlProjectionMatrix(camera, kGlZNear, kGlZFar);

  for (const auto& id_shader_pair : shader_programs_[RenderType::kLabel]) {
    const ShaderProgram& shader_program = *(id_shader_pair.second);
    shader_program.Use();

    shader_program.SetProjectionMatrix(X_DC);
    RenderAt(shader_program, RenderType::kLabel);

    shader_program.Unuse();
  }

  // Note: SetWindowVisibility must be called *after* the rendering; setting the
  // visibility is responsible for taking the target buffer and bringing it to
  // the front buffer; reversing the order means the image we've just rendered
  // wouldn't be visible.
  SetWindowVisibility(camera, show_window, render_target);
  // TODO(SeanCurtis-TRI): Apparently, we *should* be able to create a frame
  // buffer texture consisting of a single-channel, 16-bit, signed int (to match
  // the underlying RenderLabel value). Doing so would allow us to render labels
  // directly and eliminate this additional pass.
  GetLabelImage(label_image_out, render_target);
}

void RenderEngineGl::ImplementGeometry(const Sphere& sphere, void* user_data) {
  OpenGlGeometry geometry = GetSphere();
  const double r = sphere.radius();
  ImplementGeometry(geometry, user_data, Vector3d(r, r, r));
}

void RenderEngineGl::ImplementGeometry(const Cylinder& cylinder,
                                       void* user_data) {
  OpenGlGeometry geometry = GetCylinder();
  const double r = cylinder.radius();
  const double l = cylinder.length();
  ImplementGeometry(geometry, user_data, Vector3d(r, r, l));
}

void RenderEngineGl::ImplementGeometry(const HalfSpace&, void* user_data) {
  OpenGlGeometry geometry = GetHalfSpace();
  ImplementGeometry(geometry, user_data, Vector3d(1, 1, 1));
}

void RenderEngineGl::ImplementGeometry(const Box& box, void* user_data) {
  OpenGlGeometry geometry = GetBox();
  ImplementGeometry(geometry, user_data,
                    Vector3d(box.width(), box.depth(), box.height()));
}

void RenderEngineGl::ImplementGeometry(const Capsule& capsule,
                                       void* user_data) {
  const int resolution = 50;
  MeshData mesh_data =
      internal::MakeCapsule(resolution, capsule.radius(), capsule.length());

  OpenGlGeometry geometry = CreateGlGeometry(mesh_data);
  capsules_.push_back(geometry);

  ImplementGeometry(geometry, user_data, Vector3d::Ones());
}

void RenderEngineGl::ImplementGeometry(const Ellipsoid& ellipsoid,
                                       void* user_data) {
  OpenGlGeometry geometry = GetSphere();
  ImplementGeometry(geometry, user_data,
                    Vector3d(ellipsoid.a(), ellipsoid.b(), ellipsoid.c()));
}

void RenderEngineGl::ImplementGeometry(const Mesh& mesh, void* user_data) {
  OpenGlGeometry geometry = GetMesh(mesh.filename());
  ImplementGeometry(geometry, user_data, Vector3d(1, 1, 1) * mesh.scale());
}

void RenderEngineGl::ImplementGeometry(const Convex& convex, void* user_data) {
  OpenGlGeometry geometry = GetMesh(convex.filename());
  ImplementGeometry(geometry, user_data, Vector3d(1, 1, 1) * convex.scale());
}

bool RenderEngineGl::DoRegisterVisual(GeometryId id, const Shape& shape,
                                      const PerceptionProperties& properties,
                                      const RigidTransformd& X_WG) {
  opengl_context_->MakeCurrent();
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
    // Remove from the shader families to which it belongs!
    auto remove_from_family = [this](GeometryId g_id,
                                     const auto& shader_data,
                                     RenderType render_type) {
      const ShaderId s_id = shader_data[render_type].shader_id();
      auto& geometries = shader_families_[render_type].at(s_id);
      for (size_t i = 0; i < geometries.size(); ++i) {
        if (geometries[i] == g_id) {
          std::swap(geometries[i], geometries.back());
          geometries.pop_back();
          return;
        }
      }
      DRAKE_UNREACHABLE();
    };
    const OpenGlInstance& instance = iter->second;
    remove_from_family(id, instance.shader_data, RenderType::kDepth);
    remove_from_family(id, instance.shader_data, RenderType::kLabel);
    visuals_.erase(iter);
    return true;
  } else {
    return false;
  }
}

unique_ptr<RenderEngine> RenderEngineGl::DoClone() const {
  return unique_ptr<RenderEngineGl>(new RenderEngineGl(*this));
}

void RenderEngineGl::RenderAt(const ShaderProgram& shader_program,
                              RenderType render_type) const {
  // TODO(SeanCurtis-TRI) Consider storing a float-version of X_CW so it's only
  //  created once per camera declaration (and not once per shader).
  const Eigen::Matrix4f& X_CW = X_CW_.GetAsMatrix4().matrix().cast<float>();
  // We rely on the calling method to clear all appropriate buffers; this method
  // may be called multiple times per image (based on the number of shaders
  // being used) and, therefore, can't do the clearing itself.

  for (const GeometryId& g_id :
       shader_families_.at(render_type).at(shader_program.shader_id())) {
    const internal::OpenGlInstance& instance = visuals_.at(g_id);
    glBindVertexArray(instance.geometry.vertex_array);

    shader_program.SetInstanceParameters(instance.shader_data[render_type]);
    // TODO(SeanCurtis-TRI): Consider storing the float-valued pose in the
    //  OpenGl instance to avoid the conversion every time it is rendered.
    //  Generally, this wouldn't exepct much savings; an instance is only
    //  rendered once per image type. So, for three image types, I'd cast three
    //  times. Stored, I'd cast once.
    shader_program.SetModelViewMatrix(
        X_CW * instance.X_WG.GetAsMatrix4().cast<float>(), instance.scale);

    glDrawElements(GL_TRIANGLES, instance.geometry.index_buffer_size,
                   GL_UNSIGNED_INT, 0);
  }
  // Unbind the vertex array back to the default of 0.
  glBindVertexArray(0);
}

void RenderEngineGl::ImplementGeometry(const OpenGlGeometry& geometry,
                                       void* user_data, const Vector3d& scale) {
  const RegistrationData& data = *static_cast<RegistrationData*>(user_data);
  std::optional<ShaderProgramData> depth_data =
      GetShaderProgram(data.properties, RenderType::kDepth);
  std::optional<ShaderProgramData> label_data =
      GetShaderProgram(data.properties, RenderType::kLabel);
  DRAKE_DEMAND(depth_data.has_value() && label_data.has_value());
  visuals_.emplace(data.id, OpenGlInstance(geometry, data.X_WG, scale,
                                           *depth_data, *label_data));

  shader_families_[RenderType::kDepth][depth_data->shader_id()].push_back(
      data.id);
  shader_families_[RenderType::kLabel][label_data->shader_id()].push_back(
      data.id);
}

OpenGlGeometry RenderEngineGl::GetSphere() {
  if (!sphere_.is_defined()) {
    const int kLatitudeBands = 50;
    const int kLongitudeBands = 50;

    MeshData mesh_data =
        internal::MakeLongLatUnitSphere(kLongitudeBands, kLatitudeBands);

    sphere_ = CreateGlGeometry(mesh_data);
  }

  sphere_.throw_if_undefined("Built-in sphere has some invalid objects");

  return sphere_;
}

OpenGlGeometry RenderEngineGl::GetCylinder() {
  if (!cylinder_.is_defined()) {
    const int kLongitudeBands = 50;

    // For long skinny cylinders, it would be better to offer some subdivisions
    // along the length. For now, we'll simply save the triangles.
    MeshData mesh_data = internal::MakeUnitCylinder(kLongitudeBands, 1);
    cylinder_ = CreateGlGeometry(mesh_data);
  }

  cylinder_.throw_if_undefined("Built-in cylinder has some invalid objects");

  return cylinder_;
}

OpenGlGeometry RenderEngineGl::GetHalfSpace() {
  if (!half_space_.is_defined()) {
    // This matches the RenderEngineVtk half space size. Keep them matching
    // so that the common "horizon" unit test passes.
    const GLfloat kMeasure = 100.f;
    // TODO(SeanCurtis-TRI): For vertex-lighting (as opposed to fragment
    //  lighting), this will render better with tighter resolution. Consider
    //  making this configurable.
    MeshData mesh_data = internal::MakeSquarePatch(kMeasure, 1);
    half_space_ = CreateGlGeometry(mesh_data);
  }

  half_space_.throw_if_undefined(
      "Built-in half space has some invalid objects");

  return half_space_;
}

OpenGlGeometry RenderEngineGl::GetBox() {
  if (!box_.is_defined()) {
    MeshData mesh_data = internal::MakeUnitBox();
    box_ = CreateGlGeometry(mesh_data);
  }

  box_.throw_if_undefined("Built-in box has some invalid objects");

  return box_;
}

OpenGlGeometry RenderEngineGl::GetMesh(const string& filename) {
  OpenGlGeometry mesh;
  if (meshes_.count(filename) == 0) {
    MeshData mesh_data = internal::LoadMeshFromObj(filename);
    mesh = CreateGlGeometry(mesh_data);
    meshes_.insert({filename, mesh});
  } else {
    mesh = meshes_[filename];
  }

  mesh.throw_if_undefined(
      fmt::format("Error creating object for mesh {}", filename).c_str());

  return mesh;
}

std::tuple<GLint, GLenum, GLenum> RenderEngineGl::get_texture_format(
    RenderType render_type) {
  switch (render_type) {
    case RenderType::kLabel:
      // TODO(SeanCurtis-TRI): Ultimately, this should be a 16-bit, signed int.
      return std::make_tuple(GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
    case RenderType::kDepth:
      return std::make_tuple(GL_R32F, GL_RED, GL_FLOAT);
    case RenderType::kTypeCount:
      // Not an actionable type; merely included for enum completeness.
      break;
  }
  DRAKE_UNREACHABLE();
}

RenderTarget RenderEngineGl::CreateRenderTarget(const CameraProperties& camera,
                                                RenderType render_type) {
  // Create a framebuffer object (FBO).
  RenderTarget target;
  glCreateFramebuffers(1, &target.frame_buffer);

  // Create the texture object that will store the rendered result.
  const int width = camera.width;
  const int height = camera.height;
  glGenTextures(1, &target.value_texture);
  glBindTexture(GL_TEXTURE_2D, target.value_texture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  auto [internal_format, format, pixel_type] = get_texture_format(render_type);
  glTexImage2D(GL_TEXTURE_2D, 0, internal_format, width, height, 0, format,
               pixel_type, 0);
  glBindTexture(GL_TEXTURE_2D, 0);

  // TODO(SeanCurtis-TRI): This (and all glNamed*() methods we're using) might
  //  be a problem in CI. This function is only available for OpenGL >= 4.5.
  //  We're limiting ourselves to 3.3.
  //  For details on limiting OpenGl version:
  //  https://github.com/RobotLocomotion/drake/issues/12868
  //  If it's a problem, back this off to the glFramebufferTexture API (which
  //  is _a lot_ more painful to use).
  glNamedFramebufferTexture(target.frame_buffer, GL_COLOR_ATTACHMENT0,
                            target.value_texture, 0);

  // Create the render buffer object (RBO), acting as the z buffer.
  glCreateRenderbuffers(1, &target.z_buffer);
  glNamedRenderbufferStorage(target.z_buffer, GL_DEPTH_COMPONENT, width,
                             height);
  // Attach the RBO to FBO's depth attachment point.
  glNamedFramebufferRenderbuffer(target.frame_buffer, GL_DEPTH_ATTACHMENT,
                                 GL_RENDERBUFFER, target.z_buffer);

  // Check FBO status.
  GLenum status =
      glCheckNamedFramebufferStatus(target.frame_buffer, GL_FRAMEBUFFER);
  if (status != GL_FRAMEBUFFER_COMPLETE) {
    throw std::runtime_error("FBO creation failed.");
  }

  // Specify which buffer to be associated with the fragment shader output.
  GLenum buffer = GL_COLOR_ATTACHMENT0;
  glNamedFramebufferDrawBuffers(target.frame_buffer, 1, &buffer);

  return target;
}

Eigen::Matrix4f RenderEngineGl::ComputeGlProjectionMatrix(
    const CameraProperties& camera, double clip_near, double clip_far) const {
  DRAKE_ASSERT(clip_far > clip_near);
  const float inverse_frustum_depth = 1 / (clip_far - clip_near);
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
  const float A = -(clip_near + clip_far) * inverse_frustum_depth;
  const float B = -2.0f * clip_near * clip_far * inverse_frustum_depth;
  Eigen::Matrix4f P;
  // Eigen matrices are col-major, similar to OpenGL.
  // clang-format off
  P << fx, 0.0,  0.0, 0.0,
      0.0, fy,   0.0, 0.0,
      0.0, 0.0,    A,   B,
      0.0, 0.0, -1.0, 0.0;
  // clang-format on
  return P;
}

void RenderEngineGl::GetLabelImage(ImageLabel16I* label_image_out,
                                   const RenderTarget& target) const {
  ImageRgba8U image(label_image_out->width(), label_image_out->height());
  glGetTextureImage(target.value_texture, 0, GL_RGBA, GL_UNSIGNED_BYTE,
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

RenderTarget RenderEngineGl::GetRenderTarget(const CameraProperties& camera,
                                             RenderType render_type) const {
  const BufferDim dim{camera.width, camera.height};
  RenderTarget target;
  std::unordered_map<BufferDim, RenderTarget>& frame_buffers =
      frame_buffers_[render_type];
  auto iter = frame_buffers.find(dim);
  if (iter == frame_buffers.end()) {
    target = CreateRenderTarget(camera, render_type);
    frame_buffers.insert({dim, target});
  } else {
    target = iter->second;
  }
  glBindFramebuffer(GL_FRAMEBUFFER, target.frame_buffer);
  glViewport(0, 0, camera.width, camera.height);
  return target;
}

OpenGlGeometry RenderEngineGl::CreateGlGeometry(const MeshData& mesh_data) {
  OpenGlGeometry geometry;
  // Create the vertex array object (VAO).
  glCreateVertexArrays(1, &geometry.vertex_array);

  const auto& vertices = mesh_data.positions;
  const auto& indices = mesh_data.indices;

  // Create the vertex buffer object (VBO).
  glCreateBuffers(1, &geometry.vertex_buffer);
  glNamedBufferStorage(geometry.vertex_buffer,
                       vertices.size() * sizeof(GLfloat), vertices.data(), 0);
  // Bind the VBO with the VAO.
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

  // Create the index buffer object (IBO).
  glCreateBuffers(1, &geometry.index_buffer);
  glNamedBufferStorage(geometry.index_buffer, indices.size() * sizeof(GLuint),
                       indices.data(), 0);
  // Bind IBO with the VAO.
  glVertexArrayElementBuffer(geometry.vertex_array, geometry.index_buffer);

  geometry.index_buffer_size = indices.size();

  // Note: We won't need to call the corresponding glDeleteVertexArrays or
  // glDeleteBuffers. The meshes we store are "canonical" meshes. Even if a
  // particular GeometryId is removed, it was only referencing its corresponding
  // canonical mesh. We keep all canonical meshes alive for the lifetime of the
  // OpenGL context for convenient reuse.
  return geometry;
}

void RenderEngineGl::SetWindowVisibility(const CameraProperties& camera,
                                         bool show_window,
                                         const RenderTarget& target) const {
  if (show_window) {
    // Use the render target buffer as the read buffer and the default buffer
    // (0) as the draw buffer for displaying in the window. We transfer the full
    // image from source to destination. The semantics of glBlitNamedFrameBuffer
    // are inclusive of the "minimum" pixel (0, 0) and exclusive of the
    // "maximum" pixel (width, height).
    opengl_context_->DisplayWindow(camera.width, camera.height);
    glBlitNamedFramebuffer(target.frame_buffer, 0,
                           0, 0, camera.width, camera.height,  // Src bounds.
                           0, 0, camera.width, camera.height,  // Dest bounds.
                           GL_COLOR_BUFFER_BIT, GL_NEAREST);
    opengl_context_->UpdateWindow();
  } else {
    opengl_context_->HideWindow();
  }
}

ShaderId RenderEngineGl::AddShader(std::unique_ptr<ShaderProgram> program,
                                   internal::RenderType render_type) {
  const ShaderId shader_id = program->shader_id();
  shader_families_[render_type].insert({shader_id, vector<GeometryId>()});
  shader_programs_[render_type][shader_id] = move(program);
  return shader_id;
}

ShaderProgramData RenderEngineGl::GetShaderProgram(
    const PerceptionProperties& properties, RenderType render_type) const {
  std::optional<ShaderProgramData> data{std::nullopt};
  for (const auto& id_shader_pair : shader_programs_[render_type]) {
    const ShaderProgram& program = *(id_shader_pair.second);
    std::optional<ShaderProgramData> candidate_data =
        program.CreateProgramData(properties);
    if (candidate_data.has_value()) {
      if (data.has_value()) {
        if (candidate_data->shader_id() < data->shader_id()) continue;
      }
      data = move(candidate_data);
    }
  }
  // There should always be, at least, the default shader that accepts the
  // geometry.
  DRAKE_DEMAND(data.has_value());
  return *data;
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
