#include "drake/geometry/render_gl/internal_render_engine_gl.h"

#include <algorithm>
#include <filesystem>
#include <optional>
#include <unordered_set>
#include <utility>

#include <fmt/format.h>
#include <tiny_gltf.h>

#include "drake/common/diagnostic_policy.h"
#include "drake/common/drake_assert.h"
#include "drake/common/overloaded.h"
#include "drake/common/pointer_cast.h"
#include "drake/common/scope_exit.h"
#include "drake/common/string_map.h"
#include "drake/common/text_logging.h"
#include "drake/common/unused.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/proximity/polygon_to_triangle_mesh.h"

namespace drake {
namespace geometry {
namespace render_gl {
namespace internal {

using Eigen::Matrix3f;
using Eigen::Matrix4d;
using Eigen::Matrix4f;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector3f;
using geometry::internal::LoadRenderMeshesFromObj;
using geometry::internal::MakeDiffuseMaterial;
using geometry::internal::MaybeMakeMeshFallbackMaterial;
using geometry::internal::RenderMaterial;
using geometry::internal::RenderMesh;
using geometry::internal::TextureKey;
using geometry::internal::TextureSource;
using geometry::internal::UvState;
using math::RigidTransformd;
using math::RotationMatrixd;
using render::ColorRenderCamera;
using render::DepthRenderCamera;
using render::LightParameter;
using render::RenderCameraCore;
using render::RenderEngine;
using render::RenderLabel;
using std::make_shared;
using std::make_unique;
using std::map;
using std::set;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::unordered_map;
using std::vector;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;
using systems::sensors::ImageTraits;
using systems::sensors::PixelType;

namespace {

namespace fs = std::filesystem;

// A shader program that handles lighting computations. All shaders for color
// images should derive from *this* class. Depth and label do not need lighting.
class LightingShader : public ShaderProgram {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LightingShader);
  LightingShader() : ShaderProgram() {}

  void SetAllLights(const std::vector<LightParameter>& lights) const {
    DRAKE_DEMAND(lights.size() <= kMaxNumLights);
    for (int i = 0; i < ssize(lights); ++i) {
      SetLightParameters(i, lights[i]);
    }
    // Set the remaining lights off (invalid light type 0).
    for (int i = ssize(lights); i < kMaxNumLights; ++i) {
      glUniform1i(GetLightFieldLocation(i, "type"), 0);
    }
  }

  static constexpr int kMaxNumLights{5};

 protected:
  // Derived classes have the chance to configure additional uniforms.
  virtual void DoConfigureMoreUniforms() {}

  // This provides GLSL code necessary for performing lighting calculations:
  //   - Transforms the vertex into device *and* world coordinates.
  //   - Transforms the normal into world coordinates to be interpolated
  //     across the triangle (for lighting calculations).
  // Derived classes are responsible for introducing their own inputs, uniforms
  // (etc.) and defining the main() function. That main function should do
  // whatever work is unique to the shader and invoke PrepareLighting() so that
  // the transformed vertex is evaluated.
  static constexpr char kVertexShader[] = R"""(
#version 330
layout(location = 0) in vec3 p_MV;
layout(location = 1) in vec3 n_M;
uniform mat4 T_CM;  // The "model view matrix" (in OpenGl terms).
uniform mat4 T_DC;  // The "projection matrix" (in OpenGl terms).
uniform mat4 T_WM;  // The pose of the geometry (model) in the world.
uniform mat3 T_WM_normals;  // Rotation * inverse_scale to transform normals.
// TODO(SeanCurtis-TRI): Rather than propagating normal and position vertex in
// the *world* frame, compute them in camera frame. It saves one transform per
// vertex (for which there are a lot) and replaces it with CPU-side
// transformations of the lights into the camera frame. It also reduces the
// number of uniforms; T_WM is no longer necessary.
out vec3 n_W;
out vec3 p_WV; // Vertex position in world space.

void PrepareLighting() {
  // gl_Position is p_DV; the vertex position in device coordinates.
  gl_Position = T_DC * T_CM * vec4(p_MV, 1);

  n_W = normalize(T_WM_normals * n_M);
  p_WV = (T_WM * vec4(p_MV, 1)).xyz;
}
)""";

  // This provides GLSL code necessary for performing lighting calculations.
  // There is no main() method. Derived classes are responsible for introducing
  // their own inputs, uniforms (etc.) and defining the main() function. The
  // main function should compute the diffuse value at the fragment and call
  // GetIlluminatedColor() to get the illuminated result.
  static constexpr char kFragmentShader[] = R"""(
#version 330
uniform mat4 X_WC;  // Transform light position from camera to world.
in vec3 n_W;
in vec3 p_WV;

// TODO(SeanCurtis-TRI): Rather than hard-code this in this compile-time string,
// set this to the actual number of lights reported. We can still have the
// render engine subject to a hard light limit, but we can make sure the shader
// only has defined lights. This should roll into changes in how derived
// classes access these GLSL functions.
const int MAX_LIGHT_NUM = 5;

// TODO(SeanCurtis-TRI): We should packing these uniforms more tightly. vec3s
//   are stored as vec4 anyways, so we might as well reduce the uniform calls
//   and squeeze the size.
//
//   Type is the fourth field of light color.
//   intensity is the fourth field of atten_coeff.
//   cos_half_angle is the fourth field of direction.
struct Light {
    // 0 for no light
    // 1 for Point Light
    // 2 for Spot Light
    // 3 for Directional Light
    int type;
    vec3 color;
    // Only used for Point and Spot lights. position.xyz expresses the position
    // of the light in *some* frame. The fourth value determines the frame:
    // 0 := p_WL, 1 := p_CL.
    vec4 position;
    // Attenuation Coefficients (Constant, Linear, Quadratic),
    // Only used for Point and Spot lights
    vec3 atten_coeff;
    float intensity;
    // Ony used for Spot Lights
    float cos_half_angle;
    // Used for Spot lights and directional lights. dir expresses the
    // direction the light is pointing in *some* frame. position.w determines
    // the frame: 0 := dir_WL, 1 := dir_CL.
    vec3 dir;
};

uniform Light lights[MAX_LIGHT_NUM];

vec3 GetLightPositionInWorld(Light light) {
  vec3 v_W = light.position.xyz;  // Interpreting v as v_W.
  if (light.position.w == 1) {
    v_W = (X_WC * vec4(light.position.xyz, 1.0)).xyz;  // Interpreting v as v_C.
  }
  return v_W;
}

vec3 GetLightDirectionInWorld(Light light) {
  vec3 v_W = light.dir;  // Interpreting v as v_W.
  if (light.position.w == 1) {
    v_W = mat3(X_WC) * light.dir;  // Interpreting v as v_C.
  }
  return v_W;
}

float GetPointExposure(Light light, vec3 dir_FL_W, vec3 nhat_W) {
  return max(dot(nhat_W, dir_FL_W), 0.0);
}

float GetSpotExposure(Light light, vec3 dir_FL_W, vec3 nhat_W) {
  // TODO: Add a penumbra to the light.
  vec3 dir_L_W = GetLightDirectionInWorld(light);
  // If the angle θ between the light vector and the direction from fragment
  // to light is greater than the light's half cone angle θₗ it is not
  // illuminated. Alternatively, no light if cos(θ) < cos(θₗ).
  float cos_theta = max(dot(dir_FL_W, -dir_L_W), 0.0);
  if (cos_theta < light.cos_half_angle) {
      return 0.0;
  }
  return GetPointExposure(light, dir_FL_W, nhat_W);
}

float GetDirectionalExposure(Light light, vec3 nhat_W) {
  vec3 dir_L_W = GetLightDirectionInWorld(light);
  return max(dot(nhat_W, normalize(-dir_L_W)), 0.0);
}

vec3 GetLightIllumination(Light light, vec3 nhat_W) {
  // Position vector from fragment to light.
  vec3 p_WL = GetLightPositionInWorld(light);
  // p_WV is interpolated to be p_WF (position of the fragment).
  vec3 p_FL_W = p_WL - p_WV;
  float dist_FL = length(p_FL_W);
  vec3 dir_FL_W = vec3(0, 0, 0);
  if (dist_FL > 0) {
    dir_FL_W = p_FL_W / dist_FL;
  }

  // "Exposure" is the fraction of the light's full luminance that shines on
  // the given fragment.
  float exposure;
  if (light.type == 1) {
    exposure = GetPointExposure(light, dir_FL_W, nhat_W);
  } else if (light.type == 2) {
    exposure = GetSpotExposure(light, dir_FL_W, nhat_W);
  } else if (light.type == 3) {
    exposure = GetDirectionalExposure(light, nhat_W);
  } else {
      // Invalid light; no exposure.
      return vec3(0.0, 0.0, 0.0);
  }

  // Attenuation.
  float inv_attenuation = light.atten_coeff[0] +
                          (light.atten_coeff[1] +
                           light.atten_coeff[2] * dist_FL) * dist_FL;

  return light.color * exposure * light.intensity / inv_attenuation;
}

vec4 GetIlluminatedColor(vec4 diffuse) {
  // NOTE: Depending on triangle size and variance of normal direction over
  // that triangle, n_W may not be unit length; to play it safe, we blindly
  // normalize it. Consider *not* normalizing it if it improves performance
  // without degrading visual quality.
  vec3 nhat_W = normalize(n_W);

  vec3 illum = vec3(0.0, 0.0, 0.0);
  for (int i = 0; i < MAX_LIGHT_NUM; i++) {
    illum += GetLightIllumination(lights[i], nhat_W);
  }
  return vec4(illum * diffuse.rgb, diffuse.a);
}

)""";

 private:
  GLint GetLightFieldLocation(int index, std::string field_name) const {
    DRAKE_ASSERT(index >= 0 && index < kMaxNumLights);
    return GetUniformLocation(fmt::format("lights[{}].{}", index, field_name));
  }

  void DoConfigureUniforms() final {
    T_WM_normals_loc_ = GetUniformLocation("T_WM_normals");
    T_WM_loc_ = GetUniformLocation("T_WM");
    X_WC_loc_ = GetUniformLocation("X_WC");
    DoConfigureMoreUniforms();
  }

  void DoSetModelViewMatrix(const Matrix4f& X_CW, const Matrix4f& T_WM,
                            const Matrix3f& N_WM) const override {
    glUniformMatrix3fv(T_WM_normals_loc_, 1, GL_FALSE, N_WM.data());
    glUniformMatrix4fv(T_WM_loc_, 1, GL_FALSE, T_WM.data());

    Matrix4f X_WC = Eigen::Matrix4f::Identity();
    X_WC.block<3, 3>(0, 0) = X_CW.block<3, 3>(0, 0).transpose();
    X_WC.block<3, 1>(0, 3) = X_WC.block<3, 3>(0, 0) * (-X_CW.block<3, 1>(0, 3));
    glUniformMatrix4fv(X_WC_loc_, 1, GL_FALSE, X_WC.data());
  }

  void SetLightParameters(int index, const LightParameter& light) const {
    glUniform1i(GetLightFieldLocation(index, "type"),
                static_cast<int>(render::light_type_from_string(light.type)));
    Vector3f color = light.color.rgba().head<3>().cast<float>();
    glUniform3fv(GetLightFieldLocation(index, "color"), 1, color.data());
    Eigen::Vector4f position;
    position.head<3>() = light.position.cast<float>();
    const render::LightFrame frame =
        render::light_frame_from_string(light.frame);
    position(3) = frame == render::LightFrame::kWorld ? 0.0f : 1.0f;
    glUniform4fv(GetLightFieldLocation(index, "position"), 1, position.data());
    Vector3f atten_coeff = light.attenuation_values.cast<float>();
    glUniform3fv(GetLightFieldLocation(index, "atten_coeff"), 1,
                 atten_coeff.data());
    glUniform1f(GetLightFieldLocation(index, "intensity"),
                static_cast<float>(light.intensity));

    if (light.type == "spot") {
      // Note: Using the cosine here to speed up the shader so it doesn't have
      // to use cos or acos internally.
      glUniform1f(GetLightFieldLocation(index, "cos_half_angle"),
                  static_cast<float>(cos(light.cone_angle * (M_PI / 180.0))));
    }

    if (light.type != "point") {
      Vector3f direction = light.direction.cast<float>();
      glUniform3fv(GetLightFieldLocation(index, "dir"), 1, direction.data());
    }
  }

  // The location of the "T_WM_normals" uniform in the shader. This transforms
  // the *normals* to the world frame.
  GLint T_WM_normals_loc_{};

  // The location of the "T_WM" uniform in the shader.
  GLint T_WM_loc_{};

  // The transform between world and camera frame (used for transforming light
  // positions defined in the camera frame).
  GLint X_WC_loc_{};
};

/* The built-in shader for Rgba diffuse colored objects. This shader supports
 all geometries because it provides a default diffuse color if none is given. */
class DefaultRgbaColorShader final : public LightingShader {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DefaultRgbaColorShader);

  explicit DefaultRgbaColorShader(const Rgba& default_diffuse)
      : LightingShader(), default_diffuse_(default_diffuse) {
    // TODO(SeanCurtis-TRI): See if I can't come up with a more elegant way for
    // derived classes to exercise LightShader's GLSL functionality.
    LoadFromSources(
        fmt::format("{}{}", LightingShader::kVertexShader, kVertexShader),
        fmt::format("{}{}", LightingShader::kFragmentShader, kFragmentShader));
  }

  void SetInstanceParameters(const ShaderProgramData& data) const final {
    glUniform4fv(diffuse_color_loc_, 1,
                 data.value().get_value<Vector4<float>>().data());
  }

 private:
  void DoConfigureMoreUniforms() final {
    diffuse_color_loc_ = GetUniformLocation("diffuse_color");
  }

  std::unique_ptr<ShaderProgram> DoClone() const final {
    return make_unique<DefaultRgbaColorShader>(*this);
  }

  std::optional<ShaderProgramData> DoCreateProgramData(
      const PerceptionProperties& properties) const final {
    const Rgba diffuse =
        properties.GetPropertyOrDefault("phong", "diffuse", default_diffuse_);
    const Vector4<float> v4 = diffuse.rgba().template cast<float>();
    return ShaderProgramData{shader_id(), AbstractValue::Make(v4)};
  }

  // The default diffuse value to apply if missing the ("phong", "diffuse")
  // property.
  Rgba default_diffuse_;

  // The location of the "diffuse_color" uniform in the shader.
  GLint diffuse_color_loc_{};

  // For diffuse color, we only need to transform the vertex data and use the
  // diffuse color in the fragment shader. So, we'll simply invoke the lighting
  // function.
  static constexpr char kVertexShader[] = R"""(
void main() {
  PrepareLighting();
})""";

  // Simply illuminate the diffuse color at the fragment and output it.
  static constexpr char kFragmentShader[] = R"""(
uniform vec4 diffuse_color;
out vec4 color;

void main() {
  color = GetIlluminatedColor(diffuse_color);
})""";
};

/* The built-in shader for texture diffuse colored objects. This shader supports
 all geometries with a ("phong", "diffuse_map") property. */
class DefaultTextureColorShader final : public LightingShader {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DefaultTextureColorShader);

  /* Constructs the texture shader with the given library. The library will be
   used to access OpenGl textures.

   When the RenderEngineGl is cloned, instances of this shader program are
   likewise cloned, each with a shared ptr to the *same* texture library. This
   is alright, because the owning RenderEngineGl instances share that library
   as well -- so the shader program instances are consistent with the render
   engine instances. */
  explicit DefaultTextureColorShader(shared_ptr<TextureLibrary> library)
      : LightingShader(), library_(std::move(library)) {
    DRAKE_DEMAND(library_ != nullptr);
    LoadFromSources(
        fmt::format("{}{}", LightingShader::kVertexShader, kVertexShader),
        fmt::format("{}{}", LightingShader::kFragmentShader, kFragmentShader));
  }

  void SetInstanceParameters(const ShaderProgramData& data) const final {
    glActiveTexture(GL_TEXTURE0);
    glUniform1i(diffuse_map_loc_, 0);  // This texture is GL_TEXTURE0.
    const auto& my_data = data.value().get_value<InstanceData>();
    glBindTexture(GL_TEXTURE_2D, my_data.texture_id);
    glUniform2fv(diffuse_scale_loc_, 1, my_data.texture_scale.data());
    glUniform1i(texture_flip_loc_, my_data.flip_y);
  }

 private:
  void DoConfigureMoreUniforms() final {
    diffuse_map_loc_ = GetUniformLocation("diffuse_map");
    diffuse_scale_loc_ = GetUniformLocation("diffuse_map_scale");
    texture_flip_loc_ = GetUniformLocation("texture_flip_y");
  }

  std::unique_ptr<ShaderProgram> DoClone() const final {
    return make_unique<DefaultTextureColorShader>(*this);
  }

  struct InstanceData {
    GLuint texture_id{};
    Vector2<float> texture_scale;
    bool flip_y{};
  };

  std::optional<ShaderProgramData> DoCreateProgramData(
      const PerceptionProperties& properties) const final {
    if (!properties.HasProperty("phong", "diffuse_map")) return std::nullopt;

    const string& file_name =
        properties.GetProperty<string>("phong", "diffuse_map");
    std::optional<GLuint> texture_id = library_->GetTextureId(file_name);

    if (!texture_id.has_value()) return std::nullopt;

    // In constructing the material with a texture map, the UVs have already
    // been validated.

    const auto& scale = properties.GetPropertyOrDefault(
        "phong", "diffuse_scale", Vector2d(1, 1));
    const bool flip_y =
        properties.GetPropertyOrDefault("texture", "flip", false);
    return ShaderProgramData{shader_id(),
                             AbstractValue::Make(InstanceData{
                                 *texture_id, scale.cast<float>(), flip_y})};
  }

  std::shared_ptr<TextureLibrary> library_{};

  // The location of the "diffuse_map" uniform in the shader.
  GLint diffuse_map_loc_{};

  // The location of the "diffuse_scale" uniform in the shader.
  GLint diffuse_scale_loc_{};

  // The location of the "texture_flip_y" uniform in the shader.
  GLint texture_flip_loc_{};

  // For diffuse *map*, we need to propagate texture coordinates along with
  // transforming the vertex data. So, invoke the lighting function and output
  // texture coordinates.
  static constexpr char kVertexShader[] = R"""(
layout(location = 2) in vec2 tex_coord_in;
out vec2 tex_coord;
void main() {
  PrepareLighting();
  // TODO(SeanCurtis-TRI): Support transforms for texture coordinates.
  tex_coord = tex_coord_in;
})""";

  // We define the diffuse color by looking up the diffuse_map and then simply
  // illuminate it.
  static constexpr char kFragmentShader[] = R"""(
uniform sampler2D diffuse_map;
uniform vec2 diffuse_map_scale;
uniform bool texture_flip_y;
in vec2 tex_coord;
out vec4 color;

void main() {
  // Note: We're clipping the texture coordinates *here* using fract() rather
  //  than setting the texture to GL_REPEAT. Setting it GL_REPEAT can lead to
  //  unsightly visual artifacts when a texture is supposed to exactly align
  //  with a triangle edge, but there are floating point errors in interpolation
  //  which cause the texture to be sampled on the other side.
  // TODO(20234): To get parity with our other renderings, the diffuse *color*
  // should modulate the texture for the final diffuse color.
  vec2 uv = fract(tex_coord * diffuse_map_scale);
  if (texture_flip_y) {
    uv.y = 1.0 - uv.y;
  }
  vec4 map_rgba = texture(diffuse_map, uv);
  color = GetIlluminatedColor(map_rgba);
})""";
};

/* The built-in shader for objects in depth images. By default, the shader
 supports all geometries.  */
class DefaultDepthShader final : public ShaderProgram {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DefaultDepthShader);

  DefaultDepthShader() : ShaderProgram() {
    LoadFromSources(kVertexShader, kFragmentShader);
  }

  void SetDepthCameraParameters(const DepthRenderCamera& camera) const final {
    glUniform1f(depth_z_near_loc_, camera.depth_range().min_depth());
    glUniform1f(depth_z_far_loc_, camera.depth_range().max_depth());
  }

 private:
  void DoConfigureUniforms() final {
    depth_z_near_loc_ = GetUniformLocation("depth_z_near");
    depth_z_far_loc_ = GetUniformLocation("depth_z_far");
  }

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
uniform mat4 T_CM;  // The "model view matrix" (in OpenGl terms).
uniform mat4 T_DC;  // The "projection matrix" (in OpenGl terms).

void main() {
  vec4 p_CV = T_CM * vec4(p_MV, 1);
  depth = -p_CV.z;
  gl_Position = T_DC * p_CV;
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
class DefaultLabelShader final : public ShaderProgram {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DefaultLabelShader);

  /* Constructs the label shader with the given `label_encoder`.

   @param label_encoder  A function that extracts an encoded color (Vector4f)
                         from a set of PerceptionProperties representing the
                         color-encoded RenderLabel. If such a color can't be
                         defined, the function should throw.
  */
  explicit DefaultLabelShader(
      std::function<Vector4<float>(const PerceptionProperties&)> label_encoder)
      : ShaderProgram(), label_encoder_(std::move(label_encoder)) {
    LoadFromSources(kVertexShader, kFragmentShader);
  }

  void SetInstanceParameters(const ShaderProgramData& data) const final {
    glUniform4fv(encoded_label_loc_, 1,
                 data.value().get_value<Vector4<float>>().data());
  }

 private:
  void DoConfigureUniforms() final {
    encoded_label_loc_ = GetUniformLocation("encoded_label");
  }

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
uniform mat4 T_CM;  // The "model view matrix" (in OpenGl terms).
uniform mat4 T_DC;  // The "projection matrix" (in OpenGl terms).
void main() {
  // X_CM = T_CM (although X may not be a *rigid* transform).
  vec4 p_CV = T_CM * vec4(p_MV, 1);
  gl_Position = T_DC * p_CV;
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

// Given a filename (e.g., of a mesh), this produces a string that we use in
// our maps to guarantee we only load the file once.
std::string GetPathKey(const MeshSource& mesh_source, bool is_convex) {
  std::string prefix;
  if (mesh_source.is_in_memory()) {
    // TODO(SeanCurtis-TRI): This uses the sha of the core mesh file to identify
    // a unique mesh. However, there is a weird, adversarial case:
    //
    //  - User loads a geometry from foo.mesh that references foo.bin.
    //    - the data for foo.mesh and foo.bin gets processed and loaded into
    //      the cache.
    //  - The hash of foo.mesh serves as its unique key.
    //  - User then changes contents of foo.bin.
    //  - User loads another geometry with foo.mesh (which has now appreciably
    //    changed because foo.bin is different).
    //  - The cache will believe it is the same file as before, even though the
    //    geometry has changed and not load the new version, using the old
    //    instead.
    //
    // For example, if a user runs a simulation, rendering images out, resets
    // the simulation with changes to the foo.bin file (e.g. material properties
    // in a .mtl file), with the intent of creating a visual variant of the
    // previous simulation, the geometry in the second pass will not have
    // changed appearance.
    //
    // This problem applies to both on-disk and in-memory meshes. For on-disk
    // meshes, the problem is worse because it strictly uses the mesh file name
    // as cache id and therefore won't even detect changes in the core mesh file
    // (let alone any of the supporting files).
    //
    // To address this we'll have to have a key predicated on the contents on
    // the whole file *ecosystem* so that we can detect if any part of the file
    // family is unique, creating, in some sense, a unique geometry.
    //
    // The urgency is low for now because if someone is doing multiple passes
    // to create render variations, they're probably using the higher-fidelity
    // RenderEngineVtk. As we improve the fidelity of RenderEngineGl, we'll
    // want to shore up this hole.
    prefix = mesh_source.in_memory().mesh_file.sha256().to_string() +
             (is_convex ? "?convex" : "");
  } else {
    DRAKE_DEMAND(mesh_source.is_path());
    prefix = mesh_source.path().string();
    std::error_code path_error;
    const fs::path path = fs::canonical(mesh_source.path(), path_error);
    if (path_error) {
      throw std::runtime_error(
          fmt::format("RenderEngineGl: unable to access the file {}; {}",
                      prefix, path_error.message()));
    }
  }
  // Note: We're using "?". It isn't valid for filenames, so using it in the
  // key guarantees we won't collide with potential file names.
  return prefix + (is_convex ? "?convex" : "");
}

// We want to make sure the lights are as clean as possible. So, we'll
// re-normalize unit vectors (where possible). We're not testing for "bad"
// values because those values which *might* be considered "bad" can be used
// by users for debugging.
RenderEngineGlParams CleanupLights(RenderEngineGlParams params) {
  if (ssize(params.lights) > LightingShader::kMaxNumLights) {
    throw std::runtime_error(
        fmt::format("RenderEngineGl supports up to five lights; {} specified.",
                    ssize(params.lights)));
  }
  for (auto& light : params.lights) {
    if (light.type != "point") {
      const double dir_magnitude = light.direction.norm();
      if (dir_magnitude > 0) {
        // Zero vectors will remain zero, blacking the light out. But we want
        // all other vectors as close to unit length as possible.
        light.direction /= dir_magnitude;
      }
    }
  }
  return params;
}

}  // namespace

RenderEngineGl::RenderEngineGl(RenderEngineGlParams params)
    : RenderEngine(RenderLabel::kDontCare),
      opengl_context_(make_unique<OpenGlContext>()),
      texture_library_(make_shared<TextureLibrary>()),
      parameters_(CleanupLights(std::move(params))) {
  // The default light parameters have been crafted to create the default
  // "headlamp" camera.
  fallback_lights_.push_back({});
  // Configuration of basic OpenGl state.
  opengl_context_->MakeCurrent();

  InitGlState();

  // Color shaders. See documentation on GetShaderProgram. We want color from
  // texture to be "more preferred" than color from rgba, so we add the
  // texture color shader *after* the rgba color shader.
  AddShader(make_unique<DefaultRgbaColorShader>(params.default_diffuse),
            RenderType::kColor);
  AddShader(make_unique<DefaultTextureColorShader>(texture_library_),
            RenderType::kColor);
  ConfigureLights();

  // Depth shaders -- a single shader that accepts all geometry.
  AddShader(make_unique<DefaultDepthShader>(), RenderType::kDepth);

  // Label shaders -- a single shader that accepts all geometry (unless it has
  // an invalid RenderLabel -- see RenderEngine::GetLabelOrThrow).
  // Extracts the label from properties (with error checking) and returns the
  // r,g,b,a color to represent it.
  auto label_encoder = [this](const PerceptionProperties& props) {
    const RenderLabel& label = this->GetRenderLabelOrThrow(props);
    const Rgba color = RenderEngine::MakeRgbFromLabel(label);
    return Vector4<float>(color.r(), color.g(), color.b(), 1.0f);
  };
  AddShader(make_unique<DefaultLabelShader>(label_encoder), RenderType::kLabel);
}

// There are various per-RenderEngineGl-instance OpenGl objects created. These
// are enumerated in DoClone(): vertex array objects, ShaderPrograms, etc. They
// need to be deleted by hand because they require the context to be bound.
RenderEngineGl::~RenderEngineGl() {
  ScopeExit unbind([]() {
    OpenGlContext::ClearCurrent();
  });

  opengl_context_->MakeCurrent();

  // Delete vertex array objects.
  for (auto& geometry : geometries_) {
    glDeleteVertexArrays(1, &geometry.vertex_array);
  }

  // Delete programs.
  for (auto& shader_type : shader_programs_) {
    for (auto& [_, program_ptr] : shader_type) {
      program_ptr->Free();
    }
  }
}

void RenderEngineGl::UpdateViewpoint(const RigidTransformd& X_WR) {
  X_CW_ = X_WR.inverse();
}

void RenderEngineGl::ImplementGeometry(const Box& box, void* user_data) {
  const int geometry = GetBox();
  AddGeometryInstance(geometry, user_data,
                      Vector3d(box.width(), box.depth(), box.height()));
}

void RenderEngineGl::ImplementGeometry(const Capsule& capsule,
                                       void* user_data) {
  const int resolution = 50;
  RenderMesh render_mesh =
      MakeCapsule(resolution, capsule.radius(), capsule.length());

  const int geometry = CreateGlGeometry(render_mesh);

  AddGeometryInstance(geometry, user_data, Vector3d::Ones());
}

void RenderEngineGl::ImplementGeometry(const Convex& convex, void* user_data) {
  RegistrationData* data = static_cast<RegistrationData*>(user_data);
  CacheConvexHullMesh(convex, *data);
  // Note: CacheConvexHullMesh() either succeeds or throws.
  ImplementMeshesForSource(user_data, convex.scale3(), convex.source(),
                           /* is_convex=*/true);
}

void RenderEngineGl::ImplementGeometry(const Cylinder& cylinder,
                                       void* user_data) {
  const int geometry = GetCylinder();
  const double r = cylinder.radius();
  const double l = cylinder.length();
  AddGeometryInstance(geometry, user_data, Vector3d(r, r, l));
}

void RenderEngineGl::ImplementGeometry(const Ellipsoid& ellipsoid,
                                       void* user_data) {
  const int geometry = GetSphere();
  AddGeometryInstance(geometry, user_data,
                      Vector3d(ellipsoid.a(), ellipsoid.b(), ellipsoid.c()));
}

void RenderEngineGl::ImplementGeometry(const HalfSpace&, void* user_data) {
  const int geometry = GetHalfSpace();
  AddGeometryInstance(geometry, user_data, kUnitScale);
}

void RenderEngineGl::ImplementGeometry(const Mesh& mesh, void* user_data) {
  RegistrationData* data = static_cast<RegistrationData*>(user_data);
  CacheFileMeshesMaybe(mesh.source(), data);
  if (data->accepted) {
    ImplementMeshesForSource(user_data, mesh.scale3(), mesh.source(),
                             /* is_convex=*/false);
  }
}

void RenderEngineGl::ImplementGeometry(const Sphere& sphere, void* user_data) {
  const int geometry = GetSphere();
  const double r = sphere.radius();
  AddGeometryInstance(geometry, user_data, Vector3d(r, r, r));
}

void RenderEngineGl::InitGlState() {
  DRAKE_ASSERT(opengl_context_->IsCurrent());

  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  glClipControl(GL_UPPER_LEFT, GL_NEGATIVE_ONE_TO_ONE);
  glClearDepth(1.0);
  glEnable(GL_DEPTH_TEST);
  // Generally, there should be no blending for depth and label images. We'll
  // selectively enable blending for color images.
  glDisable(GL_BLEND);
  // We blend the rgb values (the first two parameters), but simply accumulate
  // transparency (the last two parameters).
  glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE);
}

void RenderEngineGl::ImplementMeshesForSource(void* user_data,
                                              const Vector3<double>& scale,
                                              const MeshSource& mesh_source,
                                              bool is_convex) {
  const std::string file_key = GetPathKey(mesh_source, is_convex);
  // If mesh_source is in memory, we want to pass an *empty* filename to
  // MaybeMakeMeshFallbackmaterial() to avoid looking for foo.png.
  const fs::path filename =
      mesh_source.is_path() ? mesh_source.path() : fs::path();
  for (const auto& gl_mesh : meshes_.at(file_key)) {
    const RegistrationData& data = *static_cast<RegistrationData*>(user_data);
    PerceptionProperties temp_props(data.properties);

    RenderMaterial material;
    // If there is a material associated with the mesh, we will use it.
    // Otherwise, we recreate the fallback material based on user data and
    // defaults.
    if (gl_mesh.mesh_material.has_value()) {
      material = gl_mesh.mesh_material.value();
    } else {
      material = *MaybeMakeMeshFallbackMaterial(
          data.properties, filename, parameters_.default_diffuse,
          drake::internal::DiagnosticPolicy(), gl_mesh.uv_state);
    }
    if (!IsEmpty(material.diffuse_map)) {
      // By the time the render material gets here, it should either be a path
      // or it is a key into an in-memory image already registered with the
      // texture library. It should never be MemoryFile.
      std::visit(overloaded{[](const auto&) {
                              throw std::logic_error("Should be path or key.");
                            },
                            [&temp_props](const fs::path& path) {
                              temp_props.UpdateProperty("phong", "diffuse_map",
                                                        path.string());
                            },
                            [&temp_props](const TextureKey& key) {
                              temp_props.UpdateProperty("phong", "diffuse_map",
                                                        key.value);
                            }},
                 material.diffuse_map);
    }
    temp_props.UpdateProperty("phong", "diffuse", material.diffuse);
    // Non-public property to communicate to the shaders.
    temp_props.UpdateProperty("texture", "flip", material.flip_y);
    RegistrationData temp_data{data.id, data.X_WG, temp_props,
                               data.default_diffuse};
    AddGeometryInstance(gl_mesh.mesh_index, &temp_data, scale);
  }
}

bool RenderEngineGl::DoRegisterVisual(GeometryId id, const Shape& shape,
                                      const PerceptionProperties& properties,
                                      const RigidTransformd& X_WG) {
  opengl_context_->MakeCurrent();
  RegistrationData data{.id = id,
                        .X_WG = RigidTransformd{X_WG},
                        .properties = properties,
                        .default_diffuse = parameters_.default_diffuse};
  shape.Reify(this, &data);
  return data.accepted;
}

bool RenderEngineGl::DoRegisterDeformableVisual(
    GeometryId id, const std::vector<RenderMesh>& render_meshes,
    const PerceptionProperties& properties) {
  opengl_context_->MakeCurrent();
  std::vector<int> gl_mesh_indices;
  for (const auto& render_mesh : render_meshes) {
    const int mesh_index =
        CreateGlGeometry(render_mesh, /* is_deformable */ true);
    DRAKE_DEMAND(mesh_index >= 0);
    gl_mesh_indices.emplace_back(mesh_index);
    const RenderMaterial& material =
        render_mesh.material.has_value()
            ? *render_mesh.material
            : MakeDiffuseMaterial(parameters_.default_diffuse);
    PerceptionProperties mesh_properties(properties);
    // TODO(SeanCurtis-TRI): For now, we'll assume that deformable materials
    // must all come from disk. Later, we'll expand it to include in-memory. The
    // challenge is that we're using geometry properties as a middle man and
    // ("phong", "diffuse_map") should only contain a string containing a file
    // path. If the image is in memory, we need to do something else.
    const auto* path_ptr = std::get_if<fs::path>(&material.diffuse_map);
    if (path_ptr != nullptr) {
      mesh_properties.UpdateProperty("phong", "diffuse_map",
                                     path_ptr->string());
      // If empty, key, or file contents, we leave it alone and do nothing.
    }
    mesh_properties.UpdateProperty("phong", "diffuse", material.diffuse);
    RegistrationData data{id, RigidTransformd::Identity(), mesh_properties,
                          parameters_.default_diffuse};
    AddGeometryInstance(mesh_index, &data, kUnitScale);
  }
  deformable_meshes_.emplace(id, std::move(gl_mesh_indices));
  return true;
}

void RenderEngineGl::DoUpdateVisualPose(GeometryId id,
                                        const RigidTransformd& X_WG) {
  const Eigen::Matrix4f X_WG_f = X_WG.GetAsMatrix4().template cast<float>();
  const Eigen::Matrix3f R_WG_f =
      X_WG.rotation().matrix().template cast<float>();
  for (auto& instance : visuals_.at(id).instances) {
    instance.T_WN = X_WG_f * instance.T_GN;
    instance.N_WN = R_WG_f * instance.N_GN;
  }
}

void RenderEngineGl::DoUpdateDeformableConfigurations(
    GeometryId id, const std::vector<VectorX<double>>& q_WGs,
    const std::vector<VectorX<double>>& nhats_W) {
  DRAKE_DEMAND(deformable_meshes_.contains(id));
  std::vector<int>& gl_mesh_indices = deformable_meshes_.at(id);
  DRAKE_DEMAND(q_WGs.size() == gl_mesh_indices.size());

  for (int i = 0; i < ssize(q_WGs); ++i) {
    const VectorX<GLfloat> q_WG = q_WGs[i].cast<GLfloat>();
    const VectorX<GLfloat> nhat_W = nhats_W[i].cast<GLfloat>();
    // Find the OpenGL geometry.
    const int geometry_index = gl_mesh_indices[i];
    DRAKE_DEMAND(0 <= geometry_index && geometry_index < ssize(geometries_));
    OpenGlGeometry& geometry = geometries_[geometry_index];
    // Update vertex position data.
    std::size_t positions_offset = 0;
    glNamedBufferSubData(geometry.vertex_buffer,
                         positions_offset * sizeof(GLfloat),
                         q_WG.size() * sizeof(GLfloat), q_WG.data());
    // Update vertex normal data.
    std::size_t normals_offset = q_WG.size();
    glNamedBufferSubData(geometry.vertex_buffer,
                         normals_offset * sizeof(GLfloat),
                         nhat_W.size() * sizeof(GLfloat), nhat_W.data());
  }
}

bool RenderEngineGl::DoRemoveGeometry(GeometryId id) {
  // Clean up the convenience look up table for deformable if the id is
  // associated with a deformable geometry.
  if (deformable_meshes_.contains(id)) {
    deformable_meshes_.erase(id);
  }
  // Now remove the instances associated with the id (stored in visuals_).
  auto iter = visuals_.find(id);
  if (iter != visuals_.end()) {
    // Multiple instances may have the same shader. We don't want to attempt
    // removing the geometry id from the corresponding family redundantly.
    std::unordered_set<ShaderId> visited_families;
    // Remove from the shader families to which it belongs!
    auto maybe_remove_from_family =
        [this, &visited_families](GeometryId g_id, const auto& shader_data,
                                  RenderType render_type) {
          const ShaderId s_id = shader_data[render_type].shader_id();
          if (visited_families.contains(s_id)) {
            return;
          }
          visited_families.insert(s_id);
          auto& geometries = shader_families_[render_type].at(s_id);
          auto num_removed = geometries.erase(g_id);
          DRAKE_DEMAND(num_removed == 1);
        };
    for (const auto& instance : iter->second.instances) {
      maybe_remove_from_family(id, instance.shader_data, RenderType::kColor);
      maybe_remove_from_family(id, instance.shader_data, RenderType::kDepth);
      maybe_remove_from_family(id, instance.shader_data, RenderType::kLabel);
    }
    visuals_.erase(iter);
    return true;
  }
  return false;
}

unique_ptr<RenderEngine> RenderEngineGl::DoClone() const {
  // The clone still requires some last-minute patching before it can work
  // correctly.
  auto clone = unique_ptr<RenderEngineGl>(new RenderEngineGl(*this));

  ScopeExit unbind([]() {
    OpenGlContext::ClearCurrent();
  });
  clone->opengl_context_->MakeCurrent();

  // Note: changes in graphics drivers have led to artifacts where frame buffers
  // are not necessarily shared across contexts. So, to be safe, we'll clear
  // the clone's frame buffers (it will recreate them as needed). This should
  // also benefit parallel rendering as two clones will no longer attempt to
  // render to the same render targets.
  for (auto& buffer : clone->frame_buffers_) {
    buffer.clear();
  }

  clone->InitGlState();

  // Update the vertex array objects on the shared vertex buffers.
  clone->UpdateVertexArrays();

  // We need to separate the ShaderProgram uniform namespaces so that setting
  // a uniform value in one thread doesn't affect the others. This uses the
  // inelegant expedient of creating a *new* shader program (in the OpenGl
  // sense) using the same compiled shaders as the original. If the OpenGl
  // context were bound during duplication, this could be done as part of the
  // copying of a ShaderProgram. For now, it has to be done as clean up here.
  for (auto& shader_type : clone->shader_programs_) {
    for (auto& [_, program_ptr] : shader_type) {
      program_ptr->Relink();
    }
  }

  // Update the shader OpenGL state to properly configure the lighting.
  clone->ConfigureLights();

  return clone;
}

void RenderEngineGl::RenderAt(const ShaderProgram& shader_program,
                              RenderType render_type) const {
  const Matrix4f& X_CW = X_CW_.GetAsMatrix4().matrix().cast<float>();
  // We rely on the calling method to clear all appropriate buffers; this method
  // may be called multiple times per image (based on the number of shaders
  // being used) and, therefore, can't do the clearing itself.

  for (const GeometryId& g_id :
       shader_families_.at(render_type).at(shader_program.shader_id())) {
    for (const auto& instance : visuals_.at(g_id).instances) {
      if (instance.shader_data.at(render_type).shader_id() !=
          shader_program.shader_id()) {
        continue;
      }
      const OpenGlGeometry& geometry = geometries_[instance.geometry];
      glBindVertexArray(geometry.vertex_array);

      shader_program.SetInstanceParameters(instance.shader_data[render_type]);
      shader_program.SetModelViewMatrix(X_CW, instance.T_WN, instance.N_WN);

      glDrawElements(geometry.mode, geometry.index_count, geometry.type, 0);
    }
  }
  // Unbind the vertex array back to the default of 0.
  glBindVertexArray(0);
}

void RenderEngineGl::DoRenderColorImage(const ColorRenderCamera& camera,
                                        ImageRgba8U* color_image_out) const {
  opengl_context_->MakeCurrent();
  // TODO(SeanCurtis-TRI): For transparency to work properly, I need to
  //  segregate objects with transparency from those without. The transparent
  //  geometries then need to be sorted from farthest to nearest the camera and
  //  rendered in that order. This may lead to shader thrashing. Without this
  //  ordering, I may not necessarily see objects through transparent surfaces.
  //  Confirm that VTK handles transparency correctly and do the same.

  const RenderTarget render_target =
      GetRenderTarget(camera.core(), RenderType::kColor);
  const Vector4<float> clear_color =
      parameters_.default_clear_color.rgba().cast<float>();
  glClearNamedFramebufferfv(render_target.frame_buffer, GL_COLOR, 0,
                            clear_color.data());
  glClear(GL_DEPTH_BUFFER_BIT);
  // We only want blending for color; not for label or depth.
  glEnable(GL_BLEND);

  // Matrix mapping a geometry vertex from the camera frame C to the device
  // frame D.
  const Matrix4f T_DC = camera.core().CalcProjectionMatrix().cast<float>();

  for (const auto& [_, shader_program] : shader_programs_[RenderType::kColor]) {
    shader_program->Use();
    shader_program->SetProjectionMatrix(T_DC);
    RenderAt(*shader_program, RenderType::kColor);
    shader_program->Unuse();
  }
  glDisable(GL_BLEND);

  // Note: SetWindowVisibility must be called *after* the rendering; setting the
  // visibility is responsible for taking the target buffer and bringing it to
  // the front buffer; reversing the order means the image we've just rendered
  // wouldn't be visible.
  SetWindowVisibility(camera.core(), camera.show_window(), render_target);
  glGetTextureImage(render_target.value_texture, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                    color_image_out->size(), color_image_out->at(0, 0));
}

void RenderEngineGl::DoRenderDepthImage(const DepthRenderCamera& camera,
                                        ImageDepth32F* depth_image_out) const {
  opengl_context_->MakeCurrent();

  const RenderTarget render_target =
      GetRenderTarget(camera.core(), RenderType::kDepth);

  // We initialize the color buffer to be all "too far" values. This is the
  // pixel value if nothing draws there -- i.e., nothing there implies that
  // whatever *might* be there is "too far" beyond the depth range.
  glClearNamedFramebufferfv(render_target.frame_buffer, GL_COLOR, 0,
                            &ImageTraits<PixelType::kDepth32F>::kTooFar);
  glClear(GL_DEPTH_BUFFER_BIT);

  // Matrix mapping a geometry vertex from the camera frame C to the device
  // frame D.
  const Matrix4f T_DC = camera.core().CalcProjectionMatrix().cast<float>();

  for (const auto& [_, shader_ptr] : shader_programs_[RenderType::kDepth]) {
    const ShaderProgram& shader_program = *shader_ptr;
    shader_program.Use();

    shader_program.SetProjectionMatrix(T_DC);
    shader_program.SetDepthCameraParameters(camera);
    RenderAt(shader_program, RenderType::kDepth);

    shader_program.Unuse();
  }

  glGetTextureImage(render_target.value_texture, 0, GL_RED, GL_FLOAT,
                    depth_image_out->size() * sizeof(GLfloat),
                    depth_image_out->at(0, 0));
}

void RenderEngineGl::DoRenderLabelImage(const ColorRenderCamera& camera,
                                        ImageLabel16I* label_image_out) const {
  opengl_context_->MakeCurrent();

  const RenderTarget render_target =
      GetRenderTarget(camera.core(), RenderType::kLabel);
  // TODO(SeanCurtis-TRI) Consider converting Rgba to float[4] as a member.
  const Rgba empty_color = RenderEngine::MakeRgbFromLabel(RenderLabel::kEmpty);
  float clear_color[4] = {static_cast<float>(empty_color.r()),
                          static_cast<float>(empty_color.g()),
                          static_cast<float>(empty_color.b()), 1.0f};
  glClearNamedFramebufferfv(render_target.frame_buffer, GL_COLOR, 0,
                            &clear_color[0]);
  glClear(GL_DEPTH_BUFFER_BIT);

  // Matrix mapping a geometry vertex from the camera frame C to the device
  // frame D.
  const Matrix4f T_DC = camera.core().CalcProjectionMatrix().cast<float>();

  for (const auto& [_, shader_ptr] : shader_programs_[RenderType::kLabel]) {
    const ShaderProgram& shader_program = *shader_ptr;
    shader_program.Use();

    shader_program.SetProjectionMatrix(T_DC);
    RenderAt(shader_program, RenderType::kLabel);

    shader_program.Unuse();
  }

  // Note: SetWindowVisibility must be called *after* the rendering; setting the
  // visibility is responsible for taking the target buffer and bringing it to
  // the front buffer; reversing the order means the image we've just rendered
  // wouldn't be visible.
  SetWindowVisibility(camera.core(), camera.show_window(), render_target);
  // TODO(SeanCurtis-TRI): Apparently, we *should* be able to create a frame
  // buffer texture consisting of a single-channel, 16-bit, signed int (to match
  // the underlying RenderLabel value). Doing so would allow us to render labels
  // directly and eliminate this additional pass.
  GetLabelImage(label_image_out, render_target);
}

std::string RenderEngineGl::DoGetParameterYaml() const {
  return yaml::SaveYamlString(parameters_, "RenderEngineGlParams");
}

void RenderEngineGl::AddGeometryInstance(int geometry_index, void* user_data,
                                         const Vector3d& scale) {
  const RegistrationData& data = *static_cast<RegistrationData*>(user_data);
  std::optional<ShaderProgramData> color_data =
      GetShaderProgram(data.properties, RenderType::kColor);
  std::optional<ShaderProgramData> depth_data =
      GetShaderProgram(data.properties, RenderType::kDepth);
  std::optional<ShaderProgramData> label_data =
      GetShaderProgram(data.properties, RenderType::kLabel);
  DRAKE_DEMAND(color_data.has_value() && depth_data.has_value() &&
               label_data.has_value());

  visuals_[data.id].instances.push_back(
      {geometry_index, scale.cast<float>(), geometries_.at(geometry_index),
       *color_data, *depth_data, *label_data});

  // For anchored geometry, we need to make sure the instance's values for
  // T_WN and N_WN are initialized based on the initial pose, X_WG.
  DoUpdateVisualPose(data.id, data.X_WG);

  shader_families_[RenderType::kColor][color_data->shader_id()].insert(data.id);
  shader_families_[RenderType::kDepth][depth_data->shader_id()].insert(data.id);
  shader_families_[RenderType::kLabel][label_data->shader_id()].insert(data.id);
}

int RenderEngineGl::GetSphere() {
  if (sphere_ < 0) {
    const int kLatitudeBands = 50;
    const int kLongitudeBands = 50;

    RenderMesh render_mesh =
        MakeLongLatUnitSphere(kLongitudeBands, kLatitudeBands);

    sphere_ = CreateGlGeometry(render_mesh);
  }

  geometries_[sphere_].throw_if_undefined(
      "Built-in sphere has some invalid objects");

  return sphere_;
}

int RenderEngineGl::GetCylinder() {
  if (cylinder_ < 0) {
    const int kLongitudeBands = 50;

    // For long skinny cylinders, it would be better to offer some subdivisions
    // along the length. For now, we'll simply save the triangles.
    RenderMesh render_mesh = MakeUnitCylinder(kLongitudeBands, 1);
    cylinder_ = CreateGlGeometry(render_mesh);
  }

  geometries_[cylinder_].throw_if_undefined(
      "Built-in cylinder has some invalid objects");

  return cylinder_;
}

int RenderEngineGl::GetHalfSpace() {
  if (half_space_ < 0) {
    // This matches the RenderEngineVtk half space size. Keep them matching
    // so that the common "horizon" unit test passes.
    const GLfloat kMeasure = 100.f;
    // TODO(SeanCurtis-TRI): For vertex-lighting (as opposed to fragment
    //  lighting), this will render better with tighter resolution. Consider
    //  making this configurable.
    RenderMesh render_mesh = MakeSquarePatch(kMeasure, 1);
    half_space_ = CreateGlGeometry(render_mesh);
  }

  geometries_[half_space_].throw_if_undefined(
      "Built-in half space has some invalid objects");

  return half_space_;
}

int RenderEngineGl::GetBox() {
  if (box_ < 0) {
    RenderMesh render_mesh = MakeUnitBox();
    box_ = CreateGlGeometry(render_mesh);
  }

  geometries_[box_].throw_if_undefined("Built-in box has some invalid objects");

  return box_;
}

void RenderEngineGl::CacheConvexHullMesh(const Convex& convex,
                                         const RegistrationData& data) {
  const std::string file_key = GetPathKey(convex.source(), /*is_convex=*/true);

  if (!meshes_.contains(file_key)) {
    const bool unscaled = (convex.scale3().array() == 1.0).all();
    // We store a hull of the mesh's *unscaled* vertices (applying a particular
    // instance's scale when rendering that instance).
    const TriangleSurfaceMesh<double> tri_hull =
        geometry::internal::MakeTriangleFromPolygonMesh(
            unscaled ? convex.GetConvexHull()
                     : Convex(convex.source()).GetConvexHull());
    RenderMesh render_mesh =
        geometry::internal::MakeFacetedRenderMeshFromTriangleSurfaceMesh(
            tri_hull, data.properties);
    // Fall back to the default diffuse material if and only if no material has
    // been assigned.
    if (!render_mesh.material.has_value()) {
      render_mesh.material = MakeDiffuseMaterial(parameters_.default_diffuse);
    }
    const int mesh_index = CreateGlGeometry(render_mesh);
    DRAKE_DEMAND(mesh_index >= 0);
    // Note: the material is left as std::nullopt, so that the instance of this
    // geometry must define its own material.
    meshes_[file_key] = vector<RenderGlMesh>{
        {.mesh_index = mesh_index, .uv_state = render_mesh.uv_state}};
  }
}

namespace {

/* Creates and configures the vertex array object for the given vertex data
 specification, assigning the resulting object to the given `geometry`.

 Note: if geometry->spec->uvs.components_per_element == 0, then the texture
 shader should never be assigned to the resulting OpenGlGeometry.

 @pre There is a valid OpenGL context bound.
 @pre geometry->vertex_buffer and geometry->index_buffer are names of valid
 buffers.
 @pre geometry->spec is properly configured. */
void CreateVertexArray(OpenGlGeometry* geometry) {
  glCreateVertexArrays(1, &geometry->vertex_array);

  auto init_array = [&g = *geometry](const VertexAttrib& props) {
    glVertexArrayVertexBuffer(g.vertex_array, props.attribute_index,
                              g.vertex_buffer, props.byte_offset, props.stride);
    glVertexArrayAttribFormat(g.vertex_array, props.attribute_index,
                              props.components_per_element, props.numeric_type,
                              GL_FALSE, 0);
    glEnableVertexArrayAttrib(g.vertex_array, props.attribute_index);
  };
  init_array(geometry->spec.positions);
  init_array(geometry->spec.normals);
  if (geometry->spec.uvs.components_per_element > 0) {
    init_array(geometry->spec.uvs);
  }

  // Bind index buffer object (IBO) with the vertex array object (VAO).
  glVertexArrayElementBuffer(geometry->vertex_array, geometry->index_buffer);
}

}  // namespace

/* Extracts mesh primitives from a glTF file. It places OpenGlGeometries in the
 provided mesh cache, registers any embedded textures into the given texture
 library, and, ultimately, produces RenderGlMeshes from the glTF contents.

 It is designed to be instantiated for each glTF file and all of the extracted
 contents should be associated with a single GeometryId. */
class RenderEngineGl::GltfMeshExtractor {
 public:
  /* Constructs the mesh extractor. During the extraction process, glTF
  primitives are turned into OpenGlGeometry instances, placed into the given
  `geometries`. Any embedded textures will be registered with the given
  `texture_library`. As such, both must remain alive and valid for the lifespan
  of the mesh extractor.

  The extractor does *not* automatically register the final RenderGlMesh
  instances. Those are created by calling ExtractMeshes() and the caller is
  responsible for taking ownership. */
  GltfMeshExtractor(const MeshSource* mesh_source,
                    std::vector<OpenGlGeometry>* geometries,
                    TextureLibrary* texture_library)
      : mesh_source_(DRAKE_DEREF(mesh_source)),
        description_(mesh_source_.is_path()
                         ? fmt::format("the on-disk glTF file: '{}'",
                                       mesh_source_.description())
                         : fmt::format("the in-memory glTF file: '{}'",
                                       mesh_source_.description())),
        geometries_(*geometries),
        texture_library_(*texture_library) {
    DRAKE_DEMAND(mesh_source != nullptr);
    DRAKE_DEMAND(geometries != nullptr);
    DRAKE_DEMAND(texture_library != nullptr);
  }

  /* Extracts the geometry data (meshes and accompanying data) from the named
   glTF file. While OpenGlGeometries and texture images will be automatically
   registered with the containers passed into the constructor, the caller is
   responsible for taking ownership of the collection of RenderGlMesh
   instances returned. */
  vector<RenderGlMesh> ExtractMeshes(const RegistrationData& data) {
    std::string embedded_prefix;
    if (mesh_source_.is_path()) {
      embedded_prefix = mesh_source_.path().lexically_normal().string();
    } else {
      DRAKE_DEMAND(mesh_source_.is_in_memory());
      // TODO(SeanCurtis-TRI): If the same image is embedded in multiple glTF
      // files, this will redundantly create them in the texture library.
      // It would probably be better to create the sha of the texture contents
      // instead.
      embedded_prefix = mesh_source_.in_memory().mesh_file.sha256().to_string();
    }

    tinygltf::TinyGLTF loader;
    string error;
    string warn;

    tinygltf::Model model;
    bool valid_parse = false;
    string_map<string> all_embedded_images;

    /* Drake's internal tinygltf has been configured to skip all external
     images. The mesh extractor handles those images later. This callback is
     designed to handle *embedded* images only.

     When encountering an image that is defined with a data URI, tinygltf will
     decode the bytes (from base64) and pass it to this callback. We'll take
     the bytes and store them for later reference, updating the image so that
     it has a URI indicating an in-memory image. */
    auto load_image_callback =
        [&all_embedded_images, this, &embedded_prefix](
            tinygltf::Image* image, const int image_index, std::string* /*err*/,
            std::string* /*warn*/, int /*req_width*/, int /*req_height*/,
            const unsigned char* bytes, int size, void* /*user_data*/) -> bool {
      /* Per glTF, an empty image URI implies an image contained in a buffer;
       that is, by definition, an "embedded image". */
      if (image->uri.empty()) {
        /* Update the image->uri to use the name we'll use in the texture
         library so that when we extract the material, the uri matches. */
        image->uri = GetEmbeddedImageName(embedded_prefix, image_index);
        /* tinygltf provides the image file's bits transiently; we need to copy
         them if they are to persist beyond this call. */
        all_embedded_images.insert(
            {image->uri, std::string(bytes, bytes + size)});
      }
      return true;
    };
    loader.SetImageLoader(load_image_callback, nullptr);

    if (mesh_source_.is_path()) {
      valid_parse = loader.LoadASCIIFromFile(&model, &error, &warn,
                                             mesh_source_.path().string());
    } else {
      DRAKE_DEMAND(mesh_source_.is_in_memory());

      loader.SetFsCallbacks(tinygltf::FsCallbacks{
          .FileExists =
              [this](const std::string& abs_filename, void*) {
                return TinyGltfInMemoryCallbackFileExists(abs_filename);
              },
          .ExpandFilePath = tinygltf::ExpandFilePath,
          .ReadWholeFile =
              [this](std::vector<unsigned char>* out, std::string* err,
                     const std::string& filepath, void*) {
                return TinyGltfInMemoryCallbackReadWholeFile(out, err,
                                                             filepath);
              },
          .WriteWholeFile = tinygltf::WriteWholeFile,
          .GetFileSizeInBytes =
              [this](size_t* filesize_out, std::string* err,
                     const std::string& filepath, void*) {
                return TinyGltfInMemoryCallbackGetFileSizeInBytes(
                    filesize_out, err, filepath);
              }});

      const std::string& gltf = mesh_source_.in_memory().mesh_file.contents();
      const std::string kEmptyBaseDir;
      valid_parse = loader.LoadASCIIFromString(
          &model, &error, &warn, gltf.c_str(), ssize(gltf), kEmptyBaseDir);
    }

    if (!valid_parse) {
      policy_.Error(fmt::format("Failed parsing {}: {}", description_, error));
    }

    /* We better not get any errors if we have a valid parse. */
    DRAKE_DEMAND(error.empty());
    if (!warn.empty()) {
      policy_.Warning(warn);
    }

    /* The root nodes of all the hierarchies that will be instantiated (by
     index). */
    vector<int> root_indices = FindTargetRootNodes(model);

    vector<RenderGlMesh> meshes =
        BuildGeometriesFromRootNodes(root_indices, model, data);

    /* Identify which embedded textures are actually referenced by materials
     and load them into the texture library. */
    string_map<string> used_embedded_images;
    for (const auto& mesh : meshes) {
      if (!mesh.mesh_material.has_value()) continue;
      /* Currently, we only support diffuse maps. */
      const auto* key_ptr =
          std::get_if<TextureKey>(&mesh.mesh_material->diffuse_map);
      if (key_ptr != nullptr) {
        // Note: we only have to worry about diffuse_maps that encode as keys.
        const std::string& diffuse_map = key_ptr->value;
        if (used_embedded_images.contains(diffuse_map)) continue;
        if (diffuse_map.starts_with(TextureLibrary::InMemoryPrefix())) {
          used_embedded_images[diffuse_map] =
              std::move(all_embedded_images[diffuse_map]);
          all_embedded_images.erase(diffuse_map);
        }
      }
    }
    texture_library_.AddInMemoryImages(used_embedded_images);
    return meshes;
  }

 private:
  /* Describes the data associated with a primitive's indices: the OpenGl
   index buffer object in which it is stored (with a zero offset), the number
   of indices, and the numerical type that represents the indices. */
  struct IndexBuffer {
    GLuint buffer{};
    int count{};
    GLenum type{};
  };

  /* Creates a unique image URI for an indexed image in a glTF file. The URI
   will be compatible with TextureLibrary's documented needs for adding
   in-memory images.

   @param gltf_id      A name associated with the glTF file. For example, if the
                       glTF came from on-disk, it could be the lexically normal
                       path to the file. If in-memory, a hash value associated
                       with the glTF file. Its purpose is to prevent accidental
                       collisions with embedded images from other glTF files.
   @param image_index  The index of the embedded image in the scope of the glTF
                       file. */
  string GetEmbeddedImageName(std::string_view gltf_id, int image_index) const {
    /* Make sure we use TextureLibrary's required prefix. We also append a
     suffix that further reduces accidentally interpreting the string as a valid
     file path. */
    return fmt::format("{}{}?image={}", TextureLibrary::InMemoryPrefix(),
                       gltf_id, image_index);
  }

  /* Returns the indices of all glTF nodes that have no parents (i.e., are root
   nodes). It searches *all* the nodes, unconstrained by what may or may not be
   indicated by the model's scenes. */
  static vector<int> FindAllRootNodes(const tinygltf::Model& model) {
    vector<bool> has_parent(model.nodes.size(), false);
    for (const tinygltf::Node& node : model.nodes) {
      for (int child_index : node.children) {
        has_parent[child_index] = true;
      }
    }
    vector<int> roots;
    for (int n = 0; n < ssize(has_parent); ++n) {
      if (!has_parent[n]) {
        roots.push_back(n);
      }
    }
    return roots;
  }

  /* Identifies a source scene from the glTF file and returns the indices of
   that scene's root nodes. If no default scene can be identified, then
   all root nodes in the file are returned. As documented in
   geometry_file_formats_doxygen.h, the only promise we make is that if the
   default scene is specified, we'll respect it. Otherwise, we get to use voodoo
   in picking the nodes from the glTF file. */
  vector<int> FindTargetRootNodes(const tinygltf::Model& model) const {
    /* The root nodes of all the hierarchies that will be instantiated (by
     index). */
    vector<int> root_indices;
    if (model.scenes.size() > 0) {
      if (model.defaultScene >= ssize(model.scenes)) {
        policy_.Error(fmt::format(
            "Error parsing a glTF file; it defines {} scenes but has an "
            "invalid value for the \"glTF.scene\" property: {}. In {}. No "
            "geometry will be added.",
            model.scenes.size(), model.defaultScene, description_));
        return root_indices;
      }

      /* Arbitrarily picking the zeroth scene *seems* consistent with glTF's
       spec: See:
       https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html#scenes

       It's unclear if the spec allows, requires, or forbids such a warning.
       We're providing the warning as the documented recommendation is that the
       user always have a default scene. */

      /* tinygltf initializes defaultScene to -1 to indicate "undefined". */
      if (model.defaultScene < 0 && ssize(model.scenes) > 1) {
        policy_.Warning(fmt::format(
            "Parsing a glTF file with multiple scene and no explicit "
            "default scene; using the zeroth scene: In {}.",
            description_));
      }
      const int scene_index = std::max(model.defaultScene, 0);
      root_indices = model.scenes[scene_index].nodes;
      /* Note: glTF doesn't require that a scene actually contain any nodes:
       https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html#reference-scene.
       */
      if (root_indices.empty()) {
        policy_.Error(fmt::format(
            "Error parsing a glTF file; scene {} has no root nodes. In {}.",
            scene_index, description_));
      }
    } else {
      if (model.nodes.size() == 0) {
        policy_.Error(fmt::format(
            "Error parsing a glTF file; it has no scenes and no nodes. In {}.",
            description_));
      }
      root_indices = FindAllRootNodes(model);
      if (root_indices.empty() && model.nodes.size() > 0) {
        policy_.Error(
            fmt::format("Error parsing a glTF file; none of its {} nodes are "
                        "root nodes. In {}.",
                        model.nodes.size(), description_));
      }
    }
    return root_indices;
  }

  /* Given a set of glTF root nodes (indicated by index), returns a collection
   of RenderGlMesh instances that represent the collection of objects.

   @pre There is an active OpenGl context. */
  vector<RenderGlMesh> BuildGeometriesFromRootNodes(
      const vector<int>& root_indices, const tinygltf::Model& model,
      const RegistrationData& data) {
    vector<RenderGlMesh> result;
    /* glTF is y-up, Drake's models are z-up. So, we'll start with a rotation
     around the x-axis. */
    const Matrix4d X_MGltf =
        RigidTransformd(
            RotationMatrixd::MakeFromOrthonormalColumns(
                Vector3d{1, 0, 0}, Vector3d{0, 0, 1}, Vector3d{0, -1, 0}))
            .GetAsMatrix4();
    for (int root_index : root_indices) {
      WalkNodeTree(root_index, model, X_MGltf, data, &result);
    }
    return result;
  }

  /* Given the name of the glTF mesh.primitive attribute, reports the attribute
   index that needs to be used to match the data to the shader definitions. */
  static int GetAttributeIndex(std::string_view attr_name) {
    if (attr_name == "POSITION") {
      return 0;
    } else if (attr_name == "NORMAL") {
      return 1;
    } else if (attr_name == "TEXCOORD_0") {
      return 2;
    }
    DRAKE_UNREACHABLE();
  }

  /* Creates the description of the named attribute (and the glTF buffer in
   which the attribute data lives).

   @returns  A pair consisting of the attribute specification and the index of
             the glTF buffer in which the data is stored. If undefined, the
             buffer will be -1.
   @throws std::exception if the requested attribute is missing (unless it is
                          "TEXCOORD_0"). */
  std::pair<VertexAttrib, int> GetAttribute(
      const std::string& attr_name, const tinygltf::Primitive& prim,
      const tinygltf::Model& model) const {
    const auto iter = prim.attributes.find(attr_name);
    VertexAttrib attribute;
    int buffer_index{-1};
    if (iter != prim.attributes.end()) {
      const int accessor_index = iter->second;
      const tinygltf::Accessor& accessor = model.accessors.at(accessor_index);
      const tinygltf::BufferView& buffer_view =
          model.bufferViews.at(accessor.bufferView);

      attribute.attribute_index = GetAttributeIndex(attr_name);
      attribute.components_per_element =
          tinygltf::GetNumComponentsInType(accessor.type);
      attribute.byte_offset = accessor.byteOffset + buffer_view.byteOffset;
      attribute.stride = accessor.ByteStride(buffer_view);
      attribute.numeric_type = accessor.componentType;

      buffer_index = buffer_view.buffer;
    } else if (attr_name != "TEXCOORD_0") {
      policy_.Error(fmt::format(
          "RenderEngineGl has limited support for glTF files. Primitives must "
          "define both 'POSITION' and 'NORMAL' attributes. A primitive is "
          "missing the attribute '{}' in {}.",
          attr_name, description_));
    }
    return {attribute, buffer_index};
  }

  /* Walks the node tree rooted at the node indicated by `node_index`. The mesh
   nodes are processed and added to `result` (with additional changes made to
   this instance's geometries_ as needed).

   @param node_index       The index of the node to process.
   @param model            The full model (used to resolve the quantities
                           referenced by the node and its child structures)
   @param T_FP             The pose of the node's parent frame in the file's
                           frame. In the parlance of OpenGlGeometry's frame
                           taxonomy, the file frame F is the model frame M
                           (i.e., we may still need to apply Drake-specified
                           scaling to put it in the geometry frame G).
   @param data             Drake's registration data associated with the file.
   @param[out] result      Newly instantiated RenderGlMesh instances will be
                           appended. For a single node, zero or more
                           RenderGlMesh instances will be added. */
  void WalkNodeTree(int node_index, const tinygltf::Model& model,
                    const Matrix4d& T_FP, const RegistrationData& data,
                    vector<RenderGlMesh>* result) {
    DRAKE_DEMAND(result != nullptr);
    const tinygltf::Node& node = model.nodes.at(node_index);

    const Matrix4d T_PN = EigenMatrixFromNode(node);
    const Matrix4d T_FN = T_FP * T_PN;
    if (node.mesh >= 0) {
      const Matrix4f T_FN_float = T_FN.cast<float>();
      InstantiateMesh(node.mesh, model, T_FN_float, data, result);
    }
    for (int child_index : node.children) {
      WalkNodeTree(child_index, model, T_FN, data, result);
    }
  }

  /* Instantiates zero or more OpenGlInstances from the glTF mesh indicated by
   `mesh_index`. Each of the primitives shares a common in-file transform
   T_MN (where the file frame is also the "model" frame M -- as documented for
   OpenGlGeometry). New RenderGlMesh instances (one per OpenGlGeometry) are
   appended to the provided `result` container.*/
  void InstantiateMesh(int mesh_index, const tinygltf::Model& model,
                       const Matrix4f& T_MN, const RegistrationData& data,
                       vector<RenderGlMesh>* result) {
    DRAKE_DEMAND(result != nullptr);
    const tinygltf::Mesh& mesh = model.meshes.at(mesh_index);
    for (int prim_i = 0; prim_i < ssize(mesh.primitives); ++prim_i) {
      const tinygltf::Primitive& prim = mesh.primitives[prim_i];
      /* Note *_buffer are indices into glTF buffers and *not* OpenGl ids. */
      const auto [p_attr, p_buffer] = GetAttribute("POSITION", prim, model);
      const auto [n_attr, n_buffer] = GetAttribute("NORMAL", prim, model);
      const auto [uv_attr, uv_buffer] = GetAttribute("TEXCOORD_0", prim, model);

      if (p_buffer != n_buffer || (uv_buffer != -1 && uv_buffer != p_buffer)) {
        policy_.Error(fmt::format(
            "RenderEngineGl has limited support for glTF files. All attributes "
            "of a primitive must ultimately reference the same buffer. A "
            "primitive {} in mesh {} has attributes referencing multiple "
            "buffers. In {}.",
            prim_i, mesh_index, description_));
      }

      OpenGlGeometry geometry;

      /* Set the transforms relating node (N) to model (M) frames. */
      geometry.T_MN = T_MN;
      /* Reference internal_opengl_geometry.h for the explanations of N_, S_,
       etc. Just extracting the upper 3x3 block does not give us N_MN. As
       documented in internal_opengl_geometry.h, N_MN = R_MN * S⁻¹_MN. The
       upper block is R_MN * S_MN. The scale factors are simply the magnitudes
       of the block's columns and we can find N_MN as follows:

           N_MN = R_MN * S⁻¹_MN
           N_MN = R_MN * S_MN * S⁻¹_MN * S⁻¹_MN
           N_MN = block * S⁻¹_MN * S⁻¹_MN          */
      geometry.N_MN = T_MN.block<3, 3>(0, 0);
      const Vector3f inv_scale_squared(
          1.0 / geometry.N_MN.col(0).squaredNorm(),
          1.0 / geometry.N_MN.col(1).squaredNorm(),
          1.0 / geometry.N_MN.col(2).squaredNorm());
      geometry.N_MN.col(0) *= inv_scale_squared.x();
      geometry.N_MN.col(1) *= inv_scale_squared.y();
      geometry.N_MN.col(2) *= inv_scale_squared.z();

      /* Set the geometry's vertex and index buffers. */
      geometry.vertex_buffer = GetOpenGlBuffer(p_buffer, model);
      ConfigureIndexBuffer(prim, model, mesh_index, &geometry);

      /* Now initialize the vertex arrays for the geometry. */
      geometry.spec =
          VertexSpec{.positions = p_attr, .normals = n_attr, .uvs = uv_attr};
      CreateVertexArray(&geometry);

      const int g_index = ssize(geometries_);
      geometries_.push_back(geometry);

      result->push_back(
          MakeRenderGlMesh(g_index, mesh_index, prim, model, data));
    }
  }

  /* Creates the RenderGlMesh associated with the OpenGlGeometry. */
  RenderGlMesh MakeRenderGlMesh(int geometry_index, int mesh_index,
                                const tinygltf::Primitive& prim,
                                const tinygltf::Model& model,
                                const RegistrationData& data) {
    RenderGlMesh gl_mesh{.mesh_index = geometry_index};

    const OpenGlGeometry& geometry = geometries_.at(geometry_index);
    gl_mesh.uv_state = geometry.spec.uvs.components_per_element == 0
                           ? UvState::kNone
                           : UvState::kFull;

    gl_mesh.mesh_material =
        MakePrimitiveMaterial(prim, model, gl_mesh, mesh_index, data);
    return gl_mesh;
  }

  /* Makes the RenderMaterial for the material associated with the primitive,
   if it exists. Otherwise, returns a fallback material. */
  RenderMaterial MakePrimitiveMaterial(const tinygltf::Primitive& prim,
                                       const tinygltf::Model& model,
                                       const RenderGlMesh& render_mesh,
                                       int gltf_mesh_index,
                                       const RegistrationData& data) {
    if (prim.material < 0 || prim.material >= ssize(model.materials)) {
      if (prim.material >= ssize(model.materials)) {
        policy_.Warning(fmt::format(
            "A primitive from mesh {} has invalid material index {}; there are "
            "only {} materials in the glTF file '{}'.",
            gltf_mesh_index, prim.material, ssize(model.materials),
            description_));
      }
      // An empty file path (for in-memory meshes) will short-circuit the
      // texture logic and not look for a texture at that path.
      const fs::path& file_path =
          mesh_source_.is_path() ? mesh_source_.path() : fs::path();
      return *MaybeMakeMeshFallbackMaterial(data.properties, file_path,
                                            data.default_diffuse, policy_,
                                            render_mesh.uv_state);
    }

    RenderMaterial material;

    const tinygltf::Material& gltf_mat = model.materials.at(prim.material);
    const tinygltf::PbrMetallicRoughness& gltf_pbr =
        gltf_mat.pbrMetallicRoughness;

    /* Not required in the glTF, but defaults to [1, 1, 1, 1], as per spec. */
    const vector<double>& gltf_rgba = gltf_pbr.baseColorFactor;
    material.diffuse =
        Rgba(gltf_rgba[0], gltf_rgba[1], gltf_rgba[2], gltf_rgba[3]);

    if (gltf_pbr.baseColorTexture.index < 0 ||
        render_mesh.uv_state == UvState::kNone) {
      /* There is no diffuse texture or no UVs for it. */
      return material;
    }

    if (gltf_pbr.baseColorTexture.texCoord > 0) {
      // TODO(SeanCurtis-TRI) Would this be better as a one-time warning?
      policy_.Warning(fmt::format(
          "Drake's support for glTF files only includes the \"TEXCOORD_0\" set "
          "of texture coordinates. Material {} specifies a baseColorTexture "
          "that references texture coordinate set {}. That texture will be "
          "ignored. In {}.",
          prim.material, gltf_pbr.baseColorTexture.texCoord, description_));
      return material;
    }
    const tinygltf::Texture& texture =
        model.textures.at(gltf_pbr.baseColorTexture.index);
    const tinygltf::Image& image = model.images.at(texture.source);
    /* We don't worry about whether the image format is supported here. It'll
     be reported downstream in the TextureLibrary. */
    /* When glTF references external images, they need to be flipped vertically
     (to accommodate the discrepancy between glTF and OpenGL image conventions).
     When *properly* embedded, they should be pre-flipped.
      - glTF image origin: top left corner.
        https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html#images
      - OpenGL image origin: bottom left corner.
        https://community.khronos.org/t/origin-of-texture-image-data/38792/2 */
    if (image.uri.starts_with(TextureLibrary::InMemoryPrefix())) {
      material.diffuse_map = TextureKey{image.uri};
      material.flip_y = false;
    } else {
      material.flip_y = true;
      if (mesh_source_.is_path()) {
        fs::path image_path = mesh_source_.path().parent_path() / image.uri;
        material.diffuse_map = image_path.lexically_normal();
      } else {
        DRAKE_DEMAND(mesh_source_.is_in_memory());
        const InMemoryMesh& mesh = mesh_source_.in_memory();
        const auto image_source_iter = mesh.supporting_files.find(image.uri);
        if (image_source_iter == mesh.supporting_files.end()) {
          policy_.Warning(fmt::format(
              "An in-memory glTF file referenced a texture as a URI ('{}') but "
              "that wasn't available in the mesh's supporting files.",
              image.uri));
          material.diffuse_map = std::monostate{};
        } else {
          material.diffuse_map = std::visit<TextureSource>(
              overloaded{[](const fs::path& path) {
                           return TextureSource(path.lexically_normal());
                         },
                         [&library = texture_library_](const MemoryFile& file) {
                           // In-memory images need to be loaded into the
                           // library directly.
                           const std::string image_key = fmt::format(
                               "{}{}", TextureLibrary::InMemoryPrefix(),
                               file.sha256().to_string());
                           // Note: if multiple materials reference the
                           // same in-memory image, it will still only be
                           // added to the texture library once.
                           library.AddInMemoryImage(image_key, file.contents());
                           return TextureSource(image_key);
                         }},
              image_source_iter->second);
        }
      }
    }

    return material;
  }

  /* Returns the name of the OpenGL buffer that contains the data in the glTF
   buffer indicated by `buffer_index` (creating the OpenGL object as needed).

   Note: the buffer is created in the current active context and only deleted
   *with the context*. */
  GLuint GetOpenGlBuffer(int buffer_index, const tinygltf::Model& model) {
    if (buffers_.contains(buffer_index)) {
      return buffers_.at(buffer_index);
    }
    const tinygltf::Buffer& buffer = model.buffers.at(buffer_index);
    GLuint buffer_id;
    glCreateBuffers(1, &buffer_id);
    glNamedBufferStorage(buffer_id, ssize(buffer.data), buffer.data.data(), 0);
    buffers_[buffer_index] = buffer_id;
    return buffer_id;
  }

  /* Configures the given `geometry` with the appropriate index data as
   extracted from the given primitive (`prim`). This may include creating a new
   buffer object for the primitive *indices*. Upon successful completion, the
   `geometry` will have its index_buffer, index_count, type, and mode
   configured.

   @pre `prim.indices` names an accessor that can be interpreted as element
        indices.

   Note: the buffer is created in the current active context and only deleted
   *with the context*. */
  void ConfigureIndexBuffer(const tinygltf::Primitive& prim,
                            const tinygltf::Model& model, int mesh_index,
                            OpenGlGeometry* geometry) {
    DRAKE_DEMAND(geometry != nullptr);
    if (prim.indices == -1) {
      policy_.Error(fmt::format(
          "RenderEngineGl has limited support for glTF files. All meshes "
          "must be indexed. A primitive in mesh {} has no 'indices'.",
          mesh_index));
    }
    if (!index_buffers_.contains(prim.indices)) {
      const tinygltf::Accessor& accessor = model.accessors.at(prim.indices);
      if (accessor.type != TINYGLTF_TYPE_SCALAR) {
        /* Note: printing the glTF-defined string value associated with
         TINYGLTF_TYPE_SCALAR is more work than it's worth. */
        policy_.Error(fmt::format(
            "glTF requires the type of an accessor for mesh indices to be "
            "'SCALAR'. Accessor {} has the wrong type.",
            prim.indices));
      }
      const tinygltf::BufferView& buffer_view =
          model.bufferViews.at(accessor.bufferView);
      if (buffer_view.byteStride != 0) {
        policy_.Error(fmt::format(
            "RenderEngineGl has limited support for glTF files. Primitive "
            "indices must be compactly stored in a buffer. The buffer view {} "
            "(referenced by mesh {}) has non-zero stride length: {}.",
            accessor.bufferView, mesh_index, buffer_view.byteStride));
      }
      const int offset = accessor.byteOffset + buffer_view.byteOffset;
      const tinygltf::Buffer& buffer = model.buffers.at(buffer_view.buffer);

      /* We have probably already loaded the whole glTF buffer into buffers_.
       The glTF buffer typically includes vertex attribute *and* primitive index
       data. However, the indices should be contained in their *own* buffer.
       (See
       https://registry.khronos.org/OpenGL/extensions/ARB/ARB_vertex_buffer_object.txt).
       This creates that separate (but redundant) buffer. In the future, we
       may choose to eliminate the redundancy by eliminating the index data from
       the vertex data buffer. */
      GLuint buffer_id;
      glCreateBuffers(1, &buffer_id);
      glNamedBufferStorage(buffer_id, buffer_view.byteLength,
                           buffer.data.data() + offset, 0);
      index_buffers_[prim.indices] = {
          .buffer = buffer_id,
          .count = static_cast<int>(accessor.count),
          .type = static_cast<GLenum>(accessor.componentType)};
    }
    const IndexBuffer& indices = index_buffers_.at(prim.indices);
    geometry->index_buffer = indices.buffer;
    geometry->index_count = indices.count;
    geometry->type = indices.type;
    /* Values of gltf.mesh.primitive.mode map to the corresponding gl values. */
    geometry->mode = static_cast<GLenum>(prim.mode);
  }

  /* Creates a transform from the given `node` data. */
  static Matrix4d EigenMatrixFromNode(const tinygltf::Node& node) {
    Matrix4d T;
    if (node.matrix.size() == 16) {
      // For glTF, transform matrix is a *column-major* matrix.
      int i = -1;
      for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
          T(r, c) = node.matrix.at(++i);
        }
      }
    } else {
      T = Matrix4d::Identity();
      if (node.translation.size() > 0) {
        DRAKE_DEMAND(node.translation.size() == 3);
        const Vector3d p(node.translation.at(0), node.translation.at(1),
                         node.translation.at(2));
        T.block<3, 1>(0, 3) = p.transpose();
      }
      if (node.rotation.size() > 0) {
        DRAKE_DEMAND(node.rotation.size() == 4);
        const Quaternion<double> quat(node.rotation[3], node.rotation[0],
                                      node.rotation[1], node.rotation[2]);
        T.block<3, 3>(0, 0) = math::RotationMatrixd(quat).matrix();
      }
      if (node.scale.size() > 0) {
        DRAKE_DEMAND(node.scale.size() == 3);
        for (int i = 0; i < 3; ++i) {
          T.block<3, 1>(0, i) *= node.scale.at(i);
        }
      }
    }
    return T;
  }

  // During ExtractMeshes when reading an InMemoryMesh, tinygltf uses this
  // callback to check if a file exists.
  bool TinyGltfInMemoryCallbackFileExists(const std::string& abs_filename) {
    const InMemoryMesh& mesh = mesh_source_.in_memory();
    return mesh.supporting_files.contains(abs_filename);
  }

  // During ExtractMeshes when reading an InMemoryMesh, tinygltf uses this
  // callback to check a file size.
  bool TinyGltfInMemoryCallbackGetFileSizeInBytes(size_t* filesize_out,
                                                  std::string* err,
                                                  const std::string& filepath) {
    DRAKE_DEMAND(filesize_out != nullptr);
    const InMemoryMesh& mesh = mesh_source_.in_memory();
    const auto file_source_iter = mesh.supporting_files.find(filepath);
    if (file_source_iter == mesh.supporting_files.end()) {
      if (err) {
        *err += fmt::format(
            "In-memory glTF referenced a URI that is not part of its "
            "supporting files: '{}'\n",
            filepath);
      }
      return false;
    }
    return std::visit(overloaded{[filesize_out, err](const fs::path& path) {
                                   return tinygltf::GetFileSizeInBytes(
                                       filesize_out, err, path.string(),
                                       nullptr);
                                 },
                                 [filesize_out](const MemoryFile& file) {
                                   *filesize_out = file.contents().size();
                                   return true;
                                 }},
                      file_source_iter->second);
  }

  // During ExtractMeshes when reading an InMemoryMesh, tinygltf uses this
  // callback to read a file.
  bool TinyGltfInMemoryCallbackReadWholeFile(std::vector<unsigned char>* out,
                                             std::string* err,
                                             const std::string& filepath) {
    DRAKE_DEMAND(out != nullptr);
    const InMemoryMesh& mesh = mesh_source_.in_memory();
    const auto file_source_iter = mesh.supporting_files.find(filepath);
    if (file_source_iter == mesh.supporting_files.end()) {
      if (err != nullptr) {
        *err += fmt::format(
            "In-memory glTF referenced a URI that is not part of its "
            "supporting files: '{}'\n",
            filepath);
      }
      return false;
    }
    return std::visit(  // BR
        overloaded{[out, err](const fs::path& path) {
                     return tinygltf::ReadWholeFile(out, err, path.string(),
                                                    nullptr);
                   },
                   [out](const MemoryFile& file) {
                     const std::string& contents = file.contents();
                     *out = std::vector<unsigned char>(contents.begin(),
                                                       contents.end());
                     return true;
                   }},
        file_source_iter->second);
  }

  const MeshSource& mesh_source_;

  /* A description to use for warnings and error messages. If the mesh source
   is a file, it includes the file name, otherwise it uses the MemoryFile
   filename hint. This value will always be non-empty, at least communicating
   the source of the glTF file. */
  const std::string description_;

  /* We load an entire glTF buffer into an OpenGl buffer object. This maps the
   glTF buffer (named by its index) to its OpenGl object. */
  map<int, GLuint> buffers_;

  // Note: this class has been written with the expectation that this policy
  // *throws* on invocation of Error(). Overriding the default throwing
  // behavior with non-throwing behavior can lead to undefined behavior.
  drake::internal::DiagnosticPolicy policy_;

  /* A map from a glTF accessor (named by its index) used as a primitive indices
   to the corresponding OpenGL index buffer. */
  map<int, IndexBuffer> index_buffers_;

  /* The repository for newly created OpenGlGeometry instances. Extracted meshes
   will strictly be appended. */
  std::vector<OpenGlGeometry>& geometries_;

  /* The texture library for registering embedded textures in the glTF file. */
  TextureLibrary& texture_library_;
};

void RenderEngineGl::CacheFileMeshesMaybe(const MeshSource& mesh_source,
                                          RegistrationData* data) {
  DRAKE_DEMAND(opengl_context_->IsCurrent());
  const std::string& extension = mesh_source.extension();
  if (!(extension == ".obj" || extension == ".gltf")) {
    static const logging::Warn one_time(
        "RenderEngineGl only supports Mesh specifications which use "
        ".obj or .gltf files. Mesh specifications using other mesh types "
        "(e.g., .stl, .dae, etc.) will be ignored.");
    data->accepted = false;
    return;
  }

  const std::string file_key = GetPathKey(mesh_source, /* is_convex= */ false);

  if (!meshes_.contains(file_key)) {
    vector<RenderGlMesh> file_meshes;
    if (extension == ".obj") {
      // Note: either the mesh has defined its own material or it hasn't. If it
      // has, that material will be defined in the RenderMesh and that material
      // will be saved in the cache, forcing every instance to use that
      // material. If it hasn't defined its own material, then every instance
      // must define its own material. Either way, we don't require whatever
      // properties were available when we triggered this cache update. That's
      // why we simply pass a set of empty properties -- to emphasize its
      // independence.
      vector<RenderMesh> meshes = LoadRenderMeshesFromObj(
          mesh_source, PerceptionProperties(), parameters_.default_diffuse,
          drake::internal::DiagnosticPolicy());

      for (const auto& render_mesh : meshes) {
        int mesh_index = CreateGlGeometry(render_mesh);
        DRAKE_DEMAND(mesh_index >= 0);

        geometries_[mesh_index].throw_if_undefined(
            fmt::format("Error creating object for mesh '{}'.",
                        mesh_source.description())
                .c_str());

        file_meshes.push_back(
            {.mesh_index = mesh_index, .uv_state = render_mesh.uv_state});

        DRAKE_DEMAND(render_mesh.material.has_value());
        const RenderMaterial& material = *render_mesh.material;
        // Only store materials defined by the mesh file; otherwise let
        // instances define their own (see ImplementMeshesForSource()).
        if (material.from_mesh_file) {
          file_meshes.back().mesh_material = material;
          // The material may have an in-memory diffuse texture. We need to
          // make sure it gets registered with the texture library and the
          // materials re-expressed to access them. This needs to happen before
          // we finish reifying in the call to ImplementMeshesForSource().
          // The input render meshes read from an .obj should either have no
          // diffuse map, a file path, or an in-memory image.
          DRAKE_DEMAND(
              !std::holds_alternative<TextureKey>(material.diffuse_map));
          const auto* memory_file_ptr =
              std::get_if<MemoryFile>(&material.diffuse_map);
          if (memory_file_ptr != nullptr) {
            // In-memory images need to be loaded into the library directly.
            const std::string image_key =
                fmt::format("{}{}", TextureLibrary::InMemoryPrefix(),
                            memory_file_ptr->sha256().to_string());
            // Note: if multiple materials reference the same in-memory image,
            // it will still only be added to the texture library once.
            texture_library_->AddInMemoryImage(image_key,
                                               memory_file_ptr->contents());
            file_meshes.back().mesh_material->diffuse_map = image_key;
            file_meshes.back().mesh_material->flip_y = true;
          }
        }
      }
    } else {
      file_meshes =
          GltfMeshExtractor(&mesh_source, &geometries_, texture_library_.get())
              .ExtractMeshes(*data);
    }
    meshes_[file_key] = std::move(file_meshes);
  }
}

std::tuple<GLint, GLenum, GLenum> RenderEngineGl::get_texture_format(
    RenderType render_type) {
  switch (render_type) {
    case RenderType::kColor:
      return std::make_tuple(GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
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

RenderTarget RenderEngineGl::CreateRenderTarget(const RenderCameraCore& camera,
                                                RenderType render_type) {
  // Create a framebuffer object (FBO).
  RenderTarget target;
  glCreateFramebuffers(1, &target.frame_buffer);

  // Create the texture object that will store the rendered result.
  const int width = camera.intrinsics().width();
  const int height = camera.intrinsics().height();
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

void RenderEngineGl::GetLabelImage(ImageLabel16I* label_image_out,
                                   const RenderTarget& target) const {
  ImageRgba8U image(label_image_out->width(), label_image_out->height());
  glGetTextureImage(target.value_texture, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                    image.size() * sizeof(GLubyte), image.at(0, 0));
  for (int y = 0; y < image.height(); ++y) {
    for (int x = 0; x < image.width(); ++x) {
      *label_image_out->at(x, y) = RenderEngine::MakeLabelFromRgb(
          image.at(x, y)[0], image.at(x, y)[1], image.at(x, y)[2]);
    }
  }
}

RenderTarget RenderEngineGl::GetRenderTarget(const RenderCameraCore& camera,
                                             RenderType render_type) const {
  const auto& intrinsics = camera.intrinsics();
  const BufferDim dim{intrinsics.width(), intrinsics.height()};
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
  DRAKE_ASSERT(glIsFramebuffer(target.frame_buffer));
  glBindFramebuffer(GL_FRAMEBUFFER, target.frame_buffer);
  glViewport(0, 0, intrinsics.width(), intrinsics.height());
  return target;
}

int RenderEngineGl::CreateGlGeometry(const RenderMesh& render_mesh,
                                     bool is_deformable) {
  // Confirm that the context is allocated.
  DRAKE_ASSERT(opengl_context_->IsCurrent());

  const int v_count = render_mesh.positions.rows();
  OpenGlGeometry geometry{
      .v_count = v_count, .type = GL_UNSIGNED_INT, .mode = GL_TRIANGLES};

  // Create the vertex buffer object (VBO).
  glCreateBuffers(1, &geometry.vertex_buffer);

  // We're representing the vertex data as a concatenation of positions,
  // normals, and texture coordinates (i.e., (VVVNNNUU)). There should be an
  // equal number of vertices, normals, and texture coordinates.
  DRAKE_DEMAND(render_mesh.positions.rows() == render_mesh.normals.rows());
  DRAKE_DEMAND(render_mesh.positions.rows() == render_mesh.uvs.rows());
  vector<GLfloat> vertex_data;
  // 3 floats each for position and normal, 2 for texture coordinates.
  const int kFloatsPerPosition = 3;
  const int kFloatsPerNormal = 3;
  const int kFloatsPerUv = 2;
  vertex_data.reserve(v_count *
                      (kFloatsPerPosition + kFloatsPerNormal + kFloatsPerUv));
  // N.B. we are implicitly converting from double to float by inserting them
  // into the vector.
  vertex_data.insert(
      vertex_data.end(), render_mesh.positions.data(),
      render_mesh.positions.data() + v_count * kFloatsPerPosition);
  vertex_data.insert(vertex_data.end(), render_mesh.normals.data(),
                     render_mesh.normals.data() + v_count * kFloatsPerNormal);
  vertex_data.insert(vertex_data.end(), render_mesh.uvs.data(),
                     render_mesh.uvs.data() + v_count * kFloatsPerUv);
  // For deformable meshes, we set the dynamic storage bit to allow modification
  // to the vertex position data.
  glNamedBufferStorage(geometry.vertex_buffer,
                       vertex_data.size() * sizeof(GLfloat), vertex_data.data(),
                       is_deformable ? GL_DYNAMIC_STORAGE_BIT : 0);

  // Create the index buffer object (IBO).
  using indices_uint_t = decltype(render_mesh.indices)::Scalar;
  static_assert(sizeof(GLuint) == sizeof(indices_uint_t),
                "If this fails, cast from unsigned int to GLuint");
  glCreateBuffers(1, &geometry.index_buffer);
  // The connectivity is always NOT modifiable.
  glNamedBufferStorage(geometry.index_buffer,
                       render_mesh.indices.size() * sizeof(GLuint),
                       render_mesh.indices.data(), 0);

  geometry.index_count = render_mesh.indices.size();

  geometry.v_count = v_count;
  // Now configure the vertex array object with vertex attributes.
  const int p_offset = 0;
  const int n_offset = v_count * kFloatsPerPosition * sizeof(GLfloat);
  const int uv_offset =
      v_count * (kFloatsPerPosition + kFloatsPerNormal) * sizeof(GLfloat);
  // clang-format off
  geometry.spec = VertexSpec{
      .positions = {.attribute_index = 0,
                    .components_per_element = kFloatsPerPosition,
                    .byte_offset = p_offset,
                    .stride = kFloatsPerPosition * sizeof(GLfloat),
                    .numeric_type = GL_FLOAT},
      .normals =   {.attribute_index = 1,
                    .components_per_element = kFloatsPerNormal,
                    .byte_offset = n_offset,
                    .stride = kFloatsPerNormal * sizeof(GLfloat),
                    .numeric_type = GL_FLOAT},
      .uvs =       {.attribute_index = 2,
                    .components_per_element = kFloatsPerUv,
                    .byte_offset = uv_offset,
                    .stride = kFloatsPerUv * sizeof(GLfloat),
                    .numeric_type = GL_FLOAT}};
  // clang-format on
  CreateVertexArray(&geometry);

  // Note: We won't need to call the corresponding glDeleteVertexArrays or
  // glDeleteBuffers. The meshes we store are "canonical" meshes. Even if a
  // particular GeometryId is removed, it was only referencing its corresponding
  // canonical mesh. We keep all canonical meshes alive for the lifetime of the
  // OpenGL context for convenient reuse.
  const int index = ssize(geometries_);
  geometries_.push_back(geometry);
  return index;
}

void RenderEngineGl::UpdateVertexArrays() {
  DRAKE_ASSERT(opengl_context_->IsCurrent());
  // Creating the vertex arrays requires the context to be bound.
  for (auto& geometry : geometries_) {
    // The only geometries in geometries_ should be fully defined.
    DRAKE_ASSERT(geometry.is_defined());
    CreateVertexArray(&geometry);
  }
}

void RenderEngineGl::SetWindowVisibility(const RenderCameraCore& camera,
                                         bool show_window,
                                         const RenderTarget& target) const {
  if (show_window) {
    const auto& intrinsics = camera.intrinsics();
    // Use the render target buffer as the read buffer and the default buffer
    // (0) as the draw buffer for displaying in the window. We transfer the full
    // image from source to destination. The semantics of glBlitNamedFrameBuffer
    // are inclusive of the "minimum" pixel (0, 0) and exclusive of the
    // "maximum" pixel (width, height).
    opengl_context_->DisplayWindow(intrinsics.width(), intrinsics.height());
    glBlitNamedFramebuffer(target.frame_buffer, 0,
                           // Src bounds.
                           0, 0, intrinsics.width(), intrinsics.height(),
                           // Dest bounds.
                           0, 0, intrinsics.width(), intrinsics.height(),
                           GL_COLOR_BUFFER_BIT, GL_NEAREST);
    opengl_context_->UpdateWindow();
  } else {
    opengl_context_->HideWindow();
  }
}

ShaderId RenderEngineGl::AddShader(std::unique_ptr<ShaderProgram> program,
                                   RenderType render_type) {
  const ShaderId shader_id = program->shader_id();
  shader_families_[render_type].insert({shader_id, set<GeometryId>()});
  shader_programs_[render_type][shader_id] = std::move(program);
  return shader_id;
}

ShaderProgramData RenderEngineGl::GetShaderProgram(
    const PerceptionProperties& properties, RenderType render_type) const {
  std::optional<ShaderProgramData> data{std::nullopt};
  for (const auto& id_shader_pair : shader_programs_[render_type]) {
    const ShaderProgram& program = *(id_shader_pair.second);

    // We prioritize the shader by id; higher ids will always win.
    if (data.has_value() && program.shader_id() < data->shader_id()) continue;

    std::optional<ShaderProgramData> candidate_data =
        program.CreateProgramData(properties);
    if (candidate_data.has_value()) {
      data = std::move(candidate_data);
    }
  }
  // There should always be, at least, the default shader that accepts the
  // geometry.
  DRAKE_DEMAND(data.has_value());
  return *data;
}

void RenderEngineGl::SetDefaultLightPosition(const Vector3<double>& p_DL) {
  DRAKE_DEMAND(fallback_lights_.size() == 1);
  // This is a stopgap solution until we can completely eliminate this method.
  // p_DC = (0, 0, 1). position = p_CL, so P_CL = p_DL - p_DC.
  fallback_lights_[0].position = p_DL - Vector3<double>{0, 0, 1};
}

void RenderEngineGl::ConfigureLights() {
  // Set the lights *once* for all color shaders. Currently, lighting can only
  // be figured upon construction.
  for (const auto& [_, shader_ptr] : shader_programs_[RenderType::kColor]) {
    const auto* lighting_program =
        dynamic_pointer_cast_or_throw<const LightingShader>(shader_ptr.get());
    // All color image shaders should inherit form LightingShader.
    DRAKE_DEMAND(lighting_program != nullptr);
    lighting_program->Use();
    lighting_program->SetAllLights(active_lights());
    lighting_program->Unuse();
  }
}

}  // namespace internal
}  // namespace render_gl
}  // namespace geometry
}  // namespace drake
