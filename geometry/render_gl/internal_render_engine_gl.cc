#include "drake/geometry/render_gl/internal_render_engine_gl.h"

#include <algorithm>
#include <filesystem>
#include <optional>
#include <unordered_set>
#include <utility>

#include <fmt/format.h>

#include "drake/common/diagnostic_policy.h"
#include "drake/common/pointer_cast.h"
#include "drake/common/scope_exit.h"
#include "drake/common/ssize.h"
#include "drake/common/text_logging.h"
#include "drake/common/unused.h"

namespace drake {
namespace geometry {
namespace render_gl {
namespace internal {

using Eigen::Vector2d;
using Eigen::Vector3d;
using geometry::internal::LoadRenderMeshesFromObj;
using geometry::internal::MakeMeshFallbackMaterial;
using geometry::internal::RenderMaterial;
using geometry::internal::RenderMesh;
using geometry::internal::UvState;
using math::RigidTransformd;
using render::ColorRenderCamera;
using render::DepthRenderCamera;
using render::LightParameter;
using render::RenderCameraCore;
using render::RenderEngine;
using render::RenderLabel;
using std::make_shared;
using std::make_unique;
using std::set;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::unordered_map;
using std::vector;
using systems::sensors::ColorD;
using systems::sensors::ColorI;
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
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LightingShader)
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

  void DoSetModelViewMatrix(const Eigen::Matrix4f& X_CW,
                            const Eigen::Matrix4f& T_WM,
                            const Eigen::Matrix4f& X_WG,
                            const Vector3d& scale) const override {
    // For lighting, we need the normal and position of a fragment in the world
    // frame. The pose of the fragment (from its corresponding vertices) comes
    // simply from T_WM. But the normals require a different transform:
    //
    //   1. No translation.
    //   2. Same rotation as vertex positions.
    //   3. *Inverse* scale as vertex positions.
    //
    // If the scale isn't identity, the normal may not be unit length. We rely
    // on the shader to normalize the scaled normals.
    // This is the quantity historically referred to as gl_NormalMatrix
    // (available to glsl in the "compatibility profile"). See
    // https://www.cs.upc.edu/~robert/teaching/idi/GLSLangSpec.4.50.pdf.
    const Eigen::DiagonalMatrix<float, 3, 3> S_GM_normal(
        Vector3<float>(1.0 / scale(0), 1.0 / scale(1), 1.0 / scale(2)));
    const Eigen::Matrix3f X_WM_Normal = X_WG.block<3, 3>(0, 0) * S_GM_normal;
    glUniformMatrix3fv(T_WM_normals_loc_, 1, GL_FALSE, X_WM_Normal.data());
    glUniformMatrix4fv(T_WM_loc_, 1, GL_FALSE, T_WM.data());

    Eigen::Matrix4f X_WC = Eigen::Matrix4f::Identity();
    X_WC.block<3, 3>(0, 0) = X_CW.block<3, 3>(0, 0).transpose();
    X_WC.block<3, 1>(0, 3) = X_WC.block<3, 3>(0, 0) * (-X_CW.block<3, 1>(0, 3));
    glUniformMatrix4fv(X_WC_loc_, 1, GL_FALSE, X_WC.data());
  }

  void SetLightParameters(int index, const LightParameter& light) const {
    glUniform1i(GetLightFieldLocation(index, "type"),
                static_cast<int>(render::light_type_from_string(light.type)));
    Eigen::Vector3f color = light.color.rgba().head<3>().cast<float>();
    glUniform3fv(GetLightFieldLocation(index, "color"), 1, color.data());
    Eigen::Vector4f position;
    position.head<3>() = light.position.cast<float>();
    const render::LightFrame frame =
        render::light_frame_from_string(light.frame);
    position(3) = frame == render::LightFrame::kWorld ? 0.0f : 1.0f;
    glUniform4fv(GetLightFieldLocation(index, "position"), 1, position.data());
    Eigen::Vector3f atten_coeff = light.attenuation_values.cast<float>();
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
      Eigen::Vector3f direction = light.direction.cast<float>();
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
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DefaultRgbaColorShader)

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
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DefaultTextureColorShader)

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
  }

 private:
  void DoConfigureMoreUniforms() final {
    diffuse_map_loc_ = GetUniformLocation("diffuse_map");
    diffuse_scale_loc_ = GetUniformLocation("diffuse_map_scale");
  }

  std::unique_ptr<ShaderProgram> DoClone() const final {
    return make_unique<DefaultTextureColorShader>(*this);
  }

  struct InstanceData {
    GLuint texture_id;
    Vector2<float> texture_scale;
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
    return ShaderProgramData{
        shader_id(),
        AbstractValue::Make(InstanceData{*texture_id, scale.cast<float>()})};
  }

  std::shared_ptr<TextureLibrary> library_{};

  // The location of the "diffuse_map" uniform in the shader.
  GLint diffuse_map_loc_{};

  // The location of the "diffuse_scale" uniform in the shader.
  GLint diffuse_scale_loc_{};

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
  vec4 map_rgba = texture(diffuse_map, fract(tex_coord * diffuse_map_scale));
  color = GetIlluminatedColor(map_rgba);
})""";
};

/* The built-in shader for objects in depth images. By default, the shader
 supports all geometries.  */
class DefaultDepthShader final : public ShaderProgram {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DefaultDepthShader)

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
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DefaultLabelShader)

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
std::string GetPathKey(const std::string& filename) {
  std::error_code path_error;
  const fs::path path = fs::canonical(filename, path_error);
  if (path_error) {
    throw std::runtime_error(
        fmt::format("RenderEngineGl: unable to access the file {}; {}",
                    filename, path_error.message()));
  }
  return path.string();
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
  GetMeshes(convex.filename(), data);
  if (data->accepted) {
    ImplementMeshesForFile(user_data, kUnitScale * convex.scale(),
                           convex.filename());
  }
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
  GetMeshes(mesh.filename(), data);
  if (data->accepted) {
    ImplementMeshesForFile(user_data, kUnitScale * mesh.scale(),
                           mesh.filename());
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

void RenderEngineGl::ImplementMeshesForFile(void* user_data,
                                            const Vector3<double>& scale,
                                            const std::string& filename) {
  const RegistrationData& data = *static_cast<RegistrationData*>(user_data);
  const std::string file_key = GetPathKey(filename);
  for (const auto& gl_mesh : meshes_.at(file_key)) {
    PerceptionProperties temp_props(data.properties);

    RenderMaterial material;
    // If there is a material associated with the mesh, we will use it.
    // Otherwise, we recreate the fallback material based on user data and
    // defaults.
    if (gl_mesh.mesh_material.has_value()) {
      material = gl_mesh.mesh_material.value();
    } else {
      material = MakeMeshFallbackMaterial(
          data.properties, filename, parameters_.default_diffuse,
          drake::internal::DiagnosticPolicy(), gl_mesh.uv_state);
    }
    temp_props.UpdateProperty("phong", "diffuse_map",
                              material.diffuse_map.string());
    temp_props.UpdateProperty("phong", "diffuse", material.diffuse);
    RegistrationData temp_data{data.id, data.X_WG, temp_props};
    AddGeometryInstance(gl_mesh.mesh_index, &temp_data, scale);
  }
}

bool RenderEngineGl::DoRegisterVisual(GeometryId id, const Shape& shape,
                                      const PerceptionProperties& properties,
                                      const RigidTransformd& X_WG) {
  opengl_context_->MakeCurrent();
  RegistrationData data{id, RigidTransformd{X_WG}, properties};
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

    PerceptionProperties mesh_properties(properties);
    mesh_properties.UpdateProperty("phong", "diffuse_map",
                                   render_mesh.material.diffuse_map.string());
    mesh_properties.UpdateProperty("phong", "diffuse",
                                   render_mesh.material.diffuse);
    RegistrationData data{id, RigidTransformd::Identity(), mesh_properties};
    AddGeometryInstance(mesh_index, &data, kUnitScale);
  }
  deformable_meshes_.emplace(id, std::move(gl_mesh_indices));
  return true;
}

void RenderEngineGl::DoUpdateVisualPose(GeometryId id,
                                        const RigidTransformd& X_WG) {
  for (auto& part : visuals_.at(id).parts) {
    if (part.T_GN.has_value()) {
      part.instance.X_WG = X_WG * part.T_GN.value();
    } else {
      part.instance.X_WG = X_WG;
    }
  }
}

void RenderEngineGl::DoUpdateDeformableConfigurations(
    GeometryId id, const std::vector<VectorX<double>>& q_WGs,
    const std::vector<VectorX<double>>& nhats_W) {
  DRAKE_DEMAND(deformable_meshes_.count(id) > 0);
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
  if (deformable_meshes_.count(id) > 0) {
    deformable_meshes_.erase(id);
  }
  // Now remove the instances associated with the id (stored in visuals_).
  auto iter = visuals_.find(id);
  if (iter != visuals_.end()) {
    // Multiple parts may have the same shader. We don't want to attempt
    // removing the geometry id from the corresponding family redundantly.
    std::unordered_set<ShaderId> visited_families;
    // Remove from the shader families to which it belongs!
    auto maybe_remove_from_family =
        [this, &visited_families](GeometryId g_id, const auto& shader_data,
                                  RenderType render_type) {
          const ShaderId s_id = shader_data[render_type].shader_id();
          if (visited_families.count(s_id) > 0) {
            return;
          }
          visited_families.insert(s_id);
          auto& geometries = shader_families_[render_type].at(s_id);
          auto num_removed = geometries.erase(g_id);
          DRAKE_DEMAND(num_removed == 1);
        };
    for (const auto& part : iter->second.parts) {
      const OpenGlInstance& instance = part.instance;
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
  const Eigen::Matrix4f& X_CW = X_CW_.GetAsMatrix4().matrix().cast<float>();
  // We rely on the calling method to clear all appropriate buffers; this method
  // may be called multiple times per image (based on the number of shaders
  // being used) and, therefore, can't do the clearing itself.

  for (const GeometryId& g_id :
       shader_families_.at(render_type).at(shader_program.shader_id())) {
    for (const auto& part : visuals_.at(g_id).parts) {
      const OpenGlInstance& instance = part.instance;
      if (instance.shader_data.at(render_type).shader_id() !=
          shader_program.shader_id()) {
        continue;
      }
      const OpenGlGeometry& geometry = geometries_[instance.geometry];
      glBindVertexArray(geometry.vertex_array);

      shader_program.SetInstanceParameters(instance.shader_data[render_type]);
      // TODO(SeanCurtis-TRI): Consider storing the float-valued pose in the
      //  OpenGl instance to avoid the conversion every time it is rendered.
      //  Generally, this wouldn't expect much savings; an instance is only
      //  rendered once per image type. So, for three image types, I'd cast
      //  three times. Stored, I'd cast once.
      shader_program.SetModelViewMatrix(X_CW, instance.X_WG, instance.scale);

      glDrawElements(GL_TRIANGLES, geometry.index_buffer_size, GL_UNSIGNED_INT,
                     0);
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
  const Eigen::Matrix4f T_DC =
      camera.core().CalcProjectionMatrix().cast<float>();

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
  const Eigen::Matrix4f T_DC =
      camera.core().CalcProjectionMatrix().cast<float>();

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
  const Eigen::Matrix4f T_DC =
      camera.core().CalcProjectionMatrix().cast<float>();

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

  visuals_[data.id].parts.push_back(
      {.instance = OpenGlInstance(geometry_index, data.X_WG, scale, *color_data,
                                  *depth_data, *label_data),
       .T_GN = std::nullopt});

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

vector<int> RenderEngineGl::GetMeshes(const string& filename_in,
                                      RegistrationData* data) {
  vector<int> mesh_indices;

  // We're checking the input filename in case the user specified name has the
  // desired extension but is a symlink to some arbitrarily named cached file.
  if (Mesh(filename_in).extension() != ".obj") {
    static const logging::Warn one_time(
        "RenderEngineGl only supports Mesh/Convex specifications which use "
        ".obj files. Mesh specifications using other mesh types (e.g., "
        ".gltf, .stl, .dae, etc.) will be ignored.");
    data->accepted = false;
    return mesh_indices;
  }

  const std::string file_key = GetPathKey(filename_in);

  if (meshes_.count(file_key) == 0) {
    const vector<RenderMesh> meshes = LoadRenderMeshesFromObj(
        filename_in, PerceptionProperties(), parameters_.default_diffuse,
        drake::internal::DiagnosticPolicy());
    vector<RenderGlMesh> file_meshes;
    for (const auto& render_mesh : meshes) {
      int mesh_index = CreateGlGeometry(render_mesh);
      DRAKE_DEMAND(mesh_index >= 0);
      const RenderMaterial& material = render_mesh.material;

      file_meshes.push_back(
          {.mesh_index = mesh_index, .uv_state = render_mesh.uv_state});
      // If the material in render_mesh was defined by the file, we store it
      // with the RenderGlMesh (so it's used for every instance). Otherwise, we
      // leave it undefined so instance properties will define the material
      // instead.
      if (material.from_mesh_file) {
        file_meshes.back().mesh_material = material;
      }
      mesh_indices.push_back(mesh_index);
    }
    meshes_[file_key] = std::move(file_meshes);
  } else {
    for (const auto& gl_mesh : meshes_[file_key]) {
      mesh_indices.push_back(gl_mesh.mesh_index);
    }
  }

  for (const auto& index : mesh_indices) {
    geometries_[index].throw_if_undefined(
        fmt::format("Error creating object for mesh {}", filename_in).c_str());
  }

  return mesh_indices;
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

  OpenGlGeometry geometry;

  // Create the vertex buffer object (VBO).
  glCreateBuffers(1, &geometry.vertex_buffer);

  // We're representing the vertex data as a concatenation of positions,
  // normals, and texture coordinates (i.e., (VVVNNNUU)). There should be an
  // equal number of vertices, normals, and texture coordinates.
  DRAKE_DEMAND(render_mesh.positions.rows() == render_mesh.normals.rows());
  DRAKE_DEMAND(render_mesh.positions.rows() == render_mesh.uvs.rows());
  const int v_count = render_mesh.positions.rows();
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

  geometry.index_buffer_size = render_mesh.indices.size();

  geometry.v_count = v_count;
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

void RenderEngineGl::CreateVertexArray(OpenGlGeometry* geometry) const {
  // Confirm that the context is allocated.
  DRAKE_ASSERT(opengl_context_->IsCurrent());

  glCreateVertexArrays(1, &geometry->vertex_array);

  // 3 floats each for position and normal, 2 for texture coordinates.
  const int kFloatsPerPosition = 3;
  const int kFloatsPerNormal = 3;
  const int kFloatsPerUv = 2;

  std::size_t vbo_offset = 0;

  const int position_attrib = 0;
  glVertexArrayVertexBuffer(geometry->vertex_array, position_attrib,
                            geometry->vertex_buffer, vbo_offset,
                            kFloatsPerPosition * sizeof(GLfloat));
  glVertexArrayAttribFormat(geometry->vertex_array, position_attrib,
                            kFloatsPerPosition, GL_FLOAT, GL_FALSE, 0);
  glEnableVertexArrayAttrib(geometry->vertex_array, position_attrib);
  vbo_offset += geometry->v_count * kFloatsPerPosition * sizeof(GLfloat);

  const int normal_attrib = 1;
  glVertexArrayVertexBuffer(geometry->vertex_array, normal_attrib,
                            geometry->vertex_buffer, vbo_offset,
                            kFloatsPerNormal * sizeof(GLfloat));
  glVertexArrayAttribFormat(geometry->vertex_array, normal_attrib,
                            kFloatsPerNormal, GL_FLOAT, GL_FALSE, 0);
  glEnableVertexArrayAttrib(geometry->vertex_array, normal_attrib);
  vbo_offset += geometry->v_count * kFloatsPerNormal * sizeof(GLfloat);

  const int uv_attrib = 2;
  glVertexArrayVertexBuffer(geometry->vertex_array, uv_attrib,
                            geometry->vertex_buffer, vbo_offset,
                            kFloatsPerUv * sizeof(GLfloat));
  glVertexArrayAttribFormat(geometry->vertex_array, uv_attrib, kFloatsPerUv,
                            GL_FLOAT, GL_FALSE, 0);
  glEnableVertexArrayAttrib(geometry->vertex_array, uv_attrib);
  vbo_offset += geometry->v_count * kFloatsPerUv * sizeof(GLfloat);

  const float float_count =
      geometry->v_count *
      (kFloatsPerPosition + kFloatsPerNormal + kFloatsPerUv);
  DRAKE_DEMAND(vbo_offset == float_count * sizeof(GLfloat));

  // Bind index buffer object (IBO) with the vertex array object (VAO).
  glVertexArrayElementBuffer(geometry->vertex_array, geometry->index_buffer);
}

void RenderEngineGl::UpdateVertexArrays() {
  DRAKE_ASSERT(opengl_context_->IsCurrent());
  // Creating the vertex arrays requires the context to be bound.
  for (auto& geometry : geometries_) {
    // The only geometries in geometries_ should be fully defined.
    DRAKE_ASSERT(geometry.is_defined());
    this->CreateVertexArray(&geometry);
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
