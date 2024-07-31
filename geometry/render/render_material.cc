#include "drake/geometry/render/render_material.h"

#include <fstream>
#include <string>
#include <vector>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/fmt_eigen.h"

namespace drake {
namespace geometry {
namespace internal {

using drake::internal::DiagnosticPolicy;

RenderMaterial MakeDiffuseMaterial(const Rgba& diffuse) {
  RenderMaterial result;
  result.diffuse = diffuse;
  return result;
}

void MaybeWarnForRedundantMaterial(const GeometryProperties& props,
                                   std::string_view mesh_name,
                                   const DiagnosticPolicy& policy) {
  std::vector<std::string> ignored_props;
  if (props.HasProperty("phong", "diffuse")) {
    ignored_props.push_back(fmt::format(
        "('phong', 'diffuse') = {}",
        fmt_eigen(
            props.GetProperty<Rgba>("phong", "diffuse").rgba().transpose())));
  }
  if (props.HasProperty("phong", "diffuse_map")) {
    ignored_props.push_back(
        fmt::format("('phong', 'diffuse_map') = {}",
                    props.GetProperty<std::string>("phong", "diffuse_map")));
  }
  if (!ignored_props.empty()) {
    policy.Warning(
        fmt::format("The mesh {} has its own materials, but material "
                    "properties have been defined as well. They will "
                    "be ignored: {}",
                    mesh_name, fmt::join(ignored_props, ", ")));
  }
}

std::optional<RenderMaterial> MaybeMakeMeshFallbackMaterial(
    const GeometryProperties& props, const std::filesystem::path& mesh_path,
    const std::optional<Rgba>& default_diffuse, const DiagnosticPolicy& policy,
    UvState uv_state) {
  // If a material is indicated *at all* in the properties, that
  // defines the material.
  if (props.HasProperty("phong", "diffuse") ||
      props.HasProperty("phong", "diffuse_map")) {
    // This call must be guarded because it will create a valid material even if
    // no properties have been defined.

    // Why is the default color white?
    //  We're here because the user specified *some* drake material property.
    //  If it's *only* diffuse_map, then the underlying color must be white to
    //  faithfully reproduce the texture. Obviously, if they've specified a
    //  diffuse color, that will be used. Objects that request an invalid
    //  texture will be drawn in white.
    return DefineMaterial(props, Rgba(1, 1, 1), policy, uv_state);
  }

  std::optional<RenderMaterial> default_result{std::nullopt};
  if (default_diffuse.has_value()) {
    default_result = MakeDiffuseMaterial(*default_diffuse);
  }
  if (mesh_path.empty()) {
    return default_result;
  }
  // Checks for foo.png for mesh filename foo.*. This the legacy behavior we
  // want to do away with.
  std::filesystem::path alt_texture_path(mesh_path);
  alt_texture_path.replace_extension("png");
  // If we find foo.png but can't access it, we'll *silently* treat it like
  // we didn't find it. No value in warning for a behavior we're cutting.
  if (!std::ifstream(alt_texture_path).is_open()) {
    return default_result;
  }

  RenderMaterial material;
  if (uv_state == UvState::kFull) {
    material.diffuse_map = alt_texture_path;
  } else {
    policy.Warning(fmt::format(
        "A png file of the same name as the mesh has been found ('{}'), "
        "but the mesh doesn't define {} texture coordinates. The map will "
        "be omitted leaving a flat white color.",
        alt_texture_path.string(),
        uv_state == UvState::kNone ? "any" : "a complete set of"));
  }
  // A white color will leave the texture unmodulated. In the case where
  // we couldn't apply the texture, flat white also corresponds to the
  // warning message.
  material.diffuse = Rgba(1, 1, 1);
  return material;
}

RenderMaterial DefineMaterial(const GeometryProperties& props,
                              const Rgba& default_diffuse,
                              const DiagnosticPolicy& policy,
                              UvState uv_state) {
  RenderMaterial material;

  material.diffuse_map =
      props.GetPropertyOrDefault<std::string>("phong", "diffuse_map", "");
  const bool has_diffuse_map = !material.diffuse_map.empty();

  // Default of white with a declared texture, otherwise the given default.
  material.diffuse = props.GetPropertyOrDefault<Rgba>(
      "phong", "diffuse", has_diffuse_map ? Rgba(1, 1, 1) : default_diffuse);

  if (has_diffuse_map) {
    // Confirm it is available.
    if (!std::ifstream(material.diffuse_map).is_open()) {
      // TODO(SeanCurtis-TRI): It would be good to be able to tie this into
      // some reference to the geometry under question. Ideally, the caller
      // would provide a custom policy that would automatically decorate this
      // message with the additional context.
      policy.Warning(fmt::format(
          "The ('phong', 'diffuse_map') property referenced a map that "
          "could not be found: '{}'",
          material.diffuse_map));
      material.diffuse_map = "";
    } else if (uv_state != UvState::kFull) {
      policy.Warning(fmt::format(
          "The ('phong', 'diffuse_map') property referenced a map, but the "
          "geometry doesn't define {} texture coordinates. The map will be "
          "omitted: '{}'.",
          uv_state == UvState::kNone ? "any" : "a complete set of",
          material.diffuse_map));
      material.diffuse_map = "";
    }
  }
  return material;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
