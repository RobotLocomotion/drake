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

void MaybeWarnForRedundantMaterial(
    const GeometryProperties& props, std::string_view mesh_name,
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

RenderMaterial MakeMeshFallbackMaterial(
    const GeometryProperties& props, const std::filesystem::path& mesh_path,
    const Rgba& default_diffuse,
    const DiagnosticPolicy& policy) {
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
    return DefineMaterial(props, Rgba(1, 1, 1), policy);
  }

  // Checks for foo.png for mesh filename foo.*. This the legacy behavior we
  // want to do away with.
  RenderMaterial material;

  // This is the fall-through condition; the final priority in the protocol.
  material.diffuse = default_diffuse;

  if (!mesh_path.empty()) {
    std::filesystem::path alt_texture_path(mesh_path);
    alt_texture_path.replace_extension("png");
    // If we find foo.png but can't access it, we'll *silently* treat it like
    // we didn't find it. No value in warning for a behavior we're cutting.
    if (std::ifstream(alt_texture_path).is_open()) {
      material.diffuse = Rgba(1, 1, 1);
      material.diffuse_map = alt_texture_path;
    }
  }
  return material;
}

RenderMaterial DefineMaterial(
    const GeometryProperties& props,
    const Rgba& default_diffuse,
    const DiagnosticPolicy& policy) {
  RenderMaterial material;

  material.diffuse_map =
      props.GetPropertyOrDefault<std::string>("phong", "diffuse_map", "");
  const bool has_texture = !material.diffuse_map.empty();
  // Default of white with a declared texture, otherwise the given default.
  material.diffuse = props.GetPropertyOrDefault<Rgba>(
      "phong", "diffuse", has_texture ? Rgba(1, 1, 1) : default_diffuse);

  if (has_texture) {
    // Confirm it is available.
    if (!std::ifstream(material.diffuse_map).is_open()) {
      // TODO(SeanCurtis-TRI): It would be good to be able to tie this into
      // some reference to the geometry under question. Ideally, the caller
      // would provide a custom policy that would automatically decorate this
      // message with the additional context.
      policy.Warning(fmt::format(
          "The ('phong', 'diffuse_map') property referenced a map that "
          "could not be found: '{}'",
          material.diffuse_map.string()));
      material.diffuse_map.clear();
    }
  }
  return material;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
