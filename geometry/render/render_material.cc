#include "drake/geometry/render/render_material.h"

#include <filesystem>
#include <fstream>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/fmt_eigen.h"

namespace drake {
namespace geometry {
namespace internal {

using drake::internal::DiagnosticPolicy;

/* Dispatches a warning to the given diagnostic policy if the props contain a
 material definition. It is assumed an intrinsic material has already been found
 for the named mesh. */
void MaybeWarnForRedundantMaterial(
    const GeometryProperties& props, const std::string& mesh_name,
    const DiagnosticPolicy& policy) {
  const bool has_diffuse = props.HasProperty("phong", "diffuse");
  const bool has_diffuse_map = props.HasProperty("phong", "diffuse_map");
  if (has_diffuse || has_diffuse_map) {
    const std::string diffuse_s =
        has_diffuse
            ? fmt::format(
                  "('phong', 'diffuse') = {}",
                  fmt_eigen(props.GetProperty<Rgba>("phong", "diffuse").rgba()))
            : std::string();
    const std::string map_s = has_diffuse_map
                                  ? fmt::format("('phong', 'diffuse_map') = {}",
                                                props.GetProperty<std::string>(
                                                    "phong", "diffuse_map"))
                                  : std::string();
    policy.Warning(
        fmt::format("The mesh {} has its own materials, but material "
                    "properties have been defined as well. They will "
                    "be ignored: {}{}{}",
                    mesh_name, diffuse_s,
                    has_diffuse && has_diffuse_map ? ", " : "", map_s));
  }
}

RenderMaterial MakeMeshFallbackMaterial(
    const GeometryProperties& props, const std::string& mesh_filename,
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

  if (!mesh_filename.empty()) {
    std::filesystem::path alt_texture_path(mesh_filename);
    alt_texture_path.replace_extension("png");
    // If we find foo.png but can't access it, we'll *silently* treat it like
    // we didn't find it. No value in warning for a behavior we're cutting.
    if (std::ifstream(alt_texture_path).is_open()) {
      material.diffuse = Rgba(1, 1, 1);
      material.diffuse_map = alt_texture_path.string();
    }
  }
  return material;
}

RenderMaterial DefineMaterial(
    const GeometryProperties& props,
    const Rgba& default_diffuse,
    const DiagnosticPolicy& policy) {
  RenderMaterial material;
  material.diffuse =
      props.GetPropertyOrDefault<Rgba>("phong", "diffuse", default_diffuse);

  material.diffuse_map =
      props.GetPropertyOrDefault<std::string>("phong", "diffuse_map", "");
  if (!material.diffuse_map.empty()) {
    // Confirm it is available.
    if (!std::ifstream(material.diffuse_map).is_open()) {
      // TODO(SeanCurtis-TRI): It would be good to be able to tie this into
      // some reference to the geometry under question.
      policy.Warning(fmt::format(
          "The ('phong', 'diffuse_map') property referenced a map that "
          "could not be found: {}",
          material.diffuse_map));
      material.diffuse_map = "";
    }
  }
  return material;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
