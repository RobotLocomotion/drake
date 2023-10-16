#pragma once

#include <filesystem>

#include "drake/common/diagnostic_policy.h"
#include "drake/geometry/geometry_properties.h"
#include "drake/geometry/rgba.h"

namespace drake {
namespace geometry {
namespace internal {

/* Reports how UVs have been assigned to the mesh receiving a material. Textures
 should only be applied to meshes with *fully* assigned UVs. */
enum class UvState { kNone, kFull, kPartial };

/* Specifies a mesh material as currently supported by Drake. We expect this
 definition to grow with time. */
struct RenderMaterial {
  /* The diffuse appearance at a point on a geometry surface is defined by the
   channel-wise product of the `diffuse` and the image color of `diffuse_map`
   applied as a texture according to the geometry's texture coordinates. If
   `diffuse_map` is empty, it acts as the multiplicative identity. */
  Rgba diffuse;
  std::filesystem::path diffuse_map;

  /* Whether the material definition comes from the mesh itself, e.g., an .mtl
   file, as opposed to the user specification or an implied texture. */
  bool from_mesh_file{false};
};

/* Dispatches a warning to the given diagnostic policy if the props contain a
 material definition. It is assumed an intrinsic material has already been found
 for the named mesh. */
void MaybeWarnForRedundantMaterial(
    const GeometryProperties& props, std::string_view mesh_name,
    const drake::internal::DiagnosticPolicy& policy);

/* If a mesh definition doesn't include a single material (e.g., as in an .mtl
 file for an .obj mesh), this function applies a cascading priority for
 otherwise defining a material for the mesh.

 The material is defined with the following protocol:

   - If the properties indicate a material at all, the material is derived
     purely from the properties (e.g., ("phong", "diffuse_map") and
     ("phong", "diffuse").
   - Otherwise, if an image can be located with a "compatible name" (e.g.,
     foo.png for a mesh foo.obj), a material with an unmodulated texture is
     created.
   - Finally, a diffuse material is created with the given default_diffuse
     color value.

 References to textures will be included in the material iff they can be read
 and the `uv_state` is full. Otherwise, a warning will be dispatched.

 @pre The mesh (named by `mesh_filename`) is a valid mesh and did not have an
      acceptable material definition). */
RenderMaterial MakeMeshFallbackMaterial(
    const GeometryProperties& props, const std::filesystem::path& mesh_path,
    const Rgba& default_diffuse,
    const drake::internal::DiagnosticPolicy& policy, UvState uv_state);

/* Creates a RenderMaterial from the given set of geometry properties. If no
 material properties exist, a material with the given default diffuse color is
 returned.

 If only a texture is specified, the diffuse color will be white. If the
 texture is not available, the diffuse map will be cleared, but the color will
 remain white, signaling a data error in the material specification.

 The `default_diffuse` color is only applied in the total absence of material
 properties. */
RenderMaterial DefineMaterial(
    const GeometryProperties& props,
    const Rgba& default_diffuse = Rgba(1, 1, 1),
    const drake::internal::DiagnosticPolicy& policy = {},
    UvState uv_state = UvState::kFull);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
