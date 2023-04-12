#pragma once

#include <string>

#include "drake/common/diagnostic_policy.h"
#include "drake/geometry/geometry_properties.h"
#include "drake/geometry/rgba.h"

namespace drake {
namespace geometry {
namespace internal {

/* Specifies a mesh material as currently supported by Drake. We expect this
 definition to grow with time. */
struct RenderMaterial {
  Rgba diffuse;
  std::string diffuse_map;
};

/* Dispatches a warning to the given diagnostic policy if the props contain a
 material definition. It is assumed an intrinsic material has already been found
 for the named mesh. */
void MaybeWarnForRedundantMaterial(
    const GeometryProperties& props, const std::string& mesh_name,
    const drake::internal::DiagnosticPolicy& policy);

/* If a mesh hasn't specified an implicit material (or it has defined multiple
 materials), this function applies a cascading priority for otherwise defining
 a material for the mesh.

 The material is defined with the following protocol:

   - If the properties indicate a material at all, the material is derived
     purely from the properties (e.g., ("phong", "diffuse_map") and
     ("phong", "diffuse").
   - Otherwise, if an image can be located with a "compatible name" (e.g.,
     foo.png for a mesh foo.obj), a material with an unmodulated texture is
     created.
   - Finally, a diffuse material is created with the given default_diffuse
     color value.

 References to textures will be included in the material iff they can be read.

 @pre The mesh (named by `mesh_filename`) is a valid mesh and did not have an
      acceptable material definition). */
RenderMaterial MakeMeshFallbackMaterial(
    const GeometryProperties& props, const std::string& mesh_filename,
    const Rgba& default_diffuse,
    const drake::internal::DiagnosticPolicy& policy);

/* Creates a RenderMaterial from the given set of geometry properties. If no
 material properties exist, a material with the given default diffuse color is
 returned. */
RenderMaterial DefineMaterial(
    const GeometryProperties& props,
    const Rgba& default_diffuse = Rgba(1, 1, 1),
    const drake::internal::DiagnosticPolicy& policy = {});

}  // namespace internal
}  // namespace geometry
}  // namespace drake
