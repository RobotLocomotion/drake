#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <variant>

#include "drake/common/diagnostic_policy.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/memory_file.h"
#include "drake/geometry/geometry_properties.h"
#include "drake/geometry/rgba.h"

namespace drake {
namespace geometry {
namespace internal {

/* The key value for a texture source when it references an entry into an
 internal image database. */
struct TextureKey {
  std::string value;
};

/* A texture can be specified for a RenderMaterial in several ways:

  - No image at all (aka an "empty" texture source).
  - A file path to an image on the disk.
  - A special key for access in some coordinated image database.
  - Contents of a known image file format.

 Note: when assigning to a %TextureSource, take *extra* care to distinguish
 between a TextureKey and a path. Simply assigning a string (or string-like
 type) will successfully compile, but the implicit conversion from those types
 to std::filesystem::path will mean you're setting a *path*. When setting a
 texture key, explicitly declare it as such. E.g.,:

 TextureSource source = TextureKey{"looks/like/a/path/but/is/not.png"};

 This can be particularly surprising if the right-hand side is a
 std::string_view. There *is* an implicit conversion from std::string_view to
 std::filesystem::path, but not to std::string. */
using TextureSource =
    std::variant<std::monostate, std::filesystem::path, TextureKey, MemoryFile>;

/* Reports if the texture source specifies no texture -- i.e., it's empty. */
bool IsEmpty(const TextureSource& source);

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

  /* The optional texture to use as diffuse map. If no diffuse texture is
   defined, it will be "empty". Otherwise, the texture can be specified by a
   path to an on-disk image, in-memory image file contents, or a database key.
   Some RenderEngine implementations construct and consume their own
   %RenderMaterial may store images in a local database. When
   `diffuse_map.is_key()` returns true, it is a key into that database. Such
   %RenderMaterial instances should be kept hidden within those RenderEngine
   implementations. */
  TextureSource diffuse_map;

  /* OpenGL defines image origin at the bottom-left corner of the texture. Some
   geometry formats (e.g., glTF), define the origin at the top-left corner.
   When set to true, the texture coordinates need to be flipped in the y
   direction for this material's texture to render correctly. */
  bool flip_y{false};

  /* Whether the material definition comes from the mesh itself, e.g., an .mtl
   file, as opposed to the user specification or an implied texture. */
  bool from_mesh_file{false};
};

/* Creates a RenderMaterial with the specified diffuse color.

 Consumers of RenderMesh can call this function to produce an untextured
 RenderMaterial with the prescribed diffuse color when presented with RenderMesh
 instances that do not come with their own material definitions.

 @param diffuse  The RGBA color to be used as the diffuse color for the
 material. */
RenderMaterial MakeDiffuseMaterial(const Rgba& diffuse);

/* Dispatches a warning to the given diagnostic policy if the props contain a
 material definition. It is assumed an intrinsic material has already been found
 for the named mesh. */
void MaybeWarnForRedundantMaterial(
    const GeometryProperties& props, std::string_view mesh_name,
    const drake::internal::DiagnosticPolicy& policy);

/* If a mesh definition doesn't include a single material (e.g., as in an .mtl
 file for an .obj mesh), this function applies a cascading priority for
 potentially defining a material. Failing everything in the priority list, it
 will return std::nullopt.

 The material is defined with the following protocol:

   - If the properties indicate a material at all, the material is derived
     purely from the properties (e.g., ("phong", "diffuse_map") and
     ("phong", "diffuse").
   - Otherwise, if an image can be located with a "compatible name" (e.g.,
     foo.png for the mesh foo.obj), a material with an unmodulated texture is
     created. An existing foo.png that can't be read and an empty `mesh_path`
     are both treated as "no compatible png could be found" and will fall
     through to the next condition. If the mesh is in-memory, there is, by
     definition, no compatible png and that should be signaled with an empty
     `mesh_path`.
   - Otherwise, if a default_diffuse value is provided, a material is created
     with the given default_diffuse color value.
   - Finally, if no material is defined, std::nullopt is returned. In such a
     case, a consumer of the returned mesh can generate its own material using
     its default diffuse color with MakeDiffuseMaterial(). Such a material would
     be compliant with the heuristic defined in @ref geometry_materials.

 References to textures will be included in the material iff they can be read
 and the `uv_state` is full. Otherwise, a warning will be dispatched.

 This doesn't account for any material properties that may or may not exist in
 a mesh. Its invocation assumes no such material exists. */
std::optional<RenderMaterial> MaybeMakeMeshFallbackMaterial(
    const GeometryProperties& props, const std::filesystem::path& mesh_path,
    const std::optional<Rgba>& default_diffuse,
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
