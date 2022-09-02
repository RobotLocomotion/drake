#pragma once

#include <memory>
#include <optional>

#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/render_vtk/render_engine_vtk_params.h"

namespace drake {
namespace geometry {

/** Constructs a RenderEngine implementation which uses a VTK-based OpenGL
 renderer.

 @anchor render_engine_vtk_properties
 <h2>Geometry perception properties</h2>

 This RenderEngine implementation looks for the following properties when
 registering visual geometry, categorized by rendered image type.

 <h3>RGB images</h3>

 | Group name | Property Name | Required |  Property Type  | Property Description |
 | :--------: | :-----------: | :------: | :-------------: | :------------------- |
 |    phong   | diffuse       | no¹      | Eigen::Vector4d | The rgba value of the object surface. |
 |    phong   | diffuse_map   | no²      | std::string     | The path to a texture to apply to the geometry.³ |

 ¹ If no diffuse value is given, a default rgba value will be applied. The
   default color is a bright orange. This default value can be changed to a
   different value at construction. <br>
 ² If no path is specified, or the file cannot be read, the diffuse rgba value
   is used (or its default). <br>
 ³ %RenderEngineVtk implements a legacy feature for associating textures with
   _meshes_. If _no_ `(phong, diffuse_map)` property is provided (or it refers
   to a file that doesn't exist), for a mesh named `/path/to/mesh.obj`,
   %RenderEngineVtk will search for a file `/path/to/mesh.png` (replacing "obj"
   with "png"). If that image exists, it will be used as a texture on the mesh
   object.

 @note RenderEngineVtk does not support the OBJ format `usemtl`
 directive. Instead, it has two ways to associate a color texture with an obj
 file:
 - File name matching; see footnote 3 above.
 - Explicit assignment of arbitrary texture files from within model files. In
   SDFormat, use the tag @ref tag_drake_diffuse_map. In URDF, use
   `//visual/material/texture`.

<!-- TODO(#11949): fix material/texture handling. -->

 <h3>Depth images</h3>

 No specific properties required.

 <h3>Label images</h3>

 | Group name | Property Name |   Required    |  Property Type  | Property Description |
 | :--------: | :-----------: | :-----------: | :-------------: | :------------------- |
 |   label    | id            | configurable⁵ |  RenderLabel    | The label to render into the image. |

 ⁵ %RenderEngineVtk has a default render label value that is applied to any
 geometry that doesn't have a (label, id) property at registration. If a value
 is not explicitly specified, %RenderEngineVtk uses RenderLabel::kUnspecified
 as this default value. It can be explicitly set upon construction. The possible
 values for this default label and the ramifications of that choice are
 documented @ref render_engine_default_label "here".

 <h3>Geometries accepted by %RenderEngineVtk</h3>

 As documented in RenderEngine::RegisterVisual(), a RenderEngine implementation
 can use the properties found in the PerceptionProperties to determine whether
 it _accepts_ a shape provided for registration. %RenderEngineVtk makes use of
 defaults to accept _all_ geometries (assuming the properties pass validation,
 e.g., render label validation).
 <!-- TODO(SeanCurtis-TRI): Change this policy to be more selective when other
      renderers with different properties are introduced. -->
 */
std::unique_ptr<render::RenderEngine> MakeRenderEngineVtk(
    const RenderEngineVtkParams& params);

}  // namespace geometry
}  // namespace drake
