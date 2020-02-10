#pragma once

#include <memory>
#include <optional>

#include "drake/geometry/render/render_engine.h"

namespace drake {
namespace geometry {
namespace render {

/** Construction parameters for the RenderEngineVtk.  */
struct RenderEngineVtkParams  {
  /** The (optional) label to apply when none is otherwise specified.  */
  std::optional<RenderLabel> default_label{};

  /** The (optional) rgba color to apply to the (phong, diffuse) property when
    none is otherwise specified. Note: currently the alpha channel is unused
    by RenderEngineVtk.  */
  std::optional<Eigen::Vector4d> default_diffuse{};

  /** The rgb color to which the color buffer is cleared (each
   channel in the range [0, 1]). The default value (in byte values) would be
   [204, 229, 255].  */
  Eigen::Vector3d default_clear_color{204 / 255., 229 / 255., 255 / 255.};
};

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
   is used (or its default).
 ³ %RenderEngineVtk implements a legacy feature for associating textures with
   _meshes_. If _no_ `(phong, diffuse_map)` property is provided (or it refers
   to a file that doesn't exist), for a mesh named `/path/to/mesh.obj`,
   %RenderEngineVtk will search for a file `/path/to/mesh.png` (replacing "obj"
   with "png"). If that image exists, it will be used as a texture on the mesh
   object.

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
std::unique_ptr<RenderEngine> MakeRenderEngineVtk(
    const RenderEngineVtkParams& params);

}  // namespace render
}  // namespace geometry
}  // namespace drake
