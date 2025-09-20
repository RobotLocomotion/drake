#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#include <array>
#include <utility>

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// #include "drake/geometry/render_vtk/factory.h"
// #include "drake/geometry/render_vtk/render_engine_vtk_params.h"

// Symbol: pydrake_doc_geometry_render_vtk
constexpr struct /* pydrake_doc_geometry_render_vtk */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::geometry
    struct /* geometry */ {
      // Symbol: drake::geometry::EnvironmentMap
      struct /* EnvironmentMap */ {
        // Source: drake/geometry/render_vtk/render_engine_vtk_params.h:43
        const char* doc = R"""()""";
        // Symbol: drake::geometry::EnvironmentMap::Serialize
        struct /* Serialize */ {
          // Source: drake/geometry/render_vtk/render_engine_vtk_params.h:47
          const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
        } Serialize;
        // Symbol: drake::geometry::EnvironmentMap::skybox
        struct /* skybox */ {
          // Source: drake/geometry/render_vtk/render_engine_vtk_params.h:54
          const char* doc =
R"""(If true, the environment map will be rendered in a sky box. If false,
it won't be visible in the background, but it will illuminate objects.)""";
        } skybox;
        // Symbol: drake::geometry::EnvironmentMap::texture
        struct /* texture */ {
          // Source: drake/geometry/render_vtk/render_engine_vtk_params.h:72
          const char* doc =
R"""(The texture to use for the environment map. If none is defined, there
is no environment map.

In yaml, this must be spelled out with the type tag. E.g.,


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    {yaml}
    environment_map:
    skybox: true
    texture: !EquirectangularMap
    path: /path/to/environment_image.hdr

.. raw:: html

    </details>

Warning:
    This feature is considered to be **experimental** and may change
    or be removed at any time, without any deprecation notice ahead of
    time.

Note:
    This will change from a simple file path to a more comprehensive
    URI soon. So, be aware you will have to change /path/image.png to
    file:///path/image.png in the near future.)""";
        } texture;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("skybox", skybox.doc),
            std::make_pair("texture", texture.doc),
          };
        }
      } EnvironmentMap;
      // Symbol: drake::geometry::EquirectangularMap
      struct /* EquirectangularMap */ {
        // Source: drake/geometry/render_vtk/render_engine_vtk_params.h:29
        const char* doc = R"""()""";
        // Symbol: drake::geometry::EquirectangularMap::Serialize
        struct /* Serialize */ {
          // Source: drake/geometry/render_vtk/render_engine_vtk_params.h:33
          const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
        } Serialize;
        // Symbol: drake::geometry::EquirectangularMap::path
        struct /* path */ {
          // Source: drake/geometry/render_vtk/render_engine_vtk_params.h:40
          const char* doc = R"""(The path to the map file.)""";
        } path;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("path", path.doc),
          };
        }
      } EquirectangularMap;
      // Symbol: drake::geometry::GltfExtension
      struct /* GltfExtension */ {
        // Source: drake/geometry/render_vtk/render_engine_vtk_params.h:78
        const char* doc =
R"""(Specifies how to deal with glTF "extensions" (non-standard
capabilities).
https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html#specifying-extensions)""";
        // Symbol: drake::geometry::GltfExtension::Serialize
        struct /* Serialize */ {
          // Source: drake/geometry/render_vtk/render_engine_vtk_params.h:82
          const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
        } Serialize;
        // Symbol: drake::geometry::GltfExtension::warn_unimplemented
        struct /* warn_unimplemented */ {
          // Source: drake/geometry/render_vtk/render_engine_vtk_params.h:90
          const char* doc =
R"""(Whether to log a warning when this extension is used (i.e., is listed
in ``extensionsUsed``) but isn't implemented by the render engine. By
default, all unimplemented extensions will log a warning, but users
can configure specific extensions to be quiet.)""";
        } warn_unimplemented;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("warn_unimplemented", warn_unimplemented.doc),
          };
        }
      } GltfExtension;
      // Symbol: drake::geometry::MakeRenderEngineVtk
      struct /* MakeRenderEngineVtk */ {
        // Source: drake/geometry/render_vtk/factory.h:87
        const char* doc =
R"""(Constructs a RenderEngine implementation which uses a VTK-based OpenGL
renderer.

Warning:
    On macOS, we've observed that RenderEngineVtk sometimes does not
    obey render::ColorRenderCamera::show_window when it's set to
    ``True``. Refer to issue `#20144
    <https://github.com/RobotLocomotion/drake/issues/20144>`_ for
    further discussion.

Note:
    On Ubuntu, render::ColorRenderCamera::show_window only shows a
    window when RenderEngineVtkParams::backend is set to "GLX"; the
    default backend value of "EGL" cannot show a window.

Geometry perception properties
------------------------------

This RenderEngine implementation looks for the following properties
when registering visual geometry, categorized by rendered image type.

**RGB images**

| Group name | Property Name | Required | Property Type | Property
Description | | :--------: | :-----------: | :------: |
:-------------: | :------------------- | | phong | diffuse | no¹ |
Eigen::Vector4d | The rgba value of the object surface. | | phong |
diffuse_map | no² | std::string | The path to a texture to apply to
the geometry.³⁴ |

¹ If no diffuse value is given, a default rgba value will be applied.
The default color is a bright orange. This default value can be
changed to a different value at construction.

² If no path is specified, or the file cannot be read, the diffuse
rgba value is used (or its default).

³ RenderEngineVtk implements a legacy feature for associating textures
with *meshes*. If *no* ``(phong, diffuse_map)`` property is provided
(or it refers to a file that doesn't exist), for a mesh named
``/path/to/mesh.obj``, RenderEngineVtk will search for a file
``/path/to/mesh.png`` (replacing "obj" with "png"). If that image
exists, it will be used as a texture on the mesh object. ⁴ The render
engine consumes pngs with uchar channels. Pngs with a different bit
depth, e.g., uint16 channels, will be converted to that.

Note:
    RenderEngineVtk does not support the OBJ format ``usemtl``
    directive. Instead, it has two ways to associate a color texture
    with an obj file: - File name matching; see footnote 3 above. -
    Explicit assignment of arbitrary texture files from within model
    files. In SDFormat, use the tag tag_drake_diffuse_map. In URDF,
    use ``//visual/material/texture``.

**Depth images**

No specific properties required.

**Label images**

| Group name | Property Name | Required | Property Type | Property
Description | | :--------: | :-----------: | :-----------: |
:-------------: | :------------------- | | label | id | no⁵ |
RenderLabel | The label to render into the image. |

⁵ When the label property is not set, RenderEngineVtk uses a default
render label of RenderLabel::kDontCare.

**Geometries accepted by RenderEngineVtk**

As documented in RenderEngine::RegisterVisual(), a RenderEngine
implementation can use the properties found in the
PerceptionProperties to determine whether it *accepts* a shape
provided for registration. RenderEngineVtk makes use of defaults to
accept *all* geometries (assuming the properties pass validation,
e.g., render label validation).)""";
      } MakeRenderEngineVtk;
      // Symbol: drake::geometry::NullTexture
      struct /* NullTexture */ {
        // Source: drake/geometry/render_vtk/render_engine_vtk_params.h:22
        const char* doc =
R"""((Internal use only) A place holder indicating that no texture has been
provided for environment map (and, therefore, no environment map).)""";
        // Symbol: drake::geometry::NullTexture::Serialize
        struct /* Serialize */ {
          // Source: drake/geometry/render_vtk/render_engine_vtk_params.h:26
          const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
        } Serialize;
      } NullTexture;
      // Symbol: drake::geometry::RenderEngineVtkParams
      struct /* RenderEngineVtkParams */ {
        // Source: drake/geometry/render_vtk/render_engine_vtk_params.h:94
        const char* doc =
R"""(Construction parameters for the RenderEngineVtk.)""";
        // Symbol: drake::geometry::RenderEngineVtkParams::Serialize
        struct /* Serialize */ {
          // Source: drake/geometry/render_vtk/render_engine_vtk_params.h:98
          const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
        } Serialize;
        // Symbol: drake::geometry::RenderEngineVtkParams::backend
        struct /* backend */ {
          // Source: drake/geometry/render_vtk/render_engine_vtk_params.h:258
          const char* doc =
R"""(Controls which graphics library will be used to perform the rendering.

Permissible values are the empty string (default), "GLX", "EGL", and
"Cocoa". Any other value will throw an error.

By default (i.e., when set to the empty string) the render engine will
choose which library to use. At the moment the default is "Cocoa" on
macOS and "EGL" on Linux.

If the option is set to one of the permissible values but the related
graphics library has not been compiled into current build (e.g., "GLX"
on macOS), then the default choice (empty string) will be used
instead, with a warning.

Note: RenderEngineVtk's API allows callers to request an interactive
display of the current rendering (such as when
systems::sensors::CameraConfig::show_rgb is set to ``True``). Whether
or not the debug window is displayed depends on the final backend
configuration of the RenderEngineVtk instance (after resolving default
values as documented above). *Currently*, the EGL background does not
support this display.)""";
        } backend;
        // Symbol: drake::geometry::RenderEngineVtkParams::cast_shadows
        struct /* cast_shadows */ {
          // Source: drake/geometry/render_vtk/render_engine_vtk_params.h:205
          const char* doc =
R"""(If ``True``, *all* lights that are *able* to cast shadows will do so.

Several important notes when designing your lighting:

- Point lights do not cast shadows.
- Spot lights will not cast shadows if the spot light angle is 90 degrees
or more. At 90 degrees, the spot light is now a half-point light.
Even though 89.9 degrees *would* enable shadows, it is still an
impractical value. To actually see shadows, the ``shadow_map_size`` value
would have to be absurdly large to support such a wide extent.
- Directional lights will create a shadow map that spans the whole scene.
If your scene includes a geometry that is significantly larger than
the locale you're rendering, this will significantly reduce the
efficacy of the directional light's shadows. Consider truncating that
larger geometry. A common case would be to use a HalfSpace to define
a ground. A half space has infinite extent, so any reasonable
approximation would be quite large. Better to use a box targeted to
where you need it.
- Transparent objects cast no shadows at all, but they do receive them.

Currently, there is no way to enable/disable shadows on a per-light
basis.)""";
        } cast_shadows;
        // Symbol: drake::geometry::RenderEngineVtkParams::default_clear_color
        struct /* default_clear_color */ {
          // Source: drake/geometry/render_vtk/render_engine_vtk_params.h:119
          const char* doc =
R"""(The rgb color to which the color buffer is cleared (each channel in
the range [0, 1]). The default value (in byte values) would be [204,
229, 255].)""";
        } default_clear_color;
        // Symbol: drake::geometry::RenderEngineVtkParams::default_diffuse
        struct /* default_diffuse */ {
          // Source: drake/geometry/render_vtk/render_engine_vtk_params.h:114
          const char* doc =
R"""(The (optional) rgba color to apply to the (phong, diffuse) property
when none is otherwise specified. Note: currently the alpha channel is
unused by RenderEngineVtk.)""";
        } default_diffuse;
        // Symbol: drake::geometry::RenderEngineVtkParams::environment_map
        struct /* environment_map */ {
          // Source: drake/geometry/render_vtk/render_engine_vtk_params.h:138
          const char* doc =
R"""(An optional environment map. When providing the environment map, the
render engine will be configured for physically-based rendering (PBR);
all materials will be promoted to be a PBR material. This may change
the appearance of any geometries introduced as primitives or .obj
meshes.

Furthermore, if an environment map is specified, it *replaces* the
default lighting (the map itself illuminates the scene). The usual
camera head lamp will not be present. Lights *can* be explicitly added
to combine with the environment map.)""";
        } environment_map;
        // Symbol: drake::geometry::RenderEngineVtkParams::exposure
        struct /* exposure */ {
          // Source: drake/geometry/render_vtk/render_engine_vtk_params.h:175
          const char* doc =
R"""(Exposure is an aspect of "tone mapping" (as described in `VTK's
description of its PBR capabilities
<https://www.kitware.com/pbrj1/>`_). Drake uses the GenericFilmic tone
mapper and this value maps to its ``exposure`` property.

By default, exposure is undefined and *no* tone mapping is applied to
the image. Providing any value at all will enable tone mapping and can
be done so whether your scene uses default Phong illumination model or
physically-based rendering (e.g., such as when an environment map is
present or a glTF model has been added).

This property is analogous to the concept of "exposure" in traditional
photography. Exposure controls how much the film material is exposed
to the light in the scene. Too little exposure and the scene is dark
and muddy. Too much exposure, and the image becomes over saturated. As
the amount of radiant energy in the scene increases, reducing the
exposure may be necessary. In essence, exposure serves as a scale
factor on the radiant energy. When thinking about it as a scale
factor, it may *seem* that setting exposure to one would leave the
image unchanged (with respect to the images that RenderEngineVtk has
historically produced). This is not the case. As noted above, setting
any value, including one, enables tone mapping. Without tone mapping
enabled, there is a direct correlation between radiant energy on the
surface and the final pixel colors. Tone mapping defines a non-linear
relationship between illumination levels and pixel color.

The most common use for the ``exposure`` parameter is to combine an
environment map with shadow-casting lights. If the environment map
contains a great deal of light energy, it may overpower the strength
of your lights. By reducing the exposure (below one), the shadows
caused by the lights will become more distinct.

If you plan on working with shadows and environment maps, best
practice will be to set exposure to one and enable tone mapping. You
can increase the value if your image seems too dark, or reduce the
value if it seems "washed out".)""";
        } exposure;
        // Symbol: drake::geometry::RenderEngineVtkParams::force_to_pbr
        struct /* force_to_pbr */ {
          // Source: drake/geometry/render_vtk/render_engine_vtk_params.h:228
          const char* doc =
R"""(RenderEngineVtk can use one of two illumination models: Phong or
Physically based rendering (PBR). It defaults to Phong. However, it
automatically switches to PBR if:

- an environment map is added, or
- a glTF mesh is added.

If ``force_to_pbr`` is set to true, it switches the engine to use PBR
regardless of the scene's contents.

Be aware, switching to PBR will lead to a qualitative change in
rendered images even if literally nothing else changes.)""";
        } force_to_pbr;
        // Symbol: drake::geometry::RenderEngineVtkParams::gltf_extensions
        struct /* gltf_extensions */ {
          // Source: drake/geometry/render_vtk/render_engine_vtk_params.h:232
          const char* doc =
R"""(Map from the name of a glTF extension (e.g., "KHR_materials_sheen") to
render engine settings related to that extension.)""";
        } gltf_extensions;
        // Symbol: drake::geometry::RenderEngineVtkParams::lights
        struct /* lights */ {
          // Source: drake/geometry/render_vtk/render_engine_vtk_params.h:127
          const char* doc =
R"""(Lights in the scene. If no lights are defined, a single directional
light, fixed to the camera frame, is used.

Note: RenderEngineVtk does not have a hard-coded limit on the number
of lights; but more lights increases rendering cost. Note: the
attenuation values have no effect on VTK *directional* lights.)""";
        } lights;
        // Symbol: drake::geometry::RenderEngineVtkParams::shadow_map_size
        struct /* shadow_map_size */ {
          // Source: drake/geometry/render_vtk/render_engine_vtk_params.h:214
          const char* doc =
R"""(The size of texture map (in pixels) to use for shadow maps. Note: this
is a *global* setting. All shadow casting lights will use a map of the
same size. Larger map sizes increase GPU memory usage and rendering
times but improve shadow fidelity (less obvious pixelation).

See the note on ``cast_shadows`` for the warning on directional lights
and shadow maps.)""";
        } shadow_map_size;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("backend", backend.doc),
            std::make_pair("cast_shadows", cast_shadows.doc),
            std::make_pair("default_clear_color", default_clear_color.doc),
            std::make_pair("default_diffuse", default_diffuse.doc),
            std::make_pair("environment_map", environment_map.doc),
            std::make_pair("exposure", exposure.doc),
            std::make_pair("force_to_pbr", force_to_pbr.doc),
            std::make_pair("gltf_extensions", gltf_extensions.doc),
            std::make_pair("lights", lights.doc),
            std::make_pair("shadow_map_size", shadow_map_size.doc),
          };
        }
      } RenderEngineVtkParams;
      // Symbol: drake::geometry::render_vtk
      struct /* render_vtk */ {
      } render_vtk;
    } geometry;
  } drake;
} pydrake_doc_geometry_render_vtk;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
