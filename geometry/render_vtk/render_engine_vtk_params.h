#pragma once

#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/name_value.h"
#include "drake/common/string_map.h"
#include "drake/geometry/render/light_parameter.h"
#include "drake/geometry/render/render_label.h"

namespace drake {
namespace geometry {

// TODO(SeanCurtis-TRI): When CubeMap is implemented, replace NullTexture with
// CubeMap.

/** (Internal use only) A place holder indicating that no texture has been
 provided for environment map (and, therefore, no environment map). */
struct NullTexture {
  /** Passes this object to an Archive.
   Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive*) {}
};

struct EquirectangularMap {
  /** Passes this object to an Archive.
   Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(path));
  }

  // TODO(SeanCurtis-TRI): It would be nice if this supported package. To that
  // end proper URIs including file:// or even data://, I suppose.
  /** The path to the map file. */
  std::string path;
};

struct EnvironmentMap {
  /** Passes this object to an Archive.
   Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(skybox));
    a->Visit(DRAKE_NVP(texture));
  }

  /** If true, the environment map will be rendered in a sky box. If false, it
   won't be visible in the background, but it will illuminate objects. */
  bool skybox{true};

  /** The texture to use for the environment map. If none is defined, there is
   no environment map.

   In yaml, this must be spelled out with the type tag. E.g.,

   @code{yaml}
     environment_map:
       skybox: true
       texture: !EquirectangularMap
         path: /path/to/environment_image.hdr
   @endcode

   @experimental
   @note This will change from a simple file path to a more comprehensive URI
   soon. So, be aware you will have to change /path/image.png to
   file:///path/image.png in the near future. */
  std::variant<NullTexture, EquirectangularMap> texture;
};

/** Specifies how to deal with glTF "extensions" (non-standard capabilities).
https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html#specifying-extensions
*/
struct GltfExtension {
  /** Passes this object to an Archive.
   Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(warn_unimplemented));
  }

  /** Whether to log a warning when this extension is used (i.e., is listed in
   `extensionsUsed`) but isn't implemented by the render engine. By default,
   all unimplemented extensions will log a warning, but users can configure
   specific extensions to be quiet. */
  bool warn_unimplemented{true};
};

/** Construction parameters for the RenderEngineVtk.  */
struct RenderEngineVtkParams {
  /** Passes this object to an Archive.
   Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(default_diffuse));
    a->Visit(DRAKE_NVP(default_clear_color));
    a->Visit(DRAKE_NVP(lights));
    a->Visit(DRAKE_NVP(environment_map));
    a->Visit(DRAKE_NVP(exposure));
    a->Visit(DRAKE_NVP(cast_shadows));
    a->Visit(DRAKE_NVP(shadow_map_size));
    a->Visit(DRAKE_NVP(force_to_pbr));
    a->Visit(DRAKE_NVP(gltf_extensions));
    a->Visit(DRAKE_NVP(backend));
  }

  /** The (optional) rgba color to apply to the (phong, diffuse) property when
    none is otherwise specified. Note: currently the alpha channel is unused
    by RenderEngineVtk.  */
  std::optional<Eigen::Vector4d> default_diffuse{};

  /** The rgb color to which the color buffer is cleared (each
   channel in the range [0, 1]). The default value (in byte values) would be
   [204, 229, 255].  */
  Eigen::Vector3d default_clear_color{204 / 255., 229 / 255., 255 / 255.};

  /** Lights in the scene. If no lights are defined, a single directional light,
   fixed to the camera frame, is used.

   Note: RenderEngineVtk does not have a hard-coded limit on the number of
         lights; but more lights increases rendering cost.
   Note: the attenuation values have no effect on VTK *directional* lights. */
  std::vector<render::LightParameter> lights;

  /** An optional environment map. When providing the environment map, the
   render engine will be configured for physically-based rendering (PBR); all
   materials will be promoted to be a PBR material. This may change the
   appearance of any geometries introduced as primitives or .obj meshes.

   Furthermore, if an environment map is specified, it *replaces* the default
   lighting (the map itself illuminates the scene). The usual camera head lamp
   will not be present. Lights *can* be explicitly added to combine with the
   environment map. */
  std::optional<EnvironmentMap> environment_map;

  /** Exposure is an aspect of "tone mapping" (as described in
   <a href="https://www.kitware.com/pbrj1/">VTK's description of its PBR
   capabilities</a>). Drake uses the GenericFilmic tone mapper and this value
   maps to its `exposure` property.

   By default, exposure is undefined and *no* tone mapping is applied to the
   image. Providing any value at all will enable tone mapping and can be done
   so whether your scene uses default Phong illumination model or
   physically-based rendering (e.g., such as when an environment map is present
   or a glTF model has been added).

   This property is analogous to the concept of "exposure" in traditional
   photography. Exposure controls how much the film material is exposed to the
   light in the scene. Too little exposure and the scene is dark and muddy. Too
   much exposure, and the image becomes over saturated. As the amount of radiant
   energy in the scene increases, reducing the exposure may be necessary. In
   essence, exposure serves as a scale factor on the radiant energy. When
   thinking about it as a scale factor, it may *seem* that setting exposure to
   one would leave the image unchanged (with respect to the images that
   %RenderEngineVtk has historically produced). This is not the case. As noted
   above, setting any value, including one, enables tone mapping. Without tone
   mapping enabled, there is a direct correlation between radiant energy on
   the surface and the final pixel colors. Tone mapping defines a non-linear
   relationship between illumination levels and pixel color.

   The most common use for the `exposure` parameter is to combine an environment
   map with shadow-casting lights. If the environment map contains a great
   deal of light energy, it may overpower the strength of your lights. By
   reducing the exposure (below one), the shadows caused by the lights will
   become more distinct.

   If you plan on working with shadows and environment maps, best practice will
   be to set exposure to one and enable tone mapping. You can increase the value
   if your image seems too dark, or reduce the value if it seems "washed out".
   */
  std::optional<double> exposure{};

  /** If `true`, *all* lights that are *able* to cast shadows will do so.

   Several important notes when designing your lighting:

       - Point lights do not cast shadows.
       - Spot lights will not cast shadows if the spot light angle is 90 degrees
         or more. At 90 degrees, the spot light is now a half-point light.
         Even though 89.9 degrees _would_ enable shadows, it is still an
         impractical value. To actually see shadows, the `shadow_map_size` value
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

   Currently, there is no way to enable/disable shadows on a per-light basis.

   <!-- TODO(SeanCurtis-TRI): Figure out a way to set this on a per-light
    basis. One *could* use the vtkLight::ShadowAttenuation property, but it
    would still incur the cost of rendering a shadow map. The best option would
    be to upstream/patch a change to VTK where each light could simply declare
    its ability to cast shadows. We'll wait to see how common this need is. -->
  */
  bool cast_shadows{false};

  /** The size of texture map (in pixels) to use for shadow maps. Note: this is
   a *global* setting. All shadow casting lights will use a map of the same
   size. Larger map sizes increase GPU memory usage and rendering times but
   improve shadow fidelity (less obvious pixelation).

   See the note on `cast_shadows` for the warning on directional lights and
   shadow maps. */
  int shadow_map_size{256};

  /** RenderEngineVtk can use one of two illumination models: Phong or
   Physically based rendering (PBR). It defaults to Phong. However, it
   automatically switches to PBR if:

       - an environment map is added, or
       - a glTF mesh is added.

   If `force_to_pbr` is set to true, it switches the engine to use PBR
   regardless of the scene's contents.

   Be aware, switching to PBR will lead to a qualitative change in rendered
   images even if literally nothing else changes. */
  bool force_to_pbr{false};

  /** Map from the name of a glTF extension (e.g., "KHR_materials_sheen") to
   render engine settings related to that extension. */
  string_map<GltfExtension> gltf_extensions{
      // The basisu extension is commonplace in the Drake ecosystem, because it
      // vastly speeds up Meshcat visualizer loading. We suppress VTK warnings
      // about it by default, to avoid too much warning spam.
      {"KHR_texture_basisu", {.warn_unimplemented = false}},
  };

  /** Controls which graphics library will be used to perform the rendering.

  Permissible values are the empty string (default), "GLX", "EGL", and "Cocoa".
  Any other value will throw an error.

  By default (i.e., when set to the empty string) the render engine will choose
  which library to use. At the moment the default is "Cocoa" on macOS and "EGL"
  on Linux.

  If the option is set to one of the permissible values but the related graphics
  library has not been compiled into current build (e.g., "GLX" on macOS), then
  the default choice (empty string) will be used instead, with a warning.

  Note: %RenderEngineVtk's API allows callers to request an interactive display
  of the current rendering (such as when
  systems::sensors::CameraConfig::show_rgb is set to `true`). A window will
  only be displayed if the render engine's `backend` is set to "GLX" or "Cocoa".
  For any other backend value, the request to show the debug window during
  rendering is ignored. */
  std::string backend;
};

namespace render_vtk {
namespace internal {

/* A parsed enum form of the RenderEngineVtkParams.backend string. */
enum class RenderEngineVtkBackend {
  kCocoa,
  kEgl,
  kGlx,
};

/* Parses the parameters.backend string to an enum and warns or throws per the
validation logic documented on the `backend` field. */
RenderEngineVtkBackend ParseRenderEngineVtkBackend(
    const RenderEngineVtkParams& parameters);

}  // namespace internal
}  // namespace render_vtk

}  // namespace geometry
}  // namespace drake
