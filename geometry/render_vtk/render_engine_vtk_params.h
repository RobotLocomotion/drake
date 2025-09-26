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

/** Screen-space ambient occlusion (SSAO) parameters.

 Ambient occlusion is a shading method used to calculate how exposed each point
 in a scene is to ambient lighting. The more occluded a point is, the darker it
 appears. SSAO is an efficient, real-time approximation of ambient occlusion
 that operates in screen space. It enhances the depth and realism of a scene by
 adding subtle shadowing effects in creases, holes, and surfaces that are close
 to each other. See https://www.kitware.com/ssao/ for an overview.

 The default parameter values below have been chosen as a best-guess effort at
 producing good quality images. They may not be optimal for your particular
 scene.

 To understand how to tune these parameters, you need a bit of insight into how
 ambient occlusion works generally, and how SSAO works specifically. For each
 pixel, its "occlusion factor" is determined by examining a hemispherical area
 around the pixel's position in 3D space (oriented based on its normal). The
 fraction of the hemisphere contained within geometry is the occlusion factor.

 SSAO produces a discrete approximation of the true occlusion factor. It
 evaluates a discrete number of samples within the hemisphere and compares their
 depths (in the OpenGL-camera sense) with that of the pixel being shaded to
 determine how much of the hemisphere is occupied by geometry. As with all
 discrete approximations, there is a tradeoff between fidelity and cost. The
 parameter documentation below provides some guidance on the effect of each
 parameter on the final rendered image. */
struct SsaoParameter {
  /** Passes this object to an Archive.
   Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(radius));
    a->Visit(DRAKE_NVP(bias));
    a->Visit(DRAKE_NVP(sample_count));
    a->Visit(DRAKE_NVP(intensity_scale));
    a->Visit(DRAKE_NVP(intensity_shift));
    a->Visit(DRAKE_NVP(blur));
  }

  /** The radius (in meters) of the sampling hemisphere. There is no "correct"
   value. Heuristically, it should scale with your scene. A rubric of 1/10th the
   size of your scene is reasonable. For example, 0.25 meters is good for a
   robot working at a table top.

   If the radius is too large, pixels will exhibit occlusion (darkness) from
   objects that might seem too distant to be relevant. A too-small radius would
   have the opposite effect; missing occlusion where it seems it should occur.
   */
  double radius{0.25};

  /** SSAO works in screen space; so it uses the depth image to determine
   whether a sample lies within the hemisphere or not. We compare the depth of
   a sample point with the recorded depth in the same direction. In principle,
   if the sample point is farther than the recorded depth, the sample is
   occluded. This bias pads that calculation by specifying how far in front (in
   meters) the recorded distance has to be before the sample is considered
   occluded. Larger bias values will classify fewer samples as occluded,
   reducing the amount of ambient occlusion (darkness) in the final image.

   A non-zero value is usually helpful to help resolve potential issues due to
   depth map precision issues (so-called "acne"), but it should generally be
   small. */
  double bias{0.01};

  /** This is simply the number of samples taken. More samples lead to smoother
   occlusion patterns. Large numbers will produce better images but at a higher
   computational cost. You should use the smallest number that still provides
   acceptable visual quality. */
  int sample_count{128};

  /** Once the occlusion factor is computed, prior to applying the factor to
   the shaded pixel, you can apply a final affine transformation:

       occlusion = (occlusion - intensity_shift) * intensity_scale

   Using these two values allows you to tune the contrast and mean occlusion
   value independent of the sampling algorithm above. Remember, the more
   occlusion, the more darkness. So, scale factors greater than one will make
   the image darker as will *negative* shift values.

   One reason to consider shifting is is based on the total lighting in the
   scene. For a very brightly lit scene with a large camera exposure, the
   ambient occlusion effects should have a smaller influence. A scale less
   than one, or a shift greater than zero, will reduce the SSAO effect. */
  double intensity_scale{1.0};
  double intensity_shift{0.0};

  /** The discrete sampling approach will ultimately produce shading with a
   noisy pattern. More samples will reduce the noise, but you can also apply a
   blur to the final occlusion image to reduce the noise. This is a screen
   space blur, so it is fast. However, it will also reduce the sharpness of
   occlusion patterns. You can disable blurring to speed things up, but it will
   emphasize the sampling noise. */
  bool blur{true};
};

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
    a->Visit(DRAKE_NVP(ssao));
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

  /** An optional SSAO (screen-space ambient occlusion) parameters set. When
   specified, VTK enables screen-space ambient occlusion configured by the
   given parameters. */
  std::optional<SsaoParameter> ssao{};

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
  systems::sensors::CameraConfig::show_rgb is set to `true`). Whether or not the
  debug window is displayed depends on the final backend configuration of the
  %RenderEngineVtk instance (after resolving default values as documented
  above). _Currently_, the EGL background does not support this display. */
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
