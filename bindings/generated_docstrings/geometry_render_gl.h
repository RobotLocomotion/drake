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

// #include "drake/geometry/render_gl/factory.h"
// #include "drake/geometry/render_gl/render_engine_gl_params.h"

// Symbol: pydrake_doc_geometry_render_gl
constexpr struct /* pydrake_doc_geometry_render_gl */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::geometry
    struct /* geometry */ {
      // Symbol: drake::geometry::MakeRenderEngineGl
      struct /* MakeRenderEngineGl */ {
        // Source: drake/geometry/render_gl/factory.h
        const char* doc =
R"""(Constructs a RenderEngine implementation which uses a purely OpenGL
renderer. The engine only works under Ubuntu. If called on a Mac, it
will throw.

Note:
    RenderEngineGl behaves a bit differently from other RenderEngine
    implementations (e.g., RenderEngineVtk) with respect to displayed
    images. RenderEngineGl can only display a *single* image type at a
    time. So, if ``show_window`` has been requested for both label and
    color images, the images will alternate quickly (flickering) in
    the shared window.

** Using RenderEngineGl in multiple threads **

Most importantly, a single RenderEngineGl should *not* be exercised in
multiple threads. One thread, one RenderEngineGl instance.

A RenderEngineGl instance and its *clones* can be used in different
threads simultaneously, but *only* the rendering APIs are threadsafe.
Do not mutate the contents of the engine (e.g., adding/removing
geometries, etc.) in parallel.

Two independently constructed RenderEngineGl instances can be freely
used in different threads -- all APIs are available.

The expected workflow is to add a RenderEngineGl instance a SceneGraph
instance (see SceneGraph∷AddRenderer()) and then to populate
SceneGraph with the desired geometry. Each systems∷Context allocated
for that SceneGraph will receive a clone of the original
RenderEngineGl. One systems∷Context can be used per thread to create
rendered images in parallel.

Raises:
    RuntimeError if kHasRenderEngineGl is false.)""";
      } MakeRenderEngineGl;
      // Symbol: drake::geometry::RenderEngineGlParams
      struct /* RenderEngineGlParams */ {
        // Source: drake/geometry/render_gl/render_engine_gl_params.h
        const char* doc =
R"""(Construction parameters for RenderEngineGl.)""";
        // Symbol: drake::geometry::RenderEngineGlParams::Serialize
        struct /* Serialize */ {
          // Source: drake/geometry/render_gl/render_engine_gl_params.h
          const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
        } Serialize;
        // Symbol: drake::geometry::RenderEngineGlParams::cast_shadows
        struct /* cast_shadows */ {
          // Source: drake/geometry/render_gl/render_engine_gl_params.h
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
If an object is made transparent strictly through its diffuse texture,
it will still cast shadows as if were fully opaque.
- If you only plan to render depth or label images, leave this as false.
Setting it as true will increase start up time and GPU memory usage to
support shadows that will never be used.

Currently, there is no way to enable/disable shadows on a per-light
basis.)""";
        } cast_shadows;
        // Symbol: drake::geometry::RenderEngineGlParams::default_clear_color
        struct /* default_clear_color */ {
          // Source: drake/geometry/render_gl/render_engine_gl_params.h
          const char* doc =
R"""(The default background color for color images.)""";
        } default_clear_color;
        // Symbol: drake::geometry::RenderEngineGlParams::default_diffuse
        struct /* default_diffuse */ {
          // Source: drake/geometry/render_gl/render_engine_gl_params.h
          const char* doc =
R"""(Default diffuse color to apply to a geometry when none is otherwise
specified in the (phong, diffuse) property.)""";
        } default_diffuse;
        // Symbol: drake::geometry::RenderEngineGlParams::lights
        struct /* lights */ {
          // Source: drake/geometry/render_gl/render_engine_gl_params.h
          const char* doc =
R"""(Lights in the scene. If no lights are defined, a single directional
light, fixed to the camera frame, is used.

Note: RenderEngineGl does not have a hard-coded limit on the number of
lights, but more lights increases rendering cost.)""";
        } lights;
        // Symbol: drake::geometry::RenderEngineGlParams::shadow_map_size
        struct /* shadow_map_size */ {
          // Source: drake/geometry/render_gl/render_engine_gl_params.h
          const char* doc =
R"""(The size of texture map (in pixels) to use for shadow maps. Note: this
is a *global* setting. All shadow casting lights will use a map of the
same size. Larger map sizes increase GPU memory usage and rendering
times but improve shadow fidelity (less obvious pixelation).

See the note on ``cast_shadows`` for the warning on directional lights
and shadow maps.

Precondition:
    shadow_map_size is positive and does not exceed the OpenGL
    implementation's maximum allowable texture size.)""";
        } shadow_map_size;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("cast_shadows", cast_shadows.doc),
            std::make_pair("default_clear_color", default_clear_color.doc),
            std::make_pair("default_diffuse", default_diffuse.doc),
            std::make_pair("lights", lights.doc),
            std::make_pair("shadow_map_size", shadow_map_size.doc),
          };
        }
      } RenderEngineGlParams;
    } geometry;
  } drake;
} pydrake_doc_geometry_render_gl;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
