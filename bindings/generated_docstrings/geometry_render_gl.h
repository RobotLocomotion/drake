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
    images. First, RenderEngineGl can only display a *single* image
    type at a time. So, if a shown window has been requested for both
    label and color images, the images will alternate in the same
    window. Second, the window display draws all images *flipped
    vertically*. The image produced will be compatible with the Drake
    ecosystem, only the visualization will be upside down. This has
    been documented in
    https://github.com/RobotLocomotion/drake/issues/14254.

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
R"""(Lights in the scene. More than five lights is an error. If no lights
are defined, a single directional light, fixed to the camera frame, is
used.)""";
        } lights;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("default_clear_color", default_clear_color.doc),
            std::make_pair("default_diffuse", default_diffuse.doc),
            std::make_pair("lights", lights.doc),
          };
        }
      } RenderEngineGlParams;
    } geometry;
  } drake;
} pydrake_doc_geometry_render_gl;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
