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

// #include "drake/geometry/render/light_parameter.h"
// #include "drake/geometry/render/render_camera.h"
// #include "drake/geometry/render/render_engine.h"
// #include "drake/geometry/render/render_label.h"
// #include "drake/geometry/render/render_material.h"
// #include "drake/geometry/render/render_mesh.h"

// Symbol: pydrake_doc_geometry_render
constexpr struct /* pydrake_doc_geometry_render */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::geometry
    struct /* geometry */ {
      // Symbol: drake::geometry::render
      struct /* render */ {
        // Symbol: drake::geometry::render::ClippingRange
        struct /* ClippingRange */ {
          // Source: drake/geometry/render/render_camera.h
          const char* doc =
R"""(Defines the near and far clipping planes for frustum-based (OpenGL)
RenderEngine cameras.

**Guidance on selecting clipping plane values**

This documentation is targeted toward those who are unfamiliar with
the OpenGL rasterization pipeline. For in-depth explanations about how
the clipping range defines the viewing volume, see ` the discussion on
projective transforms
<https://www.glprogramming.com/red/chapter03.html#name3>`_. For more
detail on its effect on determining occlusion (which geometries are in
front), try ` "A Hidden-Surface Removal Survival Kit"
<https://www.glprogramming.com/red/chapter05.html#name1>`_.

*The short summary*

- The clipping range defines the distance of the *closest* and *farthest*
things that *can* appear in the rendering.
- Objects that cross the planes placed at those distances get clipped.
- Make the range as small as reasonably possible to get the best occlusion
(a.k.a. z-buffer) results.
- For depth cameras, make sure your clipping range always includes your valid
depth range.

*The longer discussion*

Given that the clipping range defines what you can/can't see in the
camera, it *might* be tempting to just put an arbitrarily large range
in (e.g., starting 1 micrometer away and going up to 1 million
kilometers away). By doing so, you know everything you put into your
scene will appear. If making sure things are visible were the only
factor, this would be fine.

Rasterization pipelines render objects in arbitrary order but have to
be able to determine which objects are in front of other objects as
they go. They achieve this by creating a "z-buffer". It is a measure
of the depth of the triangle that rendered to a particular "pixel".
When two triangles both want to color the same pixel, the triangle
with the smallest z-value is typically selected.

The z-buffer has fixed precision. That fixed precision is spread over
the entire available depth range (as defined by the clipping range).
The greater the range, the less precision the z-buffer has per meter
of depth. A small range may have the ability to distinguish objects
separated by 1 mm. But a large range may only be able to distinuish
objects separated by 1 m. Two objects with measurably different depths
relative to the camera, can become indistinguishable in the z-buffer
due to these precision issues; the object that ends up in front is due
to random chance. This will lead to artifacts where two objects near
the same depth will flicker back and forth in front of each other
(sometimes called "z fighting").

So, it is best to define the smallest clipping range that will include
the objects of the scene that you care about most.)""";
          // Symbol: drake::geometry::render::ClippingRange::ClippingRange
          struct /* ctor */ {
            // Source: drake/geometry/render/render_camera.h
            const char* doc =
R"""(Constructs the ClippingRange.

Raises:
    RuntimeError if either value isn't finite and positive, or if
    ``near >= far``.)""";
          } ctor;
          // Symbol: drake::geometry::render::ClippingRange::far
          struct /* far */ {
            // Source: drake/geometry/render/render_camera.h
            const char* doc = R"""()""";
          } far;
          // Symbol: drake::geometry::render::ClippingRange::near
          struct /* near */ {
            // Source: drake/geometry/render/render_camera.h
            const char* doc = R"""()""";
          } near;
        } ClippingRange;
        // Symbol: drake::geometry::render::ColorRenderCamera
        struct /* ColorRenderCamera */ {
          // Source: drake/geometry/render/render_camera.h
          const char* doc =
R"""(Collection of camera properties for cameras to be used with
color/label images.)""";
          // Symbol: drake::geometry::render::ColorRenderCamera::ColorRenderCamera
          struct /* ctor */ {
            // Source: drake/geometry/render/render_camera.h
            const char* doc =
R"""(Fully-specified constructor. See the documentation on the member
getter methods for documentation of parameters.)""";
          } ctor;
          // Symbol: drake::geometry::render::ColorRenderCamera::core
          struct /* core */ {
            // Source: drake/geometry/render/render_camera.h
            const char* doc = R"""(This camera's core render properties.)""";
          } core;
          // Symbol: drake::geometry::render::ColorRenderCamera::show_window
          struct /* show_window */ {
            // Source: drake/geometry/render/render_camera.h
            const char* doc =
R"""(If true, requests that the RenderEngine display the rendered image.
Whether or not the image is able to be displayed depends on the
specific render engine and its configuration.)""";
          } show_window;
        } ColorRenderCamera;
        // Symbol: drake::geometry::render::DepthRange
        struct /* DepthRange */ {
          // Source: drake/geometry/render/render_camera.h
          const char* doc =
R"""(Defines a depth sensor's functional range. Only points that lie within
the range ``[min_depth, max_depth]`` will register meaningful values.

Note:
    It's important to carefully coordinate depth range and clipping
    planes. It might seem reasonable to use the depth range as
    clipping planes, but that would be a mistake. Objects closer than
    the depth range's minimum value have an occluding effect in
    reality. If the near clipping plane is set to the minimum depth
    range value, those objects will be clipped away and won't occlude
    as they should. In essence, the camera will see through them and
    return incorrect values from beyond the missing geometry. The near
    clipping plane should *always* be closer than the minimum depth
    range. How much closer depends on the scenario. Given the
    scenario, evaluate the closest possible distance to the camera
    that geometry in the scene could possibly achieve; the clipping
    plane should be slightly closer than that. When in doubt, some
    very small value (e.g., 1 mm) is typically safe.)""";
          // Symbol: drake::geometry::render::DepthRange::DepthRange
          struct /* ctor */ {
            // Source: drake/geometry/render/render_camera.h
            const char* doc =
R"""(Constructs the DepthRange.

Raises:
    RuntimeError if either value isn't finite and positive, or if
    ``min_in >= max_in``.)""";
          } ctor;
          // Symbol: drake::geometry::render::DepthRange::max_depth
          struct /* max_depth */ {
            // Source: drake/geometry/render/render_camera.h
            const char* doc = R"""()""";
          } max_depth;
          // Symbol: drake::geometry::render::DepthRange::min_depth
          struct /* min_depth */ {
            // Source: drake/geometry/render/render_camera.h
            const char* doc = R"""()""";
          } min_depth;
        } DepthRange;
        // Symbol: drake::geometry::render::DepthRenderCamera
        struct /* DepthRenderCamera */ {
          // Source: drake/geometry/render/render_camera.h
          const char* doc =
R"""(Collection of camera properties for cameras to be used with depth
images.)""";
          // Symbol: drake::geometry::render::DepthRenderCamera::DepthRenderCamera
          struct /* ctor */ {
            // Source: drake/geometry/render/render_camera.h
            const char* doc =
R"""(Fully-specified constructor. See the documentation on the member
getter methods for documentation of parameters.

Raises:
    RuntimeError if the depth_range is not fully contained within the
    clipping range.)""";
          } ctor;
          // Symbol: drake::geometry::render::DepthRenderCamera::core
          struct /* core */ {
            // Source: drake/geometry/render/render_camera.h
            const char* doc = R"""(This camera's core render properties.)""";
          } core;
          // Symbol: drake::geometry::render::DepthRenderCamera::depth_range
          struct /* depth_range */ {
            // Source: drake/geometry/render/render_camera.h
            const char* doc =
R"""(The range of valid values for the depth camera.)""";
          } depth_range;
        } DepthRenderCamera;
        // Symbol: drake::geometry::render::LightFrame
        struct /* LightFrame */ {
          // Source: drake/geometry/render/light_parameter.h
          const char* doc =
R"""(Specifies the frame in which a light is fixed.

Fixing the camera to the world keeps it stationary. Fixing it to the
camera moves it with the camera.)""";
          // Symbol: drake::geometry::render::LightFrame::kCamera
          struct /* kCamera */ {
            // Source: drake/geometry/render/light_parameter.h
            const char* doc = R"""()""";
          } kCamera;
          // Symbol: drake::geometry::render::LightFrame::kWorld
          struct /* kWorld */ {
            // Source: drake/geometry/render/light_parameter.h
            const char* doc = R"""()""";
          } kWorld;
        } LightFrame;
        // Symbol: drake::geometry::render::LightParameter
        struct /* LightParameter */ {
          // Source: drake/geometry/render/light_parameter.h
          const char* doc =
R"""(Light parameter for supporting RenderEngine implementations. Look at
the various RenderEngine___Params types to see if they support
declaring lights.)""";
          // Symbol: drake::geometry::render::LightParameter::Serialize
          struct /* Serialize */ {
            // Source: drake/geometry/render/light_parameter.h
            const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
          } Serialize;
          // Symbol: drake::geometry::render::LightParameter::attenuation_values
          struct /* attenuation_values */ {
            // Source: drake/geometry/render/light_parameter.h
            const char* doc =
R"""(The quadratic attenuation constants (k₀, k₁, and k₂). The intensity of
light can decrease with distance according to the following equation:

I = 1/(k₀ + k₁·d + k₂·d²)

In the physical world, light from a point source decreases with the
squared distance (with some linear attenuation due to atmospheric
effects). However, this often leads to undesirable illumination
effects in low-dynamic range rendering. Suitable values typically will
temper the quadratic effects with non-zero constant and linear terms.

If the values sum to 1, then, a surface one meter away from the light
will get full illumination (surfaces closer will get *magnified*
illumination. When tuning attenuation to achieve a particular falloff
*pattern*, it is common to *increase* the light ``intensity`` to
offset what would otherwise appear to be a rapid decay. By default,
the light is unattenuated, with constant illumination across all
distances.

When using directional lights, attenuation is best kept with the
default, non attenuated values (<1, 0, 0>). Other attenuation values
may be applied, but the effect may be surprising and difficult to
control.

Warning:
    If all three values are zero, no light is emitted.)""";
          } attenuation_values;
          // Symbol: drake::geometry::render::LightParameter::color
          struct /* color */ {
            // Source: drake/geometry/render/light_parameter.h
            const char* doc =
R"""(The illuminating color of the light (with channels in the range [0,
1]). The alpha value is currently ignored.)""";
          } color;
          // Symbol: drake::geometry::render::LightParameter::cone_angle
          struct /* cone_angle */ {
            // Source: drake/geometry/render/light_parameter.h
            const char* doc =
R"""(For a spotlight, it is the measure of the angle (in degrees) between
the light's central direction vector and the border of the light's
conical extent. It is half of the spotlight's full angular span. So, a
light with a 45° ``cone_angle`` would cast light over a 90° span.)""";
          } cone_angle;
          // Symbol: drake::geometry::render::LightParameter::direction
          struct /* direction */ {
            // Source: drake/geometry/render/light_parameter.h
            const char* doc =
R"""(The direction the light points in the frame indicated by ``frame``.
For example, in the world frame, a spotlight with direction [0, 0,
-1]ᵀ points straight down. In the camera frame, the direction [0, 0,
1]ᵀ points forward in the camera's view direction. This field only
applies to spotlight and directional lights.

Precondition:
    the vector has sufficient precision to be meaningfully normalized.)""";
          } direction;
          // Symbol: drake::geometry::render::LightParameter::frame
          struct /* frame */ {
            // Source: drake/geometry/render/light_parameter.h
            const char* doc =
R"""(Specifies the frame to which the camera is fixed, ``"world"`` or
``"camera"``.

Remember, as documented by systems∷sensors∷CameraInfo "CameraInfo",
the camera frame has +Cz pointing *into* the image, +Cx pointing to
the right, and +Cy pointing to the bottom of the image.)""";
          } frame;
          // Symbol: drake::geometry::render::LightParameter::intensity
          struct /* intensity */ {
            // Source: drake/geometry/render/light_parameter.h
            const char* doc =
R"""(A multiplier for the brightness of the light. A zero intensity will
effectively disable the light. A value of one will have an intensity
equal to the light's color. Higher values will magnify the light's
illumination (and may be necessary to offset the attenuation effects).)""";
          } intensity;
          // Symbol: drake::geometry::render::LightParameter::position
          struct /* position */ {
            // Source: drake/geometry/render/light_parameter.h
            const char* doc =
R"""(The position of the light in the frame indicated by ``frame`` F: p_FL.)""";
          } position;
          // Symbol: drake::geometry::render::LightParameter::type
          struct /* type */ {
            // Source: drake/geometry/render/light_parameter.h
            const char* doc =
R"""(The light type is either ``"point"``, `"spot"`, or ``"directional"``.)""";
          } type;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("attenuation_values", attenuation_values.doc),
              std::make_pair("color", color.doc),
              std::make_pair("cone_angle", cone_angle.doc),
              std::make_pair("direction", direction.doc),
              std::make_pair("frame", frame.doc),
              std::make_pair("intensity", intensity.doc),
              std::make_pair("position", position.doc),
              std::make_pair("type", type.doc),
            };
          }
        } LightParameter;
        // Symbol: drake::geometry::render::LightType
        struct /* LightType */ {
          // Source: drake/geometry/render/light_parameter.h
          const char* doc =
R"""(Specification of the type of light.

- kPoint: a punctual light source emitting light in all directions.
- kSpot: A conical light source emitting light from a point into a limited
set of directions.
- kDirectional: a light source, infinitely far away, casting parallel rays of
light (like a sun).)""";
          // Symbol: drake::geometry::render::LightType::kDirectional
          struct /* kDirectional */ {
            // Source: drake/geometry/render/light_parameter.h
            const char* doc = R"""()""";
          } kDirectional;
          // Symbol: drake::geometry::render::LightType::kPoint
          struct /* kPoint */ {
            // Source: drake/geometry/render/light_parameter.h
            const char* doc = R"""()""";
          } kPoint;
          // Symbol: drake::geometry::render::LightType::kSpot
          struct /* kSpot */ {
            // Source: drake/geometry/render/light_parameter.h
            const char* doc = R"""()""";
          } kSpot;
        } LightType;
        // Symbol: drake::geometry::render::RenderCameraCore
        struct /* RenderCameraCore */ {
          // Source: drake/geometry/render/render_camera.h
          const char* doc =
R"""(Collection of core parameters for modeling a pinhole-model camera in a
RenderEngine. These parameters are applicable to both depth and
color/label renderings. Parameters specific to those output image
types can be found below.

While these parameters are generally applicable to all RenderEngine
implementations, this is not guaranteed to be true. For example, the
clipping range property only applies to frustum-based RenderEngine
implementations. I.e., it wouldn't apply to a ray-tracing based
implementation.)""";
          // Symbol: drake::geometry::render::RenderCameraCore::CalcProjectionMatrix
          struct /* CalcProjectionMatrix */ {
            // Source: drake/geometry/render/render_camera.h
            const char* doc =
R"""(Expresses ``this`` camera's pinhole camera properties as the
projective transform T_DC which transforms points in a camera's frame
C to a 2D, normalized device frame D. The transform is represented by
a 4x4 matrix (i.e., a <a
href="https://strawlab.org/2011/11/05/augmented-reality-with-OpenGL/">
classic OpenGl projection matrix</a>).)""";
          } CalcProjectionMatrix;
          // Symbol: drake::geometry::render::RenderCameraCore::RenderCameraCore
          struct /* ctor */ {
            // Source: drake/geometry/render/render_camera.h
            const char* doc =
R"""(Fully-specified constructor. See the documentation on the member
getter methods for documentation of parameters.)""";
          } ctor;
          // Symbol: drake::geometry::render::RenderCameraCore::clipping
          struct /* clipping */ {
            // Source: drake/geometry/render/render_camera.h
            const char* doc =
R"""(The near and far clipping planes for this camera. This property is
ignored by RenderEngine implementations that don't use a clipping
frustum.)""";
          } clipping;
          // Symbol: drake::geometry::render::RenderCameraCore::intrinsics
          struct /* intrinsics */ {
            // Source: drake/geometry/render/render_camera.h
            const char* doc =
R"""(The camera's intrinsic properties (e.g., focal length, sensor size,
etc.) See systems∷sensors∷CameraInfo for details.)""";
          } intrinsics;
          // Symbol: drake::geometry::render::RenderCameraCore::renderer_name
          struct /* renderer_name */ {
            // Source: drake/geometry/render/render_camera.h
            const char* doc =
R"""(The name of the render engine this camera should be used with.)""";
          } renderer_name;
          // Symbol: drake::geometry::render::RenderCameraCore::sensor_pose_in_camera_body
          struct /* sensor_pose_in_camera_body */ {
            // Source: drake/geometry/render/render_camera.h
            const char* doc =
R"""(The pose of the sensor frame (S) in the camera's body frame (B). This
is the "imager" referred to in systems∷sensors∷CameraInfo's
documentation.)""";
          } sensor_pose_in_camera_body;
        } RenderCameraCore;
        // Symbol: drake::geometry::render::RenderEngine
        struct /* RenderEngine */ {
          // Source: drake/geometry/render/render_engine.h
          const char* doc =
R"""(The engine for performing rasterization operations on geometry. This
includes rgb images and depth images. The coordinate system of
RenderEngine's viewpoint ``R`` is ``X-right``, `Y-down` and
``Z-forward`` with respect to the rendered images.

**Output image format**

- RGB (ImageRgba8U) : the RGB image has four channels in the following
order: red, green, blue, and alpha. Each channel is represented by
a uint8_t.

- Depth (ImageDepth32F) : the depth image has a depth channel represented
by a float. For a point in space ``P``, the value stored in the depth
channel holds *the Z-component of the position vector ``p_RP``.*
Note that this is different from the range data used by laser
range finders (like that provided by DepthSensor) in which the depth
value represents the distance from the sensor origin to the object's
surface.

- Label (ImageLabel16I) : the label image has single channel represented
by an int16_t. The value stored in the channel holds a RenderLabel value
which corresponds to an object class in the scene or an "empty" pixel (see
RenderLabel for more details).

**RenderLabels, registering geometry, and derived classes**

By convention, when registering a geometry, the provided properties
should contain no more than one RenderLabel instance, and that should
be the ``(label, id)`` property. RenderEngine provides the notion of a
*default render label* that will be applied where no ``(label, id)``
RenderLabel property is found. This default value can be one of two
values: RenderLabel∷kDontCare or RenderLabel∷kUnspecified. The choice
of default RenderLabel can be made at construction and it affects
registration behavior when the ``(label, id)`` property is absent:

- RenderLabel∷kUnspecified: throws an exception.
- RenderLabel∷kDontCare: the geometry will be included in label images as
the generic, non-distinguishing label.

Choosing RenderLabel∷kUnspecified is best in a system that wants
explicit feedback and strict enforcement on a policy of strict label
enforcement -- everything should receive a meaningful label. The
choice of RenderLabel∷kDontCare is best for a less strict system in
which only some subset of geometry need be explicitly specified.

Derived classes configure their *de facto* default RenderLabel value,
or a user-configured default value, at construction, subject to the
requirements outlined above.

Derived classes should not access the ``(label, id)`` property
directly. RenderEngine provides a method to safely extract a
RenderLabel value from the PerceptionProperties, taking into account
the configured default value and the documented reserved_render_label
"RenderLabel semantics"; see GetRenderLabelOrThrow().)""";
          // Symbol: drake::geometry::render::RenderEngine::Clone
          struct /* Clone */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(Clones the render engine.

Template parameter ``Result``:
    must be either ``std∷unique_ptr<RenderEngine>`` or
    ``std∷shared_ptr<RenderEngine>``. In C++, it defaults to
    unique_ptr; in Python, it's hard-coded to shared_ptr.

Raises:
    RuntimeError if Result is unique_ptr but this particular class
    only supports cloning for shared_ptr.)""";
          } Clone;
          // Symbol: drake::geometry::render::RenderEngine::DoClone
          struct /* DoClone */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(The NVI-function for cloning this render engine as a unique_ptr. It
must always be overridden, but in case a subclass does not support
cloning into a unique_ptr, it may throw an exception.)""";
          } DoClone;
          // Symbol: drake::geometry::render::RenderEngine::DoCloneShared
          struct /* DoCloneShared */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(The NVI-function for cloning this render engine as a shared_ptr. When
not overridden, this base class implementation will call DoClone() to
construct a unique_ptr clone and then promote that to a shared_ptr
upon return. Note that in Python this is bound as simply "DoClone" not
"DoCloneShared", because the unique_ptr flavor is nonsense in Python.)""";
          } DoCloneShared;
          // Symbol: drake::geometry::render::RenderEngine::DoGetParameterYaml
          struct /* DoGetParameterYaml */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(The NVI-function for GetParameterYaml(). Derived classes must
implement this in order to support engine comparisons.)""";
          } DoGetParameterYaml;
          // Symbol: drake::geometry::render::RenderEngine::DoRegisterDeformableVisual
          struct /* DoRegisterDeformableVisual */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(The NVI-function for RegisterDeformableVisual(). This function
defaults to returning false. If the derived class chooses to register
this particular geometry, it should return true. This function is
invoked with the following guarantees:

- ``id`` is unique (i.e. distinct from previously registered geometries).
- ``render_meshes`` is non-empty.

Warning:
    This feature is considered to be **experimental** and may change
    or be removed at any time, without any deprecation notice ahead of
    time.)""";
          } DoRegisterDeformableVisual;
          // Symbol: drake::geometry::render::RenderEngine::DoRegisterVisual
          struct /* DoRegisterVisual */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(The NVI-function for sub-classes to implement actual rigid geometry
registration. If the derived class chooses not to register this
particular shape, it should return false.

A derived render engine can use arbitrary criteria to decide if the
rigid geometry gets registered. In accessing the RenderLabel property
in ``properties`` derived class should *exclusively* use
GetRenderLabelOrThrow().)""";
          } DoRegisterVisual;
          // Symbol: drake::geometry::render::RenderEngine::DoRemoveGeometry
          struct /* DoRemoveGeometry */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(The NVI-function for removing the geometry with the given ``id``.

Parameter ``id``:
    The id of the geometry to remove.

Returns:
    True if the geometry was registered with this RenderEngine and
    removed, false if it wasn't registered in the first place.)""";
          } DoRemoveGeometry;
          // Symbol: drake::geometry::render::RenderEngine::DoRenderColorImage
          struct /* DoRenderColorImage */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(The NVI-function for rendering color with a fully-specified camera.
When RenderColorImage calls this, it has already confirmed that
``color_image_out`` is not ``nullptr`` and its size is consistent with
the camera intrinsics.

Raises:
    RuntimeError in its default implementation indicating that it has
    not been implemented. Derived RenderEngine classes must implement
    this to support rendering color images.)""";
          } DoRenderColorImage;
          // Symbol: drake::geometry::render::RenderEngine::DoRenderDepthImage
          struct /* DoRenderDepthImage */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(The NVI-function for rendering depth with a fully-specified camera.
When RenderDepthImage calls this, it has already confirmed that
``depth_image_out`` is not ``nullptr`` and its size is consistent with
the camera intrinsics.

Raises:
    RuntimeError in its default implementation indicating that it has
    not been implemented. Derived RenderEngine classes must implement
    this to support rendering depth images.)""";
          } DoRenderDepthImage;
          // Symbol: drake::geometry::render::RenderEngine::DoRenderLabelImage
          struct /* DoRenderLabelImage */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(The NVI-function for rendering label with a fully-specified camera.
When RenderLabelImage calls this, it has already confirmed that
``label_image_out`` is not ``nullptr`` and its size is consistent with
the camera intrinsics.

Raises:
    RuntimeError in its default implementation indicating that it has
    not been implemented. Derived RenderEngine classes must implement
    this to support rendering label images.)""";
          } DoRenderLabelImage;
          // Symbol: drake::geometry::render::RenderEngine::DoUpdateDeformableConfigurations
          struct /* DoUpdateDeformableConfigurations */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(The NVI-function for UpdateDeformableConfigurations(). It is invoked
with the following guarantees:

- ``id`` references a registered deformable geometry.
- ``q_WGs`` and ``nhats_W`` are appropriately sized for the
registered meshes.

Warning:
    This feature is considered to be **experimental** and may change
    or be removed at any time, without any deprecation notice ahead of
    time.)""";
          } DoUpdateDeformableConfigurations;
          // Symbol: drake::geometry::render::RenderEngine::DoUpdateVisualPose
          struct /* DoUpdateVisualPose */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(The NVI-function for updating the pose of a rigid render geometry
(identified by ``id``) to the given pose X_WG.

Parameter ``id``:
    The id of the render geometry whose pose is being set.

Parameter ``X_WG``:
    The pose of the render geometry in the world frame.)""";
          } DoUpdateVisualPose;
          // Symbol: drake::geometry::render::RenderEngine::GetParameterYaml
          struct /* GetParameterYaml */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(Produces a yaml string that can be deserialized into this *particular*
RenderEngine's type.)""";
          } GetParameterYaml;
          // Symbol: drake::geometry::render::RenderEngine::GetRenderLabelOrThrow
          struct /* GetRenderLabelOrThrow */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(Extracts the ``(label, id)`` RenderLabel property from the given
``properties`` and validates it (or the configured default if no such
property is defined).

Raises:
    RuntimeError If the tested render label value is deemed invalid.)""";
          } GetRenderLabelOrThrow;
          // Symbol: drake::geometry::render::RenderEngine::MakeLabelFromRgb
          struct /* MakeLabelFromRgb */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(Transforms the given RGB color into its corresponding RenderLabel.)""";
          } MakeLabelFromRgb;
          // Symbol: drake::geometry::render::RenderEngine::MakeRgbFromLabel
          struct /* MakeRgbFromLabel */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(Transforms the given render label into an RGB color. The alpha channel
will always be 1.0.)""";
          } MakeRgbFromLabel;
          // Symbol: drake::geometry::render::RenderEngine::RegisterDeformableVisual
          struct /* RegisterDeformableVisual */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(Requests registration of the given deformable geometry with this
render engine.

Warning:
    This feature is considered to be **experimental** and may change
    or be removed at any time, without any deprecation notice ahead of
    time.

Parameter ``id``:
    The geometry id of the shape to register.

Parameter ``render_meshes``:
    The mesh representations of deformable geometry in its default
    state. A single geometry may be represented by more than one
    render mesh. This facilitates registering a geometry with more
    than one material for rendering.

Parameter ``properties``:
    The perception properties provided for this geometry.

Precondition:
    each RenderMesh in ``render_meshes`` is valid.

Raises:
    RuntimeError if a geometry with ``id`` has already been registered
    with ``this`` RenderEngine.

Raises:
    RuntimeError if ``render_meshes`` is empty.

Returns:
    True if the RenderEngine implementation accepted the geometry for
    registration.)""";
          } RegisterDeformableVisual;
          // Symbol: drake::geometry::render::RenderEngine::RegisterVisual
          struct /* RegisterVisual */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(Requests registration of the given shape as a rigid geometry with this
render engine.

Parameter ``id``:
    The geometry id of the shape to register.

Parameter ``shape``:
    The shape specification to add to the render engine.

Parameter ``properties``:
    The perception properties provided for this geometry.

Parameter ``X_WG``:
    The pose of the geometry relative to the world frame W.

Parameter ``needs_updates``:
    If true, the geometry's pose will be updated via UpdatePoses().

Returns:
    True if the RenderEngine implementation accepted the shape for
    registration.

Raises:
    RuntimeError if the shape is an unsupported type, the shape's
    RenderLabel value is RenderLabel∷kUnspecified or
    RenderLabel∷kEmpty, or a geometry has already been registered with
    the given ``id``.)""";
          } RegisterVisual;
          // Symbol: drake::geometry::render::RenderEngine::RemoveGeometry
          struct /* RemoveGeometry */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(Removes the geometry indicated by the given ``id`` from the engine.

Parameter ``id``:
    The id of the geometry to remove.

Returns:
    True if the geometry was removed (false implies that this id
    wasn't registered with this engine).)""";
          } RemoveGeometry;
          // Symbol: drake::geometry::render::RenderEngine::RenderColorImage
          struct /* RenderColorImage */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(Renders the registered geometry into the given color (rgb) image based
on a *fully* specified camera.

Parameter ``camera``:
    The *render engine* camera properties.

Parameter ``color_image_out``:
    The rendered color image.

Raises:
    RuntimeError if ``color_image_out`` is ``nullptr`` or the size of
    the given input image doesn't match the size declared in
    ``camera``.)""";
          } RenderColorImage;
          // Symbol: drake::geometry::render::RenderEngine::RenderDepthImage
          struct /* RenderDepthImage */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(Renders the registered geometry into the given depth image based on a
*fully* specified camera. In contrast to the other rendering
operations, depth images don't have an option to display the window;
generally, basic depth images are not readily communicative to humans.

Parameter ``camera``:
    The *render engine* camera properties.

Parameter ``depth_image_out``:
    The rendered depth image.

Raises:
    RuntimeError if ``depth_image_out`` is ``nullptr`` or the size of
    the given input image doesn't match the size declared in
    ``camera``.)""";
          } RenderDepthImage;
          // Symbol: drake::geometry::render::RenderEngine::RenderEngine
          struct /* ctor */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(Constructs a RenderEngine with the given default render label. The
default render label is applied to geometries that have not otherwise
specified a (label, id) property. The value *must* be either
RenderLabel∷kUnspecified or RenderLabel∷kDontCare. (See
render_engine_default_label "this section" for more details.)

Raises:
    RuntimeError if the default render label is not one of the two
    allowed labels.)""";
          } ctor;
          // Symbol: drake::geometry::render::RenderEngine::RenderLabelImage
          struct /* RenderLabelImage */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(Renders the registered geometry into the given label image based on a
*fully* specified camera.

Note:
    This uses the ColorRenderCamera as label images are typically
    rendered to be exactly registered with a corresponding color
    image.

Parameter ``camera``:
    The *render engine* camera properties.

Parameter ``label_image_out``:
    The rendered label image.

Raises:
    RuntimeError if ``label_image_out`` is ``nullptr`` or the size of
    the given input image doesn't match the size declared in
    ``camera``.)""";
          } RenderLabelImage;
          // Symbol: drake::geometry::render::RenderEngine::SetDefaultLightPosition
          struct /* SetDefaultLightPosition */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(Provides access to the light for manual configuration since it's
currently bound to the camera position. This is a temporary measure to
facilitate benchmarking and create visible shadows, and should not be
used publicly.

Parameter ``X_DL``:
    The pose of the light in a frame D that is attached to the camera
    position. In this frame D, the camera is located at (0, 0, 1),
    looking towards (0, 0, 0) at a distance of 1, with up being (0, 1,
    0).)""";
          } SetDefaultLightPosition;
          // Symbol: drake::geometry::render::RenderEngine::ThrowIfInvalid
          struct /* ThrowIfInvalid */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc = R"""()""";
          } ThrowIfInvalid;
          // Symbol: drake::geometry::render::RenderEngine::UpdateDeformableConfigurations
          struct /* UpdateDeformableConfigurations */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(Updates the configurations of all meshes associated with the given
deformable geometry (see RegisterDeformableVisual()). The number of
elements in the supplied vertex position vector ``q_WGs`` and the
vertex normal vector ``nhats_W`` must be equal to the number of render
meshes registered to the geometry associated with ``id``. Within each
mesh, the vertex positions and normals must be ordered the same way as
the vertices specified in the render mesh at registration when
reshaped to be an Nx3 matrix with N being the number of vertices in
the mesh.

No-op if no geometry with the given ``id`` is registered with this
engine.

Warning:
    This feature is considered to be **experimental** and may change
    or be removed at any time, without any deprecation notice ahead of
    time.

Parameter ``id``:
    The unique identifier of a deformable geometry registered with
    this RenderEngine.

Parameter ``q_WGs``:
    The vertex positions of all meshes associated with the given
    deformable geometry (measured and expressed in the world frame).

Parameter ``nhats_W``:
    The vertex normals of all meshes associated with the given
    deformable geometry (measured and expressed in the world frame).

Raises:
    RuntimeError if the sizes of ``q_WGs`` or ``nhats_W`` are
    incompatible with the number of degrees of freedom of the meshes
    registered with the deformable geometry.)""";
          } UpdateDeformableConfigurations;
          // Symbol: drake::geometry::render::RenderEngine::UpdatePoses
          struct /* UpdatePoses */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(Updates the poses of all rigid geometries marked as "needing update"
(see RegisterVisual()).

Parameter ``X_WGs``:
    The poses of *all* geometries in SceneGraph (measured and
    expressed in the world frame). The pose for a geometry is accessed
    by that geometry's id.)""";
          } UpdatePoses;
          // Symbol: drake::geometry::render::RenderEngine::UpdateViewpoint
          struct /* UpdateViewpoint */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(Updates the renderer's viewpoint with given pose X_WR.

Parameter ``X_WR``:
    The pose of renderer's viewpoint in the world coordinate system.)""";
          } UpdateViewpoint;
          // Symbol: drake::geometry::render::RenderEngine::default_render_label
          struct /* default_render_label */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(Reports the render label value this render engine has been configured
to use.)""";
          } default_render_label;
          // Symbol: drake::geometry::render::RenderEngine::has_geometry
          struct /* has_geometry */ {
            // Source: drake/geometry/render/render_engine.h
            const char* doc =
R"""(Reports true if a geometry with the given ``id`` has been registered
with ``this`` engine.)""";
          } has_geometry;
        } RenderEngine;
        // Symbol: drake::geometry::render::RenderLabel
        struct /* RenderLabel */ {
          // Source: drake/geometry/render/render_label.h
          const char* doc =
R"""(Class representing object "labels" for rendering.

In a "label image" (see RenderEngine∷RenderLabelImage() for details)
each pixel value indicates the classification of the object that
rendered into that pixel. The RenderLabel class provides that value
and one label is associated with each rendered geometry.

The labels could be unique for each geometry, or multiple geometries
could all share the same label (becoming indistinguishable in the
label image). Ultimately, it is the user's responsibility to assign
labels in a manner that is meaningful for their application.

**Reserved labels**

There are several RenderLabels that are reserved. They have specific
meaning in the context of the rendering ecosystem and are globally
available to all applications. They are:

- ``empty``: a pixel with the ``empty`` RenderLabel value indicates that *no*
geometry rendered to that pixel. Implemented as RenderLabel∷kEmpty.
- ``do not render``: any geometry assigned the ``do not render`` tag will *not* be
rendered into a label image. This is a clear declaration that a geometry
should be omitted. Useful for marking, e.g., glass windows so that the
visible geometry behind the glass is what is included in the label image.
Implemented as RenderLabel∷kDoNotRender.
- ``don't care``: the ``don't care`` label is intended as a convenient dumping
ground. This would be for geometry that *should* render into the label image,
but whose class is irrelevant (e.g., the walls of a room a robot is working
in or the background terrain in driving simulation). Implemented as
RenderLabel∷kDontCare.
- ``unspecified``: a default-constructed RenderLabel has an ``unspecified`` value.
Implemented as RenderLabel∷kUnspecified.

Generally, there is no good reason to assign ``empty`` or
``unspecified`` labels to a geometry. A RenderEngine implementation is
entitled to throw an exception if you attempt to do so.

Usage
-----

An application can simply instantiate RenderLabel with an arbitrary
value. This allows the application to define a particular mapping from
render label class to a preferred RenderLabel value. For a label image
to be *meaningful*, every pixel value should admit an unambiguous
interpretation. The application bears *full* responsibility in making
sure that a single value is not inadvertently associated with multiple
render classes. Finally, a RenderLabel cannot be explicitly
constructed with a reserved value -- those can only be accessed
through the static methods provided.

Note:
    The RenderLabel class is based on a 16-bit integer. This makes the
    label image more compact but means there are only, approximately,
    32,000 unique RenderLabel values.)""";
          // Symbol: drake::geometry::render::RenderLabel::RenderLabel
          struct /* ctor */ {
            // Source: drake/geometry/render/render_label.h
            const char* doc_0args =
R"""(Constructs a label with the reserved ``unspecified`` value.)""";
            // Source: drake/geometry/render/render_label.h
            const char* doc_1args =
R"""(Constructs a label with the given ``value``.

Raises:
    RuntimeError if a) is negative, or b) the ``value`` is one of the
    reserved values.)""";
          } ctor;
          // Symbol: drake::geometry::render::RenderLabel::ValueType
          struct /* ValueType */ {
            // Source: drake/geometry/render/render_label.h
            const char* doc = R"""()""";
          } ValueType;
          // Symbol: drake::geometry::render::RenderLabel::is_reserved
          struct /* is_reserved */ {
            // Source: drake/geometry/render/render_label.h
            const char* doc =
R"""(Reports if the label is a reserved label.)""";
          } is_reserved;
          // Symbol: drake::geometry::render::RenderLabel::operator short
          struct /* operator_short */ {
            // Source: drake/geometry/render/render_label.h
            const char* doc =
R"""(Implicit conversion to its underlying integer representation.)""";
          } operator_short;
          // Symbol: drake::geometry::render::RenderLabel::operator!=
          struct /* operator_ne */ {
            // Source: drake/geometry/render/render_label.h
            const char* doc =
R"""(Compares this label with the ``other`` label. Reports true if they
have different values.)""";
          } operator_ne;
          // Symbol: drake::geometry::render::RenderLabel::operator<
          struct /* operator_lt */ {
            // Source: drake/geometry/render/render_label.h
            const char* doc =
R"""(Allows the labels to be compared to imply a total ordering --
facilitates use in data structures which require ordering (e.g.,
std∷set). The ordering has no particular meaning for applications.)""";
          } operator_lt;
          // Symbol: drake::geometry::render::RenderLabel::to_string
          struct /* to_string */ {
            // Source: drake/geometry/render/render_label.h
            const char* doc =
R"""(Converts the RenderLabel value to a string representation.)""";
          } to_string;
        } RenderLabel;
        // Symbol: drake::geometry::render::light_frame_from_string
        struct /* light_frame_from_string */ {
          // Source: drake/geometry/render/light_parameter.h
          const char* doc =
R"""(Instantiates a LightFrame from its string representation.

Parameter ``spec``:
    Must be one of 'world' or 'camera'.

Raises:
    if ``spec`` is an unrecognized string.)""";
        } light_frame_from_string;
        // Symbol: drake::geometry::render::light_type_from_string
        struct /* light_type_from_string */ {
          // Source: drake/geometry/render/light_parameter.h
          const char* doc =
R"""(Instantiates a LightType from its string representation.

Parameter ``spec``:
    Must be one of 'point', 'spot', or 'directional'.

Raises:
    if ``spec`` is an unrecognized string.)""";
        } light_type_from_string;
        // Symbol: drake::geometry::render::to_string
        struct /* to_string */ {
          // Source: drake/geometry/render/light_parameter.h
          const char* doc_1args_t = R"""(Returns the LightType as a string.)""";
          // Source: drake/geometry/render/light_parameter.h
          const char* doc_1args_f = R"""(Returns the LightFrame as a string.)""";
        } to_string;
      } render;
    } geometry;
  } drake;
} pydrake_doc_geometry_render;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
