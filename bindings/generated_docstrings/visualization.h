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

// #include "drake/visualization/colorize_depth_image.h"
// #include "drake/visualization/colorize_label_image.h"
// #include "drake/visualization/concatenate_images.h"
// #include "drake/visualization/inertia_visualizer.h"
// #include "drake/visualization/meshcat_pose_sliders.h"
// #include "drake/visualization/visualization_config.h"
// #include "drake/visualization/visualization_config_functions.h"

// Symbol: pydrake_doc_visualization
constexpr struct /* pydrake_doc_visualization */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::visualization
    struct /* visualization */ {
      // Symbol: drake::visualization::AddDefaultVisualization
      struct /* AddDefaultVisualization */ {
        // Source: drake/visualization/visualization_config_functions.h
        const char* doc =
R"""(Adds LCM visualization publishers to communicate to Meshcat and/or
Meldis, using all of the default configuration settings.

Parameter ``meshcat``:
    An optional existing Meshcat instance. (If nullptr, then a meshcat
    instance will be created.)

**Example**


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    // Create a builder.
    DiagramBuilder<double> builder;
    
    // Add the MultibodyPlant and SceneGraph.
    const MultibodyPlantConfig plant_config = ...;
    MultibodyPlant<double>& plant = AddMultibodyPlant(plant_config, &builder);
    // ... populate the plant, e.g., with a Parser, ...
    plant.Finalize();
    
    // Add the visualization.
    AddDefaultVisualization(&builder);
    
    // Simulate.
    Simulator<double> simulator(builder.Build());
    // ... etc ...

.. raw:: html

    </details>

Precondition:
    AddMultibodyPlant() or AddMultibodyPlantSceneGraph() has already
    been called on the given ``builder``.

Precondition:
    The MultibodyPlant in the given ``builder`` is already finalized,
    as in ``plant.Finalize()``.

See also:
    drake∷visualization∷ApplyVisualizationConfig()

See also:
    drake∷multibody∷AddMultibodyPlant())""";
      } AddDefaultVisualization;
      // Symbol: drake::visualization::ApplyVisualizationConfig
      struct /* ApplyVisualizationConfig */ {
        // Source: drake/visualization/visualization_config_functions.h
        const char* doc =
R"""(Adds LCM visualization publishers to communicate to Meshcat and/or
Meldis.

**Example**


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    // Create a builder.
    DiagramBuilder<double> builder;
    
    // Add the MultibodyPlant and SceneGraph.
    const MultibodyPlantConfig plant_config = ...;
    MultibodyPlant<double>& plant = AddMultibodyPlant(plant_config, &builder);
    // ... populate the plant, e.g., with a Parser, ...
    plant.Finalize();
    
    // Add the visualization.
    const VisualizationConfig vis_config = ...;
    ApplyVisualizationConfig(vis_config, &builder);
    
    // Simulate.
    Simulator<double> simulator(builder.Build());
    // ... etc ...

.. raw:: html

    </details>

Parameter ``config``:
    The visualization configuration.

Parameter ``builder``:
    The diagram to add visualization systems into.

Parameter ``lcm_buses``:
    (Optional) The available LCM buses to use for visualization
    message publication. When not provided, uses the ``lcm`` interface
    if provided, or else the ``config.lcm_bus`` must be set to
    "default" in which case an appropriate drake∷lcm∷DrakeLcm object
    is constructed and used internally.

Parameter ``plant``:
    (Optional) The MultibodyPlant to visualize. In the common case
    where a MultibodyPlant has already been added to ``builder`` using
    either AddMultibodyPlant() or AddMultibodyPlantSceneGraph(), the
    default value (nullptr) here is suitable and generally should be
    preferred. When provided, it must be a System that's been added to
    the the given ``builder``. When not provided, visualizes the
    system named "plant" in the given ``builder``.

Parameter ``scene_graph``:
    (Optional) The SceneGraph to visualize. In the common case where a
    SceneGraph has already been added to ``builder`` using either
    AddMultibodyPlant() or AddMultibodyPlantSceneGraph(), the default
    value (nullptr) here is suitable and generally should be
    preferred. When provided, it must be a System that's been added to
    the the given ``builder``. When not provided, visualizes the
    system named "scene_graph" in the given ``builder``.

Parameter ``meshcat``:
    (Optional) A Meshcat object for visualization message publication.
    When not provided, a Meshcat object will be created unless
    ``config.enable_meshcat_creation`` is set to false.

Parameter ``lcm``:
    (Optional) The LCM interface used for visualization message
    publication. When not provided, uses the ``config.lcm_bus`` value
    to look up the appropriate interface from ``lcm_buses``.

Precondition:
    The ``builder`` is non-null.

Precondition:
    Either the ``config.lcm_bus`` is set to "default", or else
    ``lcm_buses`` is non-null and contains a bus named
    ``config.lcm_bus``, or else ``lcm`` is non-null.

Precondition:
    Either the given ``builder`` contains a MultibodyPlant system
    named "plant" or else the provided ``plant`` is non-null.

Precondition:
    Either the given ``builder`` contains a SceneGraph system named
    "scene_graph" or else the provided ``scene_graph`` is non-null.

Precondition:
    The MultibodyPlant is already finalized, as in
    ``plant.Finalize()``.

See also:
    drake∷visualization∷AddDefaultVisualization()

See also:
    drake∷multibody∷AddMultibodyPlant()

See also:
    drake∷systems∷lcm∷ApplyLcmBusConfig())""";
      } ApplyVisualizationConfig;
      // Symbol: drake::visualization::ColorizeDepthImage
      struct /* ColorizeDepthImage */ {
        // Source: drake/visualization/colorize_depth_image.h
        const char* doc =
R"""(ColorizeDepthImage converts a depth image, either 32F or 16U, to a
color image. One input port, and only one, must be connected.

.. pydrake_system::

    name: ColorizeDepthImage
    input_ports:
    - depth_image_32f
    - depth_image_16u
    output_ports:
    - color_image

Depth measurements are linearly mapped to a grayscale palette, with
smaller (closer) values brighter and larger (further) values darker.

The dynamic range of each input image determines the scale. The pixel
with the smallest depth will be fully white (#FFFFFFFF), and largest
depth will be fully black (#000000FF). Note that alpha channel is
still 100% in both cases.

Because the dynamic range is measured one input image a time, take
note that a video recording of this System will not have consistent
scaling across its entirety. The ability to set a fixed palette is
future work.

For the special depth pixel values "too close" or "too far", the color
pixel will use the ``invalid_color`` property (by default, a dim red).)""";
        // Symbol: drake::visualization::ColorizeDepthImage::Calc
        struct /* Calc */ {
          // Source: drake/visualization/colorize_depth_image.h
          const char* doc =
R"""(Colorizes the ``input`` into ``output``, without using any System port
conections nor any Context.)""";
        } Calc;
        // Symbol: drake::visualization::ColorizeDepthImage::ColorizeDepthImage<T>
        struct /* ctor */ {
          // Source: drake/visualization/colorize_depth_image.h
          const char* doc = R"""(Creates a ColorizeDepthImage system.)""";
        } ctor;
        // Symbol: drake::visualization::ColorizeDepthImage::get_invalid_color
        struct /* get_invalid_color */ {
          // Source: drake/visualization/colorize_depth_image.h
          const char* doc =
R"""(Gets the color used for pixels with too-near or too-far depth.)""";
        } get_invalid_color;
        // Symbol: drake::visualization::ColorizeDepthImage::set_invalid_color
        struct /* set_invalid_color */ {
          // Source: drake/visualization/colorize_depth_image.h
          const char* doc =
R"""(Sets the color used for pixels with too-near or too-far depth.)""";
        } set_invalid_color;
      } ColorizeDepthImage;
      // Symbol: drake::visualization::ColorizeLabelImage
      struct /* ColorizeLabelImage */ {
        // Source: drake/visualization/colorize_label_image.h
        const char* doc =
R"""(ColorizeLabelImage converts a label image to a color image.

.. pydrake_system::

    name: ColorizeLabelImage
    input_ports:
    - label_image
    output_ports:
    - color_image

Labels are mapped to colors with a built-in, fixed palette. The
palette has fewer elements than all possible labels, so not all labels
will necessarily be represented by a unique color. For label pixels
that do not represent a label ("don't care", "empty", "unspecified",
etc.), the color pixel will use the ``background_color`` property (by
default, black with 0% alpha).)""";
        // Symbol: drake::visualization::ColorizeLabelImage::Calc
        struct /* Calc */ {
          // Source: drake/visualization/colorize_label_image.h
          const char* doc =
R"""(Colorizes the ``input`` into ``output``, without using any System port
conections nor any Context.)""";
        } Calc;
        // Symbol: drake::visualization::ColorizeLabelImage::ColorizeLabelImage<T>
        struct /* ctor */ {
          // Source: drake/visualization/colorize_label_image.h
          const char* doc = R"""(Creates a ColorizeLabelImage system.)""";
        } ctor;
        // Symbol: drake::visualization::ColorizeLabelImage::get_background_color
        struct /* get_background_color */ {
          // Source: drake/visualization/colorize_label_image.h
          const char* doc =
R"""(Gets the color used for pixels with no label.)""";
        } get_background_color;
        // Symbol: drake::visualization::ColorizeLabelImage::set_background_color
        struct /* set_background_color */ {
          // Source: drake/visualization/colorize_label_image.h
          const char* doc =
R"""(Sets the color used for pixels with no label.)""";
        } set_background_color;
      } ColorizeLabelImage;
      // Symbol: drake::visualization::ConcatenateImages
      struct /* ConcatenateImages */ {
        // Source: drake/visualization/concatenate_images.h
        const char* doc =
R"""(ConcatenateImages stacks multiple input images into a single output
image.

.. pydrake_system::

    name: ConcatenateImages
    input_ports:
    - color_image_r0_c0
    - color_image_r0_c1
    - ...
    output_ports:
    - color_image

All inputs must be of type ImageRgba8U.

Any input port may be disconnected, in which case it will be
interpreted as zero-sized image.

In case of non-uniform image sizes, any gaps between images will be
filled with all-zero pixels (i.e., with 0% alpha).)""";
        // Symbol: drake::visualization::ConcatenateImages::ConcatenateImages<T>
        struct /* ctor */ {
          // Source: drake/visualization/concatenate_images.h
          const char* doc =
R"""(Constructs a ConcatenateImages system.

Parameter ``rows``:
    Number of images to stack vertically.

Parameter ``cols``:
    Number of images to stack horizontally.)""";
        } ctor;
        // Symbol: drake::visualization::ConcatenateImages::get_input_port
        struct /* get_input_port */ {
          // Source: drake/visualization/concatenate_images.h
          const char* doc =
R"""(Returns the InputPort for the given (row, col) image. Rows and columns
are 0-indexed, i.e., we have ``0 <= row < rows`` and ``0 <= col <
cols``.)""";
        } get_input_port;
      } ConcatenateImages;
      // Symbol: drake::visualization::InertiaVisualizer
      struct /* InertiaVisualizer */ {
        // Source: drake/visualization/inertia_visualizer.h
        const char* doc =
R"""(InertiaVisualizer provides illustration geometry to reflect the
equivalent inertia of all bodies in a MultibodyPlant that are not
welded to the world.

Instead of constructing this system directly, most users should use
AddDefaultVisualization() which automatically uses this system.

.. pydrake_system::

    name: InertiaVisualizer
    input_ports:
    - plant_geometry_pose
    output_ports:
    - geometry_pose

Warning:
    The visualized inertia shows the inertia of the ``plant`` bodies
    when this system was constructed. If you edit the plant's inertia
    values after construction (e.g., by changing a body's mass), the
    visualization will not be updated.)""";
        // Symbol: drake::visualization::InertiaVisualizer::AddToBuilder
        struct /* AddToBuilder */ {
          // Source: drake/visualization/inertia_visualizer.h
          const char* doc =
R"""(Adds a new InertiaVisualizer to the given ``builder`` and connects it
to the given ``plant`` and ``scene_graph``.)""";
        } AddToBuilder;
        // Symbol: drake::visualization::InertiaVisualizer::InertiaVisualizer<T>
        struct /* ctor */ {
          // Source: drake/visualization/inertia_visualizer.h
          const char* doc =
R"""(Creates an instance of InertiaVisualizer. The plant must be finalized.)""";
          // Source: drake/visualization/inertia_visualizer.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
        // Symbol: drake::visualization::InertiaVisualizer::source_id
        struct /* source_id */ {
          // Source: drake/visualization/inertia_visualizer.h
          const char* doc =
R"""((Advanced) Returns the source_id for our visualization geometries.
Most users will not need to use this.)""";
        } source_id;
      } InertiaVisualizer;
      // Symbol: drake::visualization::MeshcatPoseSliders
      struct /* MeshcatPoseSliders */ {
        // Source: drake/visualization/meshcat_pose_sliders.h
        const char* doc =
R"""(MeshcatPoseSliders adds slider bars to the MeshCat control panel for
the roll, pitch, yaw, x, y, z control of a pose. These might be useful
for interactive or teleoperation demos. The abstract-value pose is
available on the ``pose`` output port of this system.

.. pydrake_system::

    name: MeshcatPoseSliders
    input_ports:
    - pose (optional)
    output_ports:
    - pose

The optional ``pose`` input port is used ONLY at initialization; it
can be used to set the pose during an Initialization event. If
connected, it will take precedence over any ``initial_pose`` specified
in the constructor.

Beware that the output port of this system always provides the
sliders' current values, even if evaluated by multiple different
downstream input ports during a single computation. If you need to
have a synchronized view of the slider data, place a
systems∷ZeroOrderHold system between the sliders and downstream
calculations.)""";
        // Symbol: drake::visualization::MeshcatPoseSliders::Delete
        struct /* Delete */ {
          // Source: drake/visualization/meshcat_pose_sliders.h
          const char* doc =
R"""(Removes our sliders from the associated meshcat instance.

From this point on, our output port produces the initial_value from
the constructor, not any slider values.

Warning:
    It is not safe to call this when a CalcOutput might be happening
    on this instance concurrently on another thread.)""";
        } Delete;
        // Symbol: drake::visualization::MeshcatPoseSliders::MeshcatPoseSliders<T>
        struct /* ctor */ {
          // Source: drake/visualization/meshcat_pose_sliders.h
          const char* doc =
R"""(Creates MeshCat sliders for a roll-pitch-yaw-x-y-z parameterization of
a pose (aka RigidTransform).

Parameter ``meshcat``:
    The MeshCat instance where the sliders will be added.

Parameter ``initial_pose``:
    (Optional) If provided, the sliders' initial values will be as
    given.

Parameter ``lower_limit``:
    (Optional) The lower limit of the sliders for roll, pitch, yaw, x,
    y, z.

Parameter ``upper_limit``:
    (Optional) The upper limit of the sliders for roll, pitch, yaw, x,
    y, z.

Parameter ``step``:
    (Optional) The step argument of the slider, which is the smallest
    increment by which the slider can change values (and therefore
    update our output port's value). May be a single value or else a
    vector of length 6.

Parameter ``decrement_keycodes``:
    (Optional) A vector of length 6 with keycodes to assign to
    decrement the value of each slider (roll, pitch, yaw, x, y, z).
    See Meshcat∷AddSlider for more details.

Parameter ``increment_keycodes``:
    (Optional) A vector of length 6 with keycodes to assign to
    increment the value of each slider (roll, pitch, yaw, x, y, z).
    See Meshcat∷AddSlider for more details.

Parameter ``prefix``:
    (Optional) By default, the sliders will be named "roll, pitch,
    yaw, x, y, z". If prefix is non-empty, the the sliders will be
    named "{prefix}_roll, {prefix}_pitch", etc.

Parameter ``visible``:
    (Optional) If provided, determines which of the sliders (roll,
    pitch, yaw, x, y, z) are visible in the Meshcat window. This can
    be useful, for instance, if one only wants to control the pose in
    2D (and therefore only show the sliders for e.g. yaw, x, and y).

The default keycodes provide the following keyboard mapping:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ┌───────┬┬───────┬┬───────┐        ┌───────┬┬───────┬┬───────┐
    │   Q   ││   W   ││   E   │        │   U   ││   I   ││   O   │
    │ roll- ││ pitch+││ roll+ │        │   z-  ││   y+  ││   z+  │
    ├───────┼┼───────┼┼───────┤        ├───────┼┼───────┼┼───────┤
    ├───────┼┼───────┼┼───────┤        ├───────┼┼───────┼┼───────┤
    │   A   ││   S   ││   D   │        │   J   ││   K   ││   L   │
    │  yaw- ││ pitch-││  yaw+ │        │   x-  ││   y-  ││   x+  │
    └───────┴┴───────┴┴───────┘        └───────┴┴───────┴┴───────┘

.. raw:: html

    </details>)""";
        } ctor;
        // Symbol: drake::visualization::MeshcatPoseSliders::Run
        struct /* Run */ {
          // Source: drake/visualization/meshcat_pose_sliders.h
          const char* doc =
R"""(Publishes the given systems∷System (typically, a Diagram including
visualizers, to cause it to be visualized) whenever our sliders'
values change. Blocks until the user clicks a "Stop" button in the
MeshCat control panel, or if the timeout limit is reached.

Parameter ``context``:
    The systems∷Context for the systems∷Diagram.

Parameter ``timeout``:
    (Optional) In the absence of a button click, the duration (in
    seconds) to wait before returning. If the button is clicked, this
    function will return promptly, without waiting for the timeout.
    When no timeout is given, this function will block indefinitely.

Parameter ``stop_button_keycode``:
    a keycode that will be assigned to the "Stop" button. Setting this
    to the empty string means no keycode. See Meshcat∷AddButton for
    details. $*Default:* "Escape".

Returns:
    the most recently output pose.

Precondition:
    ``system`` must be this MeshcatPoseSliders system or be a
    top-level (i.e., "root") diagram that contains this
    MeshcatPoseSliders system.)""";
        } Run;
        // Symbol: drake::visualization::MeshcatPoseSliders::SetPose
        struct /* SetPose */ {
          // Source: drake/visualization/meshcat_pose_sliders.h
          const char* doc =
R"""(Sets the pose to ``X``. The MeshCat sliders will have their value
updated. Additionally, the "initial pose" tracked by this instance
will be updated to ``X``. This "initial pose" update will persist even
if sliders are removed (e.g., via Delete).)""";
        } SetPose;
      } MeshcatPoseSliders;
      // Symbol: drake::visualization::VisualizationConfig
      struct /* VisualizationConfig */ {
        // Source: drake/visualization/visualization_config.h
        const char* doc =
R"""(Settings for what MultibodyPlant and SceneGraph should send to Meshcat
and/or Meldis.

See ApplyVisualizationConfig() for how to enact this configuration.)""";
        // Symbol: drake::visualization::VisualizationConfig::Serialize
        struct /* Serialize */ {
          // Source: drake/visualization/visualization_config.h
          const char* doc = R"""()""";
        } Serialize;
        // Symbol: drake::visualization::VisualizationConfig::default_illustration_color
        struct /* default_illustration_color */ {
          // Source: drake/visualization/visualization_config.h
          const char* doc =
R"""(The color to apply to any illustration geometry that hasn't defined
one. The vector must be of size three (rgb) or four (rgba).)""";
        } default_illustration_color;
        // Symbol: drake::visualization::VisualizationConfig::default_proximity_color
        struct /* default_proximity_color */ {
          // Source: drake/visualization/visualization_config.h
          const char* doc =
R"""(The color to apply to any proximity geometry that hasn't defined one.
The vector must be of size three (rgb) or four (rgba).)""";
        } default_proximity_color;
        // Symbol: drake::visualization::VisualizationConfig::delete_on_initialization_event
        struct /* delete_on_initialization_event */ {
          // Source: drake/visualization/visualization_config.h
          const char* doc =
R"""(Determines whether to send a Meshcat∷Delete() messages to the Meshcat
object (if any) on an initialization event to remove any
visualizations, e.g., from a previous simulation.)""";
        } delete_on_initialization_event;
        // Symbol: drake::visualization::VisualizationConfig::enable_alpha_sliders
        struct /* enable_alpha_sliders */ {
          // Source: drake/visualization/visualization_config.h
          const char* doc =
R"""(Determines whether to enable alpha sliders for geometry display.)""";
        } enable_alpha_sliders;
        // Symbol: drake::visualization::VisualizationConfig::enable_meshcat_creation
        struct /* enable_meshcat_creation */ {
          // Source: drake/visualization/visualization_config.h
          const char* doc =
R"""(Whether to create a Meshcat object if needed.)""";
        } enable_meshcat_creation;
        // Symbol: drake::visualization::VisualizationConfig::initial_proximity_alpha
        struct /* initial_proximity_alpha */ {
          // Source: drake/visualization/visualization_config.h
          const char* doc =
R"""(The initial value of the proximity alpha slider. Note: the effective
transparency of the proximity geometry is the slider value multiplied
by the alpha value of ``default_proximity_color``. To have access to
the full range of opacity, the color's alpha value should be one and
the slider should be used to change it.)""";
        } initial_proximity_alpha;
        // Symbol: drake::visualization::VisualizationConfig::lcm_bus
        struct /* lcm_bus */ {
          // Source: drake/visualization/visualization_config.h
          const char* doc =
R"""(Which LCM URL to use.

See also:
    drake∷systems∷lcm∷LcmBuses)""";
        } lcm_bus;
        // Symbol: drake::visualization::VisualizationConfig::publish_contacts
        struct /* publish_contacts */ {
          // Source: drake/visualization/visualization_config.h
          const char* doc = R"""(Whether to show contact forces.)""";
        } publish_contacts;
        // Symbol: drake::visualization::VisualizationConfig::publish_illustration
        struct /* publish_illustration */ {
          // Source: drake/visualization/visualization_config.h
          const char* doc = R"""(Whether to show illustration geometry.)""";
        } publish_illustration;
        // Symbol: drake::visualization::VisualizationConfig::publish_inertia
        struct /* publish_inertia */ {
          // Source: drake/visualization/visualization_config.h
          const char* doc = R"""(Whether to show body inertia.)""";
        } publish_inertia;
        // Symbol: drake::visualization::VisualizationConfig::publish_period
        struct /* publish_period */ {
          // Source: drake/visualization/visualization_config.h
          const char* doc =
R"""(The duration (in seconds) between published LCM messages that update
visualization. (To help avoid small simulation time steps, we use a
default period that has an exact representation in binary floating
point; see drake#15021 for details.))""";
        } publish_period;
        // Symbol: drake::visualization::VisualizationConfig::publish_proximity
        struct /* publish_proximity */ {
          // Source: drake/visualization/visualization_config.h
          const char* doc = R"""(Whether to show proximity geometry.)""";
        } publish_proximity;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("default_illustration_color", default_illustration_color.doc),
            std::make_pair("default_proximity_color", default_proximity_color.doc),
            std::make_pair("delete_on_initialization_event", delete_on_initialization_event.doc),
            std::make_pair("enable_alpha_sliders", enable_alpha_sliders.doc),
            std::make_pair("enable_meshcat_creation", enable_meshcat_creation.doc),
            std::make_pair("initial_proximity_alpha", initial_proximity_alpha.doc),
            std::make_pair("lcm_bus", lcm_bus.doc),
            std::make_pair("publish_contacts", publish_contacts.doc),
            std::make_pair("publish_illustration", publish_illustration.doc),
            std::make_pair("publish_inertia", publish_inertia.doc),
            std::make_pair("publish_period", publish_period.doc),
            std::make_pair("publish_proximity", publish_proximity.doc),
          };
        }
      } VisualizationConfig;
    } visualization;
  } drake;
} pydrake_doc_visualization;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
