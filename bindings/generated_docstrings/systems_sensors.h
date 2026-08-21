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

// #include "drake/systems/sensors/accelerometer.h"
// #include "drake/systems/sensors/beam_model.h"
// #include "drake/systems/sensors/beam_model_params.h"
// #include "drake/systems/sensors/camera_config.h"
// #include "drake/systems/sensors/camera_config_functions.h"
// #include "drake/systems/sensors/camera_info.h"
// #include "drake/systems/sensors/gyroscope.h"
// #include "drake/systems/sensors/image.h"
// #include "drake/systems/sensors/image_file_format.h"
// #include "drake/systems/sensors/image_io.h"
// #include "drake/systems/sensors/image_to_lcm_image_array_t.h"
// #include "drake/systems/sensors/image_writer.h"
// #include "drake/systems/sensors/lcm_image_array_to_images.h"
// #include "drake/systems/sensors/lcm_image_traits.h"
// #include "drake/systems/sensors/pixel_types.h"
// #include "drake/systems/sensors/rgbd_sensor.h"
// #include "drake/systems/sensors/rgbd_sensor_async.h"
// #include "drake/systems/sensors/rgbd_sensor_discrete.h"
// #include "drake/systems/sensors/rotary_encoders.h"
// #include "drake/systems/sensors/sim_rgbd_sensor.h"

// Symbol: pydrake_doc_systems_sensors
constexpr struct /* pydrake_doc_systems_sensors */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::systems
    struct /* systems */ {
      // Symbol: drake::systems::scalar_conversion
      struct /* scalar_conversion */ {
        // Symbol: drake::systems::scalar_conversion::Traits
        struct /* Traits */ {
          // Source: drake/systems/sensors/beam_model.h
          const char* doc = R"""()""";
        } Traits;
      } scalar_conversion;
      // Symbol: drake::systems::sensors
      struct /* sensors */ {
        // Symbol: drake::systems::sensors::Accelerometer
        struct /* Accelerometer */ {
          // Source: drake/systems/sensors/accelerometer.h
          const char* doc =
R"""(Sensor to represent an ideal accelerometer sensor. Currently does not
represent noise or bias, but this could and should be added at a later
date. This sensor measures the proper acceleration of a point on a
given body B. Proper acceleration subtracts gravity from the
coordinate acceleration a_WS_S. That is, aproper_WS_S = a_WS_S - g_S
Note that measurement is taken with respect to the world frame, but
expressed in the coordinates of the local sensor frame S. Sensor frame
S is rigidly affixed to the given body B. Note, also, the sign of the
gravity component. For typical settings (e.g. on Earth assuming a
constant gravitational field), the direction of "-g_S" is "upwards."

There are three inputs to this sensor (nominally from a
MultibodyPlant): 1. A vector of body poses (e.g.
plant.get_body_poses_output_port()) 2. A vector of spatial velocities
(e.g. plant.get_body_spatial_velocities_output_port()) 3. A vector of
spatial accelerations (e.g.
plant.get_body_spatial_accelerations_output_port())

This class is therefore defined by: 1. The rigid body to which this
sensor is rigidly affixed. 2. A rigid transform from the body frame to
the sensor frame.

.. pydrake_system::

    name: Accelerometer
    input_ports:
    - body_poses
    - body_spatial_velocities
    - body_spatial_accelerations
    output_ports:
    - measurement)""";
          // Symbol: drake::systems::sensors::Accelerometer::Accelerometer<T>
          struct /* ctor */ {
            // Source: drake/systems/sensors/accelerometer.h
            const char* doc =
R"""(Parameter ``body``:
    the body B to which the sensor is affixed

Parameter ``X_BS``:
    the pose of sensor frame S in body B

Parameter ``gravity_vector``:
    the constant acceleration due to gravity expressed in world
    coordinates)""";
            // Source: drake/systems/sensors/accelerometer.h
            const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
          } ctor;
          // Symbol: drake::systems::sensors::Accelerometer::AddToDiagram
          struct /* AddToDiagram */ {
            // Source: drake/systems/sensors/accelerometer.h
            const char* doc =
R"""(Static factory method that creates an Accelerometer object and
connects it to the given plant. Modifies a Diagram by connecting the
input ports of the new Accelerometer to the appropriate output ports
of a MultibodyPlant. Must be called during Diagram building and given
the appropriate builder. This is a convenience method to simplify some
common boilerplate of Diagram wiring. Specifically, this makes three
connections:

1. plant.get_body_poses_output_port() to this.get_body_poses_input_port()
2. plant.get_body_spatial_velocities_output_port() to
       this.get_body_velocities_input_port()
3. plant.get_body_spatial_accelerations_output_port() to
       this.get_body_spatial_accelerations_output_port()

Parameter ``body``:
    the rigid body B to which the sensor is affixed

Parameter ``X_BS``:
    the pose of sensor frame S in body B

Parameter ``gravity_vector``:
    the constant acceleration due to gravity expressed in world
    coordinates

Parameter ``plant``:
    the plant to which the sensor will be connected

Parameter ``builder``:
    a pointer to the DiagramBuilder)""";
          } AddToDiagram;
          // Symbol: drake::systems::sensors::Accelerometer::body_index
          struct /* body_index */ {
            // Source: drake/systems/sensors/accelerometer.h
            const char* doc =
R"""(Returns the index of the RigidBody that was supplied in the
constructor.)""";
          } body_index;
          // Symbol: drake::systems::sensors::Accelerometer::get_body_accelerations_input_port
          struct /* get_body_accelerations_input_port */ {
            // Source: drake/systems/sensors/accelerometer.h
            const char* doc = R"""()""";
          } get_body_accelerations_input_port;
          // Symbol: drake::systems::sensors::Accelerometer::get_body_poses_input_port
          struct /* get_body_poses_input_port */ {
            // Source: drake/systems/sensors/accelerometer.h
            const char* doc = R"""()""";
          } get_body_poses_input_port;
          // Symbol: drake::systems::sensors::Accelerometer::get_body_velocities_input_port
          struct /* get_body_velocities_input_port */ {
            // Source: drake/systems/sensors/accelerometer.h
            const char* doc = R"""()""";
          } get_body_velocities_input_port;
          // Symbol: drake::systems::sensors::Accelerometer::get_measurement_output_port
          struct /* get_measurement_output_port */ {
            // Source: drake/systems/sensors/accelerometer.h
            const char* doc = R"""()""";
          } get_measurement_output_port;
          // Symbol: drake::systems::sensors::Accelerometer::gravity_vector
          struct /* gravity_vector */ {
            // Source: drake/systems/sensors/accelerometer.h
            const char* doc =
R"""(Returns the gravity vector supplied in the constructor, or zero if
none.)""";
          } gravity_vector;
          // Symbol: drake::systems::sensors::Accelerometer::pose
          struct /* pose */ {
            // Source: drake/systems/sensors/accelerometer.h
            const char* doc =
R"""(Gets X_BS, the pose of sensor frame S in body B)""";
          } pose;
        } Accelerometer;
        // Symbol: drake::systems::sensors::ApplyCameraConfig
        struct /* ApplyCameraConfig */ {
          // Source: drake/systems/sensors/camera_config_functions.h
          const char* doc =
R"""(Constructs a simulated camera sensor (rgbd sensor and publishing
systems) within ``builder``. As specified, the RGB, depth, and/or
label images from the camera are published via ``lcm`` on the channel
``DRAKE_RGBD_CAMERA_IMAGES_{camera_config.name}``.

Parameter ``config``:
    The camera configuration.

Parameter ``builder``:
    The diagram to add sensor and publishing systems into.

Parameter ``lcm_buses``:
    (Optional) The available LCM buses to use for camera message
    publication. When not provided, uses the ``lcm`` interface if
    provided, or else the ``config.lcm_bus`` must be set to "default"
    in which case an appropriate drake∷lcm∷DrakeLcm object is
    constructed and used internally.

Parameter ``plant``:
    (Optional) The MultibodyPlant to use for kinematics. In the common
    case where a MultibodyPlant has already been added to ``builder``
    using either AddMultibodyPlant() or AddMultibodyPlantSceneGraph(),
    the default value (nullptr) here is suitable and generally should
    be preferred. When provided, it must be a System that's been added
    to the given ``builder``. When not provided, uses the system named
    "plant" in the given ``builder``.

Parameter ``scene_graph``:
    (Optional) The SceneGraph to use for rendering. In the common case
    where a SceneGraph has already been added to ``builder`` using
    either AddMultibodyPlant() or AddMultibodyPlantSceneGraph(), the
    default value (nullptr) here is suitable and generally should be
    preferred. When provided, it must be a System that's been added to
    the given ``builder``. When not provided, uses the system named
    "scene_graph" in the given ``builder``.

Parameter ``lcm``:
    (Optional) The LCM interface used for visualization message
    publication. When not provided, uses the ``config.lcm_bus`` value
    to look up the appropriate interface from ``lcm_buses``.

Raises:
    RuntimeError if camera_config contains invalid values.

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

See also:
    drake∷multibody∷AddMultibodyPlant()

See also:
    drake∷systems∷lcm∷ApplyLcmBusConfig())""";
        } ApplyCameraConfig;
        // Symbol: drake::systems::sensors::BeamModel
        struct /* BeamModel */ {
          // Source: drake/systems/sensors/beam_model.h
          const char* doc =
R"""(Implements the "Beam Models of Range Finders" from section 6.3 of
Probabilistic Robotics (2006), by Thrun, Burgard, and Fox

This system takes a depth measurement signal as input, and outputs a
noisy measurement version of that signal, with some probability of
returning the true measurement with Gaussian noise, but also with some
probability of occlusions (short returns), of missed detections
(returning the max depth), and of returning just a (uniform) random
measurement.

Four additional input ports (each of the same dimension as the depth
signal) are provided for the random inputs: One for determining which
of the events occurred (true + noise, short return, max return, or
uniform return), and one each for modeling the distribution of short
true but noisy returns, short returns, and uniform returns).

We deviate from the textbook model in one respect: both here and in
the textbook, the distribution over short returns and the distribution
over getting a noisy version of the true return (aka a "hit") are
truncated. The short returns are from an exponential distribution but
truncated to be less than the input depth, and "hits" are drawn from a
Gaussian centered at the input depth but truncated at the maximum
range of the sensor. In the book, these distributions are normalized
so that the total probability of getting a short return and/or hit
stays constant (independent of the input depth). Here we do not
normalize, so that the probability of getting a short return decreases
as the input depth is smaller (there is a modeled obstacle closer to
the robot), and the tails of the "hit" distribution simply cause more
max returns as the input depth gets closer to the max range. This was
done both because it is arguably a better model and because it keeps
the code much simpler (to allow AutoDiff and Symbolic) given the
modeling framework we have here that builds the output out of simple
(non-truncated) random variable inputs.

.. pydrake_system::

    name: BeamModel
    input_ports:
    - depth
    - event
    - hit
    - short
    - uniform
    output_ports:
    - depth

It is convenient to use ``systems∷AddRandomInputs()`` to supply all
the random input signals:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    DiagramBuilder<double> builder;
     auto beam_model = builder.AddSystem<BeamModel>(1, 5.0);
     builder.ExportInput(beam_model->get_depth_input_port(), "depth");
     builder.ExportOutput(beam_model->get_output_port(0), "depth");
     AddRandomInputs(0.01, &builder);
     auto diagram = builder.Build();

.. raw:: html

    </details>)""";
          // Symbol: drake::systems::sensors::BeamModel::BeamModel<T>
          struct /* ctor */ {
            // Source: drake/systems/sensors/beam_model.h
            const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
          } ctor;
          // Symbol: drake::systems::sensors::BeamModel::get_depth_input_port
          struct /* get_depth_input_port */ {
            // Source: drake/systems/sensors/beam_model.h
            const char* doc = R"""()""";
          } get_depth_input_port;
          // Symbol: drake::systems::sensors::BeamModel::get_event_random_input_port
          struct /* get_event_random_input_port */ {
            // Source: drake/systems/sensors/beam_model.h
            const char* doc = R"""()""";
          } get_event_random_input_port;
          // Symbol: drake::systems::sensors::BeamModel::get_hit_random_input_port
          struct /* get_hit_random_input_port */ {
            // Source: drake/systems/sensors/beam_model.h
            const char* doc = R"""()""";
          } get_hit_random_input_port;
          // Symbol: drake::systems::sensors::BeamModel::get_mutable_parameters
          struct /* get_mutable_parameters */ {
            // Source: drake/systems/sensors/beam_model.h
            const char* doc = R"""()""";
          } get_mutable_parameters;
          // Symbol: drake::systems::sensors::BeamModel::get_short_random_input_port
          struct /* get_short_random_input_port */ {
            // Source: drake/systems/sensors/beam_model.h
            const char* doc = R"""()""";
          } get_short_random_input_port;
          // Symbol: drake::systems::sensors::BeamModel::get_uniform_random_input_port
          struct /* get_uniform_random_input_port */ {
            // Source: drake/systems/sensors/beam_model.h
            const char* doc = R"""()""";
          } get_uniform_random_input_port;
          // Symbol: drake::systems::sensors::BeamModel::max_range
          struct /* max_range */ {
            // Source: drake/systems/sensors/beam_model.h
            const char* doc = R"""()""";
          } max_range;
        } BeamModel;
        // Symbol: drake::systems::sensors::BeamModelParams
        struct /* BeamModelParams */ {
          // Source: drake/systems/sensors/beam_model_params.h
          const char* doc =
R"""(Specializes BasicVector with specific getters and setters.)""";
          // Symbol: drake::systems::sensors::BeamModelParams::BeamModelParams<T>
          struct /* ctor */ {
            // Source: drake/systems/sensors/beam_model_params.h
            const char* doc =
R"""(Default constructor. Sets all rows to their default value:

* ``lambda_short`` defaults to 1.0 dimensionless.

* ``sigma_hit`` defaults to 0.0 m.

* ``probability_short`` defaults to 0.0 dimensionless.

* ``probability_miss`` defaults to 0.0 dimensionless.

* ``probability_uniform`` defaults to 0.0 dimensionless.)""";
          } ctor;
          // Symbol: drake::systems::sensors::BeamModelParams::DoClone
          struct /* DoClone */ {
            // Source: drake/systems/sensors/beam_model_params.h
            const char* doc = R"""()""";
          } DoClone;
          // Symbol: drake::systems::sensors::BeamModelParams::GetCoordinateNames
          struct /* GetCoordinateNames */ {
            // Source: drake/systems/sensors/beam_model_params.h
            const char* doc =
R"""(See BeamModelParamsIndices∷GetCoordinateNames().)""";
          } GetCoordinateNames;
          // Symbol: drake::systems::sensors::BeamModelParams::GetElementBounds
          struct /* GetElementBounds */ {
            // Source: drake/systems/sensors/beam_model_params.h
            const char* doc = R"""()""";
          } GetElementBounds;
          // Symbol: drake::systems::sensors::BeamModelParams::IsValid
          struct /* IsValid */ {
            // Source: drake/systems/sensors/beam_model_params.h
            const char* doc =
R"""(Returns whether the current values of this vector are well-formed.)""";
          } IsValid;
          // Symbol: drake::systems::sensors::BeamModelParams::K
          struct /* K */ {
            // Source: drake/systems/sensors/beam_model_params.h
            const char* doc =
R"""(An abbreviation for our row index constants.)""";
          } K;
          // Symbol: drake::systems::sensors::BeamModelParams::Serialize
          struct /* Serialize */ {
            // Source: drake/systems/sensors/beam_model_params.h
            const char* doc =
R"""(Visit each field of this named vector, passing them (in order) to the
given Archive. The archive can read and/or write to the vector values.
One common use of Serialize is the //common/yaml tools.)""";
          } Serialize;
          // Symbol: drake::systems::sensors::BeamModelParams::SetToNamedVariables
          struct /* SetToNamedVariables */ {
            // Source: drake/systems/sensors/beam_model_params.h
            const char* doc =
R"""(Create a symbolic∷Variable for each element with the known variable
name. This is only available for T == symbolic∷Expression.)""";
          } SetToNamedVariables;
          // Symbol: drake::systems::sensors::BeamModelParams::lambda_short
          struct /* lambda_short */ {
            // Source: drake/systems/sensors/beam_model_params.h
            const char* doc =
R"""(The rate parameter of the (truncated) exponential distribution
governing short returns

Note:
    ``lambda_short`` is expressed in units of dimensionless.

Note:
    ``lambda_short`` has a limited domain of [0.0, +Inf].)""";
          } lambda_short;
          // Symbol: drake::systems::sensors::BeamModelParams::probability_miss
          struct /* probability_miss */ {
            // Source: drake/systems/sensors/beam_model_params.h
            const char* doc =
R"""(The probability of ignoring the input depth and simply returning the
max range of the sensor

Note:
    ``probability_miss`` is expressed in units of dimensionless.

Note:
    ``probability_miss`` has a limited domain of [0.0, 1.0].)""";
          } probability_miss;
          // Symbol: drake::systems::sensors::BeamModelParams::probability_short
          struct /* probability_short */ {
            // Source: drake/systems/sensors/beam_model_params.h
            const char* doc =
R"""(The total probability of getting a short return is probability_short *
p(lambda_short*w_short <= input_depth)

Note:
    ``probability_short`` is expressed in units of dimensionless.

Note:
    ``probability_short`` has a limited domain of [0.0, 1.0].)""";
          } probability_short;
          // Symbol: drake::systems::sensors::BeamModelParams::probability_uniform
          struct /* probability_uniform */ {
            // Source: drake/systems/sensors/beam_model_params.h
            const char* doc =
R"""(The probability of ignoring the input depth and simple returning a
uniform random value between 0 and the max range of the sensor

Note:
    ``probability_uniform`` is expressed in units of dimensionless.

Note:
    ``probability_uniform`` has a limited domain of [0.0, 1.0].)""";
          } probability_uniform;
          // Symbol: drake::systems::sensors::BeamModelParams::set_lambda_short
          struct /* set_lambda_short */ {
            // Source: drake/systems/sensors/beam_model_params.h
            const char* doc = R"""(Setter that matches lambda_short().)""";
          } set_lambda_short;
          // Symbol: drake::systems::sensors::BeamModelParams::set_probability_miss
          struct /* set_probability_miss */ {
            // Source: drake/systems/sensors/beam_model_params.h
            const char* doc = R"""(Setter that matches probability_miss().)""";
          } set_probability_miss;
          // Symbol: drake::systems::sensors::BeamModelParams::set_probability_short
          struct /* set_probability_short */ {
            // Source: drake/systems/sensors/beam_model_params.h
            const char* doc =
R"""(Setter that matches probability_short().)""";
          } set_probability_short;
          // Symbol: drake::systems::sensors::BeamModelParams::set_probability_uniform
          struct /* set_probability_uniform */ {
            // Source: drake/systems/sensors/beam_model_params.h
            const char* doc =
R"""(Setter that matches probability_uniform().)""";
          } set_probability_uniform;
          // Symbol: drake::systems::sensors::BeamModelParams::set_sigma_hit
          struct /* set_sigma_hit */ {
            // Source: drake/systems/sensors/beam_model_params.h
            const char* doc = R"""(Setter that matches sigma_hit().)""";
          } set_sigma_hit;
          // Symbol: drake::systems::sensors::BeamModelParams::sigma_hit
          struct /* sigma_hit */ {
            // Source: drake/systems/sensors/beam_model_params.h
            const char* doc =
R"""(The standard deviation of the (truncated) Gaussian distribution
governing the noisy returns of the true depth (aka hit)

Note:
    ``sigma_hit`` is expressed in units of m.

Note:
    ``sigma_hit`` has a limited domain of [0.0, +Inf].)""";
          } sigma_hit;
          // Symbol: drake::systems::sensors::BeamModelParams::with_lambda_short
          struct /* with_lambda_short */ {
            // Source: drake/systems/sensors/beam_model_params.h
            const char* doc =
R"""(Fluent setter that matches lambda_short(). Returns a copy of ``this``
with lambda_short set to a new value.)""";
          } with_lambda_short;
          // Symbol: drake::systems::sensors::BeamModelParams::with_probability_miss
          struct /* with_probability_miss */ {
            // Source: drake/systems/sensors/beam_model_params.h
            const char* doc =
R"""(Fluent setter that matches probability_miss(). Returns a copy of
``this`` with probability_miss set to a new value.)""";
          } with_probability_miss;
          // Symbol: drake::systems::sensors::BeamModelParams::with_probability_short
          struct /* with_probability_short */ {
            // Source: drake/systems/sensors/beam_model_params.h
            const char* doc =
R"""(Fluent setter that matches probability_short(). Returns a copy of
``this`` with probability_short set to a new value.)""";
          } with_probability_short;
          // Symbol: drake::systems::sensors::BeamModelParams::with_probability_uniform
          struct /* with_probability_uniform */ {
            // Source: drake/systems/sensors/beam_model_params.h
            const char* doc =
R"""(Fluent setter that matches probability_uniform(). Returns a copy of
``this`` with probability_uniform set to a new value.)""";
          } with_probability_uniform;
          // Symbol: drake::systems::sensors::BeamModelParams::with_sigma_hit
          struct /* with_sigma_hit */ {
            // Source: drake/systems/sensors/beam_model_params.h
            const char* doc =
R"""(Fluent setter that matches sigma_hit(). Returns a copy of ``this``
with sigma_hit set to a new value.)""";
          } with_sigma_hit;
        } BeamModelParams;
        // Symbol: drake::systems::sensors::BeamModelParamsIndices
        struct /* BeamModelParamsIndices */ {
          // Source: drake/systems/sensors/beam_model_params.h
          const char* doc =
R"""(Describes the row indices of a BeamModelParams.)""";
          // Symbol: drake::systems::sensors::BeamModelParamsIndices::GetCoordinateNames
          struct /* GetCoordinateNames */ {
            // Source: drake/systems/sensors/beam_model_params.h
            const char* doc =
R"""(Returns a vector containing the names of each coordinate within this
class. The indices within the returned vector matches that of this
class. In other words,
``BeamModelParamsIndices∷GetCoordinateNames()[i]`` is the name for
``BasicVector∷GetAtIndex(i)``.)""";
          } GetCoordinateNames;
        } BeamModelParamsIndices;
        // Symbol: drake::systems::sensors::CameraConfig
        struct /* CameraConfig */ {
          // Source: drake/systems/sensors/camera_config.h
          const char* doc =
R"""(Configuration of a camera. This covers all of the parameters for both
color (see geometry∷render∷ColorRenderCamera) and depth (see
geometry∷render∷DepthRenderCamera) cameras.

The various properties have restrictions on what they can be.

- Values must be finite.
- Some values must be positive (see notes on individual properties).
- Ranges must be specified such that the "minimum" value is less than or
equal to the "maximum" value. This includes the clipping range
[`clipping_near`, ``clipping_far`] and depth range [`z_near``, `z_far`].
- The depth range must lie *entirely* within the clipping range.

The values are only checked when the configuration is operated on:
during serialization, after deserialization, and when applying the
configuration (see ApplyCameraConfig().)

**Cameras and RenderEngines**

Every camera is supported by a geometry∷render∷RenderEngine instance.
These properties configure the render engine for this camera.

RenderEngines are uniquely identified by their name (as specified by
``renderer_name``) and configured by ``renderer_class``. Each
RenderEngine instance must have a unique name. Rendering will be more
efficient if multiple cameras share the same RenderEngine instance. To
share RenderEngine instances, cameras must have *same* value for
``renderer_name``.

Each camera can also provide the configuration of its supporting
RenderEngine (``renderer_class``). However, it is an error for two
cameras to specify the same ``renderer_name`` but provide
*conflicting* renderer configurations. It is the *user's*
responsibility to make sure that for every shared ``renderer_name``
value in a set of CameraConfig instances that the corresponding
``renderer_class`` values are compatible. How that is achieved depends
on the medium of specification. While there are multiple possible
mechanisms, these examples consist of the simplest.

*Configuring compatible ``renderer_class`` in YAML*

The simplest solution in YAML is to use the merge operator (``<<:``)
to guarantee that multiple cameras use the exact same
``renderer_class``. This terse example shows how that might be done.


.. code-block:: yaml

    cameras:
    - &BaseCamera:
    name: base_camera
    rgb: true
    depth: true
    label: true
    renderer_name: common_renderer
    renderer_class: !RenderEngineVtk
    exposure: 0.4
    cast_shadows: true
    - &DepthOnlyCamera
    <<: *BaseCamera
    name: depth_camera
    rgb: false
    label: false

In this example, we've defined two cameras named, ``base_camera`` and
``depth_camera``. Both share a RenderEngineVtk instance named
``common_renderer``. The merge operator guarantees that
``depth_camera`` has exactly duplicated ``base_camera`'s
`renderer_class`` value. However, it tweaks the camera definition by
disabling the rgb and label images. Edits to the ``RenderEngineVtk``
parameters in a single location are kept in sync across all cameras
that are supposed to share.

*Configuring compatible ``renderer_class`` in C++*

The same trick for coordinating multiple CameraConfig instances in
yaml will also work in C++. Simply instantiate a single set of
RenderEngine parameters and assign it to every instance.

In C++ there are other options available. If you have control over the
order that CameraConfig instances get applied, you can simply define
the ``renderer_class`` value for the first instance and leave it blank
in subsequent instances which share the same ``renderer_name``. It is
important to configure the RenderEngine on the *first* appearance of a
``renderer_name`` value.

*Rules for compatible ``renderer_class`` values*

Assume we have two CameraConfig instances that have a common
``renderer_name`` value (and both have well-formed ``renderer_class``
values). When we apply the first config instance, we will add a
RenderEngine. When we attempt to apply the second config instance, we
already have a RenderEngine with the specified name. Compatibility now
depends on the ``renderer_class`` value stored in the second config
instance. It will be compatible in the following cases:

- The second config instance's ``renderer_class`` value is the empty string.
- The second config instance's ``renderer_class`` value is the *class name* of
the RenderEngine instance that was already created.
- The second config instance's ``renderer_class`` contains the appropriate
parameters type for the type of RenderEngine instance that was already
created *and* the parameter values *exactly* match those in the
instantiated engine.

Every other ``renderer_class`` value in the second config instance
(e.g., naming a different type of RenderEngine, or specifying
different parameters) is considered a conflicting specification and
will give rise to runtime errors.)""";
          // Symbol: drake::systems::sensors::CameraConfig::FocalLength
          struct /* FocalLength */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(Specification of a camera's intrinsic focal properties as focal
*length* (in pixels). One or both values can be given. When only one
value is given, the other is assumed to match. At least one value must
be provided.)""";
            // Symbol: drake::systems::sensors::CameraConfig::FocalLength::Serialize
            struct /* Serialize */ {
              // Source: drake/systems/sensors/camera_config.h
              const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
            } Serialize;
            // Symbol: drake::systems::sensors::CameraConfig::FocalLength::ValidateOrThrow
            struct /* ValidateOrThrow */ {
              // Source: drake/systems/sensors/camera_config.h
              const char* doc = R"""()""";
            } ValidateOrThrow;
            // Symbol: drake::systems::sensors::CameraConfig::FocalLength::focal_x
            struct /* focal_x */ {
              // Source: drake/systems/sensors/camera_config.h
              const char* doc =
R"""(Reports focal length along x-axis.

Raises:
    RuntimeError if both ``x`` and ``y`` are null.)""";
            } focal_x;
            // Symbol: drake::systems::sensors::CameraConfig::FocalLength::focal_y
            struct /* focal_y */ {
              // Source: drake/systems/sensors/camera_config.h
              const char* doc =
R"""(Resolves focal length along y-axis.

Raises:
    RuntimeError if both ``x`` and ``y`` are null.)""";
            } focal_y;
            // Symbol: drake::systems::sensors::CameraConfig::FocalLength::x
            struct /* x */ {
              // Source: drake/systems/sensors/camera_config.h
              const char* doc =
R"""(If specified, the focal length along this axis; otherwise, use focal
length in the y-direction.)""";
            } x;
            // Symbol: drake::systems::sensors::CameraConfig::FocalLength::y
            struct /* y */ {
              // Source: drake/systems/sensors/camera_config.h
              const char* doc =
R"""(If specified, the focal length along this axis; otherwise, use focal
length in the x-direction.)""";
            } y;
            auto Serialize__fields() const {
              return std::array{
                std::make_pair("x", x.doc),
                std::make_pair("y", y.doc),
              };
            }
          } FocalLength;
          // Symbol: drake::systems::sensors::CameraConfig::FovDegrees
          struct /* FovDegrees */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(Specification of focal length via fields of view (in degrees).)""";
            // Symbol: drake::systems::sensors::CameraConfig::FovDegrees::Serialize
            struct /* Serialize */ {
              // Source: drake/systems/sensors/camera_config.h
              const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
            } Serialize;
            // Symbol: drake::systems::sensors::CameraConfig::FovDegrees::ValidateOrThrow
            struct /* ValidateOrThrow */ {
              // Source: drake/systems/sensors/camera_config.h
              const char* doc = R"""()""";
            } ValidateOrThrow;
            // Symbol: drake::systems::sensors::CameraConfig::FovDegrees::focal_x
            struct /* focal_x */ {
              // Source: drake/systems/sensors/camera_config.h
              const char* doc =
R"""(Resolves focal length along x-axis based on image dimensions and
defined value for ``x`` and/or ``y``.

Raises:
    RuntimeError if both ``x`` and ``y`` are null.)""";
            } focal_x;
            // Symbol: drake::systems::sensors::CameraConfig::FovDegrees::focal_y
            struct /* focal_y */ {
              // Source: drake/systems/sensors/camera_config.h
              const char* doc =
R"""(Resolves focal length along y-axis based on image dimensions and
defined value for ``y`` and/or ``x``.

Raises:
    RuntimeError if both ``x`` and ``y`` are null.)""";
            } focal_y;
            // Symbol: drake::systems::sensors::CameraConfig::FovDegrees::x
            struct /* x */ {
              // Source: drake/systems/sensors/camera_config.h
              const char* doc =
R"""(If specified, compute focal length along this axis; otherwise, use
focal length given computation for ``y``.)""";
            } x;
            // Symbol: drake::systems::sensors::CameraConfig::FovDegrees::y
            struct /* y */ {
              // Source: drake/systems/sensors/camera_config.h
              const char* doc =
R"""(If specified, compute focal length along this axis; otherwise, use
focal length given computation for ``x``.)""";
            } y;
            auto Serialize__fields() const {
              return std::array{
                std::make_pair("x", x.doc),
                std::make_pair("y", y.doc),
              };
            }
          } FovDegrees;
          // Symbol: drake::systems::sensors::CameraConfig::MakeCameras
          struct /* MakeCameras */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(Creates color and depth camera data from this configuration.

Raises:
    RuntimeError if configuration values do not satisfy the documented
    prerequisites.)""";
          } MakeCameras;
          // Symbol: drake::systems::sensors::CameraConfig::Serialize
          struct /* Serialize */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
          } Serialize;
          // Symbol: drake::systems::sensors::CameraConfig::ValidateOrThrow
          struct /* ValidateOrThrow */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc = R"""(Throws if the values are inconsistent.)""";
          } ValidateOrThrow;
          // Symbol: drake::systems::sensors::CameraConfig::X_BC
          struct /* X_BC */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(The pose of the color sensor relative to the camera body frame.

Precondition:
    ``X_BC.base_frame`` is empty.)""";
          } X_BC;
          // Symbol: drake::systems::sensors::CameraConfig::X_BD
          struct /* X_BD */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(The pose of the depth sensor relative to the camera body frame.

Precondition:
    ``X_BD.base_frame`` is empty.)""";
          } X_BD;
          // Symbol: drake::systems::sensors::CameraConfig::X_PB
          struct /* X_PB */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(The pose of the body camera relative to a parent frame P. If
``X_PB.base_frame`` is unspecified, then the world frame is assumed to
be the parent frame.

Precondition:
    ``X_PB.base_frame`` is empty *or* refers to a valid, unique frame.)""";
          } X_PB;
          // Symbol: drake::systems::sensors::CameraConfig::background
          struct /* background */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(The "background" color. This is the color drawn where there are no
objects visible. Its default value matches the default value for
render∷RenderEngineVtkParams∷default_clear_color. See the
documentation for geometry∷Rgba∷Serialize for how to define this value
in YAML.

This value is used only if the ``render_class`` specifies either
``"RenderEngineVtk"`` or ``"RenderEngineGl"`` by *name*
(RenderEngineGltfClient doesn't have a configurable background color.))""";
          } background;
          // Symbol: drake::systems::sensors::CameraConfig::capture_offset
          struct /* capture_offset */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(Phase offset (in seconds) for image capture, relative to the
simulator's time zero. This can be useful to stagger multiple cameras.
Refer to the RgbdSensorAsync class for a comprehensive description.

Precondition:
    capture_offset is non-negative and finite.)""";
          } capture_offset;
          // Symbol: drake::systems::sensors::CameraConfig::center_x
          struct /* center_x */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(The x-position of the principal point (in pixels). To query what the
current value is, use principal_point().

Precondition:
    0 < center_x < width or is std∷nullopt.)""";
          } center_x;
          // Symbol: drake::systems::sensors::CameraConfig::center_y
          struct /* center_y */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(The y-position of the principal point (in pixels). To query what the
current value is, use principal_point().

Precondition:
    0 < center_y < height or is std∷nullopt.)""";
          } center_y;
          // Symbol: drake::systems::sensors::CameraConfig::clipping_far
          struct /* clipping_far */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(The distance (in meters) from sensor origin to far clipping plane.

Precondition:
    clipping_far is a positive, finite number.)""";
          } clipping_far;
          // Symbol: drake::systems::sensors::CameraConfig::clipping_near
          struct /* clipping_near */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(The distance (in meters) from sensor origin to near clipping plane.

Precondition:
    clipping_near is a positive, finite number.)""";
          } clipping_near;
          // Symbol: drake::systems::sensors::CameraConfig::depth
          struct /* depth */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(If true, depth images will be produced and published via LCM.)""";
          } depth;
          // Symbol: drake::systems::sensors::CameraConfig::do_compress
          struct /* do_compress */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(Controls whether the images are broadcast in a compressed format.)""";
          } do_compress;
          // Symbol: drake::systems::sensors::CameraConfig::focal
          struct /* focal */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(The focal properties of the camera. It can be specified in one of two
ways:

- A FocalLength which specifies the focal lengths (in pixels) in the x-
and y-directions.
- An FovDegrees which defines focal length *implicitly* by deriving it from
field of view measures (in degrees).

For both specifications, if only one value is given (in either
direction), the focal length (as reported by focal_x() and focal_y())
is determined by that single value and is assumed to be symmetric for
both directions.

Precondition:
    focal length(s) is a/are finite, positive number(s).)""";
          } focal;
          // Symbol: drake::systems::sensors::CameraConfig::focal_x
          struct /* focal_x */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(Returns the focal length (in pixels) in the x-direction.)""";
          } focal_x;
          // Symbol: drake::systems::sensors::CameraConfig::focal_y
          struct /* focal_y */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(Returns the focal length (in pixels) in the y-direction.)""";
          } focal_y;
          // Symbol: drake::systems::sensors::CameraConfig::fps
          struct /* fps */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(Publishing rate (in Hz) for both RGB and depth cameras (as requested).

Precondition:
    fps is a positive, finite number.)""";
          } fps;
          // Symbol: drake::systems::sensors::CameraConfig::height
          struct /* height */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(%Image height (in pixels).

Precondition:
    height > 0.)""";
          } height;
          // Symbol: drake::systems::sensors::CameraConfig::label
          struct /* label */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(If true, label images will be produced and published via LCM.)""";
          } label;
          // Symbol: drake::systems::sensors::CameraConfig::lcm_bus
          struct /* lcm_bus */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(Which LCM URL to use.

See also:
    drake∷systems∷lcm∷LcmBuses)""";
          } lcm_bus;
          // Symbol: drake::systems::sensors::CameraConfig::name
          struct /* name */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(The camera name. This ``name`` is used as part of the corresponding
RgbdSensor system's name and serves as a suffix of the LCM channel
name.

Precondition:
    ``name`` is not empty.)""";
          } name;
          // Symbol: drake::systems::sensors::CameraConfig::output_delay
          struct /* output_delay */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(Delay (in seconds) between when the scene graph geometry is "captured"
and when the output image is published. Refer to the RgbdSensorAsync
class for a comprehensive description.

Precondition:
    output_delay is non-negative and strictly less than 1/fps.)""";
          } output_delay;
          // Symbol: drake::systems::sensors::CameraConfig::principal_point
          struct /* principal_point */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(Returns the position of the principal point. This respects the
semantics that undefined center_x and center_y place the principal
point in the center of the image.)""";
          } principal_point;
          // Symbol: drake::systems::sensors::CameraConfig::renderer_class
          struct /* renderer_class */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(The configuration of the camera's supporting RenderEngine.

The value can be one of: - An empty string. - A string containing the
class name of the RenderEngine to use (i.e., ``"RenderEngineVtk"``,
`"RenderEngineGl"`, or ``"RenderEngineGltfClient"``). - The struct of
parameters for a supported engine, i.e., RenderEngineVtkParams,
RenderEngineGlParams, or RenderEngineGltfClientParams.

For config instances which have a unique ``renderer_name`` value (one
that has not yet been added to the diagram), the various possible
``renderer_class`` values will instantiate a new RenderEngine and
associate it with a camera according to the following rules.

- Empty string: a RenderEngine of the *default* type (RenderEngineVtk)
will be instantiated with *default* parameters.
- Class name string: a RenderEngine of the *named* type will be
instantiated with *default* parameters.
- Parameter struct: a RenderEngine of the type compatible with the struct
will be instantiated, with those parameter values.

If, however, the ``renderer_name`` is not unique, as documented
camera_config_compatible_renderer "above", the values of
``renderer_class`` must be *compatible* with each other.

Note: RenderEngineVtk is the default RenderEngine type. It is slower,
but portable and robust. RenderEngineGl is an option if you are on
Ubuntu and need the improved performance (at the *possible* cost of
lesser image fidelity).

*Configuring in YAML*

It isn't always obvious what the proper spelling in YAML is. The
following examples illustrate how the ``renderer_class`` field can be
defined in YAML files.

1. Use the RenderEngineVtk with all default parameters - providing the class
name as a string.



.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    {yaml}
    renderer_class: RenderEngineVtk

.. raw:: html

    </details>

2. Use the RenderEngineVtk with all default parameters - providing the
engine parameters with default values.



.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    {yaml}
    renderer_class: !RenderEngineVtkParams {}

.. raw:: html

    </details>

3. Use the RenderEngineVtk with a customized clear color (black).



.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    {yaml}
    renderer_class: !RenderEngineVtkParams
    default_clear_color: [0, 0, 0]

.. raw:: html

    </details>

4. Use the RenderEngineGl with a customized clear color (black).



.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    {yaml}
    renderer_class: !RenderEngineGlParams
    default_clear_color:
    rgba: [0, 0, 0, 1]

.. raw:: html

    </details>

5. Use the RenderEngineGltfClient with fully specified properties.



.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    {yaml}
    renderer_class: !RenderEngineGltfClientParams
    base_url: http://10.10.10.1
    render_endpoint: server
    verbose: true
    cleanup: false

.. raw:: html

    </details>

6. Explicitly request Drake's default render engine or previously configured
RenderEngine based on shared ``renderer_name``.



.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    {yaml}
    renderer_class: ""

.. raw:: html

    </details>

Things to note:

- When providing the *parameters* for the engine, the declaration must
begin with ``!`` to announce it as a type (examples 2, 3, and 4).
- A defaulted set of parameters must have a trailing ``{}`` (example 2).
- Two engine parameter sets may have the same semantic parameter but spell
it differently (``default_clear_color`` in examples 3 and 4).

Precondition:
    ``renderer_class`` is a string and either empty or one of
    ``"RenderEngineVtk"``, `"RenderEngineGl"`, or
    ``"RenderEngineGltfClient"``, or, it is a parameter type of one of
    Drake's RenderEngine implementations.

See also:
    drake∷geometry∷SceneGraph∷GetRendererTypeName().)""";
          } renderer_class;
          // Symbol: drake::systems::sensors::CameraConfig::renderer_name
          struct /* renderer_name */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(The name of the geometry∷render∷RenderEngine that this camera uses.

Precondition:
    ``renderer_name`` is not empty.)""";
          } renderer_name;
          // Symbol: drake::systems::sensors::CameraConfig::rgb
          struct /* rgb */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(If true, RGB images will be produced and published via LCM.)""";
          } rgb;
          // Symbol: drake::systems::sensors::CameraConfig::show_rgb
          struct /* show_rgb */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(Controls whether the rendered RGB and/or label images are displayed
(in separate windows controlled by the thread in which the camera
images are rendered). Because both RGB and label images are configured
from the same ``ColorRenderCamera``, this setting applies to both
images. Even when set to true, whether or not the image is able to be
displayed depends on the specific render engine and its configuration
(see e.g., geometry∷RenderEngineVtkParams∷backend).

Note: This flag is intended for quick debug use during development
instead of serving as an image viewer. Currently, there are known
issues, e.g., flickering windows when multiple cameras share the same
renderer or upside-down images if RenderEngineGl is set. See issue
#18862 for the proposal to visualize images via Meldis.)""";
          } show_rgb;
          // Symbol: drake::systems::sensors::CameraConfig::width
          struct /* width */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(%Image width (in pixels).

Precondition:
    width > 0.)""";
          } width;
          // Symbol: drake::systems::sensors::CameraConfig::z_far
          struct /* z_far */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(The distance (in meters) from sensor origin to farthest measurable
plane.

Precondition:
    z_near is a positive, finite number.)""";
          } z_far;
          // Symbol: drake::systems::sensors::CameraConfig::z_near
          struct /* z_near */ {
            // Source: drake/systems/sensors/camera_config.h
            const char* doc =
R"""(The distance (in meters) from sensor origin to closest measurable
plane.

Precondition:
    z_near is a positive, finite number.)""";
          } z_near;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("X_BC", X_BC.doc),
              std::make_pair("X_BD", X_BD.doc),
              std::make_pair("X_PB", X_PB.doc),
              std::make_pair("background", background.doc),
              std::make_pair("capture_offset", capture_offset.doc),
              std::make_pair("center_x", center_x.doc),
              std::make_pair("center_y", center_y.doc),
              std::make_pair("clipping_far", clipping_far.doc),
              std::make_pair("clipping_near", clipping_near.doc),
              std::make_pair("depth", depth.doc),
              std::make_pair("do_compress", do_compress.doc),
              std::make_pair("focal", focal.doc),
              std::make_pair("fps", fps.doc),
              std::make_pair("height", height.doc),
              std::make_pair("label", label.doc),
              std::make_pair("lcm_bus", lcm_bus.doc),
              std::make_pair("name", name.doc),
              std::make_pair("output_delay", output_delay.doc),
              std::make_pair("renderer_class", renderer_class.doc),
              std::make_pair("renderer_name", renderer_name.doc),
              std::make_pair("rgb", rgb.doc),
              std::make_pair("show_rgb", show_rgb.doc),
              std::make_pair("width", width.doc),
              std::make_pair("z_far", z_far.doc),
              std::make_pair("z_near", z_near.doc),
            };
          }
        } CameraConfig;
        // Symbol: drake::systems::sensors::CameraInfo
        struct /* CameraInfo */ {
          // Source: drake/systems/sensors/camera_info.h
          const char* doc =
R"""(Simple class for characterizing the Drake camera model. The camera
model is based on the `pinhole *model_
<http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html>`_,
which is related to but distinct from an actual `pinhole _camera_
<https://en.wikipedia.org/wiki/Pinhole_camera>`_. The former is a
mathematical model for producing images, the latter is a physical
object.

The camera info members are directly tied to the underlying model's
mathematical parameters. In order to understand the model parameters,
we will provide a discussion of how cameras are treated in Drake with
some common terminology (liberally borrowed from computer vision).

**Pinhole camera model**

(To get an exact definition of the terms used here, please refer to
the glossary below.)

Intuitively, a camera produces images of an observed environment. The
pinhole model serves as a camera for a virtually modeled environment.
Its parameters are those required to determine where in the resultant
image a virtual object appears. In essence, the parameters of the
camera model define a mapping from points in 3D space to a point on
the image (2D space).

The full discussion of this mapping can be found in the `OpenCV
documentation
<https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html>`_.
Here, we'll highlight one or two points as it relates to this struct.

The mapping from 3D to 2D is decomposed into two sets of properties:
extrinsic and intrinsic. The extrinsic properties define the pose of
the camera in the environment -- specifically, given the camera frame
``C``, it defines ``X_WC``. Once a point ``Q`` is measured and
expressed in the camera's frame (i.e., ``p_CQ_C``), the intrinsic
matrix projects it into the 2D image. CameraInfo does _not* concern
itself with the extrinsic properties (other classes are responsible
for that -- see RgbdSensor).

CameraInfo defines the parameters of the intrinsic projection. The
projection can be captured by the camera or intrinsic matrix which
essentially maps points in the camera frame C to the image plane. The
camera looks along the positive ``Cz`` axis, and the ``Cy`` axis
points down. The projection matrix is called ``A`` in the OpenCV
documentation, but typically called ``K`` in computer vision
literature:

│ f_x 0 c_x │ K = │ 0 f_y c_y │, i.e., │ 0 0 1 │

- This matrix maps a point in the camera frame C to the projective space of the
image (via homogeneous coordinates). The resulting image coordinate ``(u, v)``
is extracted by dividing out the homogeneous scale factor.
- (c_x, c_y) defines the principal point.
- (f_x, f_y) defines the model focal length.

In other words, for point Q in the world frame, its projected position
in the 2D image ``(u_Q, v_Q)`` is calculated as:

│ u_Q │ s│ v_Q │ = K ⋅ X_CW ⋅ p_WQ │ 1 │

Note: The expression on the right will generally produce a homogeneous
coordinate vector of the form ``(s * u_Q, s * v_Q, s)``. The texture
coordinate is defined as the first two measures when the *third*
measure is 1. The magic of homogeneous coordinates allows us to simply
factor out ``s``.

**Alignment of the camera frame with the image**

When looking at the resulting image and reasoning about the camera
that produced it, one can say that Cz points into the image, Cx is
parallel with the image rows, pointing to the right, and Cy is
parallel with the image columns, pointing down leading to language
such as: "X-right", "Y-down", and "Z-forward".

**Glossary**

These terms are important to the discussion. Some refer to real world
concepts and some to the model. The application domain of the term
will be indicated and, where necessary, ambiguities will be resolved.
The terms are ordered alphabetically for ease of reference.

- **aperture**: the opening in a camera through which light passes. The origin
of the camera frame ``Co`` is located at the aperture's center.
- in a physical camera it may contain a lens and the size of the camera
affects optical artifacts such as depth of field.
- in the pinhole model, there is no lens and the aperture is a single point.
- **focal length**: a camera property that determines the field of view angle
and the scale of objects in the image (large focal length --> small field of
view angle).
- In a physical camera, it is the distance (in meters) between the center of
the lens and the plane at which all incoming, parallel light rays converge
(assuming a radially symmetric lens).
- in the pinhole model, ``(f_x, f_y)`` is described as "focal length", but the
units are in pixels and the interpretation is different. The relationship
between ``(f_x, f_y)`` and the physical focal length ``F`` is ``f_i = F * s_i``,
where ``(s_x, s_y)`` are the number of pixels per meter in the x- and
y-directions (of the image). Both values described as "focal length" have
analogous effects on field of view and scale of objects in the image.
- **frame**: (Also "pose") an origin point and a space-spanning basis in 2D/3D,
as in multibody_frames_and_bodies.
- Incidentally, the word "frame" is also used in conjunction with a single
image in a video (such as a "video frame"). Drake doesn't use this sense in
discussing cameras (unless explicitly noted).
- **image**: an array of measurements such that the measured values have
spatial relationships based on where they are in the array.
- **image frame**: the 2D frame embedded in 3D space spanning the image plane.
The image frame's x- and y-axes are parallel with ``Cx`` and ``Cy``,
respectively. Coordinates are expressed as the pair ``(u, v)`` and the camera's
image lies on the plane spanned by the frame's basis. The *center* of the
pixel in the first row and first column is at ``(u=0, v=0)``.
- **image plane**: a plane in 3D which is perpendicular to the camera's viewing
direction. Conceptually, the image lies on this plane.
- In a physical pinhole camera, the aperture is between the image plane and
the environment being imaged.
- In the pinhole model, the image plane lies between the environment and
aperture (i.e., in the positive Cz direction from Co).
- **imager**: a sensor whose measurements are reported in images.
- **principal point**: The projection of the camera origin, ``Co``, on the image
plane. Its value is measured from the image's origin in pixels.
- **sensor**: a measurement device.
- **viewing direction**: the direction the camera is facing. Defined as being
parallel with Cz.)""";
          // Symbol: drake::systems::sensors::CameraInfo::CameraInfo
          struct /* ctor */ {
            // Source: drake/systems/sensors/camera_info.h
            const char* doc_6args_width_height_focal_x_focal_y_center_x_center_y =
R"""(Constructor that directly sets the image size, principal point, and
focal lengths.

Parameter ``width``:
    The image width in pixels, must be positive.

Parameter ``height``:
    The image height in pixels, must be positive.

Parameter ``focal_x``:
    The *model* "focal length" x in pixels (see above), must be
    positive and finite.

Parameter ``focal_y``:
    The *model* "focal length" y in pixels (see above), must be
    positive and finite.

Parameter ``center_x``:
    The x coordinate of the principal point in pixels (see above),
    must lie in the range 0 < center_x < width.

Parameter ``center_y``:
    The y coordinate of the principal point in pixels (see above),
    must lie in the range 0 < center_y < height.

Raises:
    RuntimeError if the provided values don't satisfy the listed
    requirements.)""";
            // Source: drake/systems/sensors/camera_info.h
            const char* doc_3args_width_height_intrinsic_matrix =
R"""(Constructs this instance by extracting focal_x, focal_y, center_x, and
center_y from the provided intrinsic_matrix.

Parameter ``width``:
    The image width in pixels, must be positive.

Parameter ``height``:
    The image height in pixels, must be positive.

Parameter ``intrinsic_matrix``:
    The intrinsic matrix (K) as documented above. Where K is defined
    to be non-zero, the values must be finite and the focal length
    values must be positive.

Raises:
    RuntimeError if intrinsic_matrix is not of the form indicated
    above for the pinhole camera model (representing an affine /
    homogeneous transform) or the non-zero values don't meet the
    documented requirements.)""";
            // Source: drake/systems/sensors/camera_info.h
            const char* doc_3args_width_height_fov_y =
R"""(Constructs this instance from image size and vertical field of view.
We assume the principal point is in the center of the image; ``(center
x, center_y)`` is equal to ``(width / 2.0 - 0.5, height / 2.0 -
0.5)``. We also assume the focal lengths ``focal_x`` and ``focal_y``
are identical (modeling a radially symmetric lens). The value is
derived from field of view and image size as:

focal_x = focal_y = height * 0.5 / tan(0.5 * fov_y)

Parameter ``width``:
    The image width in pixels, must be positive.

Parameter ``height``:
    The image height in pixels, must be positive.

Parameter ``fov_y``:
    The vertical field of view in radians, must be positive and
    finite.

Raises:
    RuntimeError if the provided values don't satisfy the listed
    requirements.)""";
          } ctor;
          // Symbol: drake::systems::sensors::CameraInfo::center_x
          struct /* center_x */ {
            // Source: drake/systems/sensors/camera_info.h
            const char* doc =
R"""(Returns the principal point's x coordinate in pixels.)""";
          } center_x;
          // Symbol: drake::systems::sensors::CameraInfo::center_y
          struct /* center_y */ {
            // Source: drake/systems/sensors/camera_info.h
            const char* doc =
R"""(Returns the principal point's y coordinate in pixels.)""";
          } center_y;
          // Symbol: drake::systems::sensors::CameraInfo::focal_x
          struct /* focal_x */ {
            // Source: drake/systems/sensors/camera_info.h
            const char* doc = R"""(Returns the focal length x in pixels.)""";
          } focal_x;
          // Symbol: drake::systems::sensors::CameraInfo::focal_y
          struct /* focal_y */ {
            // Source: drake/systems/sensors/camera_info.h
            const char* doc = R"""(Returns the focal length y in pixels.)""";
          } focal_y;
          // Symbol: drake::systems::sensors::CameraInfo::fov_x
          struct /* fov_x */ {
            // Source: drake/systems/sensors/camera_info.h
            const char* doc =
R"""(Returns the field of view in the x-direction (in radians).)""";
          } fov_x;
          // Symbol: drake::systems::sensors::CameraInfo::fov_y
          struct /* fov_y */ {
            // Source: drake/systems/sensors/camera_info.h
            const char* doc =
R"""(Returns the field of view in the y-direction (in radians).)""";
          } fov_y;
          // Symbol: drake::systems::sensors::CameraInfo::height
          struct /* height */ {
            // Source: drake/systems/sensors/camera_info.h
            const char* doc =
R"""(Returns the height of the image in pixels.)""";
          } height;
          // Symbol: drake::systems::sensors::CameraInfo::intrinsic_matrix
          struct /* intrinsic_matrix */ {
            // Source: drake/systems/sensors/camera_info.h
            const char* doc = R"""(Returns the camera intrinsic matrix, K.)""";
          } intrinsic_matrix;
          // Symbol: drake::systems::sensors::CameraInfo::width
          struct /* width */ {
            // Source: drake/systems/sensors/camera_info.h
            const char* doc =
R"""(Returns the width of the image in pixels.)""";
          } width;
        } CameraInfo;
        // Symbol: drake::systems::sensors::ConvertDepth16UTo32F
        struct /* ConvertDepth16UTo32F */ {
          // Source: drake/systems/sensors/image.h
          const char* doc =
R"""(Converts a single channel 16-bit unsigned int depth image with depths
in millimeters to a single channel 32-bit float depth image with
depths in meters.)""";
        } ConvertDepth16UTo32F;
        // Symbol: drake::systems::sensors::ConvertDepth32FTo16U
        struct /* ConvertDepth32FTo16U */ {
          // Source: drake/systems/sensors/image.h
          const char* doc =
R"""(Converts a single channel 32-bit float depth image with depths in
meters to a single channel 16-bit unsigned int depth image with depths
in millimeters.

Note that pixels that are not kTooFar in the input image may saturate
to be kTooFar in the output image due to the smaller representable
range.)""";
        } ConvertDepth32FTo16U;
        // Symbol: drake::systems::sensors::Gyroscope
        struct /* Gyroscope */ {
          // Source: drake/systems/sensors/gyroscope.h
          const char* doc =
R"""(Sensor to represent an ideal gyroscopic sensor. Currently does not
represent noise or bias, but this could and should be added at a later
date. This sensor measures the angular velocity of a given body B
relative to the world frame. The sensor frame S is rigidly affixed to
the given body B. The measurement, written w_WS_S, is expressed in the
coordinates of frame S. Note that, since S is fixed to B, the angular
velocity of the two frames is identical, w_WS_S = w_WB_S.

There are two inputs to this sensor (nominally from a MultibodyPlant):
1. A vector of body poses (e.g. plant.get_body_poses_output_port()),
2. A vector of spatial velocities, (e.g.
plant.get_body_spatial_velocities_output_port()).

This class is therefore defined by: 1. The Body to which this sensor
is rigidly affixed, 2. The pose of the sensor frame in the body frame.
Note that the translational component of the transformation is not
strictly needed by a gyroscope, as the position of the sensor does not
affect the measurement. However, as a sensor does have a physical
location, it is included here for completeness and in case it might be
useful for display or other purposes.

.. pydrake_system::

    name: Gyroscope
    input_ports:
    - body_poses
    - body_spatial_velocities
    output_ports:
    - measurement)""";
          // Symbol: drake::systems::sensors::Gyroscope::AddToDiagram
          struct /* AddToDiagram */ {
            // Source: drake/systems/sensors/gyroscope.h
            const char* doc =
R"""(Static factory method that creates a Gyroscope object and connects it
to the given plant. Modifies a Diagram by connecting the input ports
of the new Gyroscope to the appropriate output ports of a
MultibodyPlant. Must be called during Diagram building and given the
appropriate builder. This is a convenience method to simplify some
common boilerplate of Diagram wiring. Specifically, this makes three
connections:

1. plant.get_body_poses_output_port() to this.get_body_poses_input_port()
2. plant.get_body_spatial_velocities_output_port() to
       this.get_body_velocities_input_port()

Parameter ``body``:
    the body B to which the sensor is affixed

Parameter ``X_BS``:
    X_BS the pose of sensor frame S in body B

Parameter ``plant``:
    the plant to which the sensor will be connected

Parameter ``builder``:
    a pointer to the DiagramBuilder)""";
          } AddToDiagram;
          // Symbol: drake::systems::sensors::Gyroscope::Gyroscope<T>
          struct /* ctor */ {
            // Source: drake/systems/sensors/gyroscope.h
            const char* doc =
R"""(Constructor for Gyroscope using full transform.

Parameter ``body``:
    the body B to which the sensor is affixed

Parameter ``X_BS``:
    the pose of sensor frame S in body B)""";
            // Source: drake/systems/sensors/gyroscope.h
            const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
          } ctor;
          // Symbol: drake::systems::sensors::Gyroscope::body_index
          struct /* body_index */ {
            // Source: drake/systems/sensors/gyroscope.h
            const char* doc =
R"""(Returns the index of the RigidBody that was supplied in the
constructor.)""";
          } body_index;
          // Symbol: drake::systems::sensors::Gyroscope::get_body_poses_input_port
          struct /* get_body_poses_input_port */ {
            // Source: drake/systems/sensors/gyroscope.h
            const char* doc = R"""()""";
          } get_body_poses_input_port;
          // Symbol: drake::systems::sensors::Gyroscope::get_body_velocities_input_port
          struct /* get_body_velocities_input_port */ {
            // Source: drake/systems/sensors/gyroscope.h
            const char* doc = R"""()""";
          } get_body_velocities_input_port;
          // Symbol: drake::systems::sensors::Gyroscope::get_measurement_output_port
          struct /* get_measurement_output_port */ {
            // Source: drake/systems/sensors/gyroscope.h
            const char* doc = R"""()""";
          } get_measurement_output_port;
          // Symbol: drake::systems::sensors::Gyroscope::pose
          struct /* pose */ {
            // Source: drake/systems/sensors/gyroscope.h
            const char* doc =
R"""(Gets X_BS, the pose of sensor frame S in body B.)""";
          } pose;
        } Gyroscope;
        // Symbol: drake::systems::sensors::Image
        struct /* Image */ {
          // Source: drake/systems/sensors/image.h
          const char* doc =
R"""(Simple data format for Image. For the complex calculation with the
image, consider converting this to other libaries' Matrix data format,
i.e., MatrixX in Eigen, Mat in OpenCV, and so on.

The origin of image coordinate system is on the left-upper corner.

Template parameter ``kPixelType``:
    The pixel type enum that denotes the pixel format and the data
    type of a channel. TODO(zachfang): move most of the function
    definitions in this class to the .cc file.)""";
          // Symbol: drake::systems::sensors::Image::Image<kPixelType>
          struct /* ctor */ {
            // Source: drake/systems/sensors/image.h
            const char* doc_0args = R"""(Constructs a zero-sized image.)""";
            // Source: drake/systems/sensors/image.h
            const char* doc_2args =
R"""(Image size only constructor. Specifies a width and height for the
image. The width and height can be either both zero or both strictly
positive. All the channel values in all the pixels are initialized
with zero.)""";
            // Source: drake/systems/sensors/image.h
            const char* doc_3args =
R"""(Image size and initial value constructor. Specifies a width, height
and an initial value for all the channels in all the pixels. The width
and height can be either both zero or both strictly positive.)""";
          } ctor;
          // Symbol: drake::systems::sensors::Image::NonTypeTemplateParameter
          struct /* NonTypeTemplateParameter */ {
            // Source: drake/systems/sensors/image.h
            const char* doc =
R"""(This is used by generic helpers such as drake∷Value to deduce a
non-type template argument.)""";
          } NonTypeTemplateParameter;
          // Symbol: drake::systems::sensors::Image::T
          struct /* T */ {
            // Source: drake/systems/sensors/image.h
            const char* doc = R"""(The data type for a channel.)""";
          } T;
          // Symbol: drake::systems::sensors::Image::Traits
          struct /* Traits */ {
            // Source: drake/systems/sensors/image.h
            const char* doc =
R"""(An alias for ImageTraits that contains the data type for a channel,
the number of channels and the pixel format in it.)""";
          } Traits;
          // Symbol: drake::systems::sensors::Image::at
          struct /* at */ {
            // Source: drake/systems/sensors/image.h
            const char* doc_2args_x_y_nonconst =
R"""(Access to the pixel located at (x, y) in image coordinate system where
x is the variable for horizontal direction and y is one for vertical
direction. To access to the each channel value in the pixel (x, y),
you can do:

ImageRgbaU8 image(640, 480, 255); uint8_t red = image.at(x, y)[0];
uint8_t green = image.at(x, y)[1]; uint8_t blue = image.at(x, y)[2];
uint8_t alpha = image.at(x, y)[3];)""";
            // Source: drake/systems/sensors/image.h
            const char* doc_2args_x_y_const =
R"""(Const version of at() method. See the document for the non-const
version for the detail.)""";
          } at;
          // Symbol: drake::systems::sensors::Image::height
          struct /* height */ {
            // Source: drake/systems/sensors/image.h
            const char* doc =
R"""(Returns the size of height for the image)""";
          } height;
          // Symbol: drake::systems::sensors::Image::resize
          struct /* resize */ {
            // Source: drake/systems/sensors/image.h
            const char* doc =
R"""(Changes the sizes of the width and height for the image. The new width
and height can be either both zero or both strictly positive. All the
values in the pixels become zero after resize.)""";
          } resize;
          // Symbol: drake::systems::sensors::Image::size
          struct /* size */ {
            // Source: drake/systems/sensors/image.h
            const char* doc =
R"""(Returns the result of the number of pixels in a image by the number of
channels in a pixel)""";
          } size;
          // Symbol: drake::systems::sensors::Image::width
          struct /* width */ {
            // Source: drake/systems/sensors/image.h
            const char* doc = R"""(Returns the size of width for the image)""";
          } width;
        } Image;
        // Symbol: drake::systems::sensors::ImageAny
        struct /* ImageAny */ {
          // Source: drake/systems/sensors/image.h
          const char* doc =
R"""(A sum type of all built-in images types.)""";
        } ImageAny;
        // Symbol: drake::systems::sensors::ImageBgr8U
        struct /* ImageBgr8U */ {
          // Source: drake/systems/sensors/image.h
          const char* doc =
R"""(The type for BGR image where the each channel has the type of uint8_t.)""";
        } ImageBgr8U;
        // Symbol: drake::systems::sensors::ImageBgra8U
        struct /* ImageBgra8U */ {
          // Source: drake/systems/sensors/image.h
          const char* doc =
R"""(The type for BGRA image where the each channel has the type of
uint8_t.)""";
        } ImageBgra8U;
        // Symbol: drake::systems::sensors::ImageDepth16U
        struct /* ImageDepth16U */ {
          // Source: drake/systems/sensors/image.h
          const char* doc =
R"""(The type for depth image where the channel has the type of uint16_t.)""";
        } ImageDepth16U;
        // Symbol: drake::systems::sensors::ImageDepth32F
        struct /* ImageDepth32F */ {
          // Source: drake/systems/sensors/image.h
          const char* doc =
R"""(The type for depth image where the channel has the type of float.)""";
        } ImageDepth32F;
        // Symbol: drake::systems::sensors::ImageFileFormat
        struct /* ImageFileFormat */ {
          // Source: drake/systems/sensors/image_file_format.h
          const char* doc = R"""(The image file formats known to Drake.)""";
          // Symbol: drake::systems::sensors::ImageFileFormat::kJpeg
          struct /* kJpeg */ {
            // Source: drake/systems/sensors/image_file_format.h
            const char* doc = R"""(mime-type: image/jpeg.)""";
          } kJpeg;
          // Symbol: drake::systems::sensors::ImageFileFormat::kPng
          struct /* kPng */ {
            // Source: drake/systems/sensors/image_file_format.h
            const char* doc = R"""(mime-type: image/png.)""";
          } kPng;
          // Symbol: drake::systems::sensors::ImageFileFormat::kTiff
          struct /* kTiff */ {
            // Source: drake/systems/sensors/image_file_format.h
            const char* doc = R"""(mime-type: image/tiff.)""";
          } kTiff;
        } ImageFileFormat;
        // Symbol: drake::systems::sensors::ImageGrey8U
        struct /* ImageGrey8U */ {
          // Source: drake/systems/sensors/image.h
          const char* doc =
R"""(The type for greyscale image where the channel has the type of
uint8_t.)""";
        } ImageGrey8U;
        // Symbol: drake::systems::sensors::ImageIo
        struct /* ImageIo */ {
          // Source: drake/systems/sensors/image_io.h
          const char* doc =
R"""(Utility functions for reading and writing images, from/to either files
or memory buffers.

The only file formats supported are JPEG, PNG, and TIFF.

The only format that supports floating-point scalars (e.g.,
ImageDepth32F) is TIFF. Trying to load or save a floating-point image
from/to a PNG or JPEG file will throw an exception.)""";
          // Symbol: drake::systems::sensors::ImageIo::ByteSpan
          struct /* ByteSpan */ {
            // Source: drake/systems/sensors/image_io.h
            const char* doc =
R"""(When loading from memory, this struct denotes a span of raw bytes as
input.)""";
            // Symbol: drake::systems::sensors::ImageIo::ByteSpan::data
            struct /* data */ {
              // Source: drake/systems/sensors/image_io.h
              const char* doc = R"""(Pointer to the first byte of the span.)""";
            } data;
            // Symbol: drake::systems::sensors::ImageIo::ByteSpan::size
            struct /* size */ {
              // Source: drake/systems/sensors/image_io.h
              const char* doc = R"""(Total number of bytes in the span.)""";
            } size;
          } ByteSpan;
          // Symbol: drake::systems::sensors::ImageIo::ImageIo
          struct /* ctor */ {
            // Source: drake/systems/sensors/image_io.h
            const char* doc = R"""(Default constructor.)""";
          } ctor;
          // Symbol: drake::systems::sensors::ImageIo::Load
          struct /* Load */ {
            // Source: drake/systems/sensors/image_io.h
            const char* doc_2args_path_format =
R"""(Loads and returns an image from disk.

Parameter ``format``:
    (Optionally) establishes the required image file format. When set,
    images that are a different file format will throw an exception.
    When not set, the filename extension has no bearing on the result;
    only the actual file contents determine the file format.

Raises:
    RuntimeError for any kind of error loading the image file.)""";
            // Source: drake/systems/sensors/image_io.h
            const char* doc_2args_buffer_format =
R"""(Loads and returns an image from a memory buffer.

Parameter ``format``:
    (Optionally) establishes the required image file format. When set,
    images that are a different file format will throw an exception.

Raises:
    RuntimeError for any kind of error loading the image data.)""";
          } Load;
          // Symbol: drake::systems::sensors::ImageIo::LoadMetadata
          struct /* LoadMetadata */ {
            // Source: drake/systems/sensors/image_io.h
            const char* doc_1args_path =
R"""(Returns the metadata of the given image file, or nullopt if the
metadata cannot be determined or is unsupported. The filename
extension has no bearing on the result; only the actual file contents
determine the file format.)""";
            // Source: drake/systems/sensors/image_io.h
            const char* doc_1args_buffer =
R"""(Returns the metadata of the given image buffer, or nullopt if the
metadata cannot be determined or is unsupported.)""";
          } LoadMetadata;
          // Symbol: drake::systems::sensors::ImageIo::Metadata
          struct /* Metadata */ {
            // Source: drake/systems/sensors/image_io.h
            const char* doc =
R"""(Some characteristics of an image file. Note that Drake's Image<> class
can only express ``depth == 1``.)""";
            // Symbol: drake::systems::sensors::ImageIo::Metadata::Serialize
            struct /* Serialize */ {
              // Source: drake/systems/sensors/image_io.h
              const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
            } Serialize;
            // Symbol: drake::systems::sensors::ImageIo::Metadata::channels
            struct /* channels */ {
              // Source: drake/systems/sensors/image_io.h
              const char* doc = R"""()""";
            } channels;
            // Symbol: drake::systems::sensors::ImageIo::Metadata::depth
            struct /* depth */ {
              // Source: drake/systems/sensors/image_io.h
              const char* doc = R"""()""";
            } depth;
            // Symbol: drake::systems::sensors::ImageIo::Metadata::format
            struct /* format */ {
              // Source: drake/systems/sensors/image_io.h
              const char* doc = R"""()""";
            } format;
            // Symbol: drake::systems::sensors::ImageIo::Metadata::height
            struct /* height */ {
              // Source: drake/systems/sensors/image_io.h
              const char* doc = R"""()""";
            } height;
            // Symbol: drake::systems::sensors::ImageIo::Metadata::scalar
            struct /* scalar */ {
              // Source: drake/systems/sensors/image_io.h
              const char* doc = R"""()""";
            } scalar;
            // Symbol: drake::systems::sensors::ImageIo::Metadata::width
            struct /* width */ {
              // Source: drake/systems/sensors/image_io.h
              const char* doc = R"""()""";
            } width;
            auto Serialize__fields() const {
              return std::array{
                std::make_pair("channels", channels.doc),
                std::make_pair("depth", depth.doc),
                std::make_pair("format", format.doc),
                std::make_pair("height", height.doc),
                std::make_pair("scalar", scalar.doc),
                std::make_pair("width", width.doc),
              };
            }
          } Metadata;
          // Symbol: drake::systems::sensors::ImageIo::Save
          struct /* Save */ {
            // Source: drake/systems/sensors/image_io.h
            const char* doc_3args =
R"""(Saves an image to disk.

Parameter ``format``:
    (Optionally) chooses the image file format. When not set, the
    filename extension will determine the format and the extension
    must be a supported choice (i.e., ``.jpg``, `.jpeg`, ``.png``,
    `.tif`, or ``.tiff``).

Raises:
    RuntimeError for any kind of error saving the image file.)""";
            // Source: drake/systems/sensors/image_io.h
            const char* doc_2args =
R"""(Saves an image to a new memory buffer, returning the buffer.

Raises:
    RuntimeError for any kind of error saving the image data.)""";
          } Save;
        } ImageIo;
        // Symbol: drake::systems::sensors::ImageLabel16I
        struct /* ImageLabel16I */ {
          // Source: drake/systems/sensors/image.h
          const char* doc =
R"""(The type for label image where the channel has the type of int16_t.)""";
        } ImageLabel16I;
        // Symbol: drake::systems::sensors::ImageRgb8U
        struct /* ImageRgb8U */ {
          // Source: drake/systems/sensors/image.h
          const char* doc =
R"""(The type for RGB image where the each channel has the type of uint8_t.)""";
        } ImageRgb8U;
        // Symbol: drake::systems::sensors::ImageRgba8U
        struct /* ImageRgba8U */ {
          // Source: drake/systems/sensors/image.h
          const char* doc =
R"""(The type for RGBA image where the each channel has the type of
uint8_t.)""";
        } ImageRgba8U;
        // Symbol: drake::systems::sensors::ImageToLcmImageArrayT
        struct /* ImageToLcmImageArrayT */ {
          // Source: drake/systems/sensors/image_to_lcm_image_array_t.h
          const char* doc =
R"""(An ImageToLcmImageArrayT takes as input an ImageRgba8U, ImageDepth32F
and ImageLabel16I. This system outputs an AbstractValue containing a
``Value<lcmt_image_array>`` LCM message that defines an array of
images (lcmt_image). This message can then be sent to other processes
that sbscribe it using LcmPublisherSystem. Note that you should NOT
assume any particular order of those images stored in
lcmt_image_array, instead check the semantic of those images with
lcmt_image∷pixel_format before using them.

.. pydrake_system::

    name: ImageToLcmImageArrayT
    input_ports:
    - (user assigned port name)
    - ...
    - (user assigned port name)
    output_ports:
    - y0

Note:
    The output message's header field ``seq`` is always zero.)""";
          // Symbol: drake::systems::sensors::ImageToLcmImageArrayT::DeclareImageInputPort
          struct /* DeclareImageInputPort */ {
            // Source: drake/systems/sensors/image_to_lcm_image_array_t.h
            const char* doc = R"""()""";
          } DeclareImageInputPort;
          // Symbol: drake::systems::sensors::ImageToLcmImageArrayT::ImageToLcmImageArrayT
          struct /* ctor */ {
            // Source: drake/systems/sensors/image_to_lcm_image_array_t.h
            const char* doc_1args =
R"""(Constructs an empty system with no input ports. After construction,
use DeclareImageInputPort() to add inputs.)""";
            // Source: drake/systems/sensors/image_to_lcm_image_array_t.h
            const char* doc_4args =
R"""(An ImageToLcmImageArrayT constructor. Declares three input ports --
one color image, one depth image, and one label image.

Parameter ``color_frame_name``:
    The frame name used for color image.

Parameter ``depth_frame_name``:
    The frame name used for depth image.

Parameter ``label_frame_name``:
    The frame name used for label image.

Parameter ``do_compress``:
    When true, zlib compression will be performed. The default is
    false.)""";
          } ctor;
          // Symbol: drake::systems::sensors::ImageToLcmImageArrayT::color_image_input_port
          struct /* color_image_input_port */ {
            // Source: drake/systems/sensors/image_to_lcm_image_array_t.h
            const char* doc =
R"""(Returns the input port containing a color image. Note: Only valid if
the color/depth/label constructor is used.)""";
          } color_image_input_port;
          // Symbol: drake::systems::sensors::ImageToLcmImageArrayT::depth_image_input_port
          struct /* depth_image_input_port */ {
            // Source: drake/systems/sensors/image_to_lcm_image_array_t.h
            const char* doc =
R"""(Returns the input port containing a depth image. Note: Only valid if
the color/depth/label constructor is used.)""";
          } depth_image_input_port;
          // Symbol: drake::systems::sensors::ImageToLcmImageArrayT::image_array_t_msg_output_port
          struct /* image_array_t_msg_output_port */ {
            // Source: drake/systems/sensors/image_to_lcm_image_array_t.h
            const char* doc =
R"""(Returns the abstract valued output port that contains a
``Value<lcmt_image_array>``.)""";
          } image_array_t_msg_output_port;
          // Symbol: drake::systems::sensors::ImageToLcmImageArrayT::label_image_input_port
          struct /* label_image_input_port */ {
            // Source: drake/systems/sensors/image_to_lcm_image_array_t.h
            const char* doc =
R"""(Returns the input port containing a label image. Note: Only valid if
the color/depth/label constructor is used.)""";
          } label_image_input_port;
        } ImageToLcmImageArrayT;
        // Symbol: drake::systems::sensors::ImageTraits
        struct /* ImageTraits */ {
          // Source: drake/systems/sensors/pixel_types.h
          const char* doc = R"""()""";
          // Symbol: drake::systems::sensors::ImageTraits::ChannelType
          struct /* ChannelType */ {
            // Source: drake/systems/sensors/pixel_types.h
            const char* doc = R"""()""";
          } ChannelType;
        } ImageTraits;
        // Symbol: drake::systems::sensors::ImageWriter
        struct /* ImageWriter */ {
          // Source: drake/systems/sensors/image_writer.h
          const char* doc =
R"""(A system for periodically writing images to the file system. The
system also provides direct image writing via a forced publish event.
The system does not have a fixed set of input ports; the system can
have an arbitrary number of image input ports. Each input port is
independently configured with respect to:

- publish frequency,
- write location (directory) and image name,
- input image format (which, in turn, implies a file-system format),
- port name (which needs to be unique across all input ports in this system),
and
- context time at which output starts.

By design, this system is intended to work with RgbdCamera, but can
connect to any output port that provides images.

.. pydrake_system::

    name: ImageWriter
    input_ports:
    - (user assigned port name)
    - ...
    - (user assigned port name)

ImageWriter supports three specific types of images:

- ImageRgba8U - typically a color image written to disk as .png images.
- ImageDepth32F - typically a depth image, written to disk as .tiff images.
- ImageLabel16I - typically a label image, written to disk as .png images.
- ImageDepth16U - typically a depth image, written to disk as .png images.
- ImageGrey8U - typically a grey scale image, written to disk as .png images.

Input ports are added to an ImageWriter via DeclareImageInputPort().
See that function's documentation for elaboration on how to configure
image output. It is important to note, that every declared image input
port *must* be connected; otherwise, attempting to write an image from
that port, will cause an error in the system.

If the user intends to write images directly instead of periodically,
e.g., when running this system outside of Simulator∷AdvanceTo, a call
to ``ForcedPublish(system_context)`` will write all images from each
input port simultaneously to disk. Note that one can invoke a forced
publish on this system using the same context multiple times,
resulting in multiple write operations, with each operation
overwriting the same file(s).)""";
          // Symbol: drake::systems::sensors::ImageWriter::DeclareImageInputPort
          struct /* DeclareImageInputPort */ {
            // Source: drake/systems/sensors/image_writer.h
            const char* doc =
R"""(Declares and configures a new image input port. A port is configured
by providing:

- a unique port name,
- an output file format string,
- a publish period,
- a start time, and
- an image type.

Each port is evaluated independently, so that two ports on the same
ImageWriter can write images to different locations at different
frequencies, etc. If images are to be kept in sync (e.g., registered
color and depth images), they should be given the same period and
start time.

**Specifying the times at which images are written**

Given a *positive* publish period ``p``, images will be written at
times contained in the list of times: ``t = [0, 1⋅p, 2⋅p, ...]``. The
start time parameter determines what the *first* output time will be.
Given a "start time" value ``tₛ``, the frames will be written at: ``t
= tₛ + [0, 1⋅p, 2⋅p, ...]``.

**Specifying write location and output file names**

When writing image data to disk, the location and name of the output
files are controlled by a user-defined format string. The format
string should be compatible with ``fmt∷format()``. ImageWriter
provides several *named* format arguments that can be referenced in
the format string:

- ``port_name``   - The name of the port (see below).
- ``image_type``  - One of ``color``, `depth`, or ``label``, depending on the
image type requested.
- ``time_double`` - The time (in seconds) stored in the context at the
invocation of Publish(), represented as a double.
- ``time_usec``   - The time (in microseconds) stored in the context at the
invocation of Publish(), represented as a 64-bit integer.
- ``time_msec``   - The time (in milliseconds) stored in the context at the
invocation of Publish(), represented as an integer.
- ``count``       - The number of write operations that have occurred from
this port since system creation or the last invocation of
ResetAllImageCounts() (the first write operation would
get zero, the Nᵗʰ would get N - 1). This value increments
*every* time a write operation occurs.

File names can then be specified as shown in the following examples
(assuming the port was declared as a color image port, with a name of
"my_port", a period of 0.02 s (50 Hz), and a start time of 5 s.

- ``/home/user/images/{port_name}/{time_usec}`` creates a sequence like:
- ``/home/user/images/my_port/5000000.png``
- `/home/user/images/my_port/5020000.png`
- ``/home/user/images/my_port/5040000.png``
- ...
- `/home/user/images/{image_type}/{time_msec:05}` creates a sequence like:
- ``/home/user/images/color/05000.png``
- `/home/user/images/color/05020.png`
- ``/home/user/images/color/05040.png``
- ...
- `/home/user/{port_name}/my_image_{count:03}.txt` creates a sequence like:
- ``/home/user/my_port/my_image_000.txt.png``
- `/home/user/my_port/my_image_001.txt.png`
- ``/home/user/my_port/my_image_002.txt.png``
- ...

We call attention particularly to the following:

- Note the zero-padding arguments in the second and third examples. Making
use of zero-padding typically facilitates *other* processes.
- If the file name format does not end with an appropriate extension (e.g.,
``.png`` or ``.tiff``), the extension will be added.
- The directory specified in the format will be tested for validity
(does it exist, is it a directory, can the program write to it). The
full *file name* will *not* be validated. If it is invalid (e.g., too
long, invalid characters, bad format substitution), images will silently
not be created.
- The third example uses the count flag -- regardless of start time, the
first file written will always be zero, the second one, etc.
- The directory can *only* depend ``port_name`` and ``image_type``. It *cannot*
depend on values that change over time (e.g., ``time_double``, `count`,
etc.

Parameter ``port_name``:
    The name of the port (must be unique among all image ports). This
    string is available in the format string as ``port_name``.

Parameter ``file_name_format``:
    The ``fmt∷format()``-compatible string which defines the
    context-dependent file name to write the image to.

Parameter ``publish_period``:
    The period at which images read from this input port are written
    in calls to Publish().

Parameter ``start_time``:
    The minimum value for the context's time at which images will be
    written in calls to Publish().

Parameter ``pixel_type``:
    The representation of the per-pixel data (see PixelType). Must be
    one of {PixelType∷kRgba8U, PixelType∷kDepth32F,
    PixelType∷kLabel16I, PixelType∷kDepth16U, or PixelType∷kGrey8U}.

Raises:
    RuntimeError if (1) the directory encoded in the
    ``file_name_format`` is not "valid" (see documentation above for
    definition), (2) ``publish_period`` is not positive, (3)
    ``port_name`` is used by a previous input port, or (4)
    ``pixel_type`` is not supported.)""";
          } DeclareImageInputPort;
          // Symbol: drake::systems::sensors::ImageWriter::ImageWriter
          struct /* ctor */ {
            // Source: drake/systems/sensors/image_writer.h
            const char* doc =
R"""(Constructs default instance with no image ports.)""";
          } ctor;
          // Symbol: drake::systems::sensors::ImageWriter::ResetAllImageCounts
          struct /* ResetAllImageCounts */ {
            // Source: drake/systems/sensors/image_writer.h
            const char* doc = R"""()""";
          } ResetAllImageCounts;
        } ImageWriter;
        // Symbol: drake::systems::sensors::LcmImageArrayToImages
        struct /* LcmImageArrayToImages */ {
          // Source: drake/systems/sensors/lcm_image_array_to_images.h
          const char* doc =
R"""(An LcmImageArrayToImages takes as input an AbstractValue containing a
``Value<lcmt_image_array>`` LCM message that defines an array of
images (lcmt_image). The system has output ports for one color image
as an ImageRgba8U and one depth image as ImageDepth32F (intended to be
similar to the API of RgbdCamera, though without the label image
port).

.. pydrake_system::

    name: LcmImageArrayToImages
    input_ports:
    - image_array_t
    output_ports:
    - color_image
    - depth_image
    - label_image)""";
          // Symbol: drake::systems::sensors::LcmImageArrayToImages::LcmImageArrayToImages
          struct /* ctor */ {
            // Source: drake/systems/sensors/lcm_image_array_to_images.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::systems::sensors::LcmImageArrayToImages::color_image_output_port
          struct /* color_image_output_port */ {
            // Source: drake/systems/sensors/lcm_image_array_to_images.h
            const char* doc =
R"""(Returns the abstract valued output port that contains a RGBA image of
the type ImageRgba8U. The image will be empty if no color image was
received in the most recent message (so, for example, sending color
and depth in different messages will not produce useful results).)""";
          } color_image_output_port;
          // Symbol: drake::systems::sensors::LcmImageArrayToImages::depth_image_output_port
          struct /* depth_image_output_port */ {
            // Source: drake/systems/sensors/lcm_image_array_to_images.h
            const char* doc =
R"""(Returns the abstract valued output port that contains an
ImageDepth32F. The image will be empty if no color image was received
in the most recent message (so, for example, sending color and depth
in different messages will not produce useful results).)""";
          } depth_image_output_port;
          // Symbol: drake::systems::sensors::LcmImageArrayToImages::image_array_t_input_port
          struct /* image_array_t_input_port */ {
            // Source: drake/systems/sensors/lcm_image_array_to_images.h
            const char* doc =
R"""(Returns the abstract valued input port that expects a
``Value<lcmt_image_array>``.)""";
          } image_array_t_input_port;
          // Symbol: drake::systems::sensors::LcmImageArrayToImages::label_image_output_port
          struct /* label_image_output_port */ {
            // Source: drake/systems/sensors/lcm_image_array_to_images.h
            const char* doc =
R"""(Returns the abstract valued output port that contains an
ImageLabel16I. The image will be empty if no label image was received
in the most recent message (so, for example, sending color and label
in different messages will not produce useful results).)""";
          } label_image_output_port;
        } LcmImageArrayToImages;
        // Symbol: drake::systems::sensors::LcmImageTraits
        struct /* LcmImageTraits */ {
          // Source: drake/systems/sensors/lcm_image_traits.h
          const char* doc = R"""()""";
        } LcmImageTraits;
        // Symbol: drake::systems::sensors::LcmPixelTraits
        struct /* LcmPixelTraits */ {
          // Source: drake/systems/sensors/lcm_image_traits.h
          const char* doc = R"""()""";
        } LcmPixelTraits;
        // Symbol: drake::systems::sensors::PixelFormat
        struct /* PixelFormat */ {
          // Source: drake/systems/sensors/pixel_types.h
          const char* doc =
R"""(The enum class to be used to express semantic meaning of pixels. This
also expresses the order of channels in a pixel if the pixel has
multiple channels.)""";
          // Symbol: drake::systems::sensors::PixelFormat::kBgr
          struct /* kBgr */ {
            // Source: drake/systems/sensors/pixel_types.h
            const char* doc =
R"""(The pixel format used for all the BGR images.)""";
          } kBgr;
          // Symbol: drake::systems::sensors::PixelFormat::kBgra
          struct /* kBgra */ {
            // Source: drake/systems/sensors/pixel_types.h
            const char* doc =
R"""(The pixel format used for all the BGRA images.)""";
          } kBgra;
          // Symbol: drake::systems::sensors::PixelFormat::kDepth
          struct /* kDepth */ {
            // Source: drake/systems/sensors/pixel_types.h
            const char* doc =
R"""(The pixel format used for all the depth images.)""";
          } kDepth;
          // Symbol: drake::systems::sensors::PixelFormat::kGrey
          struct /* kGrey */ {
            // Source: drake/systems/sensors/pixel_types.h
            const char* doc =
R"""(The pixel format used for all the greyscale images.)""";
          } kGrey;
          // Symbol: drake::systems::sensors::PixelFormat::kLabel
          struct /* kLabel */ {
            // Source: drake/systems/sensors/pixel_types.h
            const char* doc =
R"""(The pixel format used for all the labe images.)""";
          } kLabel;
          // Symbol: drake::systems::sensors::PixelFormat::kRgb
          struct /* kRgb */ {
            // Source: drake/systems/sensors/pixel_types.h
            const char* doc =
R"""(The pixel format used for all the RGB images.)""";
          } kRgb;
          // Symbol: drake::systems::sensors::PixelFormat::kRgba
          struct /* kRgba */ {
            // Source: drake/systems/sensors/pixel_types.h
            const char* doc =
R"""(The pixel format used for all the RGBA images.)""";
          } kRgba;
        } PixelFormat;
        // Symbol: drake::systems::sensors::PixelScalar
        struct /* PixelScalar */ {
          // Source: drake/systems/sensors/pixel_types.h
          const char* doc =
R"""(The enum class to be used to express channel type.)""";
          // Symbol: drake::systems::sensors::PixelScalar::k16I
          struct /* k16I */ {
            // Source: drake/systems/sensors/pixel_types.h
            const char* doc = R"""(int16_t)""";
          } k16I;
          // Symbol: drake::systems::sensors::PixelScalar::k16U
          struct /* k16U */ {
            // Source: drake/systems/sensors/pixel_types.h
            const char* doc = R"""(uint16_t)""";
          } k16U;
          // Symbol: drake::systems::sensors::PixelScalar::k32F
          struct /* k32F */ {
            // Source: drake/systems/sensors/pixel_types.h
            const char* doc = R"""(float (32-bit))""";
          } k32F;
          // Symbol: drake::systems::sensors::PixelScalar::k8U
          struct /* k8U */ {
            // Source: drake/systems/sensors/pixel_types.h
            const char* doc = R"""(uint8_t)""";
          } k8U;
        } PixelScalar;
        // Symbol: drake::systems::sensors::PixelType
        struct /* PixelType */ {
          // Source: drake/systems/sensors/pixel_types.h
          const char* doc =
R"""(The enum class to be used for describing pixel type in Image class.
The naming rule for the enum members is: k + (pixel format) + (bit per
a channel) + (data type for channels). For the type for channels, one
of the following capital letters is used.

- I: int
- U: unsigned int
- F: float)""";
          // Symbol: drake::systems::sensors::PixelType::kBgr8U
          struct /* kBgr8U */ {
            // Source: drake/systems/sensors/pixel_types.h
            const char* doc = R"""(The pixel format used by ImageBgr8U.)""";
          } kBgr8U;
          // Symbol: drake::systems::sensors::PixelType::kBgra8U
          struct /* kBgra8U */ {
            // Source: drake/systems/sensors/pixel_types.h
            const char* doc = R"""(The pixel format used by ImageBgra8U.)""";
          } kBgra8U;
          // Symbol: drake::systems::sensors::PixelType::kDepth16U
          struct /* kDepth16U */ {
            // Source: drake/systems/sensors/pixel_types.h
            const char* doc = R"""(The pixel format used by ImageDepth16U.)""";
          } kDepth16U;
          // Symbol: drake::systems::sensors::PixelType::kDepth32F
          struct /* kDepth32F */ {
            // Source: drake/systems/sensors/pixel_types.h
            const char* doc = R"""(The pixel format used by ImageDepth32F.)""";
          } kDepth32F;
          // Symbol: drake::systems::sensors::PixelType::kGrey8U
          struct /* kGrey8U */ {
            // Source: drake/systems/sensors/pixel_types.h
            const char* doc = R"""(The pixel format used by ImageGrey8U.)""";
          } kGrey8U;
          // Symbol: drake::systems::sensors::PixelType::kLabel16I
          struct /* kLabel16I */ {
            // Source: drake/systems/sensors/pixel_types.h
            const char* doc = R"""(The pixel format used by ImageLabel16I.)""";
          } kLabel16I;
          // Symbol: drake::systems::sensors::PixelType::kRgb8U
          struct /* kRgb8U */ {
            // Source: drake/systems/sensors/pixel_types.h
            const char* doc = R"""(The pixel format used by ImageRgb8U.)""";
          } kRgb8U;
          // Symbol: drake::systems::sensors::PixelType::kRgba8U
          struct /* kRgba8U */ {
            // Source: drake/systems/sensors/pixel_types.h
            const char* doc = R"""(The pixel format used by ImageRgba8U.)""";
          } kRgba8U;
        } PixelType;
        // Symbol: drake::systems::sensors::RgbdSensor
        struct /* RgbdSensor */ {
          // Source: drake/systems/sensors/rgbd_sensor.h
          const char* doc =
R"""(A meta-sensor that houses RGB, depth, and label cameras, producing
their corresponding images based on the contents of the
geometry∷SceneGraph.

.. pydrake_system::

    name: RgbdSensor
    input_ports:
    - geometry_query
    output_ports:
    - color_image
    - depth_image_32f
    - depth_image_16u
    - label_image
    - body_pose_in_world
    - image_time

This system models a continuous sensor, where the output ports reflect
the instantaneous images observed by the sensor. In contrast, a
discrete (sample and hold) sensor model might be a more suitable match
for a real-world camera; for that case, see RgbdSensorDiscrete or
RgbdSensorAsync.

The following text uses terminology and conventions from CameraInfo.
Please review its documentation.

This class uses the following frames:

- W - world frame
- C - color camera frame, used for both color and label cameras to guarantee
perfect registration between color and label images.
- D - depth camera frame
- B - sensor body frame. Approximately, the frame of the "physical" sensor
that contains the color, depth, and label cameras. The contained cameras
are rigidly fixed to B and X_WB is what is used to pose the sensor in the
world (or, alternatively, X_PB where P is some parent frame for which X_WP
is known).

By default, frames B, C, and D are coincident and aligned. These can
be changed using the ``camera_poses`` constructor parameter. Frames C
and D are always rigidly affixed to the sensor body frame B. As
documented in the camera_axes_in_image "CameraInfo documentation", the
color and depth cameras "look" in the positive Cz and Dz directions,
respectively with the positive Cy and Dy directions pointing to the
bottom of the image. If R_BC and R_BD are the identity rotation, we
can apply the same reasoning to the body frame: look in the +Bz
direction with the +By direction pointing down in the image. Only if
the depth or color frames are re-oriented relative to the body does
further reasoning need to be applied.

Output port image formats:

- color_image: Four channels, each channel uint8_t, in the following order:
red, green, blue, and alpha.

- depth_image_32f: One channel, float, representing the Z value in
``D`` in *meters*. The values 0 and infinity are reserved for out-of-range
depth returns (too close or too far, respectively, as defined by
geometry∷render∷DepthRenderCamera "DepthRenderCamera").

- depth_image_16u: One channel, uint16_t, representing the Z value in
``D`` in *millimeters*. The values 0 and 65535 are reserved for out-of-range
depth returns (too close or too far, respectively, as defined by
geometry∷render∷DepthRenderCamera "DepthRenderCamera").
Additionally, 65535 will also be returned if the
depth measurement exceeds the representation range of uint16_t. Thus, the
maximum valid depth return is 65534mm.

- label_image: One channel, int16_t, whose value is a unique
geometry∷render∷RenderLabel "RenderLabel" value aligned with the
color camera frame. See geometry∷render∷RenderLabel "RenderLabel"
for discussion of interpreting rendered labels.

Note:
    These depth sensor measurements differ from those of range data
    used by laser range finders (like DepthSensor), where the depth
    value represents the distance from the sensor origin to the
    object's surface.)""";
          // Symbol: drake::systems::sensors::RgbdSensor::GetColorRenderCamera
          struct /* GetColorRenderCamera */ {
            // Source: drake/systems/sensors/rgbd_sensor.h
            const char* doc =
R"""(Returns the context dependent render camera for color/label
renderings.)""";
          } GetColorRenderCamera;
          // Symbol: drake::systems::sensors::RgbdSensor::GetDepthRenderCamera
          struct /* GetDepthRenderCamera */ {
            // Source: drake/systems/sensors/rgbd_sensor.h
            const char* doc =
R"""(Returns the context dependent render camera for depth renderings.)""";
          } GetDepthRenderCamera;
          // Symbol: drake::systems::sensors::RgbdSensor::GetParentFrameId
          struct /* GetParentFrameId */ {
            // Source: drake/systems/sensors/rgbd_sensor.h
            const char* doc =
R"""(Returns the context dependent id of the frame to which the body is
affixed.)""";
          } GetParentFrameId;
          // Symbol: drake::systems::sensors::RgbdSensor::GetX_PB
          struct /* GetX_PB */ {
            // Source: drake/systems/sensors/rgbd_sensor.h
            const char* doc = R"""(Returns the context dependent ``X_PB``.)""";
          } GetX_PB;
          // Symbol: drake::systems::sensors::RgbdSensor::RgbdSensor
          struct /* ctor */ {
            // Source: drake/systems/sensors/rgbd_sensor.h
            const char* doc_individual_intrinsics =
R"""(Constructs an RgbdSensor with fully specified render camera models for
both color/label and depth cameras.)""";
            // Source: drake/systems/sensors/rgbd_sensor.h
            const char* doc_combined_intrinsics =
R"""(Constructs an RgbdSensor using a fully specified depth render camera,
inferring the color settings based on depth. The color camera in
inferred from the ``depth_camera``; it shares the same
geometry∷render∷RenderCameraCore and is configured to show the window
based on the value of ``show_color_window``.)""";
          } ctor;
          // Symbol: drake::systems::sensors::RgbdSensor::SetColorRenderCamera
          struct /* SetColorRenderCamera */ {
            // Source: drake/systems/sensors/rgbd_sensor.h
            const char* doc =
R"""(Sets the render camera for color/label renderings, as stored as
parameters in ``context``.)""";
          } SetColorRenderCamera;
          // Symbol: drake::systems::sensors::RgbdSensor::SetDepthRenderCamera
          struct /* SetDepthRenderCamera */ {
            // Source: drake/systems/sensors/rgbd_sensor.h
            const char* doc =
R"""(Sets the render camera for depth renderings, as stored as parameters
in ``context``.)""";
          } SetDepthRenderCamera;
          // Symbol: drake::systems::sensors::RgbdSensor::SetParentFrameId
          struct /* SetParentFrameId */ {
            // Source: drake/systems/sensors/rgbd_sensor.h
            const char* doc =
R"""(Sets the id of the frame to which the body is affixed, as stored as
parameters in ``context``.)""";
          } SetParentFrameId;
          // Symbol: drake::systems::sensors::RgbdSensor::SetX_PB
          struct /* SetX_PB */ {
            // Source: drake/systems/sensors/rgbd_sensor.h
            const char* doc =
R"""(Sets the ``X_PB``, as stored as parameters in ``context``.)""";
          } SetX_PB;
          // Symbol: drake::systems::sensors::RgbdSensor::body_pose_in_world_output_port
          struct /* body_pose_in_world_output_port */ {
            // Source: drake/systems/sensors/rgbd_sensor.h
            const char* doc =
R"""(Returns the abstract-valued output port (containing a RigidTransform)
which reports the pose of the body in the world frame (X_WB).)""";
          } body_pose_in_world_output_port;
          // Symbol: drake::systems::sensors::RgbdSensor::color_image_output_port
          struct /* color_image_output_port */ {
            // Source: drake/systems/sensors/rgbd_sensor.h
            const char* doc =
R"""(Returns the abstract-valued output port that contains an ImageRgba8U.)""";
          } color_image_output_port;
          // Symbol: drake::systems::sensors::RgbdSensor::default_X_PB
          struct /* default_X_PB */ {
            // Source: drake/systems/sensors/rgbd_sensor.h
            const char* doc = R"""(Returns the default ``X_PB``.)""";
          } default_X_PB;
          // Symbol: drake::systems::sensors::RgbdSensor::default_color_render_camera
          struct /* default_color_render_camera */ {
            // Source: drake/systems/sensors/rgbd_sensor.h
            const char* doc =
R"""(Returns the default render camera for color/label renderings.)""";
          } default_color_render_camera;
          // Symbol: drake::systems::sensors::RgbdSensor::default_depth_render_camera
          struct /* default_depth_render_camera */ {
            // Source: drake/systems/sensors/rgbd_sensor.h
            const char* doc =
R"""(Returns the default render camera for depth renderings.)""";
          } default_depth_render_camera;
          // Symbol: drake::systems::sensors::RgbdSensor::default_parent_frame_id
          struct /* default_parent_frame_id */ {
            // Source: drake/systems/sensors/rgbd_sensor.h
            const char* doc =
R"""(Returns the default id of the frame to which the body is affixed.)""";
          } default_parent_frame_id;
          // Symbol: drake::systems::sensors::RgbdSensor::depth_image_16U_output_port
          struct /* depth_image_16U_output_port */ {
            // Source: drake/systems/sensors/rgbd_sensor.h
            const char* doc =
R"""(Returns the abstract-valued output port that contains an
ImageDepth16U.)""";
          } depth_image_16U_output_port;
          // Symbol: drake::systems::sensors::RgbdSensor::depth_image_32F_output_port
          struct /* depth_image_32F_output_port */ {
            // Source: drake/systems/sensors/rgbd_sensor.h
            const char* doc =
R"""(Returns the abstract-valued output port that contains an
ImageDepth32F.)""";
          } depth_image_32F_output_port;
          // Symbol: drake::systems::sensors::RgbdSensor::image_time_output_port
          struct /* image_time_output_port */ {
            // Source: drake/systems/sensors/rgbd_sensor.h
            const char* doc =
R"""(Returns the vector-valued output port (with size == 1) that reports
the current simulation time, in seconds. This is provided for
consistency with RgbdSensorDiscrete and RgbdSensorAsync (where the
image time is not always the current time).)""";
          } image_time_output_port;
          // Symbol: drake::systems::sensors::RgbdSensor::label_image_output_port
          struct /* label_image_output_port */ {
            // Source: drake/systems/sensors/rgbd_sensor.h
            const char* doc =
R"""(Returns the abstract-valued output port that contains an
ImageLabel16I.)""";
          } label_image_output_port;
          // Symbol: drake::systems::sensors::RgbdSensor::query_object_input_port
          struct /* query_object_input_port */ {
            // Source: drake/systems/sensors/rgbd_sensor.h
            const char* doc =
R"""(Returns the geometry∷QueryObject<double>-valued input port.)""";
          } query_object_input_port;
          // Symbol: drake::systems::sensors::RgbdSensor::set_default_X_PB
          struct /* set_default_X_PB */ {
            // Source: drake/systems/sensors/rgbd_sensor.h
            const char* doc = R"""(Sets the default ``X_PB``.)""";
          } set_default_X_PB;
          // Symbol: drake::systems::sensors::RgbdSensor::set_default_color_render_camera
          struct /* set_default_color_render_camera */ {
            // Source: drake/systems/sensors/rgbd_sensor.h
            const char* doc =
R"""(Sets the default render camera for color/label renderings.)""";
          } set_default_color_render_camera;
          // Symbol: drake::systems::sensors::RgbdSensor::set_default_depth_render_camera
          struct /* set_default_depth_render_camera */ {
            // Source: drake/systems/sensors/rgbd_sensor.h
            const char* doc =
R"""(Sets the default render camera for depth renderings.)""";
          } set_default_depth_render_camera;
          // Symbol: drake::systems::sensors::RgbdSensor::set_default_parent_frame_id
          struct /* set_default_parent_frame_id */ {
            // Source: drake/systems/sensors/rgbd_sensor.h
            const char* doc =
R"""(Sets the default id of the frame to which the body is affixed.)""";
          } set_default_parent_frame_id;
        } RgbdSensor;
        // Symbol: drake::systems::sensors::RgbdSensorAsync
        struct /* RgbdSensorAsync */ {
          // Source: drake/systems/sensors/rgbd_sensor_async.h
          const char* doc =
R"""(A sensor similar to RgbdSensorDiscrete but the rendering occurs on a
background thread to offer improved performance.

Warning:
    This feature is considered to be **experimental** and may change
    or be removed at any time, without any deprecation notice ahead of
    time.

Warning:
    This system is intended for use only with the out-of-process glTF
    rendering engine (MakeRenderEngineGltfClient()). Other render
    engines (e.g., MakeRenderEngineVtk() or MakeRenderEngineGl()) are
    either not thread-safe or perform poorly when used here. We hope
    to add async support for those engines down the road (see #19437
    for details).

.. pydrake_system::

    name: RgbdSensorAsync
    input_ports:
    - geometry_query
    output_ports:
    - color_image (optional)
    - depth_image_32f (optional)
    - depth_image_16u (optional)
    - label_image (optional)
    - body_pose_in_world
    - image_time

This sensor works by capturing the i'th state of the world at time
``t_capture = capture_offset + i / fps`` and outputting the
corresponding rendered image at time ``t_output = t_capture +
output_delay``, i.e., ``t_output = capture_offset + i / fps +
output_delay``.

For example, with the following constructor settings: - capture_offset
= 0 ms - output_delay = 50 ms - fps = 5 Hz (i.e., 200 ms)

We have the following timing schedule:

| Time | Event | Current output images | | :----: | :-----: |
:----------------------: | | 0 ms | capture | empty (i.e., zero-sized)
| | 50 ms | output | the scene as of 0 ms | | 200 ms | capture | ^ | |
250 ms | output | the scene as of 200 ms | | 400 ms | capture | ^ | |
... | ... | ... |

The ``capture_offset`` is typically zero, but can be useful to stagger
multiple cameras in the same scene to trigger at different times if
desired (e.g., to space out rendering events for better performance).

The ``output_delay`` can be used to model delays of a physical camera
(e.g., for firmware processing on the device or transmitting the image
over a USB or network connection, etc.).

The image rendering between the capture event and the output event
happens on a background thread, allowing simulation time to continue
to advance during the rendering's ``output_delay``. This helps smooth
over the runtime latency associated with rendering.

See also RgbdSensorDiscrete for a simpler (unthreaded) discrete sensor
model, or RgbdSensor for a continuous model.

Note:
    There are some limits on what configuration can be changed after
    construction. It is not supported to change which cameras are
    present, since that would imply a change in what output ports are
    provided. However, it is possible to change the details of the
    camera configurations if they were provided at construction. It is
    not supported to change the timing variables (``fps``,
    `capture_offset`, ``output_delay``), since that would require a
    change in the event definitions.

Warning:
    As the moment, this implementation cannnot respond to changes to
    geometry shapes, textures, etc. after a simulation has started.
    The only thing it responds to are changes to geometry poses. If
    you change anything beyond poses, you must manually call
    Simulator∷Initialize() to reset things before resuming the
    simulation, or else you'll get an exception. This holds true for
    changes to both SceneGraph's model geometry and the copy of the
    geometry in a scene graph Context.)""";
          // Symbol: drake::systems::sensors::RgbdSensorAsync::GetColorRenderCamera
          struct /* GetColorRenderCamera */ {
            // Source: drake/systems/sensors/rgbd_sensor_async.h
            const char* doc =
R"""(Returns the context dependent render camera for color/label
renderings.)""";
          } GetColorRenderCamera;
          // Symbol: drake::systems::sensors::RgbdSensorAsync::GetDepthRenderCamera
          struct /* GetDepthRenderCamera */ {
            // Source: drake/systems/sensors/rgbd_sensor_async.h
            const char* doc =
R"""(Returns the context dependent render camera for depth renderings.)""";
          } GetDepthRenderCamera;
          // Symbol: drake::systems::sensors::RgbdSensorAsync::GetParentFrameId
          struct /* GetParentFrameId */ {
            // Source: drake/systems/sensors/rgbd_sensor_async.h
            const char* doc =
R"""(Returns the context dependent id of the frame to which the body is
affixed.)""";
          } GetParentFrameId;
          // Symbol: drake::systems::sensors::RgbdSensorAsync::GetX_PB
          struct /* GetX_PB */ {
            // Source: drake/systems/sensors/rgbd_sensor_async.h
            const char* doc = R"""(Returns the context dependent ``X_PB``.)""";
          } GetX_PB;
          // Symbol: drake::systems::sensors::RgbdSensorAsync::RgbdSensorAsync
          struct /* ctor */ {
            // Source: drake/systems/sensors/rgbd_sensor_async.h
            const char* doc =
R"""(Constructs a sensor with the given parameters. Refer to the class
overview documentation for additional exposition and examples of the
parameters.

Parameter ``scene_graph``:
    The SceneGraph to use for image rendering. This pointer is
    retained by the constructor so must remain valid for the lifetime
    of this object. After construction, this system's
    ``geometry_query`` input port must be connected to the relevant
    output port of the given ``scene_graph``.

Parameter ``parent_id``:
    The frame "P" to which the sensor body is affixed. See RgbdSensor
    for details.

Parameter ``X_PB``:
    The transform relating parent and sensor body. See RgbdSensor for
    details.

Parameter ``fps``:
    How often (frames per second) to sample the ``geometry_query``
    input and render the associated images. Must be strictly positive
    and finite.

Parameter ``capture_offset``:
    The time of the first sample of ``geometry_query`` input.
    Typically zero. Must be non-negative and finite.

Parameter ``output_delay``:
    How long after the ``geometry_query`` input sample the output
    ports should change to reflect the new rendered image(s). Must be
    strictly positive and strictly less than 1/fps.

Parameter ``color_camera``:
    The properties for the ``color_image`` output port. When nullopt,
    there will be no ``color_image`` output port. At least one of
    ``color_camera`` or ``depth_camera`` must be provided.

Parameter ``depth_camera``:
    The properties for the ``depth_image_32f`` and ``depth_image_16u``
    output ports. When nullopt, there will be no such output ports. At
    least one of ``color_camera`` or ``depth_camera`` must be
    provided.

Parameter ``render_label_image``:
    Whether to provide the ``label_image`` output port. May be set to
    true only when a ``color_camera`` has also been provided.)""";
          } ctor;
          // Symbol: drake::systems::sensors::RgbdSensorAsync::SetColorRenderCamera
          struct /* SetColorRenderCamera */ {
            // Source: drake/systems/sensors/rgbd_sensor_async.h
            const char* doc =
R"""(Sets the render camera for color/label renderings, as stored as
parameters in ``context``.

Raises:
    RuntimeError if no color camera was provided at construction.)""";
          } SetColorRenderCamera;
          // Symbol: drake::systems::sensors::RgbdSensorAsync::SetDepthRenderCamera
          struct /* SetDepthRenderCamera */ {
            // Source: drake/systems/sensors/rgbd_sensor_async.h
            const char* doc =
R"""(Sets the render camera for depth renderings, as stored as parameters
in ``context``.

Raises:
    RuntimeError if no depth camera was provided at construction.)""";
          } SetDepthRenderCamera;
          // Symbol: drake::systems::sensors::RgbdSensorAsync::SetParentFrameId
          struct /* SetParentFrameId */ {
            // Source: drake/systems/sensors/rgbd_sensor_async.h
            const char* doc =
R"""(Sets the id of the frame to which the body is affixed, as stored as
parameters in ``context``.)""";
          } SetParentFrameId;
          // Symbol: drake::systems::sensors::RgbdSensorAsync::SetX_PB
          struct /* SetX_PB */ {
            // Source: drake/systems/sensors/rgbd_sensor_async.h
            const char* doc =
R"""(Sets the ``X_PB``, as stored as parameters in ``context``.)""";
          } SetX_PB;
          // Symbol: drake::systems::sensors::RgbdSensorAsync::body_pose_in_world_output_port
          struct /* body_pose_in_world_output_port */ {
            // Source: drake/systems/sensors/rgbd_sensor_async.h
            const char* doc =
R"""(Returns the abstract-valued output port (containing a RigidTransform)
which reports the pose of the body in the world frame (X_WB).)""";
          } body_pose_in_world_output_port;
          // Symbol: drake::systems::sensors::RgbdSensorAsync::capture_offset
          struct /* capture_offset */ {
            // Source: drake/systems/sensors/rgbd_sensor_async.h
            const char* doc =
R"""(Returns the capture offset provided at construction.)""";
          } capture_offset;
          // Symbol: drake::systems::sensors::RgbdSensorAsync::color_image_output_port
          struct /* color_image_output_port */ {
            // Source: drake/systems/sensors/rgbd_sensor_async.h
            const char* doc =
R"""(Returns the abstract-valued output port that contains an ImageRgba8U.

Raises:
    RuntimeError when color output is not enabled.)""";
          } color_image_output_port;
          // Symbol: drake::systems::sensors::RgbdSensorAsync::default_X_PB
          struct /* default_X_PB */ {
            // Source: drake/systems/sensors/rgbd_sensor_async.h
            const char* doc = R"""(Returns the default ``X_PB``.)""";
          } default_X_PB;
          // Symbol: drake::systems::sensors::RgbdSensorAsync::default_color_render_camera
          struct /* default_color_render_camera */ {
            // Source: drake/systems/sensors/rgbd_sensor_async.h
            const char* doc =
R"""(Returns the default render camera for color renderings.)""";
          } default_color_render_camera;
          // Symbol: drake::systems::sensors::RgbdSensorAsync::default_depth_render_camera
          struct /* default_depth_render_camera */ {
            // Source: drake/systems/sensors/rgbd_sensor_async.h
            const char* doc =
R"""(Returns the default render camera for depth renderings.)""";
          } default_depth_render_camera;
          // Symbol: drake::systems::sensors::RgbdSensorAsync::default_parent_frame_id
          struct /* default_parent_frame_id */ {
            // Source: drake/systems/sensors/rgbd_sensor_async.h
            const char* doc =
R"""(Returns the default id of the frame to which the body is affixed.)""";
          } default_parent_frame_id;
          // Symbol: drake::systems::sensors::RgbdSensorAsync::depth_image_16U_output_port
          struct /* depth_image_16U_output_port */ {
            // Source: drake/systems/sensors/rgbd_sensor_async.h
            const char* doc =
R"""(Returns the abstract-valued output port that contains an
ImageDepth16U.

Raises:
    RuntimeError when depth output is not enabled.)""";
          } depth_image_16U_output_port;
          // Symbol: drake::systems::sensors::RgbdSensorAsync::depth_image_32F_output_port
          struct /* depth_image_32F_output_port */ {
            // Source: drake/systems/sensors/rgbd_sensor_async.h
            const char* doc =
R"""(Returns the abstract-valued output port that contains an
ImageDepth32F.

Raises:
    RuntimeError when depth output is not enabled.)""";
          } depth_image_32F_output_port;
          // Symbol: drake::systems::sensors::RgbdSensorAsync::fps
          struct /* fps */ {
            // Source: drake/systems/sensors/rgbd_sensor_async.h
            const char* doc =
R"""(Returns the frame rate provided at construction.)""";
          } fps;
          // Symbol: drake::systems::sensors::RgbdSensorAsync::image_time_output_port
          struct /* image_time_output_port */ {
            // Source: drake/systems/sensors/rgbd_sensor_async.h
            const char* doc =
R"""(Returns the vector-valued output port (with size == 1) which reports
the simulation time when the image outputs were captured, in seconds.
When there are no images available on the output ports (e.g., at the
beginning of a simulation), the value will be NaN.)""";
          } image_time_output_port;
          // Symbol: drake::systems::sensors::RgbdSensorAsync::label_image_output_port
          struct /* label_image_output_port */ {
            // Source: drake/systems/sensors/rgbd_sensor_async.h
            const char* doc =
R"""(Returns the abstract-valued output port that contains an
ImageLabel16I.

Raises:
    RuntimeError when label output is not enabled.)""";
          } label_image_output_port;
          // Symbol: drake::systems::sensors::RgbdSensorAsync::output_delay
          struct /* output_delay */ {
            // Source: drake/systems/sensors/rgbd_sensor_async.h
            const char* doc =
R"""(Returns the output delay provided at construction.)""";
          } output_delay;
          // Symbol: drake::systems::sensors::RgbdSensorAsync::set_default_X_PB
          struct /* set_default_X_PB */ {
            // Source: drake/systems/sensors/rgbd_sensor_async.h
            const char* doc = R"""(Sets the default ``X_PB``.)""";
          } set_default_X_PB;
          // Symbol: drake::systems::sensors::RgbdSensorAsync::set_default_color_render_camera
          struct /* set_default_color_render_camera */ {
            // Source: drake/systems/sensors/rgbd_sensor_async.h
            const char* doc =
R"""(Sets the default render camera for color/label renderings.

Raises:
    RuntimeError if no color camera was provided at construction.)""";
          } set_default_color_render_camera;
          // Symbol: drake::systems::sensors::RgbdSensorAsync::set_default_depth_render_camera
          struct /* set_default_depth_render_camera */ {
            // Source: drake/systems/sensors/rgbd_sensor_async.h
            const char* doc =
R"""(Sets the default render camera for depth renderings.

Raises:
    RuntimeError if no depth camera was provided at construction.)""";
          } set_default_depth_render_camera;
          // Symbol: drake::systems::sensors::RgbdSensorAsync::set_default_parent_frame_id
          struct /* set_default_parent_frame_id */ {
            // Source: drake/systems/sensors/rgbd_sensor_async.h
            const char* doc =
R"""(Sets the default id of the frame to which the body is affixed.)""";
          } set_default_parent_frame_id;
        } RgbdSensorAsync;
        // Symbol: drake::systems::sensors::RgbdSensorDiscrete
        struct /* RgbdSensorDiscrete */ {
          // Source: drake/systems/sensors/rgbd_sensor_discrete.h
          const char* doc =
R"""(Wraps a continuous RgbdSensor with a zero-order hold to create a
discrete sensor.

.. pydrake_system::

    name: RgbdSensorDiscrete
    input_ports:
    - geometry_query
    output_ports:
    - color_image
    - depth_image_32f
    - depth_image_16u
    - label_image
    - body_pose_in_world
    - image_time

See also RgbdSensorAsync for a slightly different discrete sensor
model.

Note:
    Be mindful that the discrete dynamics of a zero-order hold mean
    that all three image types (color, depth, label) are rendered at
    the given ``period``, even if nothing ends up using the images on
    the output ports; this might be computationally wasteful. If you
    only need color and depth, be sure to pass ``render_label_image =
    false`` to the constructor. If you only need one image type,
    eschew this system in favor of adding your own zero-order hold on
    just the one RgbdSensor output port that you need.)""";
          // Symbol: drake::systems::sensors::RgbdSensorDiscrete::RgbdSensorDiscrete
          struct /* ctor */ {
            // Source: drake/systems/sensors/rgbd_sensor_discrete.h
            const char* doc =
R"""(Constructs a diagram containing a (non-registered) RgbdSensor that
will update at a given rate.

Parameter ``sensor``:
    The continuous sensor used to generate periodic images.

Parameter ``period``:
    Update period (sec).

Parameter ``render_label_image``:
    If true, renders label image (which requires additional overhead).
    If false, ``label_image_output_port`` will raise an error if
    called.

Warning:
    a System (i.e., ``sensor``) may only be added to at most one
    Diagram (i.e., this ``RgbdSensorDiscrete``) so should not be
    re-used outside of the ``RgbdSensorDiscrete``.)""";
          } ctor;
          // Symbol: drake::systems::sensors::RgbdSensorDiscrete::body_pose_in_world_output_port
          struct /* body_pose_in_world_output_port */ {
            // Source: drake/systems/sensors/rgbd_sensor_discrete.h
            const char* doc =
R"""(See also:
    RgbdSensor∷body_pose_in_world_output_port().)""";
          } body_pose_in_world_output_port;
          // Symbol: drake::systems::sensors::RgbdSensorDiscrete::color_image_output_port
          struct /* color_image_output_port */ {
            // Source: drake/systems/sensors/rgbd_sensor_discrete.h
            const char* doc =
R"""(See also:
    RgbdSensor∷color_image_output_port().)""";
          } color_image_output_port;
          // Symbol: drake::systems::sensors::RgbdSensorDiscrete::depth_image_16U_output_port
          struct /* depth_image_16U_output_port */ {
            // Source: drake/systems/sensors/rgbd_sensor_discrete.h
            const char* doc =
R"""(See also:
    RgbdSensor∷depth_image_16U_output_port().)""";
          } depth_image_16U_output_port;
          // Symbol: drake::systems::sensors::RgbdSensorDiscrete::depth_image_32F_output_port
          struct /* depth_image_32F_output_port */ {
            // Source: drake/systems/sensors/rgbd_sensor_discrete.h
            const char* doc =
R"""(See also:
    RgbdSensor∷depth_image_32F_output_port().)""";
          } depth_image_32F_output_port;
          // Symbol: drake::systems::sensors::RgbdSensorDiscrete::image_time_output_port
          struct /* image_time_output_port */ {
            // Source: drake/systems/sensors/rgbd_sensor_discrete.h
            const char* doc =
R"""(Returns the vector-valued output port (with size == 1) which reports
the simulation time when the image outputs were captured, in seconds
(i.e., the time of the most recent zero-order hold discrete update).)""";
          } image_time_output_port;
          // Symbol: drake::systems::sensors::RgbdSensorDiscrete::label_image_output_port
          struct /* label_image_output_port */ {
            // Source: drake/systems/sensors/rgbd_sensor_discrete.h
            const char* doc =
R"""(See also:
    RgbdSensor∷label_image_output_port().)""";
          } label_image_output_port;
          // Symbol: drake::systems::sensors::RgbdSensorDiscrete::period
          struct /* period */ {
            // Source: drake/systems/sensors/rgbd_sensor_discrete.h
            const char* doc =
R"""(Returns the update period for the discrete camera.)""";
          } period;
          // Symbol: drake::systems::sensors::RgbdSensorDiscrete::query_object_input_port
          struct /* query_object_input_port */ {
            // Source: drake/systems/sensors/rgbd_sensor_discrete.h
            const char* doc =
R"""(See also:
    RgbdSensor∷query_object_input_port().)""";
          } query_object_input_port;
          // Symbol: drake::systems::sensors::RgbdSensorDiscrete::sensor
          struct /* sensor */ {
            // Source: drake/systems/sensors/rgbd_sensor_discrete.h
            const char* doc =
R"""(Returns a reference to the underlying RgbdSensor object.)""";
          } sensor;
        } RgbdSensorDiscrete;
        // Symbol: drake::systems::sensors::RotaryEncoders
        struct /* RotaryEncoders */ {
          // Source: drake/systems/sensors/rotary_encoders.h
          const char* doc =
R"""(Simple model to capture the quantization and calibration offset
effects of a rotary encoder. Consider combining this with a
ZeroOrderHold system to capture the sampled-data effects.

The inputs to this system are assumed to be in radians, and the
outputs of the system are also in radians.

.. pydrake_system::

    name: RotaryEncoders
    input_ports:
    - u0
    output_ports:
    - y0)""";
          // Symbol: drake::systems::sensors::RotaryEncoders::RotaryEncoders<T>
          struct /* ctor */ {
            // Source: drake/systems/sensors/rotary_encoders.h
            const char* doc_1args =
R"""(Quantization-only constructor. Specifies one ticks_per_revolution
count for every element of the input port.)""";
            // Source: drake/systems/sensors/rotary_encoders.h
            const char* doc_2args =
R"""(Selector-only constructor. Provides arguments to select particular
indices from the input signal to use in the output. Since
ticks_per_revolution is not being set, the outputs will NOT be
quantized.

Parameter ``input_port_size``:
    Dimension of the expected input signal

Parameter ``input_vector_indices``:
    List of indices)""";
            // Source: drake/systems/sensors/rotary_encoders.h
            const char* doc_3args = R"""(Quantization and Selector constructor.)""";
            // Source: drake/systems/sensors/rotary_encoders.h
            const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
          } ctor;
          // Symbol: drake::systems::sensors::RotaryEncoders::get_calibration_offsets
          struct /* get_calibration_offsets */ {
            // Source: drake/systems/sensors/rotary_encoders.h
            const char* doc =
R"""(Retrieve the calibration offset parameters.)""";
          } get_calibration_offsets;
          // Symbol: drake::systems::sensors::RotaryEncoders::set_calibration_offsets
          struct /* set_calibration_offsets */ {
            // Source: drake/systems/sensors/rotary_encoders.h
            const char* doc = R"""(Set the calibration offset parameters.)""";
          } set_calibration_offsets;
        } RotaryEncoders;
        // Symbol: drake::systems::sensors::SaveToPng
        struct /* SaveToPng */ {
          // Source: drake/systems/sensors/image_writer.h
          const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Writes the color (8-bit, RGBA) image data to disk.)""";
        } SaveToPng;
        // Symbol: drake::systems::sensors::SaveToTiff
        struct /* SaveToTiff */ {
          // Source: drake/systems/sensors/image_writer.h
          const char* doc =
R"""(Writes the depth (32-bit) image data to disk. A 32-bit depth image can
only be saved as TIFF (not PNG) because PNG files do not support
channels larger than 16-bits and its support for floating point values
is also limited at best. See ConvertDepth32FTo16U() for converting
32-bit to 16-bit to enable saving as PNG.)""";
        } SaveToTiff;
        // Symbol: drake::systems::sensors::to_string
        struct /* to_string */ {
          // Source: drake/systems/sensors/image_file_format.h
          const char* doc = R"""()""";
        } to_string;
      } sensors;
    } systems;
  } drake;
} pydrake_doc_systems_sensors;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
