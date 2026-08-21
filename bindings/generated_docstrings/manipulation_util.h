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

// #include "drake/manipulation/util/apply_driver_configs.h"
// #include "drake/manipulation/util/make_arm_controller_model.h"
// #include "drake/manipulation/util/move_ik_demo_base.h"
// #include "drake/manipulation/util/moving_average_filter.h"
// #include "drake/manipulation/util/named_positions_functions.h"
// #include "drake/manipulation/util/robot_plan_interpolator.h"
// #include "drake/manipulation/util/robot_plan_utils.h"
// #include "drake/manipulation/util/zero_force_driver.h"
// #include "drake/manipulation/util/zero_force_driver_functions.h"

// Symbol: pydrake_doc_manipulation_util
constexpr struct /* pydrake_doc_manipulation_util */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::manipulation
    struct /* manipulation */ {
      // Symbol: drake::manipulation::ApplyDriverConfig
      struct /* ApplyDriverConfig */ {
        // Source: drake/manipulation/util/zero_force_driver_functions.h
        const char* doc =
R"""(Applies zero actuation to every joint of a model mainly for debugging
and testing purposes.

Precondition:
    The sim_plant.is_finalized() is true.)""";
      } ApplyDriverConfig;
      // Symbol: drake::manipulation::ApplyDriverConfigs
      struct /* ApplyDriverConfigs */ {
        // Source: drake/manipulation/util/apply_driver_configs.h
        const char* doc =
R"""(Apply many driver configurations to a model.

A "driver configuration" helps stack Drake systems between an LCM
interface subscriber system and the actuation input ports of a
MultibodyPlant (to enact the driver command), as well as the output
ports of a MultibodyPlant back to an LCM interface publisher system
(to provide the driver status).

These conceptually simulate "drivers" -- they take the role that
driver software and control cabinets would take in real life -- but
may also model some physical properties of the robot that are not
easily reflected in MultibodyPlant (e.g., the WSG belt drive).

The caller of this function is responsible for including the variant
members' apply function, such as schunk_wsg_driver_functions.h.

``driver_configs`` The configurations to apply. ``sim_plant`` The
plant containing the model. ``models_from_directives`` All of the
``ModelInstanceInfo`s from previously- loaded directives.
``lcm_buses`` The available LCM buses to drive and/or sense from this
driver. ``builder`` The `DiagramBuilder`` into which to install this
driver.)""";
      } ApplyDriverConfigs;
      // Symbol: drake::manipulation::ApplyNamedPositionsAsDefaults
      struct /* ApplyNamedPositionsAsDefaults */ {
        // Source: drake/manipulation/util/named_positions_functions.h
        const char* doc =
R"""(Sets default joint positions in the given ``plant`` according to the
given ``input`` (which specifies joints by their string name). The
default positions for joints not mentioned will remain unchanged.

Raises:
    RuntimeError if the number of position values in the input and the
    actual number of positions for the intended joint do not match.)""";
      } ApplyNamedPositionsAsDefaults;
      // Symbol: drake::manipulation::NamedPositions
      struct /* NamedPositions */ {
        // Source: drake/manipulation/util/named_positions_functions.h
        const char* doc =
R"""(A map-of-maps {model_instance_name: {joint_name: joint_positions}}
that describes joint positions. Note that a given joint's positions
are always spelled as a vector, even though for the most common joint
types (prismatic, revolute, screw, etc.) the vector will only contain
a single element.)""";
      } NamedPositions;
      // Symbol: drake::manipulation::ZeroForceDriver
      struct /* ZeroForceDriver */ {
        // Source: drake/manipulation/util/zero_force_driver.h
        const char* doc =
R"""(A driver that applies zero actuation to every joint of a model. Useful
for debugging and testing; useless in reality. No LCM channels are
created.)""";
        // Symbol: drake::manipulation::ZeroForceDriver::Serialize
        struct /* Serialize */ {
          // Source: drake/manipulation/util/zero_force_driver.h
          const char* doc = R"""()""";
        } Serialize;
        // Symbol: drake::manipulation::ZeroForceDriver::ZeroForceDriver
        struct /* ctor */ {
          // Source: drake/manipulation/util/zero_force_driver.h
          const char* doc = R"""()""";
        } ctor;
      } ZeroForceDriver;
      // Symbol: drake::manipulation::util
      struct /* util */ {
        // Symbol: drake::manipulation::util::ApplyJointVelocityLimits
        struct /* ApplyJointVelocityLimits */ {
          // Source: drake/manipulation/util/robot_plan_utils.h
          const char* doc =
R"""(Scales a plan so that no step exceeds the robot's maximum joint
velocities. The size of ``keyframes`` must match the size of
``times``. Times must be in strictly increasing order and start with
zero. Per-joint velocity limits are specified by ``limits``, which
much be the same size ad the number of joints in each element of
``keyframes``. Assumes that velocity limits are equal regardless of
direction. If any step does exceed the maximum velocities in
``limits``, ``times`` will be modified to reduce the velocity.)""";
        } ApplyJointVelocityLimits;
        // Symbol: drake::manipulation::util::EncodeKeyFrames
        struct /* EncodeKeyFrames */ {
          // Source: drake/manipulation/util/robot_plan_utils.h
          const char* doc =
R"""(Makes an lcmt_robot_plan message. The entries in ``joint_names``
should be unique, though the behavior if names are duplicated depends
on how the returned plan is evaluated. The size of each vector in
``keyframes`` must match the size of ``joint_names``. The size of
``keyframes`` must match the size of ``times``. Times must be in
strictly increasing order.)""";
        } EncodeKeyFrames;
        // Symbol: drake::manipulation::util::GetJointNames
        struct /* GetJointNames */ {
          // Source: drake/manipulation/util/robot_plan_utils.h
          const char* doc =
R"""(Returns:
    A vector of joint names corresponding to the positions in
    ``plant`` in the order of the joint indices. If joints with
    duplicate names exist in different model instance in the plant,
    the names will be duplicated in the output.)""";
        } GetJointNames;
        // Symbol: drake::manipulation::util::InterpolatorType
        struct /* InterpolatorType */ {
          // Source: drake/manipulation/util/robot_plan_interpolator.h
          const char* doc =
R"""(This enum specifies the type of interpolator to use in constructing
the piece-wise polynomial.)""";
          // Symbol: drake::manipulation::util::InterpolatorType::Cubic
          struct /* Cubic */ {
            // Source: drake/manipulation/util/robot_plan_interpolator.h
            const char* doc = R"""()""";
          } Cubic;
          // Symbol: drake::manipulation::util::InterpolatorType::FirstOrderHold
          struct /* FirstOrderHold */ {
            // Source: drake/manipulation/util/robot_plan_interpolator.h
            const char* doc = R"""()""";
          } FirstOrderHold;
          // Symbol: drake::manipulation::util::InterpolatorType::Pchip
          struct /* Pchip */ {
            // Source: drake/manipulation/util/robot_plan_interpolator.h
            const char* doc = R"""()""";
          } Pchip;
          // Symbol: drake::manipulation::util::InterpolatorType::ZeroOrderHold
          struct /* ZeroOrderHold */ {
            // Source: drake/manipulation/util/robot_plan_interpolator.h
            const char* doc = R"""()""";
          } ZeroOrderHold;
        } InterpolatorType;
        // Symbol: drake::manipulation::util::MoveIkDemoBase
        struct /* MoveIkDemoBase */ {
          // Source: drake/manipulation/util/move_ik_demo_base.h
          const char* doc =
R"""(This class provides some common functionality for generating IK plans
for robot arms, including things like creating a MultibodyPlant,
setting joint velocity limits, implementing a robot status update
handler suitable for invoking from an LCM callback, and generating
plans to move a specified link to a goal configuration.

This can be useful when building simple demonstration programs to move
a robot arm, for example when testing new arms which haven't been
previously used with Drake, or testing modifications to existing robot
configurations. See the kuka_iiwa_arm and kinova_jaco_arm examples for
existing uses.)""";
          // Symbol: drake::manipulation::util::MoveIkDemoBase::HandleStatus
          struct /* HandleStatus */ {
            // Source: drake/manipulation/util/move_ik_demo_base.h
            const char* doc =
R"""(Update the current robot status.

Parameter ``q``:
    must be equal to the number of positions in the MultibodyPlant
    (see plant()).)""";
          } HandleStatus;
          // Symbol: drake::manipulation::util::MoveIkDemoBase::MoveIkDemoBase
          struct /* ctor */ {
            // Source: drake/manipulation/util/move_ik_demo_base.h
            const char* doc =
R"""(Parameter ``robot_description``:
    A description file to load of the robot to plan.

Parameter ``base_link``:
    Name of the base link of the robot, will be welded to the world in
    the planning model.

Parameter ``ik_link``:
    Name of the link to plan a pose for.

Parameter ``print_interval``:
    Print an updated end effector position every N calls to
    HandleStatus.)""";
          } ctor;
          // Symbol: drake::manipulation::util::MoveIkDemoBase::Plan
          struct /* Plan */ {
            // Source: drake/manipulation/util/move_ik_demo_base.h
            const char* doc =
R"""(Attempt to generate a plan moving ik_link (specified at construction
time) from the joint configuration specified in the last call to
``HandleStatus`` to a configuration with ik_link at ``goal_pose``.
Returns nullopt if planning failed.

Raises:
    If HandleStatus has not been invoked.)""";
          } Plan;
          // Symbol: drake::manipulation::util::MoveIkDemoBase::plant
          struct /* plant */ {
            // Source: drake/manipulation/util/move_ik_demo_base.h
            const char* doc =
R"""(Returns:
    a reference to the internal plant.)""";
          } plant;
          // Symbol: drake::manipulation::util::MoveIkDemoBase::set_joint_velocity_limits
          struct /* set_joint_velocity_limits */ {
            // Source: drake/manipulation/util/move_ik_demo_base.h
            const char* doc =
R"""(Set the joint velocity limits when building the plan. The default
velocity limits from the robot description will be used if this isn't
set.

Precondition:
    The size of the input vector must be equal to the number of
    velocities in the MultibodyPlant (see plant()).)""";
          } set_joint_velocity_limits;
          // Symbol: drake::manipulation::util::MoveIkDemoBase::status_count
          struct /* status_count */ {
            // Source: drake/manipulation/util/move_ik_demo_base.h
            const char* doc =
R"""(Returns a count of how many times ``HandleStatus`` has been called.)""";
          } status_count;
        } MoveIkDemoBase;
        // Symbol: drake::manipulation::util::MovingAverageFilter
        struct /* MovingAverageFilter */ {
          // Source: drake/manipulation/util/moving_average_filter.h
          const char* doc =
R"""(The implementation of a Moving Average Filter. This discrete time
filter outputs the average of the last n samples i.e. y[k] = 1/n ∑ⱼ
x[k-j] ∀ j = 0..n-1, when n<k and, = 1/k ∑ⱼ x[j] ∀ j = 0..k otherwise;
where n is the window size and x being the discrete-time signal that
is to be filtered, y is the filtered signal and k is the index of
latest element in the signal time-series.

Note that this class is meant to serve as a standalone simple utility
and a filter of this form in a more ``drake∷systems`` flavour can be
generated from a ``systems∷AffineSystem`` since this is a LTI filter.

Template parameter ``T``:
    The element type. Instantiated templates for the following kinds
    of T's are provided:

- double
- VectorX<double>)""";
          // Symbol: drake::manipulation::util::MovingAverageFilter::MovingAverageFilter<T>
          struct /* ctor */ {
            // Source: drake/manipulation/util/moving_average_filter.h
            const char* doc =
R"""(Constructs the filter with the specified ``window_size``.

Parameter ``window_size``:
    The size of the window.

Raises:
    RuntimeError when window_size <= 0.)""";
          } ctor;
          // Symbol: drake::manipulation::util::MovingAverageFilter::Update
          struct /* Update */ {
            // Source: drake/manipulation/util/moving_average_filter.h
            const char* doc =
R"""(Updates the average filter result. Every call to this method modifies
the internal state of this filter thus resulting in a computation of
the moving average of the data present within the filter window.

Parameter ``new_data``:
    $Returns:)""";
          } Update;
          // Symbol: drake::manipulation::util::MovingAverageFilter::moving_average
          struct /* moving_average */ {
            // Source: drake/manipulation/util/moving_average_filter.h
            const char* doc =
R"""(Returns the most recent result of the averaging filter.)""";
          } moving_average;
          // Symbol: drake::manipulation::util::MovingAverageFilter::window
          struct /* window */ {
            // Source: drake/manipulation/util/moving_average_filter.h
            const char* doc = R"""()""";
          } window;
        } MovingAverageFilter;
        // Symbol: drake::manipulation::util::RobotPlanInterpolator
        struct /* RobotPlanInterpolator */ {
          // Source: drake/manipulation/util/robot_plan_interpolator.h
          const char* doc =
R"""(This class implements a source of joint positions for a robot. It has
one input port for lcmt_robot_plan messages containing a plan to
follow.

The system has two output ports, one with the current desired state
(q,v) of the robot and another for the accelerations.

.. pydrake_system::

    name: RobotPlanInterpolator
    input_ports:
    - plan
    output_ports:
    - state
    - acceleration

If a plan is received with no knot points, the system will create a
plan which commands the robot to hold at the measured position.)""";
          // Symbol: drake::manipulation::util::RobotPlanInterpolator::Initialize
          struct /* Initialize */ {
            // Source: drake/manipulation/util/robot_plan_interpolator.h
            const char* doc =
R"""(Makes a plan to hold at the measured joint configuration ``q0``
starting at ``plan_start_time``. This function needs to be explicitly
called before any simulation. Otherwise this aborts in CalcOutput().)""";
          } Initialize;
          // Symbol: drake::manipulation::util::RobotPlanInterpolator::RobotPlanInterpolator
          struct /* ctor */ {
            // Source: drake/manipulation/util/robot_plan_interpolator.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::manipulation::util::RobotPlanInterpolator::UpdatePlan
          struct /* UpdatePlan */ {
            // Source: drake/manipulation/util/robot_plan_interpolator.h
            const char* doc =
R"""(Updates the plan if there is a new message on the input port. Normally
this is done automatically by a Simulator; here we invoke the same
periodic event handler that the Simulator would use.)""";
          } UpdatePlan;
          // Symbol: drake::manipulation::util::RobotPlanInterpolator::get_acceleration_output_port
          struct /* get_acceleration_output_port */ {
            // Source: drake/manipulation/util/robot_plan_interpolator.h
            const char* doc = R"""()""";
          } get_acceleration_output_port;
          // Symbol: drake::manipulation::util::RobotPlanInterpolator::get_plan_input_port
          struct /* get_plan_input_port */ {
            // Source: drake/manipulation/util/robot_plan_interpolator.h
            const char* doc =
R"""(N.B. This input port is useless and may be left disconnected.)""";
          } get_plan_input_port;
          // Symbol: drake::manipulation::util::RobotPlanInterpolator::get_state_output_port
          struct /* get_state_output_port */ {
            // Source: drake/manipulation/util/robot_plan_interpolator.h
            const char* doc = R"""()""";
          } get_state_output_port;
          // Symbol: drake::manipulation::util::RobotPlanInterpolator::plant
          struct /* plant */ {
            // Source: drake/manipulation/util/robot_plan_interpolator.h
            const char* doc = R"""()""";
          } plant;
        } RobotPlanInterpolator;
      } util;
    } manipulation;
  } drake;
} pydrake_doc_manipulation_util;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
