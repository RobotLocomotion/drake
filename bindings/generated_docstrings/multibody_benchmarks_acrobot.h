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

// #include "drake/multibody/benchmarks/acrobot/acrobot.h"
// #include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"

// Symbol: pydrake_doc_multibody_benchmarks_acrobot
constexpr struct /* pydrake_doc_multibody_benchmarks_acrobot */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::multibody
    struct /* multibody */ {
      // Symbol: drake::multibody::benchmarks
      struct /* benchmarks */ {
        // Symbol: drake::multibody::benchmarks::Acrobot
        struct /* Acrobot */ {
          // Source: drake/multibody/benchmarks/acrobot/acrobot.h
          const char* doc =
R"""(The Acrobot - a canonical underactuated system as described in <a
href="http://underactuated.mit.edu/underactuated.html?chapter=3">Chapter
3 of Underactuated Robotics</a>.

This system essentially is a double pendulum consisting of two links.
Link 1 is connected to the world by a "shoulder" revolute joint
parameterized by angle theta1 and Link 2 is connected to Link 1 by an
"elbow" revolute joint parameterized by angle theta2.)""";
          // Symbol: drake::multibody::benchmarks::Acrobot::Acrobot<T>
          struct /* ctor */ {
            // Source: drake/multibody/benchmarks/acrobot/acrobot.h
            const char* doc =
R"""(Creates an acrobot model in a plane passing through the world's origin
and normal to ``normal``. Vector ``up`` defines the upwards direction
on this plane. Both ``normal`` and ``up`` are expressed in the world's
frame. Essentially the two dimensional equations of the acrobot are
described in a model frame D within a x-y plane with y the vertical
direction and gravity pointing downwards. Therefore the axes defining
the model frame D are:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    z_W = normal_W.normalized()
      y_W = (up - up.dot(z_W) * z_W).normalized()
      x_W = y_W.cross(z_W)

.. raw:: html

    </details>

The remaining arguments define the properties of the double pendulum
system:

- m1: mass of the first link.
- m2: mass of the second link.
- l1: length of the first link.
- l2: length of the second link.
- lc1: length from the shoulder to the center of mass of the first link.
- lc2: length from the elbow to the center of mass of the second link.
- Ic1: moment of inertia about the center of mass for the first link.
- Ic2: moment of inertia about the center of mass for the second link.
- b1: damping coefficient of the shoulder joint.
- b2: damping coefficient of the elbow joint.
- g: acceleration of gavity.)""";
          } ctor;
          // Symbol: drake::multibody::benchmarks::Acrobot::CalcCoriolisVector
          struct /* CalcCoriolisVector */ {
            // Source: drake/multibody/benchmarks/acrobot/acrobot.h
            const char* doc =
R"""(Computes the bias term ``C(q, v) * v`` containing Coriolis and
gyroscopic effects as a function of the state of the pendulum.)""";
          } CalcCoriolisVector;
          // Symbol: drake::multibody::benchmarks::Acrobot::CalcElbowOutboardFramePoseInWorldFrame
          struct /* CalcElbowOutboardFramePoseInWorldFrame */ {
            // Source: drake/multibody/benchmarks/acrobot/acrobot.h
            const char* doc =
R"""(Computes the pose of the elbow outboard frame ``Eo`` in the world
frame W.

Parameter ``theta1``:
    The shoulder angle in radians.

Parameter ``theta2``:
    The elbow angle in radians.

Returns:
    X_WEo the pose of the elbow frame Eo in the world frame W.)""";
          } CalcElbowOutboardFramePoseInWorldFrame;
          // Symbol: drake::multibody::benchmarks::Acrobot::CalcGravityVector
          struct /* CalcGravityVector */ {
            // Source: drake/multibody/benchmarks/acrobot/acrobot.h
            const char* doc =
R"""(Computes the effective joint-space torques induced by gravity
``tau_g(q)`` containing the effect of gravity as a function of the
configuration of the pendulum. Unlike
http://underactuated.mit.edu/underactuated.html?chapter=3, cited in
this class's documentation, we define ``tau_g(q)`` to be on the right
hand side of the equations of motion, that is, ``Mv̇ + C(q, v)v =
tau_g(q)``.)""";
          } CalcGravityVector;
          // Symbol: drake::multibody::benchmarks::Acrobot::CalcLink1PoseInWorldFrame
          struct /* CalcLink1PoseInWorldFrame */ {
            // Source: drake/multibody/benchmarks/acrobot/acrobot.h
            const char* doc =
R"""(Computes the pose of the center of mass of link 1 measured and
expressed in the world frame.

Parameter ``theta1``:
    The shoulder angle in radians.

Returns:
    X_WL1 the pose of link 1 measured and expressed in the world
    frame.)""";
          } CalcLink1PoseInWorldFrame;
          // Symbol: drake::multibody::benchmarks::Acrobot::CalcLink1SpatialAccelerationInWorldFrame
          struct /* CalcLink1SpatialAccelerationInWorldFrame */ {
            // Source: drake/multibody/benchmarks/acrobot/acrobot.h
            const char* doc =
R"""(Computes the spatial acceleration of the center of mass of link 1
expressed in the world frame.

Parameter ``theta1``:
    The shoulder angle in radians.

Parameter ``theta1dot``:
    The shoulder angular velocity in radians per second.

Parameter ``theta1dotdot``:
    The elbow angular acceleration in radians per second squared.

Returns ``A_WL1_W``:
    the spatial acceleration of the center of mass of link 1 with
    respect to the world and expressed in the world frame.)""";
          } CalcLink1SpatialAccelerationInWorldFrame;
          // Symbol: drake::multibody::benchmarks::Acrobot::CalcLink1SpatialVelocityInWorldFrame
          struct /* CalcLink1SpatialVelocityInWorldFrame */ {
            // Source: drake/multibody/benchmarks/acrobot/acrobot.h
            const char* doc =
R"""(Computes the spatial velocity of the center of mass of link 1
expressed in the world frame.

Parameter ``theta1``:
    The shoulder angle in radians.

Parameter ``theta1dot``:
    The shoulder angular velocity in radians per second.

Returns:
    V_WL1_W the spatial velocity of the center of mass of link 1 with
    respect to the world and expressed in the world frame.)""";
          } CalcLink1SpatialVelocityInWorldFrame;
          // Symbol: drake::multibody::benchmarks::Acrobot::CalcLink2PoseInWorldFrame
          struct /* CalcLink2PoseInWorldFrame */ {
            // Source: drake/multibody/benchmarks/acrobot/acrobot.h
            const char* doc =
R"""(Computes the pose of the center of mass of link 2 measured and
expressed in the world frame.

Parameter ``theta1``:
    The shoulder angle in radians.

Parameter ``theta2``:
    The elbow angle in radians.

Returns:
    X_WL2 the pose of link 2 measured and expressed in the world
    frame.)""";
          } CalcLink2PoseInWorldFrame;
          // Symbol: drake::multibody::benchmarks::Acrobot::CalcLink2SpatialAccelerationInWorldFrame
          struct /* CalcLink2SpatialAccelerationInWorldFrame */ {
            // Source: drake/multibody/benchmarks/acrobot/acrobot.h
            const char* doc =
R"""(Computes the spatial acceleration of the center of mass of link 2
expressed in the world frame.

Parameter ``theta1``:
    The shoulder angle in radians.

Parameter ``theta2``:
    The elbow angle in radians.

Parameter ``theta1dot``:
    The shoulder angular velocity in radians per second.

Parameter ``theta2dot``:
    The elbow angular velocity in radians per second.

Parameter ``theta1dotdot``:
    The shoulder angular acceleration in radians per second squared.

Parameter ``theta2dotdot``:
    The elbow angular acceleration in radians per second squared.

Returns ``A_WL2_W``:
    the spatial acceleration of the center of mass of link 2 with
    respect to the world and expressed in the world frame.)""";
          } CalcLink2SpatialAccelerationInWorldFrame;
          // Symbol: drake::multibody::benchmarks::Acrobot::CalcLink2SpatialVelocityInWorldFrame
          struct /* CalcLink2SpatialVelocityInWorldFrame */ {
            // Source: drake/multibody/benchmarks/acrobot/acrobot.h
            const char* doc =
R"""(Computes the spatial velocity of the center of mass of link 2
expressed in the world frame.

Parameter ``theta1``:
    The shoulder angle in radians.

Parameter ``theta2``:
    The elbow angle in radians.

Parameter ``theta1dot``:
    The shoulder angular velocity in radians per second.

Parameter ``theta2dot``:
    The elbow angular velocity in radians per second.

Returns:
    V_WL2_W the spatial velocity of the center of mass of link 2 with
    respect to the world and expressed in the world frame.)""";
          } CalcLink2SpatialVelocityInWorldFrame;
          // Symbol: drake::multibody::benchmarks::Acrobot::CalcMassMatrix
          struct /* CalcMassMatrix */ {
            // Source: drake/multibody/benchmarks/acrobot/acrobot.h
            const char* doc =
R"""(Computes the mass matrix ``H(q)`` for the double pendulum system. It
turns out that for this system the mass matrix is independent of the
shoulder angle ``theta1``.)""";
          } CalcMassMatrix;
          // Symbol: drake::multibody::benchmarks::Acrobot::CalcPotentialEnergy
          struct /* CalcPotentialEnergy */ {
            // Source: drake/multibody/benchmarks/acrobot/acrobot.h
            const char* doc =
R"""(Computes the total potential energy due to gravity of the acrobot
system for the state given by angles ``theta1`` and ``theta2``. The
zero potential energy is defined for ``y = 0``.)""";
          } CalcPotentialEnergy;
        } Acrobot;
        // Symbol: drake::multibody::benchmarks::acrobot
        struct /* acrobot */ {
          // Symbol: drake::multibody::benchmarks::acrobot::AcrobotParameters
          struct /* AcrobotParameters */ {
            // Source: drake/multibody/benchmarks/acrobot/make_acrobot_plant.h
            const char* doc =
R"""(This class is used to store the numerical parameters defining the
model of an acrobot with the method MakeAcrobotPlant(). Refer to this
the documentation of this class's constructor for further details on
the parameters stored by this class and their default values.

Note:
    The default constructor initializes the parameters in accordance
    to the ``acrobot.sdf`` file in this same directory. Therefore this
    file and ``acrobot.sdf`` MUST be kept in sync.)""";
            // Symbol: drake::multibody::benchmarks::acrobot::AcrobotParameters::AcrobotParameters
            struct /* ctor */ {
              // Source: drake/multibody/benchmarks/acrobot/make_acrobot_plant.h
              const char* doc =
R"""(Constructor used to initialize the physical parameters for an acrobot
model. The parameters are defaulted to values in Spong's paper [Spong
1994].

Parameter ``m1``:
    Mass of link 1 (kg).

Parameter ``m2``:
    Mass of link 2 (kg).

Parameter ``l1``:
    Length of link 1 (m).

Parameter ``l2``:
    Length of link 2 (m).

Parameter ``lc1``:
    Vertical distance from shoulder joint to center of mass of link 1
    (m).

Parameter ``lc2``:
    Vertical distance from elbow joint to center of mass of link 2
    (m).

Parameter ``Ic1``:
    Inertia of link 1 about the center of mass of link 1 (kg⋅m²).

Parameter ``Ic2``:
    Inertia of link 2 about the center of mass of link 2 (kg*m^2).

Parameter ``b1``:
    Damping coefficient of the shoulder joint (N⋅m⋅s).

Parameter ``b2``:
    Damping coefficient of the elbow joint (N⋅m⋅s).

Parameter ``g``:
    Gravitational constant (m/s²).

- [Spong 1994] Spong, M.W., 1994. Swing up control of the acrobot. In
Robotics and Automation, 1994. Proceedings., 1994 IEEE International
Conference on (pp. 2356-2361). IEEE.)""";
            } ctor;
            // Symbol: drake::multibody::benchmarks::acrobot::AcrobotParameters::Gc1
            struct /* Gc1 */ {
              // Source: drake/multibody/benchmarks/acrobot/make_acrobot_plant.h
              const char* doc = R"""()""";
            } Gc1;
            // Symbol: drake::multibody::benchmarks::acrobot::AcrobotParameters::Gc2
            struct /* Gc2 */ {
              // Source: drake/multibody/benchmarks/acrobot/make_acrobot_plant.h
              const char* doc = R"""()""";
            } Gc2;
            // Symbol: drake::multibody::benchmarks::acrobot::AcrobotParameters::Ic1
            struct /* Ic1 */ {
              // Source: drake/multibody/benchmarks/acrobot/make_acrobot_plant.h
              const char* doc = R"""()""";
            } Ic1;
            // Symbol: drake::multibody::benchmarks::acrobot::AcrobotParameters::Ic2
            struct /* Ic2 */ {
              // Source: drake/multibody/benchmarks/acrobot/make_acrobot_plant.h
              const char* doc = R"""()""";
            } Ic2;
            // Symbol: drake::multibody::benchmarks::acrobot::AcrobotParameters::actuator_name
            struct /* actuator_name */ {
              // Source: drake/multibody/benchmarks/acrobot/make_acrobot_plant.h
              const char* doc = R"""()""";
            } actuator_name;
            // Symbol: drake::multibody::benchmarks::acrobot::AcrobotParameters::b1
            struct /* b1 */ {
              // Source: drake/multibody/benchmarks/acrobot/make_acrobot_plant.h
              const char* doc = R"""()""";
            } b1;
            // Symbol: drake::multibody::benchmarks::acrobot::AcrobotParameters::b2
            struct /* b2 */ {
              // Source: drake/multibody/benchmarks/acrobot/make_acrobot_plant.h
              const char* doc = R"""()""";
            } b2;
            // Symbol: drake::multibody::benchmarks::acrobot::AcrobotParameters::elbow_joint_name
            struct /* elbow_joint_name */ {
              // Source: drake/multibody/benchmarks/acrobot/make_acrobot_plant.h
              const char* doc = R"""()""";
            } elbow_joint_name;
            // Symbol: drake::multibody::benchmarks::acrobot::AcrobotParameters::g
            struct /* g */ {
              // Source: drake/multibody/benchmarks/acrobot/make_acrobot_plant.h
              const char* doc = R"""()""";
            } g;
            // Symbol: drake::multibody::benchmarks::acrobot::AcrobotParameters::l1
            struct /* l1 */ {
              // Source: drake/multibody/benchmarks/acrobot/make_acrobot_plant.h
              const char* doc = R"""()""";
            } l1;
            // Symbol: drake::multibody::benchmarks::acrobot::AcrobotParameters::l2
            struct /* l2 */ {
              // Source: drake/multibody/benchmarks/acrobot/make_acrobot_plant.h
              const char* doc = R"""()""";
            } l2;
            // Symbol: drake::multibody::benchmarks::acrobot::AcrobotParameters::lc1
            struct /* lc1 */ {
              // Source: drake/multibody/benchmarks/acrobot/make_acrobot_plant.h
              const char* doc = R"""()""";
            } lc1;
            // Symbol: drake::multibody::benchmarks::acrobot::AcrobotParameters::lc2
            struct /* lc2 */ {
              // Source: drake/multibody/benchmarks/acrobot/make_acrobot_plant.h
              const char* doc = R"""()""";
            } lc2;
            // Symbol: drake::multibody::benchmarks::acrobot::AcrobotParameters::link1_name
            struct /* link1_name */ {
              // Source: drake/multibody/benchmarks/acrobot/make_acrobot_plant.h
              const char* doc = R"""()""";
            } link1_name;
            // Symbol: drake::multibody::benchmarks::acrobot::AcrobotParameters::link2_name
            struct /* link2_name */ {
              // Source: drake/multibody/benchmarks/acrobot/make_acrobot_plant.h
              const char* doc = R"""()""";
            } link2_name;
            // Symbol: drake::multibody::benchmarks::acrobot::AcrobotParameters::m1
            struct /* m1 */ {
              // Source: drake/multibody/benchmarks/acrobot/make_acrobot_plant.h
              const char* doc = R"""()""";
            } m1;
            // Symbol: drake::multibody::benchmarks::acrobot::AcrobotParameters::m2
            struct /* m2 */ {
              // Source: drake/multibody/benchmarks/acrobot/make_acrobot_plant.h
              const char* doc = R"""()""";
            } m2;
            // Symbol: drake::multibody::benchmarks::acrobot::AcrobotParameters::r1
            struct /* r1 */ {
              // Source: drake/multibody/benchmarks/acrobot/make_acrobot_plant.h
              const char* doc = R"""()""";
            } r1;
            // Symbol: drake::multibody::benchmarks::acrobot::AcrobotParameters::r2
            struct /* r2 */ {
              // Source: drake/multibody/benchmarks/acrobot/make_acrobot_plant.h
              const char* doc = R"""()""";
            } r2;
            // Symbol: drake::multibody::benchmarks::acrobot::AcrobotParameters::shoulder_joint_name
            struct /* shoulder_joint_name */ {
              // Source: drake/multibody/benchmarks/acrobot/make_acrobot_plant.h
              const char* doc = R"""()""";
            } shoulder_joint_name;
          } AcrobotParameters;
          // Symbol: drake::multibody::benchmarks::acrobot::MakeAcrobotPlant
          struct /* MakeAcrobotPlant */ {
            // Source: drake/multibody/benchmarks/acrobot/make_acrobot_plant.h
            const char* doc =
R"""(This method makes a MultibodyPlant model of the Acrobot - a canonical
underactuated system as described in <a
href="http://underactuated.mit.edu/underactuated.html?chapter=3">Chapter
3 of Underactuated Robotics</a>.

Parameter ``default_parameters``:
    Default parameters of the model set at construction. These
    parameters include masses, link lengths, rotational inertias, etc.
    Refer to the documentation of AcrobotParameters for further
    details.

Parameter ``finalize``:
    If ``True``, MultibodyPlant∷Finalize() gets called on the new
    plant.

Parameter ``scene_graph``:
    If a SceneGraph is provided with this argument, this factory
    method will register the new multibody plant to be a source for
    that geometry system and it will also register geometry for
    visualization. If this argument is omitted, no geometry will be
    registered.)""";
          } MakeAcrobotPlant;
        } acrobot;
      } benchmarks;
    } multibody;
  } drake;
} pydrake_doc_multibody_benchmarks_acrobot;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
