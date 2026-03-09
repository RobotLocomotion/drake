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

// #include "drake/examples/quadrotor/quadrotor_geometry.h"
// #include "drake/examples/quadrotor/quadrotor_plant.h"

// Symbol: pydrake_doc_examples_quadrotor
constexpr struct /* pydrake_doc_examples_quadrotor */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::examples
    struct /* examples */ {
      // Symbol: drake::examples::quadrotor
      struct /* quadrotor */ {
        // Symbol: drake::examples::quadrotor::QuadrotorGeometry
        struct /* QuadrotorGeometry */ {
          // Source: drake/examples/quadrotor/quadrotor_geometry.h
          const char* doc =
R"""(Expresses a QuadrotorPlant's geometry to a SceneGraph.

.. pydrake_system::

    name: QuadrotorGeometry
    input_ports:
    - state
    output_ports:
    - geometry_pose

This class has no public constructor; instead use the AddToBuilder()
static method to create and add it to a DiagramBuilder directly.)""";
          // Symbol: drake::examples::quadrotor::QuadrotorGeometry::AddToBuilder
          struct /* AddToBuilder */ {
            // Source: drake/examples/quadrotor/quadrotor_geometry.h
            const char* doc =
R"""(Creates, adds, and connects a QuadrotorGeometry system into the given
``builder``. Both the ``quadrotor_state.get_system()`` and
``scene_graph`` systems must have been added to the given ``builder``
already.

The ``scene_graph`` pointer is not retained by the QuadrotorGeometry
system. The return value pointer is an alias of the new
QuadrotorGeometry system that is owned by the ``builder``.)""";
          } AddToBuilder;
          // Symbol: drake::examples::quadrotor::QuadrotorGeometry::QuadrotorGeometry
          struct /* ctor */ {
            // Source: drake/examples/quadrotor/quadrotor_geometry.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::examples::quadrotor::QuadrotorGeometry::get_frame_id
          struct /* get_frame_id */ {
            // Source: drake/examples/quadrotor/quadrotor_geometry.h
            const char* doc =
R"""(Returns the frame of the geometry registered with a SceneGraph. This
can be useful, e.g., if one would like to add a camera to the
quadrotor.)""";
          } get_frame_id;
        } QuadrotorGeometry;
        // Symbol: drake::examples::quadrotor::QuadrotorPlant
        struct /* QuadrotorPlant */ {
          // Source: drake/examples/quadrotor/quadrotor_plant.h
          const char* doc =
R"""(The Quadrotor - an underactuated aerial vehicle. This version of the
Quadrotor is implemented to match the dynamics of the plant specified
in the ``package://drake_models/skydio_2/quadrotor.urdf`` model file.

.. pydrake_system::

    name: QuadrotorPlant
    input_ports:
    - propeller_force (optional)
    output_ports:
    - state

Note: If the propeller_force input port is not connected, then the
force is taken to be zero.)""";
          // Symbol: drake::examples::quadrotor::QuadrotorPlant::QuadrotorPlant<T>
          struct /* ctor */ {
            // Source: drake/examples/quadrotor/quadrotor_plant.h
            const char* doc = R"""()""";
            // Source: drake/examples/quadrotor/quadrotor_plant.h
            const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
          } ctor;
          // Symbol: drake::examples::quadrotor::QuadrotorPlant::force_constant
          struct /* force_constant */ {
            // Source: drake/examples/quadrotor/quadrotor_plant.h
            const char* doc = R"""()""";
          } force_constant;
          // Symbol: drake::examples::quadrotor::QuadrotorPlant::g
          struct /* g */ {
            // Source: drake/examples/quadrotor/quadrotor_plant.h
            const char* doc = R"""()""";
          } g;
          // Symbol: drake::examples::quadrotor::QuadrotorPlant::inertia
          struct /* inertia */ {
            // Source: drake/examples/quadrotor/quadrotor_plant.h
            const char* doc = R"""()""";
          } inertia;
          // Symbol: drake::examples::quadrotor::QuadrotorPlant::length
          struct /* length */ {
            // Source: drake/examples/quadrotor/quadrotor_plant.h
            const char* doc = R"""()""";
          } length;
          // Symbol: drake::examples::quadrotor::QuadrotorPlant::m
          struct /* m */ {
            // Source: drake/examples/quadrotor/quadrotor_plant.h
            const char* doc = R"""()""";
          } m;
          // Symbol: drake::examples::quadrotor::QuadrotorPlant::moment_constant
          struct /* moment_constant */ {
            // Source: drake/examples/quadrotor/quadrotor_plant.h
            const char* doc = R"""()""";
          } moment_constant;
        } QuadrotorPlant;
        // Symbol: drake::examples::quadrotor::StabilizingLQRController
        struct /* StabilizingLQRController */ {
          // Source: drake/examples/quadrotor/quadrotor_plant.h
          const char* doc =
R"""(Generates an LQR controller to move to ``nominal_position``.
Internally computes the nominal input corresponding to a hover at
position ``x0``.

See also:
    systems∷LinearQuadraticRegulator.)""";
        } StabilizingLQRController;
      } quadrotor;
    } examples;
  } drake;
} pydrake_doc_examples_quadrotor;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
