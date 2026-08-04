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

// #include "drake/multibody/benchmarks/pendulum/make_pendulum_plant.h"

// Symbol: pydrake_doc_multibody_benchmarks_pendulum
constexpr struct /* pydrake_doc_multibody_benchmarks_pendulum */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::multibody
    struct /* multibody */ {
      // Symbol: drake::multibody::benchmarks
      struct /* benchmarks */ {
        // Symbol: drake::multibody::benchmarks::pendulum
        struct /* pendulum */ {
          // Symbol: drake::multibody::benchmarks::pendulum::MakePendulumPlant
          struct /* MakePendulumPlant */ {
            // Source: drake/multibody/benchmarks/pendulum/make_pendulum_plant.h
            const char* doc =
R"""(This method makes a MultibodyPlant model of an idealized pendulum with
a point mass at the end of a massless rigid rod. The pendulum
oscillates in the x-z plane with its revolute axis coincident with the
y-axis. Gravity points downwards in the -z axis direction.

The parameters of the plant are:

- mass: the mass of the idealized point mass.
- length: the length of the massless rod on which the mass is suspended.
- gravity: the acceleration of gravity.

The simple pendulum is a canonical dynamical system as described in <a
href="http://underactuated.csail.mit.edu/underactuated.html?chapter=pend">
Chapter 2 of Underactuated Robotics</a>.

Parameter ``default_parameters``:
    Default parameters of the model set at construction. Refer to the
    documentation of PendulumParameters for further details.

Parameter ``scene_graph``:
    If a SceneGraph is provided with this argument, this factory
    method will register the new multibody plant to be a source for
    that geometry system and it will also register geometry for
    visualization. If this argument is omitted, no geometry will be
    registered.)""";
          } MakePendulumPlant;
          // Symbol: drake::multibody::benchmarks::pendulum::PendulumParameters
          struct /* PendulumParameters */ {
            // Source: drake/multibody/benchmarks/pendulum/make_pendulum_plant.h
            const char* doc =
R"""(This class is used to store the numerical parameters defining the
model of a simple pendulum with the method MakePendulumPlant(). Refer
to this the documentation of this class's constructor for further
details on the parameters stored by this class and their default
values.)""";
            // Symbol: drake::multibody::benchmarks::pendulum::PendulumParameters::PendulumParameters
            struct /* ctor */ {
              // Source: drake/multibody/benchmarks/pendulum/make_pendulum_plant.h
              const char* doc =
R"""(Constructor used to initialize the physical parameters for a simple
pendulum model.

Parameter ``mass``:
    Value of the mass of the pendulum's point mass [kg].

Parameter ``length``:
    Length of the massless rod connecting the point mass to the world
    [m].

Parameter ``damping``:
    The joint's damping in N⋅m⋅s.

Parameter ``gravity``:
    Gravitational constant (m/s²).)""";
            } ctor;
            // Symbol: drake::multibody::benchmarks::pendulum::PendulumParameters::actuator_name
            struct /* actuator_name */ {
              // Source: drake/multibody/benchmarks/pendulum/make_pendulum_plant.h
              const char* doc = R"""()""";
            } actuator_name;
            // Symbol: drake::multibody::benchmarks::pendulum::PendulumParameters::body_name
            struct /* body_name */ {
              // Source: drake/multibody/benchmarks/pendulum/make_pendulum_plant.h
              const char* doc = R"""()""";
            } body_name;
            // Symbol: drake::multibody::benchmarks::pendulum::PendulumParameters::damping
            struct /* damping */ {
              // Source: drake/multibody/benchmarks/pendulum/make_pendulum_plant.h
              const char* doc = R"""()""";
            } damping;
            // Symbol: drake::multibody::benchmarks::pendulum::PendulumParameters::g
            struct /* g */ {
              // Source: drake/multibody/benchmarks/pendulum/make_pendulum_plant.h
              const char* doc = R"""()""";
            } g;
            // Symbol: drake::multibody::benchmarks::pendulum::PendulumParameters::l
            struct /* l */ {
              // Source: drake/multibody/benchmarks/pendulum/make_pendulum_plant.h
              const char* doc = R"""()""";
            } l;
            // Symbol: drake::multibody::benchmarks::pendulum::PendulumParameters::m
            struct /* m */ {
              // Source: drake/multibody/benchmarks/pendulum/make_pendulum_plant.h
              const char* doc = R"""()""";
            } m;
            // Symbol: drake::multibody::benchmarks::pendulum::PendulumParameters::massless_rod_radius
            struct /* massless_rod_radius */ {
              // Source: drake/multibody/benchmarks/pendulum/make_pendulum_plant.h
              const char* doc = R"""()""";
            } massless_rod_radius;
            // Symbol: drake::multibody::benchmarks::pendulum::PendulumParameters::pin_joint_name
            struct /* pin_joint_name */ {
              // Source: drake/multibody/benchmarks/pendulum/make_pendulum_plant.h
              const char* doc = R"""()""";
            } pin_joint_name;
            // Symbol: drake::multibody::benchmarks::pendulum::PendulumParameters::point_mass_radius
            struct /* point_mass_radius */ {
              // Source: drake/multibody/benchmarks/pendulum/make_pendulum_plant.h
              const char* doc = R"""()""";
            } point_mass_radius;
          } PendulumParameters;
        } pendulum;
      } benchmarks;
    } multibody;
  } drake;
} pydrake_doc_multibody_benchmarks_pendulum;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
