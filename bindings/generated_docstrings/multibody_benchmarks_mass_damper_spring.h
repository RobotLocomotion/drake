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

// #include "drake/multibody/benchmarks/mass_damper_spring/mass_damper_spring_analytical_solution.h"

// Symbol: pydrake_doc_multibody_benchmarks_mass_damper_spring
constexpr struct /* pydrake_doc_multibody_benchmarks_mass_damper_spring */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::multibody
    struct /* multibody */ {
      // Symbol: drake::multibody::benchmarks
      struct /* benchmarks */ {
        // Symbol: drake::multibody::benchmarks::MassDamperSpringAnalyticalSolution
        struct /* MassDamperSpringAnalyticalSolution */ {
          // Source: drake/multibody/benchmarks/mass_damper_spring/mass_damper_spring_analytical_solution.h
          const char* doc =
R"""(This class provides an analytical solution to a mass-damper-spring
system. The system consists of a particle Q of mass m that can only
move left/right on flat Earth (frame N). Particle Q is connected by an
ideal translational spring/damper. The other end of the spring/damper
is welded to point No (the origin of frame N). Q's position from No is
x*Nx where x(t) is a time- dependent variable (to-be-calculated) and
Nx is a horizontal unit vector fixed in Earth (N). The spring force on
Q is -k*x*Nx, where k is a spring constant. The damper force on Q is
-b*ẋ*Nx where b is a damper constant and ẋ is the time-derivative of
x.

Note:
    All units must be self-consistent (e.g., standard SI with MKS
    units). The solution provided herein is also applicable to a
    rotating system, e.g., having rigid-body inertia, rotational
    damper, rotational spring.)""";
          // Symbol: drake::multibody::benchmarks::MassDamperSpringAnalyticalSolution::CalculateOutput
          struct /* CalculateOutput */ {
            // Source: drake/multibody/benchmarks/mass_damper_spring/mass_damper_spring_analytical_solution.h
            const char* doc =
R"""(For ``this`` mass-damper-spring system, and with the given initial
values, this method calculates the values of x, ẋ, ẍ at time t.

Parameter ``t``:
    The value of time at which output is requested.

Returns:
    Three-element matrix consisting of x, ẋ, ẍ, respectively.)""";
          } CalculateOutput;
          // Symbol: drake::multibody::benchmarks::MassDamperSpringAnalyticalSolution::MassDamperSpringAnalyticalSolution<T>
          struct /* ctor */ {
            // Source: drake/multibody/benchmarks/mass_damper_spring/mass_damper_spring_analytical_solution.h
            const char* doc =
R"""(This constructs the aforementioned mass-damper-spring system.

Parameter ``mass``:
    Mass of system (particle Q).

Parameter ``b``:
    Linear damping constant.

Parameter ``k``:
    Linear spring constant.)""";
          } ctor;
          // Symbol: drake::multibody::benchmarks::MassDamperSpringAnalyticalSolution::SetInitialValue
          struct /* SetInitialValue */ {
            // Source: drake/multibody/benchmarks/mass_damper_spring/mass_damper_spring_analytical_solution.h
            const char* doc =
R"""(Sets the initial values of x and ẋ for ``this`` system.

Parameter ``x0``:
    Initial value of x (value of x at time t = 0).

Parameter ``xDt0``:
    Initial value of ẋ (value of ẋ at time t = 0).)""";
          } SetInitialValue;
          // Symbol: drake::multibody::benchmarks::MassDamperSpringAnalyticalSolution::get_x
          struct /* get_x */ {
            // Source: drake/multibody/benchmarks/mass_damper_spring/mass_damper_spring_analytical_solution.h
            const char* doc =
R"""(Returns x (Nx measure of Q's position from No) at time t.)""";
          } get_x;
          // Symbol: drake::multibody::benchmarks::MassDamperSpringAnalyticalSolution::get_xDt
          struct /* get_xDt */ {
            // Source: drake/multibody/benchmarks/mass_damper_spring/mass_damper_spring_analytical_solution.h
            const char* doc =
R"""(Returns ẋ (Nx measure of Q's velocity in N) at time t.)""";
          } get_xDt;
          // Symbol: drake::multibody::benchmarks::MassDamperSpringAnalyticalSolution::get_xDtDt
          struct /* get_xDtDt */ {
            // Source: drake/multibody/benchmarks/mass_damper_spring/mass_damper_spring_analytical_solution.h
            const char* doc =
R"""(Returns ẍ (Nx measure of Q's acceleration in N) at time t.)""";
          } get_xDtDt;
        } MassDamperSpringAnalyticalSolution;
      } benchmarks;
    } multibody;
  } drake;
} pydrake_doc_multibody_benchmarks_mass_damper_spring;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
