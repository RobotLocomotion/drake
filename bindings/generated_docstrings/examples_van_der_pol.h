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

// #include "drake/examples/van_der_pol/van_der_pol.h"

// Symbol: pydrake_doc_examples_van_der_pol
constexpr struct /* pydrake_doc_examples_van_der_pol */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::examples
    struct /* examples */ {
      // Symbol: drake::examples::van_der_pol
      struct /* van_der_pol */ {
        // Symbol: drake::examples::van_der_pol::VanDerPolOscillator
        struct /* VanDerPolOscillator */ {
          // Source: drake/examples/van_der_pol/van_der_pol.h
          const char* doc =
R"""(van der Pol oscillator

The van der Pol oscillator, governed by the following equations: q̈ +
μ(q² - 1)q̇ + q = 0, μ > 0 y₀ = q y₁ = [q,q̇]' is a canonical example
of a nonlinear system that exhibits a limit cycle stability. As such
it serves as an important for examining nonlinear stability and
stochastic stability.

(Examples involving region of attraction analysis and analyzing the
stationary distribution of the oscillator under process noise are
coming soon).

.. pydrake_system::

    name: VanDerPolOscillator
    output_ports:
    - y0
    - y1)""";
          // Symbol: drake::examples::van_der_pol::VanDerPolOscillator::CalcLimitCycle
          struct /* CalcLimitCycle */ {
            // Source: drake/examples/van_der_pol/van_der_pol.h
            const char* doc =
R"""(Returns a 2-row matrix containing the result of simulating the
oscillator with the default mu=1 from (approximately) one point on the
limit cycle for (approximately) one period. The first row is q, and
the second row is q̇.)""";
          } CalcLimitCycle;
          // Symbol: drake::examples::van_der_pol::VanDerPolOscillator::VanDerPolOscillator<T>
          struct /* ctor */ {
            // Source: drake/examples/van_der_pol/van_der_pol.h
            const char* doc = R"""(Constructs a default oscillator.)""";
            // Source: drake/examples/van_der_pol/van_der_pol.h
            const char* doc_copyconvert = R"""(Scalar-converting copy constructor.)""";
          } ctor;
          // Symbol: drake::examples::van_der_pol::VanDerPolOscillator::get_full_state_output_port
          struct /* get_full_state_output_port */ {
            // Source: drake/examples/van_der_pol/van_der_pol.h
            const char* doc =
R"""(Returns the output port containing the full state. This is provided
primarily as a tool for debugging/visualization.)""";
          } get_full_state_output_port;
          // Symbol: drake::examples::van_der_pol::VanDerPolOscillator::get_position_output_port
          struct /* get_position_output_port */ {
            // Source: drake/examples/van_der_pol/van_der_pol.h
            const char* doc =
R"""(Returns the output port containing the output configuration (only).)""";
          } get_position_output_port;
        } VanDerPolOscillator;
      } van_der_pol;
    } examples;
  } drake;
} pydrake_doc_examples_van_der_pol;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
