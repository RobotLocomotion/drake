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

// #include "drake/multibody/triangle_quadrature/gaussian_triangle_quadrature_rule.h"
// #include "drake/multibody/triangle_quadrature/triangle_quadrature.h"
// #include "drake/multibody/triangle_quadrature/triangle_quadrature_rule.h"

// Symbol: pydrake_doc_multibody_triangle_quadrature
constexpr struct /* pydrake_doc_multibody_triangle_quadrature */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::multibody
    struct /* multibody */ {
      // Symbol: drake::multibody::GaussianTriangleQuadratureRule
      struct /* GaussianTriangleQuadratureRule */ {
        // Source: drake/multibody/triangle_quadrature/gaussian_triangle_quadrature_rule.h:11
        const char* doc = R"""()""";
        // Symbol: drake::multibody::GaussianTriangleQuadratureRule::GaussianTriangleQuadratureRule
        struct /* ctor */ {
          // Source: drake/multibody/triangle_quadrature/gaussian_triangle_quadrature_rule.h:15
          const char* doc =
R"""(Constructs the Gaussian quadrature rule of the specified order, which
must be between 1 and 5.)""";
        } ctor;
      } GaussianTriangleQuadratureRule;
      // Symbol: drake::multibody::TriangleQuadrature
      struct /* TriangleQuadrature */ {
        // Source: drake/multibody/triangle_quadrature/triangle_quadrature.h:25
        const char* doc =
R"""(A class for integrating a function using numerical quadrature over
triangular domains.

Template parameter ``NumericReturnType``:
    the output type of the function being integrated. Commonly will be
    a IEEE floating point scalar (e.g., ``double``), but could be an
    Eigen::VectorXd, a multibody::SpatialForce, or any other numeric
    type that supports both scalar multiplication (i.e.,
    operator*(const NumericReturnType&, double) and addition with
    another of the same type (i.e., operator+(const
    NumericReturnType&, const NumericReturnType&)).

Template parameter ``T``:
    the scalar type of the function being integrated over. Supported
    types are currently only IEEE floating point scalars.)""";
        // Symbol: drake::multibody::TriangleQuadrature::Integrate
        struct /* Integrate */ {
          // Source: drake/multibody/triangle_quadrature/triangle_quadrature.h:34
          const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Numerically integrates the function f over a triangle using the given
quadrature rule and the initial value.

Parameter ``f``:
    a function f(p) that returns a numerical value for point p in the
    domain of the triangle, specified in barycentric coordinates. The
    barycentric coordinates are given by (p[0], p[1], 1 - p[0] -
    p[1]).

Parameter ``area``:
    the area of the triangle.)""";
        } Integrate;
      } TriangleQuadrature;
      // Symbol: drake::multibody::TriangleQuadratureRule
      struct /* TriangleQuadratureRule */ {
        // Source: drake/multibody/triangle_quadrature/triangle_quadrature_rule.h:12
        const char* doc =
R"""(A "rule" (weights and quadrature points) for computing quadrature over
triangular domains.)""";
        // Symbol: drake::multibody::TriangleQuadratureRule::TriangleQuadratureRule
        struct /* ctor */ {
          // Source: drake/multibody/triangle_quadrature/triangle_quadrature_rule.h:14
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::multibody::TriangleQuadratureRule::do_order
        struct /* do_order */ {
          // Source: drake/multibody/triangle_quadrature/triangle_quadrature_rule.h:38
          const char* doc =
R"""(Derived classes shall return the order (>= 1) of this rule.)""";
        } do_order;
        // Symbol: drake::multibody::TriangleQuadratureRule::do_quadrature_points
        struct /* do_quadrature_points */ {
          // Source: drake/multibody/triangle_quadrature/triangle_quadrature_rule.h:44
          const char* doc =
R"""(Derived classes shall return the vector of quadrature points. Each of
these Vector2 objects represents the barycentric coordinates of a
triangle (the third barycentric coordinate is implicit: it is the
difference between unity and the sum of the other two coordinates).)""";
        } do_quadrature_points;
        // Symbol: drake::multibody::TriangleQuadratureRule::do_weights
        struct /* do_weights */ {
          // Source: drake/multibody/triangle_quadrature/triangle_quadrature_rule.h:48
          const char* doc =
R"""(Derived classes shall return the vector of weights. The sum of all
weights must equal 1.0.)""";
        } do_weights;
        // Symbol: drake::multibody::TriangleQuadratureRule::order
        struct /* order */ {
          // Source: drake/multibody/triangle_quadrature/triangle_quadrature_rule.h:19
          const char* doc = R"""(Returns the order of this rule.)""";
        } order;
        // Symbol: drake::multibody::TriangleQuadratureRule::quadrature_points
        struct /* quadrature_points */ {
          // Source: drake/multibody/triangle_quadrature/triangle_quadrature_rule.h:28
          const char* doc =
R"""(Returns the vector of quadrature points. These are returned as the
first two barycentric coordinates b0 b1; the third is just b2 = 1 - b0
- b1. Each of these has a corresponding weight returned by weights().)""";
        } quadrature_points;
        // Symbol: drake::multibody::TriangleQuadratureRule::weights
        struct /* weights */ {
          // Source: drake/multibody/triangle_quadrature/triangle_quadrature_rule.h:34
          const char* doc =
R"""(Returns the vector of weights. These sum to 1 and there is one weight
for each point returned by quadrature_points().)""";
        } weights;
      } TriangleQuadratureRule;
    } multibody;
  } drake;
} pydrake_doc_multibody_triangle_quadrature;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
