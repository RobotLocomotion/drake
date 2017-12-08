#pragma once

#include <gtest/gtest.h>

#include "drake/multibody/rigid_body.h"
#include "drake/multibody/shapes/drake_shapes.h"

namespace drake {
namespace examples {
namespace double_pendulum {
namespace test {


/// Checks if @p elem1 and @p elem2 are equivalent, where equivalence
/// means equality of elements' geometries and local transforms.
/// @return ::testing::AssertionSuccess if elements are
/// equivalent,  otherwise ::testing::AssertionFailure.
::testing::AssertionResult AreElementsEquivalent(
     const DrakeShapes::Element& elem1,
     const DrakeShapes::Element& elem2);


/// Checks if @p body1 and @p body2 are equivalent, where equivalence means
/// equality of bodies' naming, inertial characteritics, collisions and visuals.
/// @return ::testing::AssertionSuccess if bodies are
/// equivalent,  otherwise ::testing::AssertionFailure.
::testing::AssertionResult AreBodiesEquivalent(
     const RigidBody<double>& body1,
     const RigidBody<double>& body2);

}  // namespace test
}  // namespace double_pendulum
}  // namespace examples
}  // namespace drake
