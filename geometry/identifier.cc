#include "drake/geometry/identifier.h"

namespace {

// Ensure that the deprecated header still compiles.
using FooId = drake::geometry::Identifier<class FooTag>;

}  // namespace
