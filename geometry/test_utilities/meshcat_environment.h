#pragma once

#include <memory>

#include "drake/geometry/meshcat.h"

namespace drake {
namespace geometry {

/** Returns a singleton Meshcat object to be used during unit tests. The same
object is used for all test cases. If test code needs to reset the scene
between tests, it must do so explicitly. */
std::shared_ptr<Meshcat> GetTestEnvironmentMeshcat();

}  // namespace geometry
}  // namespace drake
