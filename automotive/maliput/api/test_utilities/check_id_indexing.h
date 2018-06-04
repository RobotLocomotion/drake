#pragma once

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/road_geometry.h"

namespace drake {
namespace maliput {
namespace api {
namespace test {

/// Walks the object graph of @p road_geometry and checks that every
/// component can be found via `ById().Get*(const *Id& id)` methods.
///
/// @return ::testing::AssertionSuccess() when all objects are found,
/// otherwise ::testing::AssertionFailure().
::testing::AssertionResult CheckIdIndexing(const RoadGeometry* road_geometry);

}  // namespace test
}  // namespace api
}  // namespace maliput
}  // namespace drake
