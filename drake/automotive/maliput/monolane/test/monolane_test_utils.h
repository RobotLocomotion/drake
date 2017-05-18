#pragma once

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/lane_data.h"

namespace drake {
namespace maliput {
namespace monolane {

#define EXPECT_GEO_NEAR(actual_arg, expected_arg, tolerance_arg) \
  do {                                                           \
    const api::GeoPosition actual(actual_arg);                   \
    const api::GeoPosition expected expected_arg;                \
    const double tolerance = (tolerance_arg);                    \
    EXPECT_NEAR(actual.x(), expected.x(), tolerance);            \
    EXPECT_NEAR(actual.y(), expected.y(), tolerance);            \
    EXPECT_NEAR(actual.z(), expected.z(), tolerance);            \
  } while (0)

#define EXPECT_LANE_NEAR(actual_arg, expected_arg, tolerance_arg) \
  do {                                                            \
    const api::LanePosition actual(actual_arg);                   \
    const api::LanePosition expected expected_arg;                \
    const double tolerance = (tolerance_arg);                     \
    EXPECT_NEAR(actual.s(), expected.s(), tolerance);             \
    EXPECT_NEAR(actual.r(), expected.r(), tolerance);             \
    EXPECT_NEAR(actual.h(), expected.h(), tolerance);             \
  } while (0)

#define EXPECT_ROT_NEAR(actual_arg, expected_arg, tolerance_arg) \
  do {                                                           \
    const api::Rotation actual(actual_arg);                      \
    const api::Rotation expected(api::Rotation::FromRpy expected_arg); \
    const double tolerance = (tolerance_arg);                    \
    EXPECT_NEAR(actual.yaw(), expected.yaw(), tolerance);        \
    EXPECT_NEAR(actual.pitch(), expected.pitch(), tolerance);    \
    EXPECT_NEAR(actual.roll(), expected.roll(), tolerance);      \
  } while (0)

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
