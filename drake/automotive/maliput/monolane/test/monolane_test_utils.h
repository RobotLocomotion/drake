#pragma once

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/lane_data.h"

namespace drake {
namespace maliput {
namespace monolane {

#define EXPECT_GEO_NEAR(actual, expected, tolerance)         \
  do {                                                       \
    const api::GeoPosition _actual(actual);                  \
    const api::GeoPosition _expected expected;               \
    const double _tolerance = (tolerance);                   \
    EXPECT_NEAR(_actual.x(), _expected.x(), _tolerance);     \
    EXPECT_NEAR(_actual.y(), _expected.y(), _tolerance);     \
    EXPECT_NEAR(_actual.z(), _expected.z(), _tolerance);     \
  } while (0)

#define EXPECT_LANE_NEAR(actual, expected, tolerance)         \
  do {                                                        \
    const api::LanePosition _actual(actual);                  \
    const api::LanePosition _expected expected;               \
    const double _tolerance = (tolerance);                    \
    EXPECT_NEAR(_actual.s(), _expected.s(), _tolerance);      \
    EXPECT_NEAR(_actual.r(), _expected.r(), _tolerance);      \
    EXPECT_NEAR(_actual.h(), _expected.h(), _tolerance);      \
  } while (0)

#define EXPECT_ROT_NEAR(actual, expected, tolerance)                 \
  do {                                                               \
    const api::Rotation _actual(actual);                             \
    const api::Rotation _expected expected;                          \
    const double _tolerance = (tolerance);                           \
    EXPECT_NEAR(_actual.yaw, _expected.yaw, _tolerance);             \
    EXPECT_NEAR(_actual.pitch, _expected.pitch, _tolerance);         \
    EXPECT_NEAR(_actual.roll, _expected.roll, _tolerance);           \
  } while (0)

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
