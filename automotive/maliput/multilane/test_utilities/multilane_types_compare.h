#pragma once

#include <gtest/gtest.h>

#include "drake/automotive/maliput/multilane/builder.h"

namespace drake {
namespace maliput {
namespace multilane {
namespace test {

// Compares equality within @p tolerance deviation of two EndpointXy objects.
// @param xy1 An EndpointXy object to compare.
// @param xy2 An EndpointXy object to compare.
// @param tolerance An allowable absolute deviation for each EndpointXy's
// coordinate.
// @return ::testing::AssertionFailure() When EndpointXy objects are different.
// @return ::testing::AssertionSuccess() When EndpointXy objects are within
// the @p tolerance deviation.

// TODO(agalbachicar)    Given that EndpointXy is composed of two lengths and
//                       one angle, tolerance must be replaced into two distinct
//                       values to match each magnitude.
::testing::AssertionResult IsEndpointXyClose(const EndpointXy& xy1,
                                             const EndpointXy& xy2,
                                             double tolerance);

// Compares equality within @p tolerance deviation of two EndpointZ objects.
// @param z1 An EndpointZ object to compare.
// @param z2 An EndpointZ object to compare.
// @param tolerance An allowable absolute deviation for each EndpointZ's
// coordinate.
// @return ::testing::AssertionFailure() When EndpointZ objects are different.
// @return ::testing::AssertionSuccess() When EndpointZ objects are within
// the @p tolerance deviation.

// TODO(agalbachicar)    Given that EndpointZ is composed of different
//                       magnitudes, tolerance must be replaced into distinct
//                       values to match each magnitude.
::testing::AssertionResult IsEndpointZClose(const EndpointZ& z1,
                                            const EndpointZ& z2,
                                            double tolerance);

// Compares equality within @p tolerance deviation of two Endpoint objects.
// @param pos1 An Endpoint object to compare.
// @param pos2 An Endpoint object to compare.
// @param tolerance An allowable absolute deviation for each Endpoint's
// coordinate.
// @return ::testing::AssertionFailure() When Endpoint objects are different.
// @return ::testing::AssertionSuccess() When Endpoint objects are within
// the @p tolerance deviation.

// TODO(agalbachicar)    Given that EndpointXy and EndpointZ are composed of
//                       different magnitudes, tolerance must be replaced into
//                       distinct values to match each magnitude.
::testing::AssertionResult IsEndpointClose(const Endpoint& pos1,
                                           const Endpoint& pos2,
                                           double tolerance);
}  // namespace test
}  // namespace multilane
}  // namespace maliput
}  // namespace drake
