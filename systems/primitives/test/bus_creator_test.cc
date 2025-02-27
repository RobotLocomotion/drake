#include "drake/systems/primitives/bus_creator.h"

#include <gtest/gtest.h>

#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace systems {
namespace {

GTEST_TEST(BusCreatorTest, Construction) {
  BusCreator<double> dut;
}

}  // namespace
}  // namespace systems
}  // namespace drake
