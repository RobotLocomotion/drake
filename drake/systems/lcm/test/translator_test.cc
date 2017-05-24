#include "drake/systems/lcm/translator.h"

#include <gtest/gtest.h>

namespace drake {
namespace systems {
namespace lcm {
namespace {

GTEST_TEST(TranslatorTest, Ha) {
  MyLcmtDrakeSignalTranslator dut(1);
}

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake
