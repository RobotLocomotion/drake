#include "drake/systems/framework/primitives/cascade3.h"

#include <memory>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"

namespace drake {
namespace systems {
namespace {

class CascadeTest : public ::testing::Test {
 protected:
  void SetUp() override {}
};

TEST_F(CascadeTest, Test1) {}

}  // namespace
}  // namespace systems
}  // namespace drake
