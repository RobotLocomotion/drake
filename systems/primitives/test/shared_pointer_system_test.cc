#include "common/unique_ptr_system.h"

#include <string>
#include <string_view>

#include <gtest/gtest.h>

namespace anzu {
namespace common {
namespace {

using drake::systems::DiagramBuilder;
using drake::systems::System;

GTEST_TEST(SharedPointerSystemTest, CtorFromUnique) {
  auto held = std::make_unique<std::string>("held");
  auto dut = std::make_unique<SharedPointerSystem<double>>(std::move(held));
  std::string* gotten = dut->get<std::string>();
  ASSERT_NE(gotten, nullptr);
  EXPECT_EQ(*gotten, "held");
}

GTEST_TEST(SharedPointerSystemTest, CtorFromShared) {
  auto held = std::make_shared<std::string>("held");
  auto dut = std::make_unique<SharedPointerSystem<double>>(std::move(held));
  std::string* gotten = dut->get<std::string>();
  ASSERT_NE(gotten, nullptr);
  EXPECT_EQ(*gotten, "held");
}

GTEST_TEST(SharedPointerSystemTest, BuilderFromUnique) {
  auto held = std::make_unique<std::string>("held");
  auto builder = std::make_unique<DiagramBuilder<double>>();
  std::string* gotten = SharedPointerSystem<double>::AddToBuilder(
      builder.get(), std::move(held));
  auto diagram = builder->Build();
  ASSERT_NE(gotten, nullptr);
  EXPECT_EQ(*gotten, "held");
}

GTEST_TEST(SharedPointerSystemTest, BuilderFromShared) {
  auto held = std::make_shared<std::string>("held");
  auto builder = std::make_unique<DiagramBuilder<double>>();
  std::string* gotten = SharedPointerSystem<double>::AddToBuilder(
      builder.get(), std::move(held));
  auto diagram = builder->Build();
  ASSERT_NE(gotten, nullptr);
  EXPECT_EQ(*gotten, "held");
}

GTEST_TEST(SharedPointerSystemTest, ScalarConversion) {
  auto dut1 = std::make_unique<SharedPointerSystem<double>>(
      std::make_unique<std::string>("held"));
  auto dut2 = System<double>::ToSymbolic(*dut1);
  ASSERT_NE(dut2, nullptr);
  dut1.release();
  std::string* gotten = dut2->get<std::string>();
  ASSERT_NE(gotten, nullptr);
  EXPECT_EQ(*gotten, "held");
}

// Ensure that type-checking guards operate correctly.
GTEST_TEST(SharedPointerSystemTest, BadCast) {
  auto held = std::make_shared<std::string>("held");
  auto dut = std::make_unique<SharedPointerSystem<double>>(std::move(held));
  EXPECT_THROW(dut->get<std::string_view>(), std::bad_cast);
}

}  // namespace
}  // namespace common
}  // namespace anzu
