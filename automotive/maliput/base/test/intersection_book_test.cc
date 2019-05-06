#include "drake/automotive/maliput/base/intersection_book.h"

#include <exception>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/base/intersection.h"
#include "drake/automotive/maliput/base/manual_phase_provider.h"

namespace drake {
namespace maliput {
namespace {

using api::rules::LaneSRange;

GTEST_TEST(IntersectionBookTest, BasicTest) {
  ManualPhaseProvider phase_provider;
  const Intersection::Id id("my intersection");
  const std::vector<api::rules::LaneSRange> region;
  auto intersection = std::make_unique<Intersection>(
      id, region, api::rules::PhaseRing::Id("phase ring"), &phase_provider);
  const Intersection* intersection_ptr = intersection.get();
  IntersectionBook dut;
  EXPECT_THROW(dut.AddIntersection(nullptr), std::exception);
  dut.AddIntersection(std::move(intersection));
  EXPECT_EQ(dut.GetIntersection(Intersection::Id("unknown")), nullptr);
  EXPECT_EQ(dut.GetIntersection(id), intersection_ptr);
}

}  // namespace
}  // namespace maliput
}  // namespace drake
