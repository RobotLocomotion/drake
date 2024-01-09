#include "drake/systems/primitives/delay_line.h"

#include <string>
#include <type_traits>

#include <gtest/gtest.h>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"
#include "drake/systems/primitives/sine.h"

namespace drake {
namespace systems {
namespace {

using Eigen::Vector2d;

const int kVectorSize = 2;
const int kNumSamples = 3;
const double kPeriod = 0.25;
const double kOffset = 0.125;

// The test fixture is parameterized on two characteristics:
// - The scalar type `T`.
// - bool is_abstract`.
template <typename Pair>
class DelayLineTest : public ::testing::Test {
 protected:
  using T = typename Pair::first_type;

  void SetUp() override {
    if (is_abstract_) {
      dut_ = std::make_unique<DelayLine<T>>(Value<std::string>("default"),
                                            kNumSamples, kPeriod, kOffset);
    } else {
      dut_ = std::make_unique<DelayLine<T>>(kVectorSize, kNumSamples, kPeriod,
                                            kOffset);
    }
  }

  PeriodicEventData GetPeriodicEventData() const {
    const auto map = dut_->MapPeriodicEventsByTiming();
    EXPECT_EQ(map.size(), 1);
    if (map.empty()) {
      return {};
    }
    const auto& [schedule, events] = *map.begin();
    EXPECT_EQ(events.size(), 1);
    return schedule;
  }

  const bool is_abstract_{Pair::second_type::value};
  std::unique_ptr<DelayLine<T>> dut_;
};

using AllTypes =
    ::testing::Types<std::pair<double, std::false_type>,
                     std::pair<double, std::true_type>,
                     std::pair<AutoDiffXd, std::false_type>,
                     std::pair<AutoDiffXd, std::true_type>,
                     std::pair<symbolic::Expression, std::false_type>,
                     std::pair<symbolic::Expression, std::true_type>>;
TYPED_TEST_SUITE(DelayLineTest, AllTypes);

// Tests that the constructor establishes the proper attributes.
TYPED_TEST(DelayLineTest, CtorAndGetters) {
  const auto& dut = *this->dut_;
  EXPECT_EQ(dut.num_input_ports(), 1);
  EXPECT_EQ(dut.num_output_ports(), 1);
  EXPECT_FALSE(dut.HasAnyDirectFeedthrough());
  EXPECT_EQ(dut.num_samples(), kNumSamples);
  EXPECT_EQ(dut.period(), kPeriod);
  EXPECT_EQ(dut.offset(), kOffset);
  EXPECT_EQ(this->GetPeriodicEventData().period_sec(), kPeriod);
  EXPECT_EQ(this->GetPeriodicEventData().offset_sec(), kOffset);
}

// XXX check more stuff
TYPED_TEST(DelayLineTest, ToAutoDiff) {
  using Scalar = typename TypeParam::first_type;
  if constexpr (std::is_same_v<Scalar, double>) {
    const auto& dut = *this->dut_;
    EXPECT_TRUE(is_autodiffxd_convertible(dut));
  }
}

TYPED_TEST(DelayLineTest, ToSymbolic) {
  using Scalar = typename TypeParam::first_type;
  if constexpr (std::is_same_v<Scalar, double>) {
    const auto& dut = *this->dut_;
    EXPECT_TRUE(is_symbolic_convertible(dut));
  }
}

namespace {

template <typename T>
class ToStringSystem final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ToStringSystem);
  ToStringSystem() = default;
};

}  // namespace

#if 0
// Tests that the simulation output is as expected.
TYPED_TEST(DelayLineTest, Output) {
  DiagramBuilder<T> builder;

  const Vector2d amplitude{10.0, 10.0};
  const Vector2d frequency{2.0 * M_PI, 2.0 * M_PI};
  const Vector2d phase{M_PI / 4, M_PI / 4};
  auto sine = builder.AddSystem<Sine<T>>(amplitude, frequency, phase);
  auto dut = builder.AddSystem(std::move(dut_));
  if (is_abstract_) {
    auto converter = builder.AddSystem<ToStringSystem<T>>();
    builder.Connect(sine->get_output_port(0), converter->get_input_port());
    builder.Connect(converter->get_output_port(), dut->get_input_port());
  } else {
    builder.Connect(sine->get_output_port(0), dut->get_input_port());
  }
}
#endif

}  // namespace
}  // namespace systems
}  // namespace drake
