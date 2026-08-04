#include "drake/multibody/plant/discrete_step_memory.h"

#include <utility>

#include <gtest/gtest.h>

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
class MultibodyPlantTester {
 public:
  MultibodyPlantTester() = delete;
  static const internal::MultibodyTree<double>& internal_tree(
      const MultibodyPlant<double>& plant) {
    return plant.internal_tree();
  }
};
namespace internal {
namespace {

GTEST_TEST(DiscreteStepMemoryTest, Lifecycle) {
  MultibodyPlant<double> plant{0.01};
  const internal::MultibodyTree<double>& tree =
      MultibodyPlantTester::internal_tree(plant);
  plant.Finalize();

  // Initially empty.
  DiscreteStepMemory dut;
  EXPECT_EQ(dut.template get<double>(), nullptr);

  // Allocation matches get().
  const auto& data = dut.template Allocate<double>(tree.forest());
  EXPECT_EQ(dut.template get<double>(), &data);

  // Access the wrong type yields empty.
  EXPECT_EQ(dut.template get<AutoDiffXd>(), nullptr);

  // Moved from values are also empty, even with the right type.
  DiscreteStepMemory moved_to{std::move(dut)};
  EXPECT_EQ(moved_to.template get<double>(), &data);
  EXPECT_EQ(dut.template get<double>(), nullptr);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
