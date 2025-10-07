#include "drake/multibody/plant/make_discrete_update_manager.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/is_dynamic_castable.h"
#include "drake/multibody/plant/compliant_contact_manager.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

GTEST_TEST(MakeDiscreteUpdateManagerTest, Sap) {
  std::unique_ptr<DiscreteUpdateManager<double>> double_manager =
      MakeDiscreteUpdateManager<double>(DiscreteContactSolver::kSap);
  EXPECT_TRUE(
      is_dynamic_castable<CompliantContactManager<double>>(double_manager));

  std::unique_ptr<DiscreteUpdateManager<AutoDiffXd>> autodiff_manager =
      MakeDiscreteUpdateManager<AutoDiffXd>(DiscreteContactSolver::kSap);
  EXPECT_TRUE(is_dynamic_castable<CompliantContactManager<AutoDiffXd>>(
      autodiff_manager));

  std::unique_ptr<DiscreteUpdateManager<symbolic::Expression>>
      symbolic_manager = MakeDiscreteUpdateManager<symbolic::Expression>(
          DiscreteContactSolver::kSap);
  EXPECT_TRUE(
      is_dynamic_castable<CompliantContactManager<symbolic::Expression>>(
          symbolic_manager));
}

GTEST_TEST(MakeDiscreteUpdateManagerTest, Tamsi) {
  std::unique_ptr<DiscreteUpdateManager<double>> double_manager =
      MakeDiscreteUpdateManager<double>(DiscreteContactSolver::kTamsi);
  EXPECT_TRUE(
      is_dynamic_castable<CompliantContactManager<double>>(double_manager));

  std::unique_ptr<DiscreteUpdateManager<AutoDiffXd>> autodiff_manager =
      MakeDiscreteUpdateManager<AutoDiffXd>(DiscreteContactSolver::kTamsi);
  EXPECT_TRUE(is_dynamic_castable<CompliantContactManager<AutoDiffXd>>(
      autodiff_manager));

  std::unique_ptr<DiscreteUpdateManager<symbolic::Expression>>
      symbolic_manager = MakeDiscreteUpdateManager<symbolic::Expression>(
          DiscreteContactSolver::kTamsi);
  EXPECT_TRUE(
      is_dynamic_castable<CompliantContactManager<symbolic::Expression>>(
          symbolic_manager));
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
