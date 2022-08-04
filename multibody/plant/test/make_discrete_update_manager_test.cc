#include "drake/multibody/plant/make_discrete_update_manager.h"

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

  DRAKE_EXPECT_THROWS_MESSAGE(MakeDiscreteUpdateManager<symbolic::Expression>(
                                  DiscreteContactSolver::kSap),
                              "SAP.*not supported.*symbolic::Expression.");
}

GTEST_TEST(MakeDiscreteUpdateManagerTest, Tamsi) {
  EXPECT_EQ(nullptr,
            MakeDiscreteUpdateManager<double>(DiscreteContactSolver::kTamsi));
  EXPECT_EQ(nullptr, MakeDiscreteUpdateManager<AutoDiffXd>(
                         DiscreteContactSolver::kTamsi));
  EXPECT_EQ(nullptr, MakeDiscreteUpdateManager<symbolic::Expression>(
                         DiscreteContactSolver::kTamsi));
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
