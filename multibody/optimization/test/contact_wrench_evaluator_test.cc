#include "drake/multibody/optimization/contact_wrench_evaluator.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/optimization/test/optimization_with_contact_utilities.h"

namespace drake {
namespace multibody {
namespace {
class ContactWrenchEvaluatorTest : public ::testing::Test {
 public:
  ContactWrenchEvaluatorTest() {
    std::vector<test::SphereSpecification> spheres;
    spheres.emplace_back(0.1, 1E3, CoulombFriction<double>(0.5, 0.4));
    spheres.emplace_back(0.2, 2E3, CoulombFriction<double>(1, 0.9));
    spheres.emplace_back(0.3, 2E3, CoulombFriction<double>(1, 0.8));
    std::vector<test::BoxSpecification> boxes;

    const CoulombFriction<double> ground_friction(1.5, 0.9);
    free_spheres_ = std::make_unique<test::FreeSpheresAndBoxes<AutoDiffXd>>(
        spheres, boxes, ground_friction);
  }

 protected:
  // Only add free spheres, no boxes yet, as we can't compute the signed
  // distance between boxes with high precision yet.
  std::unique_ptr<test::FreeSpheresAndBoxes<AutoDiffXd>> free_spheres_;
};

TEST_F(ContactWrenchEvaluatorTest,
       ContactWrenchFromForceInWorldFrameEvaluator) {
  // Test the constructor, `ComposeVariableValues` and `Eval` functions.
  systems::Context<AutoDiffXd>* plant_context =
      free_spheres_->get_mutable_plant_context();
  ContactWrenchFromForceInWorldFrameEvaluator evaluator(
      &(free_spheres_->plant()), plant_context,
      SortedPair<geometry::GeometryId>(
          free_spheres_->sphere_geometry_ids()[0],
          free_spheres_->sphere_geometry_ids()[1]));

  EXPECT_EQ(evaluator.num_lambda(), 3);
  EXPECT_EQ(evaluator.num_vars(), free_spheres_->plant().num_positions() + 3);
  EXPECT_EQ(evaluator.num_outputs(), 6);

  const Vector3<AutoDiffXd> lambda_val =
      math::InitializeAutoDiff(Eigen::Vector3d(1, 2, 3));
  const VectorX<AutoDiffXd> x_value =
      evaluator.ComposeVariableValues(*plant_context, lambda_val);
  AutoDiffVecXd y;
  evaluator.Eval(x_value, &y);
  EXPECT_EQ(math::ExtractValue(y.head<3>()), Eigen::Vector3d::Zero());
  EXPECT_EQ(y.tail<3>(), lambda_val);
  EXPECT_TRUE(CompareMatrices(math::ExtractGradient(y.tail<3>()),
                              math::ExtractGradient(lambda_val)));
}
}  // namespace
}  // namespace multibody
}  // namespace drake
