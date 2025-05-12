#include "drake/planning/iris/iris_np2.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/planning/iris/iris_common.h"
#include "drake/planning/iris/test/iris_test_utilities.h"

namespace drake {
namespace planning {
namespace {

using Eigen::VectorX;
using geometry::Sphere;
using geometry::optimization::HPolyhedron;
using symbolic::Expression;
using symbolic::Variable;

// Reproduced from the IrisInConfigurationSpace unit tests.
TEST_F(JointLimits1D, JointLimitsBasic) {
  IrisNp2Options options;
  auto sgcc_ptr = dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(sgcc_ptr != nullptr);
  HPolyhedron region =
      IrisNp2(*sgcc_ptr, starting_ellipsoid_, domain_, options);
  CheckRegion(region);
}

// Check unsupported features.
TEST_F(JointLimits1D, UnsupportedOptions) {
  IrisNp2Options options;
  auto sgcc_ptr = dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(sgcc_ptr != nullptr);

  options.sampled_iris_options.containment_points = Eigen::MatrixXd::Zero(1, 3);
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisNp2(*sgcc_ptr, starting_ellipsoid_, domain_, options),
      ".*additional containment points.*");
  options.sampled_iris_options.containment_points = std::nullopt;

  solvers::MathematicalProgram prog;
  options.sampled_iris_options.prog_with_additional_constraints = &prog;
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisNp2(*sgcc_ptr, starting_ellipsoid_, domain_, options),
      ".*additional constriants.*");
  options.sampled_iris_options.prog_with_additional_constraints = nullptr;

  VectorX<Variable> varable_vector(1);
  VectorX<Expression> expression_vector(1);
  expression_vector[0] = varable_vector[0] + 1;
  options.parameterization =
      IrisParameterizationFunction(expression_vector, varable_vector);
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisNp2(*sgcc_ptr, starting_ellipsoid_, domain_, options),
      ".*parameterized subspace.*");
  options.parameterization = IrisParameterizationFunction();

  const Sphere sphere(0.1);
  const BodyShapeDescription body_shape{sphere, {}, "limits", "movable"};
  sgcc_ptr->AddCollisionShape("test", body_shape);
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisNp2(*sgcc_ptr, starting_ellipsoid_, domain_, options),
      ".*added collision shapes.*");
  sgcc_ptr->RemoveAllAddedCollisionShapes();
}

// Check padding as an unsupported feature. (We have to use a test environment
// with multiple collision geometries.)
TEST_F(DoublePendulum, PaddingUnsupported) {
  IrisNp2Options options;
  auto sgcc_ptr = dynamic_cast<SceneGraphCollisionChecker*>(checker_.get());
  ASSERT_TRUE(sgcc_ptr != nullptr);

  sgcc_ptr->SetPaddingAllRobotEnvironmentPairs(1.0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      IrisNp2(*sgcc_ptr, starting_ellipsoid_, domain_, options),
      ".*nonzero padding.*");
}

}  // namespace
}  // namespace planning
}  // namespace drake
