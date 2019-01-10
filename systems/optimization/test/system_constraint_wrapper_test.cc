#include "drake/systems/optimization/system_constraint_wrapper.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/optimization/test/system_optimization_test_util.h"

namespace drake {
namespace systems {
namespace {
const double kInf = std::numeric_limits<double>::infinity();
const double kEps = std::numeric_limits<double>::epsilon();
// val contains [x(0); p0; x(2)]
template <typename T>
void Selector1(const System<T>&, const Eigen::Ref<const VectorX<T>>& val,
               Context<T>* context) {
  context->get_mutable_continuous_state_vector().SetAtIndex(0, val(0));
  context->get_mutable_continuous_state_vector().SetAtIndex(2, val(2));
  context->get_mutable_numeric_parameter(0).SetAtIndex(0, val(1));
}

// Just a simple test to call the Eval function.
GTEST_TEST(SystemConstraintWrapperTest, BasicTest) {
  DummySystem<double> system_double;
  auto context_double = system_double.CreateDefaultContext();
  // Set p0 = 2.
  context_double->get_mutable_numeric_parameter(0).set_value(Vector1d(2));
  // Set x = [3, 4, 5]
  context_double->get_mutable_continuous_state_vector().SetFromVector(
      Eigen::Vector3d(3, 4, 5));

  auto system_autodiff = system_double.ToAutoDiffXd();
  auto context_autodiff = system_autodiff->CreateDefaultContext();
  context_autodiff->SetTimeStateAndParametersFrom(*context_double);

  SystemConstraintWrapper constraint(
      &system_double, system_autodiff.get(), system_double.constraint_index(),
      *context_double, Selector1<double>, Selector1<AutoDiffXd>, 3);
  EXPECT_TRUE(CompareMatrices(constraint.lower_bound(), Eigen::Vector2d(2, 0)));
  EXPECT_TRUE(
      CompareMatrices(constraint.upper_bound(), Eigen::Vector2d(kInf, kInf)));

  // [x(0); p0; x(2)] = val = [10, 11, 12]
  const Eigen::Vector3d val(10, 11, 12);
  Eigen::VectorXd y;
  constraint.Eval(val, &y);

  Selector1<double>(system_double, val, context_double.get());
  Eigen::VectorXd y_expected;
  DummySystemConstraintCalc<double>(*context_double, &y_expected);
  const double tol = 3 * kEps;
  EXPECT_TRUE(CompareMatrices(y, y_expected, tol));

  Eigen::Matrix3Xd val_gradient(3, 2);
  val_gradient << 1, 2, 3, 4, 5, 6;
  const auto val_autodiff = math::initializeAutoDiffGivenGradientMatrix(
      Eigen::Vector3d(10, 11, 12), val_gradient);
  AutoDiffVecXd y_autodiff;
  constraint.Eval(val_autodiff, &y_autodiff);

  Selector1<AutoDiffXd>(*system_autodiff, val_autodiff, context_autodiff.get());
  AutoDiffVecXd y_autodiff_expected;
  DummySystemConstraintCalc<AutoDiffXd>(*context_autodiff,
                                        &y_autodiff_expected);
  EXPECT_TRUE(CompareMatrices(math::autoDiffToValueMatrix(y_autodiff),
                              math::autoDiffToValueMatrix(y_autodiff_expected),
                              tol));
  EXPECT_TRUE(CompareMatrices(
      math::autoDiffToGradientMatrix(y_autodiff),
      math::autoDiffToGradientMatrix(y_autodiff_expected), tol));
}

// Assumes x only contains the position q.
template <typename T>
void Selector2(const System<T>& plant, const Eigen::Ref<const VectorX<T>>& x,
               Context<T>* context) {
  dynamic_cast<const multibody::MultibodyPlant<T>&>(plant).SetPositions(context,
                                                                        x);
}

GTEST_TEST(SystemConstraintWrapperTest, FreeBodyPlantTest) {
  // Test if SystemConstraintWrapper works for MultibodyPlant.
  const double time_step{0};
  FreeBodyPlant<double> plant_double(time_step);
  auto context_double = plant_double.CreateDefaultContext();

  const Eigen::Quaterniond quat0(
      Eigen::AngleAxisd(0.2 * M_PI, Eigen::Vector3d(1, 2, 3).normalized()));
  Eigen::Matrix<double, 7, 1> q0;
  q0.head<4>() << quat0.w(), quat0.x(), quat0.y(), quat0.z();
  q0.tail<3>() << 2, 3, 4;
  plant_double.SetPositions(context_double.get(), q0);
  Vector6<double> v0;
  v0 << 2, 3, 4, 5, 6, 7;
  plant_double.SetVelocities(context_double.get(), v0);

  FreeBodyPlant<AutoDiffXd> plant_autodiff(time_step);
  auto context_autodiff = plant_autodiff.CreateDefaultContext();
  context_autodiff->SetTimeStateAndParametersFrom(*context_double);
  SystemConstraintWrapper constraint(
      &plant_double, &plant_autodiff,
      plant_double.unit_quaternion_constraint_index(), *context_double,
      Selector2<double>, Selector2<AutoDiffXd>, 7);

  // First test Eval with VectorX<double>
  Eigen::Matrix<double, 7, 1> x_double;
  x_double << 1, 2, 3, 4, 5, 6, 7, 8;
  Eigen::VectorXd y_double;
  constraint.Eval(x_double, &y_double);
  const Vector1d y_double_expected(29);
  EXPECT_TRUE(CompareMatrices(y_double, y_double_expected, 3 * kEps));

  // Now test Eval with VectorX<AutoDiffXd>
  Eigen::Matrix<double, 7, Eigen::Dynamic> x_gradient(7, 2);
  x_gradient << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14;
  const auto x_autodiff =
      math::initializeAutoDiffGivenGradientMatrix(x_double, x_gradient);
  AutoDiffVecXd y_autodiff;
  constraint.Eval(x_autodiff, &y_autodiff);

  Selector2<AutoDiffXd>(plant_autodiff, x_autodiff, context_autodiff.get());

  AutoDiffVecXd y_autodiff_expected;
  plant_autodiff
      .get_constraint(plant_autodiff.unit_quaternion_constraint_index())
      .Calc(*context_autodiff, &y_autodiff_expected);

  EXPECT_TRUE(CompareMatrices(math::autoDiffToValueMatrix(y_autodiff),
                              math::autoDiffToValueMatrix(y_autodiff_expected),
                              3 * kEps));
  EXPECT_TRUE(CompareMatrices(
      math::autoDiffToGradientMatrix(y_autodiff),
      math::autoDiffToGradientMatrix(y_autodiff_expected), 3 * kEps));
}

}  // namespace
}  // namespace systems
}  // namespace drake
