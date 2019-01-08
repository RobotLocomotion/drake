#include "drake/systems/optimization/system_constraint_wrapper.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace {
// Declare a constraint p * x(0) + x(1) >= 2
//                      x(1) * x(1) + p * x(0) * x(0) <= x(2)
template <typename T>
void DummySystemConstraintCalc(const Context<T>& context, VectorX<T>* y) {
  const T p = context.get_numeric_parameter(0).GetAtIndex(0);
  const Vector3<T> x = context.get_continuous_state_vector().CopyToVector();
  y->resize(2);
  (*y)(0) = p * x(0) + x(1);
  (*y)(1) = x(2) - x(1) * x(1) + p * x(0) * x(0);
}

template <typename T>
class DummySystem : public LeafSystem<T> {
 public:
  DummySystem() : LeafSystem<T>(SystemTypeTag<DummySystem>{}) {
    this->DeclareContinuousState(3);  // 3 state variable.
    this->DeclareNumericParameter(BasicVector<T>(1));
    this->constraint_index_ = this->DeclareInequalityConstraint(
        DummySystemConstraintCalc<T>, Eigen::Vector2d(2, 0), nullopt,
        "dummy_system_constraint");
  }

  template <typename U>
  explicit DummySystem(const DummySystem<U>& system) : DummySystem() {}

  SystemConstraintIndex constraint_index() const { return constraint_index_; }

 private:
  // xdot = [p0 * x(0); p0 * x(1) + x(0), x(2) - x(1)]
  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override {
    const Vector3<T>& x = context.get_continuous_state_vector().CopyToVector();
    const T& p0 = context.get_numeric_parameter(0).GetAtIndex(0);
    const Vector3<T> xdot(p0 * x(0), p0 * x(1) + x(0), x(2) - x(1));
    derivatives->SetFromVector(xdot);
  }

  SystemConstraintIndex constraint_index_;
};

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

  // [x(0); p0; x(2)] = val = [10, 11, 12]
  const Eigen::Vector3d val(10, 11, 12);
  Eigen::VectorXd y;
  constraint.Eval(val, &y);

  Selector1<double>(system_double, val, context_double.get());
  Eigen::VectorXd y_expected;
  DummySystemConstraintCalc(*context_double, &y_expected);
  const double tol = 3 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(y, y_expected, tol));

  Eigen::Matrix3Xd val_gradient(3, 2);
  val_gradient << 1, 2, 3, 4, 5, 6;
  const auto val_autodiff = math::initializeAutoDiffGivenGradientMatrix(
      Eigen::Vector3d(10, 11, 12), val_gradient);
  AutoDiffVecXd y_autodiff;
  constraint.Eval(val_autodiff, &y_autodiff);

  Selector1<AutoDiffXd>(*system_autodiff, val_autodiff, context_autodiff.get());
  AutoDiffVecXd y_autodiff_expected;
  DummySystemConstraintCalc(*context_autodiff, &y_autodiff_expected);
  EXPECT_TRUE(CompareMatrices(math::autoDiffToValueMatrix(y_autodiff),
                              math::autoDiffToValueMatrix(y_autodiff_expected),
                              tol));
  EXPECT_TRUE(CompareMatrices(
      math::autoDiffToGradientMatrix(y_autodiff),
      math::autoDiffToGradientMatrix(y_autodiff_expected), tol));
}

}  // namespace
}  // namespace systems
}  // namespace drake
