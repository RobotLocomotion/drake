#include "drake/systems/analysis/integrator_base.h"

#include <gtest/gtest.h>

#include "drake/systems/plants/spring_mass_system/spring_mass_system.h"

namespace drake {
namespace systems {
namespace {

// A class for testing protected integration functions.
template <typename T>
class DummyIntegrator : public IntegratorBase<T> {
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DummyIntegrator)

 public:
  DummyIntegrator(const System<T>& system, Context<T>* context)
      : IntegratorBase<T>(system, context) {}

  // Necessary implementations of pure virtual methods.
  bool supports_error_estimation() const override { return true; }
  int get_error_estimate_order() const override { return 1; }

  // Promote CalcStateChangeNorm() to public.
  using IntegratorBase<T>::CalcStateChangeNorm;

 private:
  // Should not be called.
  bool DoStep(const T&) override { DRAKE_UNREACHABLE(); }
};

// Tests that resetting the integrator with a null pointer throws.
GTEST_TEST(IntegratorBaseTest, DoubleStateChangeNormPropagatesNaN) {
  // We need a system with q, v, and z variables. Constants and absence of
  // forcing are arbitrary (irrelevant for this test).
  SpringMassSystem<double> spring_mass(10.0, 1.0, false);
  std::unique_ptr<Context<double>> context = spring_mass.CreateDefaultContext();
  DummyIntegrator<double> integrator(spring_mass, context.get());
  integrator.Initialize();

  // Set q = v = z = 0 and verify that the state change norm is zero.
  context->get_mutable_continuous_state()[0] = 0;
  context->get_mutable_continuous_state()[1] = 0;
  context->get_mutable_continuous_state()[2] = 0;
  EXPECT_EQ(integrator.CalcStateChangeNorm(context->get_continuous_state()), 0);

  // Set q to NaN, and v = z = 0 and verify that NaN is returned from
  // CalcStateChangeNorm().
  using std::isnan;
  context->get_mutable_continuous_state()[0] =
      std::numeric_limits<double>::quiet_NaN();
  context->get_mutable_continuous_state()[1] = 0;
  context->get_mutable_continuous_state()[2] = 0;
  EXPECT_TRUE(
      isnan(integrator.CalcStateChangeNorm(context->get_continuous_state())));

  // Set v to NaN, and q = z = 0 and verify that NaN is returned from
  // CalcStateChangeNorm().
  context->get_mutable_continuous_state()[0] = 0;
  context->get_mutable_continuous_state()[1] =
      std::numeric_limits<double>::quiet_NaN();
  context->get_mutable_continuous_state()[2] = 0;
  EXPECT_TRUE(
      isnan(integrator.CalcStateChangeNorm(context->get_continuous_state())));

  // Set v to NaN, and q = z = 0 and verify that NaN is returned from
  // CalcStateChangeNorm().
  context->get_mutable_continuous_state()[0] = 0;
  context->get_mutable_continuous_state()[1] =
      std::numeric_limits<double>::quiet_NaN();
  context->get_mutable_continuous_state()[2] = 0;
  EXPECT_TRUE(
      isnan(integrator.CalcStateChangeNorm(context->get_continuous_state())));
}

// Tests that resetting the integrator with a null pointer throws.
GTEST_TEST(IntegratorBaseTest, AutoDiffXdStateChangeNormPropagatesNaN) {
  // We need a system with q, v, and z variables. Constants and absence of
  // forcing are arbitrary (irrelevant for this test).
  SpringMassSystem<AutoDiffXd> spring_mass(10.0, 1.0, false);
  std::unique_ptr<Context<AutoDiffXd>> context =
      spring_mass.CreateDefaultContext();
  DummyIntegrator<AutoDiffXd> integrator(spring_mass, context.get());
  integrator.Initialize();

  // Set q = v = z = 0 and verify that the state change norm is zero.
  context->get_mutable_continuous_state()[0] = 0;
  context->get_mutable_continuous_state()[1] = 0;
  context->get_mutable_continuous_state()[2] = 0;
  EXPECT_EQ(integrator.CalcStateChangeNorm(context->get_continuous_state()), 0);

  // Set q to NaN, and v = z = 0 and verify that NaN is returned from
  // CalcStateChangeNorm().
  using std::isnan;
  context->get_mutable_continuous_state()[0] =
      std::numeric_limits<double>::quiet_NaN();
  context->get_mutable_continuous_state()[1] = 0;
  context->get_mutable_continuous_state()[2] = 0;
  EXPECT_TRUE(
      isnan(integrator.CalcStateChangeNorm(context->get_continuous_state())));

  // Set v to NaN, and q = z = 0 and verify that NaN is returned from
  // CalcStateChangeNorm().
  context->get_mutable_continuous_state()[0] = 0;
  context->get_mutable_continuous_state()[1] =
      std::numeric_limits<double>::quiet_NaN();
  context->get_mutable_continuous_state()[2] = 0;
  EXPECT_TRUE(
      isnan(integrator.CalcStateChangeNorm(context->get_continuous_state())));

  // Set v to NaN, and q = z = 0 and verify that NaN is returned from
  // CalcStateChangeNorm().
  context->get_mutable_continuous_state()[0] = 0;
  context->get_mutable_continuous_state()[1] =
      std::numeric_limits<double>::quiet_NaN();
  context->get_mutable_continuous_state()[2] = 0;
  EXPECT_TRUE(
      isnan(integrator.CalcStateChangeNorm(context->get_continuous_state())));
}

}  // namespace
}  // namespace systems
}  // namespace drake
