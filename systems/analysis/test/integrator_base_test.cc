#include "drake/systems/analysis/integrator_base.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/analysis/test_utilities/spring_mass_system.h"

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
  std::pair<bool, T> CalcAdjustedStepSize(const T& err, const T& step_taken,
                                          bool* at_minimum_step_size) const {
    return IntegratorBase<T>::CalcAdjustedStepSize(err, step_taken,
                                                   at_minimum_step_size);
  }

  // Promote CalcStateChangeNorm() to public.
  using IntegratorBase<T>::CalcStateChangeNorm;

 private:
  // We want the Step function to fail whenever the step size is greater than
  // or equal to unity (see FixedStepFailureIndicatesSubstepFailure).
  bool DoStep(const T& step_size) override {
      Context<T>* context = this->get_mutable_context();
      context->SetTime(context->get_time() + step_size);
      return (step_size < 1.0);
  }
};

// Tests that IntegratorBase::IntegrateNoFurtherThanTime(.) records a substep
// failure when running in fixed step mode and stepping fails.
GTEST_TEST(IntegratorBaseTest, FixedStepFailureIndicatesSubstepFailure) {
  // Use the spring-mass system because we need some system (and this one will
  // do as well as any other).
  SpringMassSystem<double> spring_mass(10.0, 1.0, false);
  std::unique_ptr<Context<double>> context = spring_mass.CreateDefaultContext();
  DummyIntegrator<double> integrator(spring_mass, context.get());

  // Set the integrator to fixed step mode.
  integrator.set_fixed_step_mode(true);

  // Verify the statistics are clear before integrating.
  EXPECT_EQ(integrator.get_num_step_shrinkages_from_substep_failures(), 0);
  EXPECT_EQ(integrator.get_num_substep_failures(), 0);

  // Call the integration function.
  const double arbitrary_time = 1.0;
  integrator.Initialize();
  integrator.IntegrateNoFurtherThanTime(arbitrary_time, arbitrary_time,
                                        arbitrary_time);

  // Verify the step statistics have been updated. We expect DoStep() to be
  // called just twice.
  EXPECT_EQ(integrator.get_num_step_shrinkages_from_substep_failures(), 1);
  EXPECT_EQ(integrator.get_num_substep_failures(), 1);
}

// Tests that CalcAdjustedStepSize() shrinks the step size when encountering
// NaN and Inf.
GTEST_TEST(IntegratorBaseTest, CalcAdjustedStepSizeShrinksOnNaNAndInf) {
  // We expect the shrinkage to be *at least* a factor of two.
  const double kShrink = 0.5;

  // Various "errors" that will be passed into CalcAdjustedStepSize.
  const double zero_error = 0.0;
  const double nan_error = std::numeric_limits<double>::quiet_NaN();
  const double inf_error = std::numeric_limits<double>::infinity();

  // Arbitrary step size taken.
  const double step_taken = 1.0;

  // The two possible values that the at_minimum_step_size input/output
  // parameter can take on entry.
  bool at_minimum_step_size_true_on_entry = true;
  bool at_minimum_step_size_false_on_entry = false;

  // Use the spring-mass system (system and context will be unused for this
  // test).
  SpringMassSystem<double> spring_mass(10.0, 1.0, false);
  std::unique_ptr<Context<double>> context = spring_mass.CreateDefaultContext();
  DummyIntegrator<double> integrator(spring_mass, context.get());

  // Verify that there is no shrinkage for zero error.
  std::pair<bool, double> result;
  result = integrator.CalcAdjustedStepSize(zero_error, step_taken,
                                           &at_minimum_step_size_true_on_entry);
  EXPECT_EQ(result.first, true);
  EXPECT_GE(result.second, step_taken);
  result = integrator.CalcAdjustedStepSize(
      zero_error, step_taken, &at_minimum_step_size_false_on_entry);
  EXPECT_EQ(result.first, true);
  EXPECT_GE(result.second, step_taken);

  // Neither should be at the minimum step size.
  EXPECT_EQ(at_minimum_step_size_true_on_entry, false);
  EXPECT_EQ(at_minimum_step_size_false_on_entry, false);

  // Reset the minimum step size Booleans.
  at_minimum_step_size_true_on_entry = true;
  at_minimum_step_size_false_on_entry = false;

  // Verify shrinkage for NaN error.
  result = integrator.CalcAdjustedStepSize(nan_error, step_taken,
                                           &at_minimum_step_size_true_on_entry);
  EXPECT_EQ(result.first, false);
  EXPECT_LT(result.second, kShrink * step_taken);
  result = integrator.CalcAdjustedStepSize(
      nan_error, step_taken, &at_minimum_step_size_false_on_entry);
  EXPECT_EQ(result.first, false);
  EXPECT_LT(result.second, kShrink * step_taken);

  // Minimum step size should be unchanged.
  EXPECT_EQ(at_minimum_step_size_true_on_entry, true);
  EXPECT_EQ(at_minimum_step_size_false_on_entry, false);

  // Verify shrinkage for Inf error.
  result = integrator.CalcAdjustedStepSize(inf_error, step_taken,
                                           &at_minimum_step_size_true_on_entry);
  EXPECT_EQ(result.first, false);
  EXPECT_LT(result.second, kShrink * step_taken);
  result = integrator.CalcAdjustedStepSize(
      inf_error, step_taken, &at_minimum_step_size_false_on_entry);
  EXPECT_EQ(result.first, false);
  EXPECT_LT(result.second, kShrink * step_taken);

  // Minimum step size should be unchanged.
  EXPECT_EQ(at_minimum_step_size_true_on_entry, true);
  EXPECT_EQ(at_minimum_step_size_false_on_entry, false);
}

// Tests that CalcStateChangeNorm() propagates NaNs in state.
GTEST_TEST(IntegratorBaseTest, DoubleStateChangeNormPropagatesNaN) {
  // We need a system with q, v, and z variables. Constants and absence of
  // forcing are arbitrary (irrelevant for this test).
  SpringMassSystem<double> spring_mass(10.0, 1.0, false);
  std::unique_ptr<Context<double>> context = spring_mass.CreateDefaultContext();
  DummyIntegrator<double> integrator(spring_mass, context.get());
  integrator.Initialize();

  // Set q = v = z = 0 and verify that the state change norm is zero.
  ASSERT_EQ(context->get_continuous_state().size(), 3);
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
  context->get_mutable_continuous_state()[1] = 0;
  context->get_mutable_continuous_state()[2] =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(
      isnan(integrator.CalcStateChangeNorm(context->get_continuous_state())));
}

// Tests that CalcStateChangeNorm() propagates NaNs in state.
GTEST_TEST(IntegratorBaseTest, AutoDiffXdStateChangeNormPropagatesNaN) {
  // We need a system with q, v, and z variables. Constants and absence of
  // forcing are arbitrary (irrelevant for this test).
  SpringMassSystem<AutoDiffXd> spring_mass(10.0, 1.0, false);
  std::unique_ptr<Context<AutoDiffXd>> context =
      spring_mass.CreateDefaultContext();
  DummyIntegrator<AutoDiffXd> integrator(spring_mass, context.get());
  integrator.Initialize();

  // Set q = v = z = 0 and verify that the state change norm is zero.
  ASSERT_EQ(context->get_continuous_state().size(), 3);
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
  context->get_mutable_continuous_state()[1] = 0;
  context->get_mutable_continuous_state()[2] =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(
      isnan(integrator.CalcStateChangeNorm(context->get_continuous_state())));
}

// Check that dense integration handles repeated evaluations, as seen in the
// witness isolation use case in Simulator.
GTEST_TEST(IntegratorBaseTest, DenseOutputTest) {
  SpringMassSystem<double> spring_mass(10.0, 1.0, false);
  std::unique_ptr<Context<double>> context = spring_mass.CreateDefaultContext();
  DummyIntegrator<double> integrator(spring_mass, context.get());
  integrator.set_fixed_step_mode(true);
  integrator.Initialize();

  EXPECT_EQ(integrator.get_dense_output(), nullptr);
  integrator.StartDenseIntegration();
  const trajectories::PiecewisePolynomial<double>* dense_output =
      integrator.get_dense_output();
  EXPECT_EQ(dense_output->get_number_of_segments(), 0);
  EXPECT_TRUE(integrator.IntegrateWithSingleFixedStepToTime(0.1));
  EXPECT_EQ(dense_output->get_number_of_segments(), 1);
  EXPECT_TRUE(integrator.IntegrateWithSingleFixedStepToTime(0.2));
  EXPECT_EQ(dense_output->get_number_of_segments(), 2);

  // Now repeat a step, and make sure that I replace rather than append the new
  // segment.
  context->SetTime(0.1);
  EXPECT_TRUE(integrator.IntegrateWithSingleFixedStepToTime(0.15));
  EXPECT_EQ(dense_output->get_number_of_segments(), 2);

  EXPECT_EQ(dense_output->start_time(), 0.0);
  EXPECT_EQ(dense_output->end_time(), 0.15);

  context->SetTime(0.2);
  DRAKE_EXPECT_THROWS_MESSAGE(
      static_cast<void>(integrator.IntegrateWithSingleFixedStepToTime(0.3)),
      std::runtime_error, ".*ConcatenateInTime.*time_offset.*");
}

}  // namespace
}  // namespace systems
}  // namespace drake
