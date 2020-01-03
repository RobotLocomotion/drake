#include "drake/systems/analysis/implicit_integrator.h"

#include <gtest/gtest.h>

#include "drake/systems/plants/spring_mass_system/spring_mass_system.h"

using Eigen::VectorXd;

namespace drake {
namespace systems {
namespace {

// Dummy implicit integrator for testing protected methods of
// ImplicitIntegrator class.
class DummyImplicitIntegrator final : public ImplicitIntegrator<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DummyImplicitIntegrator)
  DummyImplicitIntegrator(const System<double>& system,
                          Context<double>* context)
      : ImplicitIntegrator(system, context) {}
  bool supports_error_estimation() const override { return false; }
  int get_error_estimate_order() const override { return 0; }

  using ImplicitIntegrator<double>::IsUpdateZero;

 private:
  // There is no stepping so no stats should accumulate.
  int64_t do_get_num_newton_raphson_iterations() const final { return 0; }
  int64_t do_get_num_error_estimator_derivative_evaluations() const final {
    return 0;
  }
  int64_t do_get_num_error_estimator_jacobian_evaluations() const final {
    return 0;
  }
  int64_t do_get_num_error_estimator_derivative_evaluations_for_jacobian()
      const final {
    return 0;
  }
  int64_t do_get_num_error_estimator_newton_raphson_iterations() const final {
    return 0;
  }
  int64_t do_get_num_error_estimator_iteration_matrix_factorizations()
      const final {
    return 0;
  }

  bool DoImplicitIntegratorStep(const double& h) override {
    throw std::logic_error("Dummy integrator not meant to be stepped.");
  }
};

GTEST_TEST(ImplicitIntegratortest, IsUpdateZero) {
  const double mass = 1.0;
  const double spring_k = 1.0;
  SpringMassSystem<double> dummy_system(spring_k, mass, false /* unforced */);
  std::unique_ptr<Context<double>> context =
      dummy_system.CreateDefaultContext();
  DummyImplicitIntegrator dummy_integrator(dummy_system, context.get());

  // Machine epsilon is used in the tests below. The tolerance used to check
  // whether a dimension of the update is zero in IsUpdateZero() will be
  // 10 * eps. eps will also be used to construct the test below.
  const double eps = std::numeric_limits<double>::epsilon();

  // Test the case where the value of interest is unity and the update is near
  // machine epsilon. The other state and update are set to zero so that these
  // would *not* cause the update to be treated as *non*-zero.
  VectorXd xc(2), dxc(2);
  xc << 1.0, 0.0;
  dxc << eps * 5, 0.0;
  EXPECT_TRUE(dummy_integrator.IsUpdateZero(xc, dxc, eps * 10));

  // Test the case where the value of interest is large and the update is still
  // large, but not large enough relative to the value of interest.
  xc << 1e10, 0.0;
  dxc << eps * 1e6, 0.0;
  EXPECT_TRUE(dummy_integrator.IsUpdateZero(xc, dxc, eps * 10));

  // Test the case where we have two values of interest, one large and the
  // other small with both updates "significant".
  xc << 1e10, 1.0;
  dxc << 1e0, eps * 1e6;
  EXPECT_FALSE(dummy_integrator.IsUpdateZero(xc, dxc, eps * 10));

  // Verify that a reasonable tolerance is used when a negative or zero
  // tolerance is used.
  xc << 1.0, 0.0;
  dxc << eps, 0.0;
  EXPECT_TRUE(dummy_integrator.IsUpdateZero(xc, dxc, -1.0));
  EXPECT_TRUE(dummy_integrator.IsUpdateZero(xc, dxc, 0.0));
  dxc << 0.1, 0.0;
  EXPECT_FALSE(dummy_integrator.IsUpdateZero(xc, dxc, -1.0));
  EXPECT_FALSE(dummy_integrator.IsUpdateZero(xc, dxc, 0.0));
}

}  // namespace
}  // namespace systems
}  // namespace drake

