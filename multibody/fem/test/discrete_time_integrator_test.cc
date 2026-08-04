#include "drake/multibody/fem/discrete_time_integrator.h"

#include <memory>

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

using Eigen::Vector3d;
using Eigen::VectorXd;

/* A dummy implementation of the DiscreteTimeIntegrator class. Used in the unit
 test in this file only. */
class DummyScheme final : public DiscreteTimeIntegrator<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DummyScheme);

  explicit DummyScheme(double dt) : DiscreteTimeIntegrator<double>(dt) {}

  ~DummyScheme() = default;

 private:
  std::unique_ptr<DiscreteTimeIntegrator<double>> DoClone() const final {
    return std::make_unique<DummyScheme>(dt());
  }

  Vector3d DoGetWeights() const final { return {1, 2, 3}; }

  const VectorXd& DoGetUnknowns(const FemState<double>& state) const final {
    return state.GetPositions();
  }

  // Dummy implementation to set the position to `dz`.
  void DoUpdateStateFromChangeInUnknowns(const VectorXd& dz,
                                         FemState<double>* state) const final {
    state->SetPositions(dz);
  }

  // Dummy implementation to set the position of the next state to the
  // entry-wise product of previous state's position and the unknown variable.
  void DoAdvanceOneTimeStep(const FemState<double>& prev_state,
                            const VectorXd& z,
                            FemState<double>* state) const final {
    state->SetPositions(prev_state.GetPositions().cwiseProduct(z));
  }
};

class DiscreteTimeIntegratorTest : public ::testing::Test {
 protected:
  static constexpr int kNumDofs = 12;
  static constexpr double kDt = 0.1;

  void SetUp() {
    fem_state_system_ = std::make_unique<internal::FemStateSystem<double>>(
        q(), VectorXd::Zero(kNumDofs), VectorXd::Zero(kNumDofs));
  }

  VectorXd q() {
    Vector<double, kNumDofs> q;
    q << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2;
    return q;
  }

  std::unique_ptr<internal::FemStateSystem<double>> fem_state_system_;
  DummyScheme scheme_{kDt};
};

TEST_F(DiscreteTimeIntegratorTest, Dt) {
  EXPECT_EQ(scheme_.dt(), kDt);
}

TEST_F(DiscreteTimeIntegratorTest, Weights) {
  EXPECT_EQ(scheme_.GetWeights(), Vector3d(1, 2, 3));
}

TEST_F(DiscreteTimeIntegratorTest, GetUnknowns) {
  const FemState<double> state(fem_state_system_.get());
  EXPECT_EQ(scheme_.GetUnknowns(state), state.GetPositions());
}

TEST_F(DiscreteTimeIntegratorTest, UpdateStateFromChangeInUnknowns) {
  FemState<double> state(fem_state_system_.get());
  const VectorXd dz = VectorXd::LinSpaced(kNumDofs, 0.0, 1.0);
  scheme_.UpdateStateFromChangeInUnknowns(dz, &state);
  EXPECT_EQ(state.GetPositions(), dz);
}

TEST_F(DiscreteTimeIntegratorTest, AdvanceOneTimeStep) {
  const FemState<double> state0(fem_state_system_.get());
  FemState<double> state(fem_state_system_.get());
  const VectorXd z = VectorXd::LinSpaced(kNumDofs, 0.0, 1.0);
  scheme_.AdvanceOneTimeStep(state0, z, &state);
  EXPECT_EQ(state.GetPositions(), state0.GetPositions().cwiseProduct(z));
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
