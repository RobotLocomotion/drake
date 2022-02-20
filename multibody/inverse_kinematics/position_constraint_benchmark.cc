// @file
// Benchmarks for PositionConstraint.
//

#include <benchmark/benchmark.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/inverse_kinematics/position_constraint.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/tools/performance/fixture_common.h"

namespace drake {
namespace multibody {
namespace inverse_kinematics {
namespace {

using Eigen::MatrixXd;
using systems::Context;
using systems::Diagram;
using systems::System;

class IiwaPositionConstraintFixture : public benchmark::Fixture {
 public:
  using benchmark::Fixture::SetUp;
  void SetUp(const ::benchmark::State&) override {
    tools::performance::AddMinMaxStatistics(this);

    const int kNumIiwas = 10;

    const std::string iiwa_path = FindResourceOrThrow(
        "drake/manipulation/models/iiwa_description/sdf/"
        "iiwa14_no_collision.sdf");
    systems::DiagramBuilder<double> builder{};
    plant_ = builder.AddSystem<MultibodyPlant<double>>(0.0);
    multibody::Parser parser{plant_};
    for (int i = 0; i < kNumIiwas; ++i) {
      const ModelInstanceIndex model_instance =
          parser.AddModelFromFile(iiwa_path, fmt::format("iiwa{}", i));
      plant_->WeldFrames(plant_->world_frame(),
                         plant_->GetFrameByName("iiwa_link_0", model_instance));
    }
    plant_->Finalize();

    diagram_ = builder.Build();
    diagram_context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &diagram_->GetMutableSubsystemContext(*plant_, diagram_context_.get());

    const Eigen::Vector3d p_BQ(0.1, 0.2, 0.3);
    const Eigen::Vector3d p_AQ_lower(-0.2, -0.3, -0.4);
    const Eigen::Vector3d p_AQ_upper(0.2, 0.3, 0.4);
    const ModelInstanceIndex first_iiwa =
        plant_->GetModelInstanceByName("iiwa0");
    const Frame<double>& frameA =
        plant_->GetFrameByName("iiwa_link_7", first_iiwa);
    const Frame<double>& frameB =
        plant_->GetFrameByName("iiwa_link_3", first_iiwa);
    constraint_ = std::make_unique<PositionConstraint>(
        plant_, frameA, p_AQ_lower, p_AQ_upper, frameB, p_BQ, plant_context_);

    q_.resize(7 * kNumIiwas);
    for (int i = 0; i < kNumIiwas; ++i) {
      q_.segment(7 * i, 7) << 0.1, 0.2, 0.3, 0.4, -0.1, -0.2, -0.3;
    }
  }

 protected:
  std::unique_ptr<Diagram<double>> diagram_{};
  MultibodyPlant<double>* plant_{};
  std::unique_ptr<Context<double>> diagram_context_{};
  Context<double>* plant_context_{};
  std::unique_ptr<PositionConstraint> constraint_{};
  Eigen::VectorXd q_;
};

BENCHMARK_F(IiwaPositionConstraintFixture, EvalWGradientMbpDouble)
// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
(benchmark::State& state) {
  Eigen::VectorXd y(constraint_->num_constraints());
  Eigen::MatrixXd dydx(constraint_->num_constraints(), q_.size());
  for (auto _ : state) {
    q_(0) += 0.01;  // avoid caching.
    constraint_->Eval(q_, &y, &dydx);
  }
}

BENCHMARK_F(IiwaPositionConstraintFixture, EvalAutoDiffMbpDouble)
// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
(benchmark::State& state) {
  for (auto _ : state) {
    q_(0) += 0.01;  // avoid caching.
    // We include initialization / extraction on each step to mimic the work
    // that a solver would do using the original original Eval() with
    // AutoDiffXd.
    VectorX<AutoDiffXd> q_autodiff = math::InitializeAutoDiff(q_);
    AutoDiffVecXd y_autodiff;
    constraint_->Eval(q_autodiff, &y_autodiff);
    math::ExtractValue(y_autodiff);
    math::ExtractGradient(y_autodiff);
  }
}

}  // namespace
}  // namespace inverse_kinematics
}  // namespace multibody
}  // namespace drake

BENCHMARK_MAIN();
