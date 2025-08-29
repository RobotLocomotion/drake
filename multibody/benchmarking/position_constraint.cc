// @file
// Benchmarks for PositionConstraint.
//
// This highlights the performance difference between using the analytical
// derivatives (via CalcJacobian) vs the AutoDiffXd differences.

#include "drake/multibody/inverse_kinematics/position_constraint.h"

#include <memory>
#include <string>

#include <benchmark/benchmark.h>

#include "drake/multibody/parsing/parser.h"
#include "drake/tools/performance/fixture_common.h"

namespace drake {
namespace multibody {
namespace inverse_kinematics {
namespace {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using systems::Context;

class IiwaPositionConstraintFixture : public benchmark::Fixture {
 public:
  void SetUp(::benchmark::State&) override {
    tools::performance::AddMinMaxStatistics(this);

    const int kNumIiwas = 10;

    const std::string iiwa_url =
        "package://drake_models/iiwa_description/sdf/iiwa14_no_collision.sdf";
    plant_ = std::make_unique<MultibodyPlant<double>>(0.0);
    multibody::Parser parser{plant_.get()};
    parser.SetAutoRenaming(true);
    for (int i = 0; i < kNumIiwas; ++i) {
      const ModelInstanceIndex model_instance =
          parser.AddModelsFromUrl(iiwa_url).at(0);
      plant_->WeldFrames(plant_->world_frame(),
                         plant_->GetFrameByName("iiwa_link_0", model_instance));
    }
    plant_->Finalize();

    context_ = plant_->CreateDefaultContext();

    const Eigen::Vector3d p_BQ(0.1, 0.2, 0.3);
    const Eigen::Vector3d p_AQ_lower(-0.2, -0.3, -0.4);
    const Eigen::Vector3d p_AQ_upper(0.2, 0.3, 0.4);
    const ModelInstanceIndex first_iiwa =
        plant_->GetModelInstanceByName("iiwa14");
    constraint_ = std::make_unique<PositionConstraint>(
        plant_.get(), plant_->GetFrameByName("iiwa_link_7", first_iiwa),
        p_AQ_lower, p_AQ_upper,
        plant_->GetFrameByName("iiwa_link_3", first_iiwa), p_BQ,
        context_.get());

    q_.resize(7 * kNumIiwas);
    for (int i = 0; i < kNumIiwas; ++i) {
      q_.segment(7 * i, 7) << 0.1, 0.2, 0.3, 0.4, -0.1, -0.2, -0.3;
    }

    ad_plant_ = systems::System<double>::ToAutoDiffXd(*plant_);
    DRAKE_DEMAND(ad_plant_ != nullptr);
    ad_context_ = ad_plant_->CreateDefaultContext();
    const ModelInstanceIndex ad_first_iiwa =
        ad_plant_->GetModelInstanceByName("iiwa14");
    ad_constraint_ = std::make_unique<PositionConstraint>(
        ad_plant_.get(),
        ad_plant_->GetFrameByName("iiwa_link_7", ad_first_iiwa), p_AQ_lower,
        p_AQ_upper, ad_plant_->GetFrameByName("iiwa_link_3", ad_first_iiwa),
        p_BQ, ad_context_.get());
  }

 protected:
  std::unique_ptr<MultibodyPlant<double>> plant_{};
  std::unique_ptr<Context<double>> context_{};
  std::unique_ptr<PositionConstraint> constraint_{};

  std::unique_ptr<MultibodyPlant<AutoDiffXd>> ad_plant_{};
  std::unique_ptr<Context<AutoDiffXd>> ad_context_{};
  std::unique_ptr<PositionConstraint> ad_constraint_{};

  Eigen::VectorXd q_;
};

BENCHMARK_F(IiwaPositionConstraintFixture, EvalAutoDiffMbpDouble)
// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
(benchmark::State& state) {
  for (auto _ : state) {
    q_(0) += 0.01;  // avoid caching.
    // We include initialization / extraction on each step to mimic the work
    // that a solver would do using the original Eval() with AutoDiffXd.
    VectorX<AutoDiffXd> q_autodiff = math::InitializeAutoDiff(q_);
    AutoDiffVecXd y_autodiff;
    constraint_->Eval(q_autodiff, &y_autodiff);
    VectorXd value;
    benchmark::DoNotOptimize(value = math::ExtractValue(y_autodiff));
    MatrixXd gradient;
    benchmark::DoNotOptimize(gradient = math::ExtractGradient(y_autodiff));
  }
}

BENCHMARK_F(IiwaPositionConstraintFixture, EvalAutoDiffMbpAutoDiff)
// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
(benchmark::State& state) {
  for (auto _ : state) {
    q_(0) += 0.01;  // avoid caching.
    // We include initialization / extraction on each step to mimic the work
    // that a solver would do using the original Eval() with AutoDiffXd.
    VectorX<AutoDiffXd> q_autodiff = math::InitializeAutoDiff(q_);
    AutoDiffVecXd y_autodiff;
    ad_constraint_->Eval(q_autodiff, &y_autodiff);
    VectorXd value;
    benchmark::DoNotOptimize(value = math::ExtractValue(y_autodiff));
    MatrixXd gradient;
    benchmark::DoNotOptimize(gradient = math::ExtractGradient(y_autodiff));
  }
}

}  // namespace
}  // namespace inverse_kinematics
}  // namespace multibody
}  // namespace drake

BENCHMARK_MAIN();
