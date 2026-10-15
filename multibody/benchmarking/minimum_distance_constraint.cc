// @file
// Benchmarks for MinimumDistanceLowerBoundConstraint evaluated in AutoDiffXd.
//
// Compares the constraint constructed from a SceneGraphCollisionChecker with
// the one constructed from a MultibodyPlant<double>, over a row of closely
// spaced iiwa arms with sphere collision geometry. The number of arms
// sets the number of positions and of geometry pairs in range; the number of
// derivatives is varied independently.

#include <memory>
#include <utility>
#include <vector>

#include <benchmark/benchmark.h>

#include "drake/common/drake_assert.h"
#include "drake/geometry/query_object.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/inverse_kinematics/minimum_distance_lower_bound_constraint.h"
#include "drake/planning/robot_diagram_builder.h"
#include "drake/planning/scene_graph_collision_checker.h"
#include "drake/tools/performance/fixture_common.h"

namespace drake {
namespace multibody {
namespace inverse_kinematics {
namespace {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using planning::CollisionCheckerContext;
using planning::CollisionCheckerParams;
using planning::RobotDiagram;
using planning::RobotDiagramBuilder;
using planning::SceneGraphCollisionChecker;
using systems::Context;

constexpr double kBound = 0.01;
constexpr double kInfluenceDistanceOffset = 0.1;
constexpr char kIiwaUrl[] =
    "package://drake_models/iiwa_description/urdf/"
    "iiwa14_spheres_collision.urdf";

class IiwaRowFixture : public benchmark::Fixture {
 public:
  void SetUp(::benchmark::State& state) override {
    tools::performance::AddMinMaxStatistics(this);

    const int num_iiwas = state.range(0);
    const int num_derivatives = state.range(1);

    RobotDiagramBuilder<double> builder;
    builder.parser().SetAutoRenaming(true);
    std::vector<ModelInstanceIndex> iiwas;
    for (int i = 0; i < num_iiwas; ++i) {
      const ModelInstanceIndex iiwa =
          builder.parser().AddModelsFromUrl(kIiwaUrl).at(0);
      builder.plant().WeldFrames(
          builder.plant().world_frame(),
          builder.plant().GetFrameByName("base", iiwa),
          math::RigidTransformd(Eigen::Vector3d(0.6 * i, 0, 0)));
      iiwas.push_back(iiwa);
    }
    model_ = builder.Build();
    const MultibodyPlant<double>& plant = model_->plant();
    const int nq = plant.num_positions();
    DRAKE_DEMAND(nq == 7 * num_iiwas);

    // Each arm leans toward its neighbor.
    q_.resize(nq);
    for (int i = 0; i < num_iiwas; ++i) {
      q_.segment<7>(7 * i) << 0, 0.6, 0, -1.2, 0, 0.6, 0;
    }
    gradient_ = MatrixXd::Identity(nq, num_derivatives);

    diagram_context_ = model_->CreateDefaultContext();
    Context<double>* plant_context =
        &model_->mutable_plant_context(diagram_context_.get());
    plant_constraint_ = std::make_unique<MinimumDistanceLowerBoundConstraint>(
        &plant, kBound, plant_context, solvers::ExponentiallySmoothedHingeLoss,
        kInfluenceDistanceOffset);

    CollisionCheckerParams params;
    params.model = model_;
    params.robot_model_instances = iiwas;
    params.edge_step_size = 0.05;
    params.implicit_context_parallelism = Parallelism::None();
    checker_ = std::make_unique<SceneGraphCollisionChecker>(std::move(params));
    checker_context_ = checker_->MakeStandaloneModelContext();
    checker_constraint_ = std::make_unique<MinimumDistanceLowerBoundConstraint>(
        checker_.get(), kBound, checker_context_.get(),
        solvers::ExponentiallySmoothedHingeLoss, kInfluenceDistanceOffset);

    const double influence_distance = kBound + kInfluenceDistanceOffset;
    num_checker_pairs_ = checker_
                             ->CalcContextRobotClearance(checker_context_.get(),
                                                         q_, influence_distance)
                             .size();
    plant.SetPositions(plant_context, q_);
    num_plant_pairs_ =
        plant.get_geometry_query_input_port()
            .Eval<geometry::QueryObject<double>>(*plant_context)
            .ComputeSignedDistancePairwiseClosestPoints(influence_distance)
            .size();
  }

  void TearDown(::benchmark::State&) override {
    checker_constraint_.reset();
    checker_context_.reset();
    checker_.reset();
    plant_constraint_.reset();
    diagram_context_.reset();
    model_.reset();
  }

 protected:
  // NOLINTNEXTLINE(runtime/references)
  void RunEval(benchmark::State& state,
               const MinimumDistanceLowerBoundConstraint& constraint,
               int num_pairs) {
    for (auto _ : state) {
      q_(0) += 1e-6;  // avoid caching.
      const AutoDiffVecXd q_autodiff = math::InitializeAutoDiff(q_, gradient_);
      AutoDiffVecXd y_autodiff;
      constraint.Eval(q_autodiff, &y_autodiff);
      benchmark::DoNotOptimize(y_autodiff);
    }
    state.counters["nq"] = q_.size();
    state.counters["derivatives"] = gradient_.cols();
    state.counters["pairs"] = num_pairs;
  }

  std::shared_ptr<RobotDiagram<double>> model_;
  std::unique_ptr<Context<double>> diagram_context_;
  std::unique_ptr<MinimumDistanceLowerBoundConstraint> plant_constraint_;
  std::unique_ptr<SceneGraphCollisionChecker> checker_;
  std::shared_ptr<CollisionCheckerContext> checker_context_;
  std::unique_ptr<MinimumDistanceLowerBoundConstraint> checker_constraint_;
  int num_checker_pairs_{};
  int num_plant_pairs_{};
  VectorXd q_;
  MatrixXd gradient_;
};

// Arguments are {number of iiwas, number of derivatives}.
void IiwaRowArgs(benchmark::Benchmark* b) {
  for (int num_iiwas : {1, 2, 4, 8}) {
    const int nq = 7 * num_iiwas;
    b->Args({num_iiwas, 1});
    b->Args({num_iiwas, 7});
    if (nq > 7) {
      b->Args({num_iiwas, nq});
    }
  }
  b->Unit(benchmark::kMicrosecond);
}

BENCHMARK_DEFINE_F(IiwaRowFixture, CheckerAutoDiff)
// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
(benchmark::State& state) {
  RunEval(state, *checker_constraint_, num_checker_pairs_);
}
BENCHMARK_REGISTER_F(IiwaRowFixture, CheckerAutoDiff)->Apply(IiwaRowArgs);

BENCHMARK_DEFINE_F(IiwaRowFixture, PlantAutoDiff)
// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
(benchmark::State& state) {
  RunEval(state, *plant_constraint_, num_plant_pairs_);
}
BENCHMARK_REGISTER_F(IiwaRowFixture, PlantAutoDiff)->Apply(IiwaRowArgs);

}  // namespace
}  // namespace inverse_kinematics
}  // namespace multibody
}  // namespace drake

BENCHMARK_MAIN();
