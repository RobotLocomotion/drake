// @file
// Benchmark for GlobalInverseKinematics on the dual-arm TRI Homecart.

#include "drake/common/find_resource.h"
#include "drake/multibody/inverse_kinematics/global_inverse_kinematics.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/solve.h"
#include "drake/tools/performance/fixture_common.h"

namespace drake {
namespace multibody {
namespace inverse_kinematics {
namespace {

class HomecartGlobalIkBenchmark : public benchmark::Fixture {
 public:
  HomecartGlobalIkBenchmark() {
    tools::performance::AddMinMaxStatistics(this);
    this->Unit(benchmark::kSecond);

    // Set a fixed seed for random generation.
    std::srand(1234);
  }
};

// This benchmark adds only the posture cost (joint limits are also added in
// the constructor), so the optimal solution is the q0 passed to
// AddPostureCost. Passing this solution to SetInitialGuess demonstrates a
// limit on the speedup that can be obtained using SetInitialGuess; that
// speedup is substantial.
//
// clang-format off
BENCHMARK_F(HomecartGlobalIkBenchmark, PostureCost)
    // NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
    (benchmark::State& state)  // clang-format on
{
  multibody::MultibodyPlant<double> plant(0.0);
  geometry::SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);
  multibody::Parser parser(&plant);
  parser.AddModelsFromUrl(
      "package://drake_models/tri_homecart/homecart_no_grippers.dmd.yaml");
  plant.Finalize();

  Eigen::VectorXd q0(plant.num_positions());
  // A somewhat arbitrary configuration with the arms posed comfortably inside
  // the workspace.
  q0 << 1.75, -1.04, 1.27, -1.79, -2.75, 0.25, -1.45, -2.5, -1.04, -1.32, 3.11,
      0;

  GlobalInverseKinematics global_ik(plant);
  global_ik.AddPostureCost(q0,
                           Eigen::VectorXd::Constant(plant.num_bodies(), 1.0),
                           Eigen::VectorXd::Constant(plant.num_bodies(), 1));

  for (auto _ : state) {
    global_ik.SetInitialGuess(q0);
    auto result = solvers::Solve(global_ik.prog());
    DRAKE_DEMAND(result.is_success());
  }
}

}  // namespace
}  // namespace inverse_kinematics
}  // namespace multibody
}  // namespace drake
