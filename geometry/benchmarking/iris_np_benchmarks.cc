#include <memory>
#include <string>
#include <vector>

#include <benchmark/benchmark.h>
#include <gflags/gflags.h>

#include "drake/common/text_logging.h"
#include "drake/geometry/optimization/iris.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/tools/performance/fixture_common.h"

/* These scenarios should capture the most important/representative use cases
for IrisNp. They will be used to guide performance optimizations of the core
algorithm. */

namespace drake {
namespace geometry {
namespace optimization {
namespace {

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;
using multibody::MultibodyPlant;
using solvers::SnoptSolver;
using systems::Context;

DEFINE_bool(test, false, "Enable unit test mode.");

// This IIWA + Shelves + Bins fixture is one of the primary examples from
// "Motion Planning around Obstacles with Convex Optimization" by Tobia
// Marcucci, Mark Petersen, David von Wrangel, Russ Tedrake.
// https://arxiv.org/abs/2205.04422
class IiwaWithShelvesAndBins : public benchmark::Fixture {
 public:
  IiwaWithShelvesAndBins() { tools::performance::AddMinMaxStatistics(this); }

  void SetUp(benchmark::State& state) override {
    if (!(SnoptSolver::is_enabled() && SnoptSolver::is_available())) {
      state.SkipWithError(
          "SNOPT is either not enabled or not available. This benchmark should "
          "be evaluated with SNOPT.");
      return;
    }

    // Configure IRIS.
    if (FLAGS_test) {
      iris_options_.iteration_limit = 1;
      iris_options_.num_collision_infeasible_samples = 1;
    } else {
      iris_options_.iteration_limit = 10;
      iris_options_.num_collision_infeasible_samples = 3;
    }
    iris_options_.require_sample_point_is_contained = true;
    iris_options_.relative_termination_threshold = 0.01;
    iris_options_.termination_threshold = -1;

    systems::DiagramBuilder<double> builder;
    plant_ = &multibody::AddMultibodyPlantSceneGraph(&builder, 0.0).plant;
    LoadRobot();
    plant_->Finalize();
    diagram_ = builder.Build();
    diagram_context_ = diagram_->CreateDefaultContext();

    GenerateSeeds();
  }

  void LoadRobot() {
    multibody::Parser parser(plant_);

    std::string model_directives = R"""( 
directives:

# Add iiwa
- add_model:
    name: iiwa
    file: package://drake_models/iiwa_description/urdf/iiwa14_primitive_collision.urdf
    default_joint_positions:
        iiwa_joint_1: [0]
        iiwa_joint_2: [0.3]
        iiwa_joint_3: [0]
        iiwa_joint_4: [-1.8]
        iiwa_joint_5: [0]
        iiwa_joint_6: [1]
        iiwa_joint_7: [1.57]

- add_weld:
    parent: world
    child: iiwa::base

# Add schunk
- add_model:
    name: wsg
    file: package://drake_models/wsg_50_description/sdf/schunk_wsg_50_welded_fingers.sdf

- add_weld:
    parent: iiwa::iiwa_link_7
    child: wsg::body
    X_PC:
      translation: [0, 0, 0.114]
      rotation: !Rpy { deg: [90.0, 0.0, 0.0 ]}

# Add bins
- add_model:
    name: binR
    file: package://drake_models/manipulation_station/bin2.sdf

- add_weld:
    parent: world
    child: binR::bin_base
    X_PC:
      translation: [0, -0.6, 0]
      rotation: !Rpy { deg: [0.0, 0.0, 90.0 ]}

- add_model:
    name: binL
    file: package://drake_models/manipulation_station/bin2.sdf

- add_weld:
    parent: world
    child: binL::bin_base
    X_PC:
      translation: [0, 0.6, 0]
      rotation: !Rpy { deg: [0.0, 0.0, 90.0 ]}
# Add shelves
- add_model:
    name: shelves
    file: package://drake_models/manipulation_station/shelves.sdf

- add_weld:
    parent: world
    child: shelves::shelves_body
    X_PC:
      translation: [0.85, 0, 0.4]
      rotation: !Rpy { deg: [0.0, 0.0, 180.0 ]}

# Add table
- add_model:
    name: table
    file: package://drake_models/manipulation_station/table_wide.sdf

- add_weld:
    parent: world
    child: table::table_body
    X_PC:
      translation: [0.4, 0.0, 0.0]
)""";

    parser.AddModelsFromString(model_directives, ".dmd.yaml");
  }

  VectorXd MyInverseKinematics(const RigidTransformd& X_WE) {
    const auto& E = plant_->GetBodyByName("body").body_frame();
    multibody::InverseKinematics ik(*plant_);
    ik.AddPositionConstraint(E, Vector3d::Zero(), plant_->world_frame(),
                             X_WE.translation(), X_WE.translation());
    ik.AddOrientationConstraint(E, RotationMatrixd(), plant_->world_frame(),
                                X_WE.rotation(), 0.001);

    auto* prog = ik.get_mutable_prog();
    const auto& q = ik.q();
    VectorXd q0 = plant_->GetPositions(ik.context());
    prog->AddQuadraticErrorCost(MatrixXd::Identity(q.size(), q.size()), q0, q);
    prog->SetInitialGuess(q, q0);
    auto result = solvers::Solve(*prog);
    DRAKE_DEMAND(result.is_success());
    return result.GetSolution(q);
  }

  void GenerateSeeds() {
    auto context = plant_->CreateDefaultContext();
    seeds_.clear();
    seeds_.emplace_back("Home Position", plant_->GetPositions(*context));
    seeds_.emplace_back("Left Bin", MyInverseKinematics(RigidTransformd(
                                        RollPitchYawd(M_PI / 2, M_PI, 0),
                                        Vector3d(0.0, 0.6, 0.22))));
    if (FLAGS_test) {
      // In test mode, only use the first two seeds.
      return;
    }
    seeds_.emplace_back("Right Bin", MyInverseKinematics(RigidTransformd(
                                         RollPitchYawd(M_PI / 2, M_PI, M_PI),
                                         Vector3d(0.0, -0.6, 0.22))));
    seeds_.emplace_back("Above Shelve", MyInverseKinematics(RigidTransformd(
                                            RollPitchYawd(0, -M_PI, -M_PI / 2),
                                            Vector3d(0.75, 0, 0.9))));
    seeds_.emplace_back("Top Rack", MyInverseKinematics(RigidTransformd(
                                        RollPitchYawd(0, -M_PI, -M_PI / 2),
                                        Vector3d(0.75, 0, 0.67))));
    seeds_.emplace_back("Middle Rack", MyInverseKinematics(RigidTransformd(
                                           RollPitchYawd(0, -M_PI, -M_PI / 2),
                                           Vector3d(0.75, 0, 0.41))));
  }

  void GenerateAllRegions() {
    // Because we use each computed region as an obstacle for the next, we need
    // to run them all in a single benchmark.
    iris_options_.configuration_obstacles.clear();
    Context<double>& plant_context =
        plant_->GetMyMutableContextFromRoot(diagram_context_.get());
    for (const auto& [name, q0] : seeds_) {
      log()->info("Computing region for seed: {}", name);
      plant_->SetPositions(&plant_context, q0);
      HPolyhedron hpoly = IrisNp(*plant_, plant_context, iris_options_);
      iris_options_.configuration_obstacles.emplace_back(hpoly.Scale(0.95));
    }
  }

 protected:
  IrisOptions iris_options_{};
  std::unique_ptr<systems::Diagram<double>> diagram_;
  std::unique_ptr<Context<double>> diagram_context_;
  MultibodyPlant<double>* plant_;
  std::vector<std::pair<std::string, VectorXd>> seeds_;
};

BENCHMARK_DEFINE_F(IiwaWithShelvesAndBins, GenerateAllRegions)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  for (auto _ : state) {
    GenerateAllRegions();
  }
}
BENCHMARK_REGISTER_F(IiwaWithShelvesAndBins, GenerateAllRegions)
    ->Unit(benchmark::kSecond);

}  // namespace
}  // namespace optimization
}  // namespace geometry
}  // namespace drake

// TODO(russt): Get rid of this iff we can find a way to make the benchmark run
// only once without it.
int main(int argc, char** argv) {
  benchmark::Initialize(&argc, argv);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  benchmark::RunSpecifiedBenchmarks();
  return 0;
}
