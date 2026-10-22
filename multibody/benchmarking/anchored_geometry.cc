#include <memory>
#include <string>
#include <utility>

#include <benchmark/benchmark.h>
#include <fmt/format.h>

#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/tools/performance/fixture_common.h"

/* Measures the cost of collision geometry that is fixed in the world but is
registered on bodies welded to the world (as the Parser does for static models)
versus the same geometry registered on the world body (anchored geometry).

The scene is a grid of N static boxes that share their edges, so neighboring
boxes have overlapping bounding boxes (like terrain made of convex pieces), and
one free box resting on the grid. Both variants have identical geometry, poses
and contacts; only the body the static boxes are registered on differs. See
issues #8943 and #24888. */

namespace drake {
namespace multibody {
namespace {

using geometry::Box;
using geometry::ProximityProperties;
using geometry::QueryObject;
using geometry::SceneGraph;
using math::RigidTransformd;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;

constexpr double kTimeStep = 1e-3;
constexpr double kCell = 1.0;  // Grid pitch and box size (m).

// Benchmark "Args": the number of static boxes per grid side, and where the
// static boxes are registered.
enum Registration { kWelded = 0, kWorld = 1 };

class AnchoredGeometry : public benchmark::Fixture {
 public:
  AnchoredGeometry() { tools::performance::AddMinMaxStatistics(this); }

  void SetUp(benchmark::State& state) override {
    const int side = state.range(0);
    const auto registration = static_cast<Registration>(state.range(1));

    DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) =
        AddMultibodyPlantSceneGraph(&builder, kTimeStep);
    plant_->set_discrete_contact_approximation(
        DiscreteContactApproximation::kSap);

    ProximityProperties properties;
    geometry::AddContactMaterial({}, {}, CoulombFriction<double>(0.8, 0.8),
                                 &properties);
    const Box cell(kCell, kCell, kCell);
    for (int i = 0; i < side; ++i) {
      for (int j = 0; j < side; ++j) {
        const std::string name = fmt::format("cell_{}_{}", i, j);
        const RigidTransformd X_WC(
            Eigen::Vector3d(i * kCell, j * kCell, -0.5 * kCell));
        if (registration == kWorld) {
          plant_->RegisterCollisionGeometry(plant_->world_body(), X_WC, cell,
                                            name, properties);
        } else {
          const RigidBody<double>& body = plant_->AddRigidBody(name);
          plant_->WeldFrames(plant_->world_frame(), body.body_frame(), X_WC);
          plant_->RegisterCollisionGeometry(body, RigidTransformd(), cell, name,
                                            properties);
        }
      }
    }
    // The free box rests on the middle of the grid.
    const RigidBody<double>& box = plant_->AddRigidBody(
        "box", SpatialInertia<double>::SolidBoxWithMass(10, 0.5, 0.5, 0.5));
    plant_->RegisterCollisionGeometry(box, RigidTransformd(),
                                      Box(0.5, 0.5, 0.5), "box", properties);
    plant_->Finalize();
    diagram_ = builder.Build();

    context_ = diagram_->CreateDefaultContext();
    plant_context_ = &plant_->GetMyMutableContextFromRoot(context_.get());
    root_context_ = context_.get();
    const double middle = 0.5 * (side - 1) * kCell;
    plant_->SetFreeBodyPose(
        plant_context_, box,
        RigidTransformd(Eigen::Vector3d(middle, middle, 0.249)));
    simulator_ = std::make_unique<systems::Simulator<double>>(
        *diagram_, std::move(context_));
    simulator_->Initialize();
  }

  void TearDown(benchmark::State&) override {
    simulator_.reset();
    diagram_.reset();
  }

 protected:
  // Invalidates the poses so that every query recomputes them, as a
  // simulation step does.
  void TouchPositions() {
    plant_context_->get_mutable_discrete_state()
        .get_mutable_vector()
        .SetAtIndex(0, plant_context_->get_discrete_state().value()[0]);
  }

  MultibodyPlant<double>* plant_{};
  SceneGraph<double>* scene_graph_{};
  std::unique_ptr<Diagram<double>> diagram_;
  std::unique_ptr<Context<double>> context_;
  Context<double>* root_context_{};  // Owned by simulator_ after SetUp().
  Context<double>* plant_context_{};
  std::unique_ptr<systems::Simulator<double>> simulator_;
};

// The point-pair contact query: geometry pose update and broadphase.
BENCHMARK_DEFINE_F(AnchoredGeometry, PointPairPenetration)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  const auto& query_port = scene_graph_->get_query_output_port();
  const Context<double>& sg_context =
      scene_graph_->GetMyContextFromRoot(*root_context_);
  for (auto _ : state) {
    TouchPositions();
    const auto& query = query_port.Eval<QueryObject<double>>(sg_context);
    benchmark::DoNotOptimize(query.ComputePointPairPenetration());
  }
}

// One simulation step: the plant's discrete update (SAP), with its contact
// query.
BENCHMARK_DEFINE_F(AnchoredGeometry, SimulationStep)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  for (auto _ : state) {
    simulator_->AdvanceTo(root_context_->get_time() + kTimeStep);
  }
}

// Grid sides 10, 20, 40 (100, 400, 1600 static boxes); welded vs. world.
BENCHMARK_REGISTER_F(AnchoredGeometry, PointPairPenetration)
    ->Unit(benchmark::kMicrosecond)
    ->ArgsProduct({{10, 20, 40}, {kWelded, kWorld}});
BENCHMARK_REGISTER_F(AnchoredGeometry, SimulationStep)
    ->Unit(benchmark::kMicrosecond)
    ->ArgsProduct({{10, 20, 40}, {kWelded, kWorld}});

}  // namespace
}  // namespace multibody
}  // namespace drake

BENCHMARK_MAIN();
