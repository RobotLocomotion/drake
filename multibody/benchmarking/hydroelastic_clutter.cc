#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <numbers>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <benchmark/benchmark.h>
#include <fmt/format.h>
#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/memory_file.h"
#include "drake/common/never_destroyed.h"
#include "drake/geometry/in_memory_mesh.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/geometry/rgba.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/tools/performance/fixture_common.h"

/* Measures hydroelastic-with-fallback contact on a cluttered, procedurally
generated "desk" scene, without needing any model files.

The scene mimics what makes real cluttered scenes expensive for the
collision/contact pipeline:

  - Anchored furniture made of many small, oriented rigid pieces: a desk
    surface tiled with yawed rigid boxes, plus thin rigid sticks (boxes and
    convex prisms) at random orientations. These add many geometries that are
    near, but not in, contact with the free bodies.
  - Free "manipulands" in stacked columns resting on the desk. Like convex
    decompositions of real objects, each has one compliant convex hull plus a
    ring of rigid convex parts. The result is a mix of compliant-rigid,
    compliant-compliant and (via the rigid parts) point-pair fallback contact.

All geometry is generated in code with a deterministic, platform-independent
random sequence, so the scene is identical on every machine.

Two things are timed, both on the scene after it has settled:
  - ContactSurfaces: the SceneGraph geometry query,
    ComputeContactSurfacesWithFallback().
  - SimulationStep: one full discrete step (geometry query, contact-pair
    assembly, and the SAP solve).

The Args are {number of columns, desk piece edge length in mm}. Each column has
two free bodies, so nv = 12 * columns. The default 12-column, 60 mm case has
nv = 144 and produces roughly 80 hydroelastic contacts, similar in size
to a cluttered tabletop. The 250 mm case has the same free bodies on a coarser
desk (fewer, larger contacting pairs). The 64-column case checks how costs
scale with the number of degrees of freedom (e.g., Jacobian assembly).

To inspect a scene, pass --scene_html_dir=<dir>. For each case that runs (see
--benchmark_filter), a recording of the scene settling is written to
<dir>/hydroelastic_clutter_<columns>_<tile>.html, which can be opened in any
browser. The recording uses a separate copy of the scene, so the timed one is
unaffected. */

DEFINE_string(scene_html_dir, "",
              "If not empty, writes a Meshcat recording of each case's scene "
              "settling to an html file in this directory. A relative path is "
              "relative to the directory `bazel run` was invoked from.");

namespace drake {
namespace multibody {
namespace {

using geometry::Box;
using geometry::Convex;
using geometry::HydroelasticContactRepresentation;
using geometry::InMemoryMesh;
using geometry::PenetrationAsPointPair;
using geometry::ProximityProperties;
using geometry::QueryObject;
using geometry::SceneGraph;
using math::RigidTransformd;
using math::RollPitchYawd;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;

constexpr double kTimeStep = 0.01;
// Time to let the scene settle (s). At 0.5 s the bodies are still falling
// and tumbling. See the settled_max_speed counter for how well a given case
// has settled.
constexpr double kWarmupTime = 1.5;

constexpr int kLayers = 2;        // Free bodies per column.
constexpr int kPartsPerBody = 6;  // Rigid convex parts per free body.
constexpr int kNumSticks = 60;    // Anchored thin rigid pieces.
constexpr double kSize = 0.1;     // Free body size (m).
constexpr double kDeskTop = 0.0;  // Height of the desk surface (m).
// Hunt-Crossley dissipation (s/m) for all geometries; a typical value for
// hydroelastic contact, so that bodies settle realistically.
constexpr double kDissipation = 1.0;

// A small deterministic random sequence (splitmix64). We avoid <random>
// distributions because their output is implementation-defined, and we want
// the identical scene on every platform.
class Rng {
 public:
  explicit Rng(uint64_t seed) : state_(seed) {}

  // Uniform in [0, 1).
  double Uniform() {
    state_ += 0x9e3779b97f4a7c15ULL;
    uint64_t z = state_;
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
    z ^= z >> 31;
    return static_cast<double>(z >> 11) * (1.0 / 9007199254740992.0);
  }

  double Uniform(double lo, double hi) { return lo + (hi - lo) * Uniform(); }

 private:
  uint64_t state_;
};

// Returns a convex n-sided prism (centered at the origin, axis along z) as an
// in-memory .obj.
InMemoryMesh MakePrism(double radius, double height, int sides) {
  std::string obj;
  for (const double z : {-0.5 * height, 0.5 * height}) {
    for (int i = 0; i < sides; ++i) {
      const double a = 2 * std::numbers::pi * i / sides;
      obj += fmt::format("v {:.6f} {:.6f} {:.6f}\n", radius * std::cos(a),
                         radius * std::sin(a), z);
    }
  }
  std::string bottom = "f";
  std::string top = "f";
  for (int i = 0; i < sides; ++i) {
    bottom += fmt::format(" {}", sides - i);
    top += fmt::format(" {}", sides + i + 1);
  }
  obj += bottom + "\n" + top + "\n";
  for (int i = 0; i < sides; ++i) {
    const int j = (i + 1) % sides;
    obj += fmt::format("f {} {} {} {}\n", i + 1, j + 1, sides + j + 1,
                       sides + i + 1);
  }
  return InMemoryMesh{MemoryFile(std::move(obj), ".obj", "prism")};
}

ProximityProperties MakeRigidProperties(const geometry::Rgba& color) {
  ProximityProperties properties;
  geometry::AddContactMaterial(kDissipation, {}, {}, &properties);
  geometry::AddRigidHydroelasticProperties(0.05, &properties);
  // The ("phong", "diffuse") `color` is only used by --scene_html_dir; it has
  // no effect on contact.
  properties.AddProperty("phong", "diffuse", color);
  return properties;
}

ProximityProperties MakeCompliantProperties() {
  ProximityProperties properties;
  geometry::AddContactMaterial(kDissipation, {}, {}, &properties);
  geometry::AddCompliantHydroelasticProperties(kSize / 3, 1e6, &properties);
  // The ("phong", "diffuse") `color` is only used by --scene_html_dir; it has
  // no effect on contact.
  properties.AddProperty("phong", "diffuse", geometry::Rgba(0.3, 0.5, 0.9));
  return properties;
}

struct FreeBody {
  const RigidBody<double>* body;
  RigidTransformd X_WB;
};

// Populates `plant` with the scene and returns the free bodies and their
// initial poses. `tile` is the edge length of the desk pieces (m).
std::vector<FreeBody> AddScene(int columns, double tile,
                               MultibodyPlant<double>* plant) {
  // Each part of the scene has its own generator. The number of desk tiles
  // depends on `tile`; sharing one generator would make the sticks and free
  // bodies differ whenever `tile` changes, so that cases that differ only in
  // `tile` would not be comparable.
  Rng tile_rng(1);
  Rng stick_rng(2);
  Rng body_rng(3);
  // The desk is translucent so that the sticks beneath it can be seen.
  const ProximityProperties desk =
      MakeRigidProperties(geometry::Rgba(0.6, 0.6, 0.6, 0.4));
  const ProximityProperties stick =
      MakeRigidProperties(geometry::Rgba(0.8, 0.5, 0.2));
  const ProximityProperties rigid =
      MakeRigidProperties(geometry::Rgba(0.7, 0.2, 0.2));
  const ProximityProperties compliant = MakeCompliantProperties();

  const int side = static_cast<int>(std::ceil(std::sqrt(columns)));
  const double pitch = kSize * 1.6;
  const double extent = side * pitch + 0.4;

  // The floor, so that nothing can fall forever.
  plant->RegisterCollisionGeometry(
      plant->world_body(), RigidTransformd(Eigen::Vector3d(0, 0, -1.05)),
      Box(20, 20, 0.1), "floor", desk);

  // The desk: overlapping yawed tiles whose tops are flush with the surface.
  const int num_tiles = static_cast<int>(std::ceil(extent / (tile * 0.8)));
  int k = 0;
  for (int i = 0; i < num_tiles; ++i) {
    for (int j = 0; j < num_tiles; ++j) {
      const double x = (i - num_tiles / 2.0) * tile * 0.8;
      const double y = (j - num_tiles / 2.0) * tile * 0.8;
      const double yaw = tile_rng.Uniform(0, std::numbers::pi);
      plant->RegisterCollisionGeometry(
          plant->world_body(),
          RigidTransformd(RollPitchYawd(0, 0, yaw),
                          Eigen::Vector3d(x, y, kDeskTop - 0.02)),
          Box(tile, tile, 0.04), fmt::format("tile{}", k++), desk);
    }
  }

  // Sticks: thin rigid pieces below the surface at random orientations,
  // alternating convex prisms and boxes. They are near-misses for the free
  // bodies: close, but never in contact.
  //
  // A stick of length L tilted at any angle reaches at most L/2 (plus a
  // little for its cross-section) above its center. The sticks are placed deep
  // enough that even the tip of a vertical stick stays below the desk surface.
  constexpr double kStickLength = kSize * 1.5;
  constexpr double kMinStickDepth = 0.5 * kStickLength + 0.015;
  const InMemoryMesh stick_mesh = MakePrism(0.01, kStickLength, 6);
  for (int s = 0; s < kNumSticks; ++s) {
    const double x = stick_rng.Uniform(-extent / 2, extent / 2);
    const double y = stick_rng.Uniform(-extent / 2, extent / 2);
    const double z =
        kDeskTop - stick_rng.Uniform(kMinStickDepth, kMinStickDepth + 0.1);
    const double roll = stick_rng.Uniform(0, std::numbers::pi);
    const double pitch_angle = stick_rng.Uniform(0, std::numbers::pi);
    const double yaw = stick_rng.Uniform(0, std::numbers::pi);
    const RigidTransformd X_WS(RollPitchYawd(roll, pitch_angle, yaw),
                               Eigen::Vector3d(x, y, z));
    const std::string name = fmt::format("stick{}", s);
    if (s % 2 == 0) {
      plant->RegisterCollisionGeometry(plant->world_body(), X_WS,
                                       Convex(InMemoryMesh(stick_mesh)), name,
                                       stick);
    } else {
      plant->RegisterCollisionGeometry(plant->world_body(), X_WS,
                                       Box(0.02, 0.02, kStickLength), name,
                                       stick);
    }
  }

  // The free bodies: columns of stacked "manipulands".
  std::vector<InMemoryMesh> hulls;
  for (int v = 0; v < 6; ++v) {
    hulls.push_back(MakePrism(kSize * (0.28 + 0.02 * v), kSize * 0.6, 8 + v));
  }
  const InMemoryMesh part_mesh = MakePrism(kSize * 0.12, kSize * 0.5, 6);
  const auto inertia =
      SpatialInertia<double>::SolidBoxWithDensity(500.0, kSize, kSize, kSize);

  std::vector<FreeBody> free_bodies;
  int n = 0;
  for (int c = 0; c < columns; ++c) {
    const double cx = (c % side - side / 2.0) * pitch;
    const double cy = (c / side - side / 2.0) * pitch;
    for (int layer = 0; layer < kLayers; ++layer) {
      const RigidBody<double>& body =
          plant->AddRigidBody(fmt::format("b{}", n), inertia);
      plant->RegisterCollisionGeometry(body, RigidTransformd(),
                                       Convex(InMemoryMesh(hulls[n % 6])),
                                       fmt::format("hull{}", n), compliant);
      for (int q = 0; q < kPartsPerBody; ++q) {
        const double angle =
            2 * std::numbers::pi * q / kPartsPerBody + body_rng.Uniform(0, 1);
        const double yaw = body_rng.Uniform(0, std::numbers::pi);
        plant->RegisterCollisionGeometry(
            body,
            RigidTransformd(RollPitchYawd(0, 0, yaw),
                            Eigen::Vector3d(kSize * 0.45 * std::cos(angle),
                                            kSize * 0.45 * std::sin(angle), 0)),
            Convex(InMemoryMesh(part_mesh)), fmt::format("part{}_{}", n, q),
            rigid);
      }
      const double roll = body_rng.Uniform(-0.1, 0.1);
      const double pitch_angle = body_rng.Uniform(-0.1, 0.1);
      const double yaw = body_rng.Uniform(-0.1, 0.1);
      // Each layer starts a little above where it will rest (up to a few
      // centimeters), so the bodies drop onto the desk and onto each other
      // and settle during the warmup.
      free_bodies.push_back(
          {&body,
           RigidTransformd(
               RollPitchYawd(roll, pitch_angle, yaw),
               Eigen::Vector3d(
                   cx, cy, kDeskTop + kSize * (layer + 0.5) * 0.99 + 0.003))});
      ++n;
    }
  }
  return free_bodies;
}

// A settled scene, ready to simulate.
struct Scene {
  MultibodyPlant<double>* plant{};
  SceneGraph<double>* scene_graph{};
  std::unique_ptr<Diagram<double>> diagram;
  // Declared after `diagram`, which it references, so it's destroyed first.
  std::unique_ptr<systems::Simulator<double>> simulator;
};

// Builds the scene for one case and lets it settle. If `meshcat` is not null,
// the scene's proximity geometry is visualized and its settling is recorded.
Scene MakeSettledScene(int columns, double tile,
                       std::shared_ptr<geometry::Meshcat> meshcat = nullptr) {
  Scene scene;
  MultibodyPlantConfig config;
  config.time_step = kTimeStep;
  config.contact_model = "hydroelastic_with_fallback";
  DiagramBuilder<double> builder;
  std::tie(scene.plant, scene.scene_graph) =
      AddMultibodyPlant(config, &builder);
  const std::vector<FreeBody> free_bodies =
      AddScene(columns, tile, scene.plant);
  scene.plant->Finalize();
  geometry::MeshcatVisualizer<double>* visualizer{};
  if (meshcat != nullptr) {
    geometry::MeshcatVisualizerParams params;
    params.role = geometry::Role::kProximity;
    visualizer = &geometry::MeshcatVisualizer<double>::AddToBuilder(
        &builder, *scene.scene_graph, std::move(meshcat), std::move(params));
  }
  scene.diagram = builder.Build();

  std::unique_ptr<Context<double>> context =
      scene.diagram->CreateDefaultContext();
  Context<double>& plant_context =
      scene.plant->GetMyMutableContextFromRoot(context.get());
  for (const FreeBody& fb : free_bodies) {
    scene.plant->SetFreeBodyPose(&plant_context, *fb.body, fb.X_WB);
  }
  scene.simulator = std::make_unique<systems::Simulator<double>>(
      *scene.diagram, std::move(context));

  // Let the scene settle, so that we time steady-state contact.
  if (visualizer != nullptr) visualizer->StartRecording();
  scene.simulator->Initialize();
  scene.simulator->AdvanceTo(kWarmupTime);
  if (visualizer != nullptr) {
    visualizer->StopRecording();
    visualizer->PublishRecording();
  }
  return scene;
}

// If requested by --scene_html_dir, writes the recording of the given case's
// scene settling. Each case is written at most once per program run.
void MaybeWriteSceneHtml(int columns, int tile_mm) {
  if (FLAGS_scene_html_dir.empty()) return;
  static never_destroyed<std::set<std::pair<int, int>>> written;
  if (!written.access().insert({columns, tile_mm}).second) return;

  // Resolve a relative path against the directory the user expects (rather
  // than the runfiles tree) under `bazel run`, or the undeclared outputs
  // directory under `bazel test`.
  std::filesystem::path dir(FLAGS_scene_html_dir);
  if (dir.is_relative()) {
    for (const char* env :
         {"BUILD_WORKING_DIRECTORY", "TEST_UNDECLARED_OUTPUTS_DIR"}) {
      if (const char* base = std::getenv(env)) {
        dir = std::filesystem::path(base) / dir;
        break;
      }
    }
  }
  std::filesystem::create_directories(dir);
  const std::filesystem::path path =
      dir / fmt::format("hydroelastic_clutter_{}_{}.html", columns, tile_mm);

  auto meshcat = std::make_shared<geometry::Meshcat>();
  MakeSettledScene(columns, tile_mm / 1000.0, meshcat);
  std::ofstream(path) << meshcat->StaticHtml();
  fmt::print("Wrote {}\n", path.string());
}

class HydroelasticClutter : public benchmark::Fixture {
 public:
  HydroelasticClutter() { tools::performance::AddMinMaxStatistics(this); }

  // Google Benchmark calls SetUp() once per timing run, and more than once
  // per case while it searches for an iteration count. Building and settling
  // the larger scenes takes seconds; pass --benchmark_min_time=<N>x to bound
  // the number of runs.
  void SetUp(benchmark::State& state) override {
    const int columns = state.range(0);
    const int tile_mm = state.range(1);
    MaybeWriteSceneHtml(columns, tile_mm);

    Scene scene = MakeSettledScene(columns, tile_mm / 1000.0);
    plant_ = scene.plant;
    scene_graph_ = scene.scene_graph;
    diagram_ = std::move(scene.diagram);
    simulator_ = std::move(scene.simulator);
    root_context_ = &simulator_->get_mutable_context();
    plant_context_ = &plant_->GetMyMutableContextFromRoot(root_context_);
    next_step_ = static_cast<int>(std::round(kWarmupTime / kTimeStep));

    // Record the size of the problem, for the benchmark report.
    const auto& query_port = scene_graph_->get_query_output_port();
    const auto& query = query_port.Eval<QueryObject<double>>(
        scene_graph_->GetMyContextFromRoot(*root_context_));
    std::vector<geometry::ContactSurface<double>> surfaces;
    std::vector<PenetrationAsPointPair<double>> point_pairs;
    query.ComputeContactSurfacesWithFallback(
        HydroelasticContactRepresentation::kPolygon, &surfaces, &point_pairs);
    num_nv_ = plant_->num_velocities();
    num_hydro_ = static_cast<int>(surfaces.size());
    num_point_pair_ = static_cast<int>(point_pairs.size());

    // Record how well the scene has settled, as the largest speed of any
    // generalized velocity (m/s or rad/s).
    settled_max_speed_ =
        plant_->GetVelocities(*plant_context_).cwiseAbs().maxCoeff();

    // A scene with no contact would make every number meaningless.
    DRAKE_DEMAND(num_hydro_ + num_point_pair_ > 0);
  }

  void TearDown(benchmark::State&) override {
    simulator_.reset();
    diagram_.reset();
  }

 protected:
  void ReportProblemSize(benchmark::State* state) const {
    state->counters["nv"] = num_nv_;
    state->counters["hydro_contacts"] = num_hydro_;
    state->counters["point_pair_contacts"] = num_point_pair_;
    state->counters["settled_max_speed"] = settled_max_speed_;
  }

  // Invalidates the poses so that every query recomputes them, as a
  // simulation step does. We deliberately leave the cache enabled (rather than
  // disabling it) because users run with caching on, and that is the
  // configuration we want to measure. Writing the positions (even with their
  // current values) bumps the plant state's cache tickets. If this ever
  // stopped forcing recomputation, ContactSurfaces would time a cache hit and
  // its per-iteration cost would collapse to nearly zero.
  void TouchPositions() {
    plant_->SetPositions(plant_context_, plant_->GetPositions(*plant_context_));
  }

  MultibodyPlant<double>* plant_{};
  SceneGraph<double>* scene_graph_{};
  std::unique_ptr<Diagram<double>> diagram_;
  Context<double>* root_context_{};  // Owned by simulator_.
  Context<double>* plant_context_{};
  std::unique_ptr<systems::Simulator<double>> simulator_;
  int next_step_{};
  int num_nv_{};
  int num_hydro_{};
  int num_point_pair_{};
  double settled_max_speed_{};
};

// The geometry query alone: poses, broadphase, and all narrowphase contact
// computation (compliant-rigid, compliant-compliant, and point-pair fallback).
BENCHMARK_DEFINE_F(HydroelasticClutter, ContactSurfaces)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  const auto& query_port = scene_graph_->get_query_output_port();
  const Context<double>& sg_context =
      scene_graph_->GetMyContextFromRoot(*root_context_);
  std::vector<geometry::ContactSurface<double>> surfaces;
  std::vector<PenetrationAsPointPair<double>> point_pairs;
  for (auto _ : state) {
    // The query appends to its outputs, so start each iteration empty. Freeing
    // the previous results is not part of what we measure.
    state.PauseTiming();
    surfaces.clear();
    point_pairs.clear();
    state.ResumeTiming();
    TouchPositions();
    const auto& query = query_port.Eval<QueryObject<double>>(sg_context);
    query.ComputeContactSurfacesWithFallback(
        HydroelasticContactRepresentation::kPolygon, &surfaces, &point_pairs);
    benchmark::DoNotOptimize(surfaces);
    benchmark::DoNotOptimize(point_pairs);
  }
  ReportProblemSize(&state);
}

// One full discrete step: the geometry query, contact-pair assembly
// (including Jacobians), and the SAP solve.
BENCHMARK_DEFINE_F(HydroelasticClutter, SimulationStep)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  for (auto _ : state) {
    simulator_->AdvanceTo(++next_step_ * kTimeStep);
  }
  ReportProblemSize(&state);
}

// {columns, desk piece edge length in mm}. The first (tiny) case exists for
// the smoke test.
void AddArgs(benchmark::Benchmark* b) {
  b->Unit(benchmark::kMillisecond)
      ->Args({3, 60})
      ->Args({12, 250})
      ->Args({12, 60})
      ->Args({64, 60});
}

BENCHMARK_REGISTER_F(HydroelasticClutter, ContactSurfaces)->Apply(AddArgs);
BENCHMARK_REGISTER_F(HydroelasticClutter, SimulationStep)->Apply(AddArgs);

}  // namespace
}  // namespace multibody
}  // namespace drake

// Like BENCHMARK_MAIN(), but also parses gflags. We don't use
// //tools/performance:gflags_main; its memory manager adds another run, and so
// another (slow) SetUp(), per case.
int main(int argc, char** argv) {
  benchmark::MaybeReenterWithoutASLR(argc, argv);
  benchmark::Initialize(&argc, argv);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (benchmark::ReportUnrecognizedArguments(argc, argv)) return 1;
  benchmark::RunSpecifiedBenchmarks();
  benchmark::Shutdown();
  return 0;
}
