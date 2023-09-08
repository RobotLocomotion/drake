#include <benchmark/benchmark.h>
#include <fmt/format.h>

#include "drake/geometry/scene_graph.h"

namespace drake {
namespace geometry {
namespace internal {

class HydroelastizeBenchmark : public benchmark::Fixture {
 public:
  void SetupScene(const benchmark::State& state) {
    SceneGraphConfig scene_graph_config;
    scene_graph_config.hydroelastize = state.range(0);
    int num_geoms = state.range(1);
    scene_graph_ = std::make_unique<SceneGraph<double>>(scene_graph_config);

    auto source_id = scene_graph_->RegisterSource("benchmark");
    auto frame_id =
        scene_graph_->RegisterFrame(source_id, GeometryFrame{"frame0"});
    for (int k = 0; k < num_geoms; ++k) {
      auto geom_id = scene_graph_->RegisterGeometry(
          source_id, frame_id,
          std::make_unique<GeometryInstance>(math::RigidTransformd(),
                                             Sphere(1000.0),
                                             fmt::format("sphere{}", k)));
      scene_graph_->AssignRole(source_id, geom_id, ProximityProperties());
    }
  }

  std::unique_ptr<SceneGraph<double>> scene_graph_;
};

BENCHMARK_DEFINE_F(HydroelastizeBenchmark, CreateDefaultContext)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  SetupScene(state);
  for (auto _ : state) {
    scene_graph_->CreateDefaultContext();
  }
}

BENCHMARK_REGISTER_F(HydroelastizeBenchmark, CreateDefaultContext)
    ->Args({false, 1})
    ->Args({true, 1})
    ->Args({false, 4})
    ->Args({true, 4});

}  // namespace internal
}  // namespace geometry
}  // namespace drake
