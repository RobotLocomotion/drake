#include <memory>

#include <benchmark/benchmark.h>
#include <fmt/format.h>

#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph.h"

// These benchmarks help measure the impact of applying proximity defaults
// during scene graph context creation. Applying those settings can trigger
// expensive operations in proximity engine. These benchmarks can help
// demonstrate that caching within SceneGraph can avoid repeating expensive
// operations when they are not necessary: in creating multiple contexts when
// the configuration has not changed.

namespace drake {
namespace geometry {
namespace internal {

class ProximityDefaultsBenchmark : public benchmark::Fixture {
 public:
  void SetupScene() {
    // Choose compliant hydroelastic type, since it will likely be used often
    // and likely has the most expensive reification costs.
    scene_graph_config_.default_proximity_properties.compliance_type =
        "compliant";

    // Choose a moderate number of geometries. Try varying this; the benefits
    // of caching will likely still be evident with smaller or larger numbers.
    const int num_geoms = 10;

    // Populate a scene graph with `num_geoms` geometries, and give them all a
    // proximity role.
    scene_graph_ = std::make_unique<SceneGraph<double>>(scene_graph_config_);
    auto source_id = scene_graph_->RegisterSource("benchmark");
    auto frame_id =
        scene_graph_->RegisterFrame(source_id, GeometryFrame{"frame0"});
    ProximityProperties props;
    for (int k = 0; k < num_geoms; ++k) {
      auto geom_id = scene_graph_->RegisterGeometry(
          source_id, frame_id,
          // Even though 'Box' is one of the cheapest shapes to reify, it is
          // still enough to see a measurable effect of repeated reification
          // when caching is defeated.
          std::make_unique<GeometryInstance>(math::RigidTransformd(),
                                             Box(1.0, 1.0, 1.0),
                                             fmt::format("box{}", k)));
      scene_graph_->AssignRole(source_id, geom_id, props);
    }
  }

  std::unique_ptr<SceneGraph<double>> scene_graph_;
  SceneGraphConfig scene_graph_config_;
};

BENCHMARK_DEFINE_F(ProximityDefaultsBenchmark, CreateDefaultContext)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  SetupScene();
  for (auto _ : state) {
    scene_graph_->CreateDefaultContext();
    if (state.range(0)) {
      // This apparently-redundant set_config() call has the side effect of
      // spoiling caching.
      scene_graph_->set_config(scene_graph_config_);
    }
  }
}

BENCHMARK_REGISTER_F(ProximityDefaultsBenchmark, CreateDefaultContext)
    ->Unit(benchmark::kMillisecond)
    ->Args({/* spoil-caching is */ false})
    ->Args({/* spoil-caching is */ true});

}  // namespace internal
}  // namespace geometry
}  // namespace drake
