#include <benchmark/benchmark.h>
#include <fmt/format.h>

#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph.h"

namespace drake {
namespace geometry {
namespace internal {

class HydroelasticateBenchmark : public benchmark::Fixture {
 public:
  void SetupScene(const benchmark::State& state) {
    SceneGraphConfig scene_graph_config;
    scene_graph_config.hydroelastic.enabled = state.range(0);
    ProximityProperties props;
    if (scene_graph_config.hydroelastic.enabled == false) {
      // Simulate a manually annotated model.
      props.UpdateProperty(kHydroGroup, kComplianceType,
                           HydroelasticType::kSoft);
      props.UpdateProperty(
          kHydroGroup, kElastic,
          scene_graph_config.hydroelastic.default_hydroelastic_modulus);
      props.UpdateProperty(
          kHydroGroup, kRezHint,
          scene_graph_config.hydroelastic.default_mesh_resolution_hint);
    }
    int num_geoms = state.range(1);
    scene_graph_ = std::make_unique<SceneGraph<double>>(scene_graph_config);

    auto source_id = scene_graph_->RegisterSource("benchmark");
    auto frame_id =
        scene_graph_->RegisterFrame(source_id, GeometryFrame{"frame0"});
    for (int k = 0; k < num_geoms; ++k) {
      auto geom_id = scene_graph_->RegisterGeometry(
          source_id, frame_id,
          std::make_unique<GeometryInstance>(math::RigidTransformd(),
                                             Box(1.0, 1.0, 1.0),
                                             fmt::format("box{}", k)));
      scene_graph_->AssignRole(source_id, geom_id, props);
    }
  }

  std::unique_ptr<SceneGraph<double>> scene_graph_;
};

BENCHMARK_DEFINE_F(HydroelasticateBenchmark, CreateDefaultContext)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  SetupScene(state);
  for (auto _ : state) {
    scene_graph_->CreateDefaultContext();
  }
}

BENCHMARK_REGISTER_F(HydroelasticateBenchmark, CreateDefaultContext)
    ->Unit(benchmark::kMillisecond)
    ->Args({false, 1})
    ->Args({true, 1})
    ->Args({false, 100})
    ->Args({true, 100});

}  // namespace internal
}  // namespace geometry
}  // namespace drake
