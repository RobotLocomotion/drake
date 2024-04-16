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

namespace {

// We can't pass enums through the google benchmark arguments feature. Here are
// some conversion operators.

int64_t enum2int(HydroelasticType type) {
  return static_cast<int64_t>(type);
}

HydroelasticType int2enum(int64_t arg) {
  return static_cast<HydroelasticType>(arg);
}

}  // namespace

class ProximityDefaultsBenchmark : public benchmark::Fixture {
 public:
  void SetupScene(const benchmark::State& state) {
    // Pull in the benchmark arguments:
    // * compliance_type
    // * num_geoms
    scene_graph_config_.default_proximity_properties.compliance_type =
        GetStringFromHydroelasticType(int2enum(state.range(0)));
    int num_geoms = state.range(1);

    // If the default properties won't build hydroelastic assets, prepare some
    // properties to simulate a manually annotated model.
    ProximityProperties props;
    // Make a convenient alias for the configured default properties.
    const auto& config_props = scene_graph_config_.default_proximity_properties;
    if (config_props.compliance_type == "unknown") {
      // Simulate a manually annotated model.
      AddCompliantHydroelasticProperties(*config_props.resolution_hint,
                                         *config_props.hydroelastic_modulus,
                                         &props);
    }

    // Populate a scene graph with `num_geoms` geometries, and give them all a
    // proximity role.
    scene_graph_ = std::make_unique<SceneGraph<double>>(scene_graph_config_);
    auto source_id = scene_graph_->RegisterSource("benchmark");
    auto frame_id =
        scene_graph_->RegisterFrame(source_id, GeometryFrame{"frame0"});
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
  SetupScene(state);
  for (auto _ : state) {
    scene_graph_->CreateDefaultContext();
    if (state.range(2)) {
      // This apparently-redundant set_config() call has the side effect of
      // spoiling caching.
      scene_graph_->set_config(scene_graph_config_);
    }
  }
}

BENCHMARK_REGISTER_F(ProximityDefaultsBenchmark, CreateDefaultContext)
    ->Unit(benchmark::kMillisecond)
    ->ArgsProduct({// Sweep hydroelastic types.
                   {enum2int(HydroelasticType::kUndefined),
                    enum2int(HydroelasticType::kRigid),
                    enum2int(HydroelasticType::kSoft)},
                   // Sweep some sample numbers of geometries.
                   {1, 100},
                   // Sweep spoil-caching = {false, true}.
                   {false, true}});

}  // namespace internal
}  // namespace geometry
}  // namespace drake
