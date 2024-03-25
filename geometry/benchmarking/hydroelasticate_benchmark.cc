#include <benchmark/benchmark.h>
#include <fmt/format.h>

#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph.h"

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

class HydroelasticateBenchmark : public benchmark::Fixture {
 public:
  void SetupScene(const benchmark::State& state) {
    // Pull in the benchmark arguments:
    // * compliance_type
    // * num_geoms
    SceneGraphConfig scene_graph_config;
    scene_graph_config.default_proximity_properties.compliance_type =
        GetStringFromHydroelasticType(int2enum(state.range(0)));
    int num_geoms = state.range(1);

    // If the default properties won't build hydroelastic assets, prepare some
    // properties to simulate a manually annotated model.
    ProximityProperties props;
    // Make a convenient alias for the configured default properties.
    const auto& config_props = scene_graph_config.default_proximity_properties;
    if (config_props.compliance_type == "unknown") {
      // Simulate a manually annotated model.
      AddCompliantHydroelasticProperties(*config_props.mesh_resolution_hint,
                                         *config_props.hydroelastic_modulus,
                                         &props);
    }

    // Populate a scene graph with `num_geoms` geometries, and give them all a
    // proximity role.
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
    ->ArgsProduct({{enum2int(HydroelasticType::kUndefined),
                    enum2int(HydroelasticType::kRigid),
                    enum2int(HydroelasticType::kSoft)},
                   {1, 100}});

}  // namespace internal
}  // namespace geometry
}  // namespace drake
