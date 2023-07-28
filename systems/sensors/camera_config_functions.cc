#include "drake/systems/sensors/camera_config_functions.h"

#include <initializer_list>
#include <map>
#include <memory>
#include <string>

#include <fmt/format.h>

#include "drake/common/eigen_types.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/nice_type_name.h"
#include "drake/geometry/render/render_camera.h"
#include "drake/geometry/render_gl/factory.h"
#include "drake/geometry/render_vtk/factory.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/scoped_names.h"
#include "drake/systems/lcm/lcm_config_functions.h"
#include "drake/systems/sensors/rgbd_sensor.h"
#include "drake/systems/sensors/sim_rgbd_sensor.h"

namespace drake {
namespace systems {
namespace sensors {

using drake::lcm::DrakeLcmInterface;
using drake::systems::lcm::LcmBuses;
using Eigen::Vector3d;
using geometry::MakeRenderEngineGl;
using geometry::MakeRenderEngineVtk;
using geometry::RenderEngineGlParams;
using geometry::RenderEngineVtkParams;
using geometry::SceneGraph;
using geometry::render::ColorRenderCamera;
using geometry::render::DepthRenderCamera;
using internal::AddSimRgbdSensor;
using internal::AddSimRgbdSensorLcmPublisher;
using internal::SimRgbdSensor;
using math::RigidTransformd;
using multibody::Frame;
using multibody::MultibodyPlant;
using multibody::parsing::GetScopedFrameByName;

namespace {

// Boilerplate for std::visit.
template <class... Ts>
struct overloaded : Ts... {
  using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

template <typename ParamsType>
const std::string& GetEngineTypeNameFromParameters(const ParamsType&) {
  // TODO(SeanCurtis-TRI): Find some slicker way of getting these names, one
  // doesn't require instantiating throw away engines. Although, this only
  // happens once per ParamType, so that might not be so bad.
  if constexpr (std::is_same_v<ParamsType, RenderEngineVtkParams>) {
    static const never_destroyed<std::string> type_name(
        NiceTypeName::Get(MakeRenderEngineVtk({})));
    return type_name.access();
  } else if constexpr (std::is_same_v<ParamsType, RenderEngineGlParams>) {
    static const never_destroyed<std::string> type_name(
        NiceTypeName::Get(MakeRenderEngineGl({})));
    return type_name.access();
  } else if constexpr (std::is_same_v<ParamsType,
                                      RenderEngineGltfClientParams>) {
    static const never_destroyed<std::string> type_name(
        NiceTypeName::Get(MakeRenderEngineGltfClient({})));
    return type_name.access();
  } else {
    DRAKE_UNREACHABLE();
  }
}

// Validates the render engine specification in `config`. If the specification
// is valid, upon return, the specified RenderEngine is defined in `scene_graph`
// (this may require creating the RenderEngine instance). Throws if the
// specification is invalid.
void ValidateEngineAndMaybeAdd(const CameraConfig& config,
                               SceneGraph<double>* scene_graph) {
  DRAKE_DEMAND(scene_graph != nullptr);

  // Querying for the type_name of the named renderer will simultaneously tell
  // if a render engine already exists (non-empty value) *and* give us a string
  // to match against config.renderer_class.
  const std::string type_name =
      scene_graph->GetRendererTypeName(config.renderer_name);

  // Non-empty type name says that it already exists.
  bool already_exists = !type_name.empty();
  if (already_exists) {
    std::visit(
        [&type_name, &config](auto&& params) {
          if (GetEngineTypeNameFromParameters(params) != type_name) {
            throw std::logic_error(fmt::format(
                "Invalid camera configuration; requested renderer_name = '{}' "
                "and render_params = '{}'. The name is already used with a "
                "different type: {}.",
                config.renderer_name, GetEngineTypeNameFromParameters(params),
                type_name));
          }
        },
        config.render_params);
  }

  if (already_exists) return;

  std::visit(overloaded{
      [&name = config.renderer_name,
       scene_graph](const RenderEngineVtkParams& params) {
        scene_graph->AddRenderer(name, MakeRenderEngineVtk(params));
      },
      [&name = config.renderer_name,
       scene_graph](const RenderEngineGlParams& params) {
        if (!geometry::kHasRenderEngineGl) {
          throw std::logic_error(
              "Invalid camera configuration; renderer_class = 'RenderEngineGl' "
              "is not supported in current build.");
        }
        scene_graph->AddRenderer(name, MakeRenderEngineGl(params));
      },
      [&name = config.renderer_name,
       scene_graph](const RenderEngineGltfClientParams& params) {
        scene_graph->AddRenderer(name, MakeRenderEngineGltfClient(params));
      }}, config.render_params);
}

}  // namespace

void ApplyCameraConfig(const CameraConfig& config,
                       DiagramBuilder<double>* builder,
                       const LcmBuses* lcm_buses,
                       const MultibodyPlant<double>* plant,
                       SceneGraph<double>* scene_graph,
                       DrakeLcmInterface* lcm) {
  if (!(config.rgb || config.depth)) {
    return;
  }

  // Find the plant and scene graph.
  if (plant == nullptr) {
    plant = &builder->GetDowncastSubsystemByName<MultibodyPlant>("plant");
  }
  if (scene_graph == nullptr) {
    scene_graph =
        &builder->GetMutableDowncastSubsystemByName<SceneGraph>("scene_graph");
  }

  config.ValidateOrThrow();

  ValidateEngineAndMaybeAdd(config, scene_graph);

  // Extract the camera extrinsics from the config struct.
  const Frame<double>& base_frame =
      config.X_PB.base_frame
          ? GetScopedFrameByName(*plant, *config.X_PB.base_frame)
          : plant->world_frame();
  const RigidTransformd X_PB = config.X_PB.GetDeterministicValue();

  // Extract camera intrinsics from the config struct.
  const auto [color_camera, depth_camera] = config.MakeCameras();

  // Construct the sensor itself.
  const SimRgbdSensor sim_camera(config.name, base_frame, config.fps, X_PB,
                                 color_camera, depth_camera);
  const RgbdSensor* camera_sys =
      AddSimRgbdSensor(*scene_graph, *plant, sim_camera, builder);

  // Find the LCM bus.
  lcm = FindOrCreateLcmBus(lcm, lcm_buses, builder, "ApplyCameraConfig",
                           config.lcm_bus);
  DRAKE_DEMAND(lcm != nullptr);

  // Connect the sensor the the lcm system.
  const auto* rgb_port =
      config.rgb ? &camera_sys->color_image_output_port() : nullptr;
  const auto* depth_16u_port =
      config.depth ? &camera_sys->depth_image_16U_output_port() : nullptr;
  AddSimRgbdSensorLcmPublisher(sim_camera, rgb_port, depth_16u_port,
                               config.do_compress, builder, lcm);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
