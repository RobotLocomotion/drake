#include "drake/visualization/inertia_visualizer.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

namespace drake {
namespace visualization {

using geometry::Ellipsoid;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryInstance;
using geometry::IllustrationProperties;
using geometry::Rgba;
using geometry::SceneGraph;
using math::RigidTransform;
using multibody::SpatialInertia;
using systems::Context;
using systems::DiagramBuilder;

template <typename T>
InertiaVisualizer<T>::InertiaVisualizer(
    const multibody::MultibodyPlant<T>& plant, SceneGraph<T>* scene_graph,
    InertiaVisualizerParams params)
    : params_{std::move(params)} {
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  source_id_ = scene_graph->RegisterSource("inertia_visualizer");

  // For all MbP bodies, except for those welded to the world ...
  const std::vector<const multibody::Body<T>*> world_bodies =
      plant.GetBodiesWeldedTo(plant.world_body());
  const int num_bodies = plant.num_bodies();
  for (multibody::BodyIndex i{0}; i < num_bodies; ++i) {
    bool welded = false;
    for (const auto* world_body : world_bodies) {
      if (world_body->index() == i) {
        welded = true;
        break;
      }
    }
    if (welded) {
      continue;
    }
    const multibody::Body<T>& body = plant.get_body(i);

    // Add a Bcm geometry frame.
    Item item;
    item.body = i;
    item.Bo_frame = plant.GetBodyFrameIdIfExists(i).value();
    item.Bcm_frame = scene_graph->RegisterFrame(
        source_id_, SceneGraph<T>::world_frame_id(),
        GeometryFrame{fmt::format(
            "InertiaVisualizer::{}::{}",
            plant.GetModelInstanceName(body.model_instance()), body.name())});

    // Add an illustration shape to be placed at Bcm.
    // This shape will be replaced by a properly-sized geometry in the
    // subsequent call to UpdateItems() so its specifics don't matter.
    auto shape = std::make_unique<Ellipsoid>(0.001, 0.001, 0.001);
    auto geom = std::make_unique<GeometryInstance>(
        RigidTransform<double>(), std::move(shape),
        fmt::format("$inertia({})", i));
    IllustrationProperties props;
    props.AddProperty("meshcat", "accepting", "inertia");
    props.AddProperty("phong", "diffuse", params.color);
    geom->set_illustration_properties(std::move(props));
    item.geometry = scene_graph->RegisterGeometry(source_id_, item.Bcm_frame,
                                                  std::move(geom));
    items_.push_back(std::move(item));
  }

  // Update the geometry information to reflect the initial inertia values
  // from the plant.
  UpdateItems(plant, *plant.CreateDefaultContext(), scene_graph);

  this->DeclareAbstractInputPort("plant_geometry_pose",
                                 Value<FramePoseVector<T>>());
  this->DeclareAbstractOutputPort("geometry_pose",
                                  &InertiaVisualizer<T>::CalcFramePoseOutput);
}

template <typename T>
template <typename U>
InertiaVisualizer<T>::InertiaVisualizer(const InertiaVisualizer<U>& other)
    : params_{other.params_},
      source_id_{other.source_id_},
      items_{other.items_} {}

template <typename T>
const InertiaVisualizer<T>& InertiaVisualizer<T>::AddToBuilder(
    DiagramBuilder<T>* builder, const multibody::MultibodyPlant<T>& plant,
    SceneGraph<T>* scene_graph, InertiaVisualizerParams params) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  auto result = builder->template AddSystem<InertiaVisualizer<T>>(
      plant, scene_graph, std::move(params));
  result->set_name("inertia_visualizer");
  builder->Connect(plant.get_geometry_poses_output_port(),
                   result->get_input_port());
  builder->Connect(result->get_output_port(),
                   scene_graph->get_source_pose_port(result->source_id_));
  return *result;
}

template <typename T>
InertiaVisualizer<T>::~InertiaVisualizer() = default;

// TODO(trowell-tri): This method also needs to run whenever mass or inertia
// changes in plant.getBody().
template <typename T>
void InertiaVisualizer<T>::UpdateItems(
    const multibody::MultibodyPlant<T>& plant, const Context<T>& plant_context,
    SceneGraph<T>* scene_graph) {
  for (auto& item : items_) {
    // Interrogate the plant context for the inertia information.
    const multibody::Body<T>& body = plant.get_body(item.body);
    const SpatialInertia<T> M_B =
        body.CalcSpatialInertiaInBodyFrame(plant_context);
    const Matrix6<double> inertia_matrix =
        ExtractDoubleOrThrow(M_B.CopyToFullMatrix6());

    item.X_BBcm = RigidTransform<double>(ExtractDoubleOrThrow(M_B.get_com()));

    // Compute the illustration geometry and update the visualization.
    double i_xx = inertia_matrix(0, 0);
    double i_yy = inertia_matrix(1, 1);
    double i_zz = inertia_matrix(2, 2);

    // The moments of inertia of a uniform-density solid ellipsoid with
    // semi-axes a, b, and c are:
    //   MIa = (b^2 + c^2) / 5 ; MIb = (a^2 + b^2) / 5 ; MIc = (a^2 + c^2) / 5
    //
    // (See also multibody/tree/unit_inertia.[h,cc].)
    //
    // Here we compute the ellipsoid dimensions as the inverse of the above.
    double ellipsoid_a =
        std::max(sqrt((5 / 2) * (-i_xx + i_yy + i_zz)) * params_.scale_factor,
                 std::numeric_limits<double>::epsilon());
    double ellipsoid_b =
        std::max(sqrt((5 / 2) * (+i_xx - i_yy + i_zz)) * params_.scale_factor,
                 std::numeric_limits<double>::epsilon());
    double ellipsoid_c =
        std::max(sqrt((5 / 2) * (+i_xx + i_yy - i_zz)) * params_.scale_factor,
                 std::numeric_limits<double>::epsilon());
    const Eigen::Vector3d dimensions{ellipsoid_a, ellipsoid_b, ellipsoid_c};
    scene_graph->ChangeShape(source_id_, item.geometry, Ellipsoid(dimensions));
  }
}

template <typename T>
void InertiaVisualizer<T>::CalcFramePoseOutput(
    const Context<T>& context, FramePoseVector<T>* poses) const {
  const auto& plant_poses =
      this->get_input_port().template Eval<FramePoseVector<T>>(context);

  poses->clear();
  for (const auto& item : items_) {
    const RigidTransform<T>& X_WBo = plant_poses.value(item.Bo_frame);
    const RigidTransform<T> X_WBcm = X_WBo * item.X_BBcm.template cast<T>();
    poses->set_value(item.Bcm_frame, X_WBcm);
  }
}

}  // namespace visualization
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::visualization::InertiaVisualizer)
