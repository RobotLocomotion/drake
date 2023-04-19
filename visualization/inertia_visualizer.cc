#include "drake/visualization/inertia_visualizer.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

namespace drake {
namespace visualization {

using Eigen::Vector3d;
using geometry::Ellipsoid;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryInstance;
using geometry::IllustrationProperties;
using geometry::Rgba;
using geometry::SceneGraph;
using geometry::Sphere;
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

    // Pose the ellipsoid at the center of mass.
    item.X_BBcm = RigidTransform<double>(ExtractDoubleOrThrow(M_B.get_com()));

    // For a massless body, use a very tiny sphere.
    const double mass = ExtractDoubleOrThrow(M_B.get_mass());
    if (mass == 0) {
      scene_graph->ChangeShape(source_id_, item.geometry, Sphere(0.001));
      continue;
    }

    // We need to find the equivalent solid ellipsoid for M_B, as described by
    // its semi-major axes (a, b, c) and density (p).
    //
    // Note that the scale of a, b, c relative to each other are independent of
    // the mass; they only depend on Ixx, Iyy, Izz. So first, we'll convert the
    // unit inertia to an ellipsoid to determine the relative axes. Then, we'll
    // re-scale the axes to match the body's mass.
    //
    // Per multibody/tree/unit_inertia the unit inertia of a solid ellipsoid is:
    //
    //  Ixx = (b² + c²) / 5
    //  Iyy = (a² + b²) / 5
    //  Izz = (a² + c²) / 5
    //
    // Solving this for positive a b c, we have:
    const Vector3d unit_inertia_moments =
        ExtractDoubleOrThrow(M_B.get_unit_inertia().get_moments());
    const double Ixx = unit_inertia_moments(0);
    const double Iyy = unit_inertia_moments(1);
    const double Izz = unit_inertia_moments(2);
    Vector3d solved_abc{std::sqrt((5.0 / 2.0) * (-Ixx + Iyy + Izz)),
                        std::sqrt((5.0 / 2.0) * (+Ixx - Iyy + Izz)),
                        std::sqrt((5.0 / 2.0) * (+Ixx + Iyy - Izz))};
    // A zero for any axis means we're dealing with a point mass.
    const double eps = std::numeric_limits<double>::epsilon();
    if (solved_abc.minCoeff() < 10 * eps) {
      solved_abc.setConstant(1.0);
    }
    // Using an assumed density, we'll scale the ellipsoid to have a volume that
    // matches the body's mass.
    const double density = 1000.0;
    const double solved_mass = density * (4.0 / 3.0) * M_PI * solved_abc(0) *
                               solved_abc(1) * solved_abc(2);
    const double volume_scale = mass / solved_mass;
    const Vector3d abc = solved_abc * std::cbrt(volume_scale);
    scene_graph->ChangeShape(source_id_, item.geometry, Ellipsoid(abc));
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
