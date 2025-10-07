#include "drake/visualization/inertia_visualizer.h"

#include <algorithm>
#include <limits>
#include <memory>

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
using math::RigidTransform;
using multibody::SpatialInertia;
using systems::Context;
using systems::DiagramBuilder;

template <typename T>
InertiaVisualizer<T>::InertiaVisualizer(
    const multibody::MultibodyPlant<T>& plant, SceneGraph<T>* scene_graph) {
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  source_id_ = scene_graph->RegisterSource("inertia_visualizer");

  // For all MbP bodies, except for those welded to the world ...
  const std::vector<const multibody::RigidBody<T>*> world_bodies =
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
    const multibody::RigidBody<T>& body = plant.get_body(i);

    // Add a Bcm geometry frame.
    Item item;
    item.body = i;
    item.Bo_frame = plant.GetBodyFrameIdIfExists(i).value();
    item.Bcm_frame = scene_graph->RegisterFrame(
        source_id_, SceneGraph<T>::world_frame_id(),
        GeometryFrame{fmt::format(
            "InertiaVisualizer::{}::{}",
            plant.GetModelInstanceName(body.model_instance()), body.name())});

    // Add an illustration ellipsoid to be placed at Bcm.
    // This ellipsoid will be replaced by a properly-sized geometry in the
    // subsequent call to UpdateItems() so its specifics don't matter.
    auto ellipsoid = std::make_unique<Ellipsoid>(0.001, 0.001, 0.001);
    auto geom = std::make_unique<GeometryInstance>(
        RigidTransform<double>(), std::move(ellipsoid),
        fmt::format("$inertia({})", i));
    IllustrationProperties props;
    props.AddProperty("meshcat", "accepting", "inertia");
    // TODO(trowell-tri) This color is a placeholder until texturing is added.

    // We set inherent opacity (alpha) to 1. This means if InertiaVisualizer is
    // used in a viewer without a slider control for opacity, the inertia
    // geometry will be permanently visible (but does allow us to use
    // meshcat.js's "modulated_opacity" property to manage its opacity
    // in a manner consistent with MeshcatVisualizer).
    props.AddProperty("phong", "diffuse", Rgba{0.0, 0.0, 1.0, 1.0});
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
    : source_id_{other.source_id_}, items_{other.items_} {}

template <typename T>
const InertiaVisualizer<T>& InertiaVisualizer<T>::AddToBuilder(
    DiagramBuilder<T>* builder, const multibody::MultibodyPlant<T>& plant,
    SceneGraph<T>* scene_graph) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  auto result =
      builder->template AddSystem<InertiaVisualizer<T>>(plant, scene_graph);
  result->set_name("inertia_visualizer");
  builder->Connect(plant.get_geometry_pose_output_port(),
                   result->get_input_port());
  builder->Connect(result->get_output_port(),
                   scene_graph->get_source_pose_port(result->source_id_));
  return *result;
}

template <typename T>
InertiaVisualizer<T>::~InertiaVisualizer() = default;

// TODO(trowell-tri): This method also needs to run whenever mass or inertia
// changes in the plant.
template <typename T>
void InertiaVisualizer<T>::UpdateItems(
    const multibody::MultibodyPlant<T>& plant, const Context<T>& plant_context,
    SceneGraph<T>* scene_graph) {
  for (auto& item : items_) {
    const multibody::RigidBody<T>& body = plant.get_body(item.body);

    auto [ellipsoid, X_BBcm] =
        internal::CalculateInertiaGeometry(body, plant_context);
    item.X_BBcm = X_BBcm;
    scene_graph->ChangeShape(source_id_, item.geometry, ellipsoid);
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

namespace internal {

// TODO(jwnimmer-tri) Add a system Parameter to configure whether to display
// ellipsoids or boxes as the shape, and whether or not to rescale the shape
// to match the body's mass.
template <typename T>
std::pair<Ellipsoid, RigidTransform<double>> CalculateInertiaGeometry(
    const multibody::RigidBody<T>& body, const Context<T>& plant_context) {
  // Interrogate the plant context for the spatial inertia of body B about its
  // origin Bo, expressed in body frame B.
  const SpatialInertia<T> M_BBo_B =
      body.CalcSpatialInertiaInBodyFrame(plant_context);

  // For a massless body, use a very tiny sphere (though expressed as an
  // ellipsoid for consistency).
  const double mass = ExtractDoubleOrThrow(M_BBo_B.get_mass());
  if (mass == 0) {
    const double radius = 0.001;
    return std::make_pair(
        Ellipsoid(radius, radius, radius),
        RigidTransform<double>(ExtractDoubleOrThrow(M_BBo_B.get_com())));
  }

  // Find the equivalent solid ellipsoid E for M_BBo_B, as described by an
  // assumed density, a computed pose X_BE, and computed semi-major axes
  // (a, b, c). Note that Eo is the same point as Bcm.
  RigidTransform<double> X_BE;
  Vector3<double> radii;
  std::tie(radii, X_BE) =
      M_BBo_B.CalcPrincipalSemiDiametersAndPoseForSolidEllipsoid();
  const double max_radius = radii.maxCoeff();
  Vector3d abc;
  if (max_radius == 0.0) {
    // A point mass has no volume, so all radii will be zero. We still need to
    // visualize it with *something*. So, we'll pick a sphere whose radius is
    // proportional to its mass; specifically, with the constraint that a 1-kg
    // mass gets a 6.2-cm radius. 6.2 cm is the radius of a 1-kg sphere of
    // water.
    abc = Vector3d::Constant(std::cbrt(mass) * 0.062);
    return std::make_pair(
        Ellipsoid(abc),
        RigidTransform<double>(ExtractDoubleOrThrow(M_BBo_B.get_com())));
  }

  // At least one measure is non-zero. An ellipsoid that has one of its
  // diameters >100x greater than another one does not render well on the
  // screen, so we'll make sure no dimension of this unit inertia's ellipsoid
  // is smaller than 1/100 of its maximum dimension.
  radii = radii.array().max(1e-2 * radii.maxCoeff());

  // Assuming the ~density of water for the visualization ellipsoid, scale up
  // the ellipsoid representation of the unit inertia to have a volume that
  // matches the body's actual mass, so that our ellipsoid actually has the
  // same inertia as M_BBo_B. (We're illustrating M_BBo, not G_BBo.)
  const double density = 1000.0;
  const double unit_inertia_ellipsoid_mass =
      density * (4.0 / 3.0) * M_PI * radii(0) * radii(1) * radii(2);
  const double volume_scale = mass / unit_inertia_ellipsoid_mass;
  abc = radii * std::cbrt(volume_scale);
  return std::make_pair(Ellipsoid(abc), X_BE);
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    (&CalculateInertiaGeometry<T>));

}  // namespace internal
}  // namespace visualization
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::visualization::InertiaVisualizer);
