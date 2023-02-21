#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace visualization {

struct InertiaVisualizerParams {};

/** InertiaVisualizer is ...

 @system
 name: InertiaVisualizer
 output_ports:
 - geometry_pose
 @endsystem

@tparam_default_scalar
@ingroup visualization */
template <typename T>
class InertiaVisualizer final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InertiaVisualizer)

  /** Creates an instance of %InertiaVisualizer.
  The plant must be finalized. */
  InertiaVisualizer(const multibody::MultibodyPlant<T>& plant,
                    geometry::SceneGraph<T>* scene_graph,
                    InertiaVisualizerParams params = {})
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

      // Add a Bcm geometry frame.
      Item item;
      item.body = i;
      item.body_frame = plant.GetBodyFrameIdIfExists(i).value();
      item.bcm_frame = scene_graph->RegisterFrame(
          source_id_, item.body_frame,
          geometry::GeometryFrame{fmt::format("Bcm({})", i)});

      // Add an illustration shape on Bcm.
      auto shape = std::make_unique<geometry::Box>(0.001, 0.001, 0.001);
      auto geom = std::make_unique<geometry::GeometryInstance>(
          math::RigidTransform<double>(), std::move(shape), "$inertia");
      geometry::IllustrationProperties props;
      // FIXME Make this invisible by default.
      props.AddProperty("phong", "diffuse", geometry::Rgba{0.0, 0.0, 1.0, 0.2});
      geom->set_illustration_properties(std::move(props));
      item.geometry = scene_graph->RegisterGeometry(source_id_, item.bcm_frame,
                                                    std::move(geom));
      items_.push_back(std::move(item));
    }

    // Update the geometry information to reflect the default inertia values
    // form the plant.
    UpdateItems(plant, *plant.CreateDefaultContext(), scene_graph);

    this->DeclareAbstractOutputPort("geometry_pose",
                                    &InertiaVisualizer<T>::CalcFramePoseOutput);
  }

  /** Scalar-converting copy constructor. See @ref system_scalar_conversion. */
  template <typename U>
  explicit InertiaVisualizer(const InertiaVisualizer<U>& other)
      : params_{other.params_},
        source_id_{other.source_id_},
        items_{other.items_} {}

  ~InertiaVisualizer() final = default;

  /** Adds a InertiaVisualizer and connects it ... */
  static const InertiaVisualizer<T>& AddToBuilder(
      systems::DiagramBuilder<T>* builder,
      const multibody::MultibodyPlant<T>& plant,
      geometry::SceneGraph<T>* scene_graph,
      InertiaVisualizerParams params = {}) {
    DRAKE_THROW_UNLESS(builder != nullptr);
    DRAKE_THROW_UNLESS(scene_graph != nullptr);
    auto result = builder->template AddSystem<InertiaVisualizer<T>>(
        plant, scene_graph, std::move(params));
    builder->Connect(result->get_output_port(),
                     scene_graph->get_source_pose_port(result->source_id_));
    return *result;
  }

#if 0
  /** Sizes the inertial visualization to match the plant properties in the
  given context. This should be called if you've customized a context-specific
  mass or inertia. */
  void Initialize(const systems::Context<T>& plant_context) {
    // Call UpdateItems with the plant context.
  }
#endif

  /** (Advanced) ... */
  geometry::SourceId source_id() const { return source_id_; }

 private:
  /* InertiaVisualizer of different scalar types can all access each
  other's data. */
  template <typename>
  friend class InertiaVisualizer;

  struct Item {
    multibody::BodyIndex body;
    geometry::FrameId body_frame;
    geometry::FrameId bcm_frame;
    geometry::GeometryId geometry;
    math::RigidTransform<double> X_BBcm;
  };

  void UpdateItems(const multibody::MultibodyPlant<T>& plant,
                   const systems::Context<T>& plant_context,
                   geometry::SceneGraph<T>* scene_graph) {
    for (auto& item : items_) {
      // Interrogate the plant context for the inertia information.
      const multibody::Body<T>& body = plant.get_body(item.body);
      const double mass = ExtractDoubleOrThrow(body.get_mass(plant_context));
      // const Vector3<T> com = body.CalcCenterOfMassInBodyFrame(plant_context);

      // Compute the equivalent illustration box.
      (void)(mass);                                  // FIXME
      const Eigen::Vector3d box_dim{0.1, 0.1, 0.1};  // FIXME
      const math::RigidTransform<double> X_BBcm;     // FIXME

      // Update the visualization.
      const geometry::Box box(box_dim);
      scene_graph->ChangeShape(source_id_, item.geometry, box);
      item.X_BBcm = X_BBcm;
    }
  }

  // TODO(jwnimmer-tri) Ideally, we could declare that this output port depends
  // on the MbP's parameters.
  void CalcFramePoseOutput(const systems::Context<T>&,
                           geometry::FramePoseVector<T>* poses) const {
    poses->clear();
    for (const auto& item : items_) {
      poses->set_value(item.bcm_frame, item.X_BBcm.template cast<T>());
    }
  }

  const InertiaVisualizerParams params_;
  geometry::SourceId source_id_;
  std::vector<Item> items_;
};

}  // namespace visualization
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::visualization::InertiaVisualizer)
