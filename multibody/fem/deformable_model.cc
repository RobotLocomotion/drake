#include "drake/multibody/fem/deformable_model.h"

#include <memory>
#include <utility>

#include "drake/multibody/fem/mesh_utilities.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
template <typename T>
void DeformableModel<T>::RegisterDeformableBody(
    const geometry::Box& box, std::string name,
    geometry::ProximityProperties properties) {
  /* Must have resolution hint property to create the mesh. */
  DRAKE_THROW_UNLESS(properties.HasProperty(geometry::internal::kHydroGroup,
                                            geometry::internal::kRezHint));
  /* Must have friction coefficient specified as all collision geometries. */
  DRAKE_THROW_UNLESS(properties.HasProperty(geometry::internal::kMaterialGroup,
                                            geometry::internal::kFriction));
  const double resolution_hint = properties.GetProperty<double>(
      geometry::internal::kHydroGroup, geometry::internal::kRezHint);
  const geometry::VolumeMesh<double> geometry =
      MakeDiamondCubicBoxVolumeMesh<double>(box, resolution_hint);
  const std::vector<Vector3<double>>& vertices = geometry.vertices();
  VectorX<T> X(3 * vertices.size());
  for (int i = 0; i < static_cast<int>(vertices.size()); ++i) {
    X.template segment<3>(3 * i) = vertices[i];
  }
  reference_positions_.push_back(std::move(X));
  geometry::SceneGraph<T>& scene_graph = this->mutable_scene_graph(plant_);
  geometry::SourceId source_id = plant_->get_source_id().value();
  std::unique_ptr<geometry::GeometryInstance> geometry_instance =
      std::make_unique<geometry::GeometryInstance>(
          math::RigidTransform<double>::Identity(), box.Clone(), name);
  /* All deformable bodies are attached to the world frame at the moment. */
  const geometry::FrameId world_frame_id = scene_graph.world_frame_id();
  scene_graph.RegisterGeometry(source_id, world_frame_id,
                               std::move(geometry_instance));
  ++num_bodies_;
}

template <typename T>
void DeformableModel<T>::DoDeclareSystemResources(MultibodyPlant<T>* plant) {
  /* Ensure that the owning plant is the one declaring system resources. */
  DRAKE_DEMAND(plant == plant_);
  /* Declare output ports. */
  vertex_positions_port_ = &this->DeclareAbstractOutputPort(
      plant, "vertex_positions",
      []() { return AbstractValue::Make<std::vector<Matrix3X<T>>>(); },
      [this](const systems::Context<T>& context, AbstractValue* output) {
        // TODO(xuchenhan-tri): Right now the vertex positions are copied
        //  directly from the discrete state. They should come from FEM models.
        std::vector<Matrix3X<T>>& output_value =
            output->get_mutable_value<std::vector<Matrix3X<T>>>();
        output_value.resize(num_bodies());
        const systems::DiscreteValues<T>& all_discrete_states =
            context.get_discrete_state();
        for (int i = 0; i < num_bodies(); ++i) {
          const VectorX<T>& positions =
              all_discrete_states.value(discrete_state_indexes_[i]);
          output_value[i] = Eigen::Map<const Matrix3X<T>>(positions.data(), 3,
                                                          positions.size() / 3);
        }
      },
      {systems::System<double>::xd_ticket()});

  /* Declare discrete states. */
  for (int i = 0; i < num_bodies(); ++i) {
    discrete_state_indexes_.emplace_back(
        this->DeclareDiscreteState(plant, reference_positions_[i]));
  }
}
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::internal::DeformableModel);
