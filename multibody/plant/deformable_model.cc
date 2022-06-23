#include "drake/multibody/plant/deformable_model.h"

#include <utility>

#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/multibody/fem/corotated_model.h"
#include "drake/multibody/fem/fem_state.h"
#include "drake/multibody/fem/linear_constitutive_model.h"
#include "drake/multibody/fem/linear_simplex_element.h"
#include "drake/multibody/fem/simplex_gaussian_quadrature.h"
#include "drake/multibody/fem/volumetric_model.h"

namespace drake {
namespace multibody {
namespace internal {

using geometry::FrameId;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::SceneGraph;
using geometry::SourceId;

using fem::DeformableBodyConfig;
using fem::MaterialModel;

template <typename T>
DeformableBodyId DeformableModel<T>::RegisterDeformableBody(
    std::unique_ptr<geometry::GeometryInstance> geometry_instance,
    const fem::DeformableBodyConfig<T>& config, double resolution_hint) {
  this->ThrowIfSystemResourcesDeclared(__func__);

  /* Register the geometry with SceneGraph. */
  SceneGraph<T>& scene_graph = this->mutable_scene_graph(plant_);
  SourceId source_id = plant_->get_source_id().value();
  /* All deformable bodies are registered with the world frame at the moment. */
  const FrameId world_frame_id = scene_graph.world_frame_id();
  GeometryId geometry_id = scene_graph.RegisterDeformableGeometry(
      source_id, world_frame_id, std::move(geometry_instance), resolution_hint);

  /* Record the reference positions. */
  const geometry::SceneGraphInspector<T>& inspector =
      scene_graph.model_inspector();
  const geometry::VolumeMesh<double>* mesh_ptr =
      inspector.GetReferenceMesh(geometry_id);
  DRAKE_DEMAND(mesh_ptr != nullptr);
  const auto& mesh = *mesh_ptr;
  VectorX<T> reference_position(3 * mesh.num_vertices());
  for (int v = 0; v < mesh.num_vertices(); ++v) {
    reference_position.template segment<3>(3 * v) = mesh.vertex(v);
  }

  const DeformableBodyId body_id = DeformableBodyId::get_new_id();
  /* Build FEM model for the deformable body. */
  BuildLinearVolumetricModel(body_id, mesh, config);

  /* Do the book-keeping. */
  reference_positions_.emplace(body_id, std::move(reference_position));
  body_id_to_geometry_id_.emplace(body_id, geometry_id);
  return body_id;
}

template <typename T>
systems::DiscreteStateIndex DeformableModel<T>::GetDiscreteStateIndex(
    DeformableBodyId id) const {
  this->ThrowIfSystemResourcesNotDeclared(__func__);
  ThrowUnlessRegistered(__func__, id);
  return discrete_state_indexes_.at(id);
}

template <typename T>
const fem::FemModel<T>& DeformableModel<T>::GetFemModel(
    DeformableBodyId id) const {
  ThrowUnlessRegistered(__func__, id);
  return *fem_models_.at(id);
}

template <typename T>
const VectorX<T>& DeformableModel<T>::GetReferencePositions(
    DeformableBodyId id) const {
  ThrowUnlessRegistered(__func__, id);
  return reference_positions_.at(id);
}

template <typename T>
void DeformableModel<T>::BuildLinearVolumetricModel(
    DeformableBodyId id, const geometry::VolumeMesh<double>& mesh,
    const fem::DeformableBodyConfig<T>& config) {
  if (fem_models_.find(id) != fem_models_.end()) {
    throw std::logic_error("An FEM model with id: " + to_string(id) +
                           " already exists.");
  }
  switch (config.material_model()) {
    case MaterialModel::kLinear:
      BuildLinearVolumetricModelHelper<fem::internal::LinearConstitutiveModel>(
          id, mesh, config);
      break;
    case MaterialModel::kCorotated:
      BuildLinearVolumetricModelHelper<fem::internal::CorotatedModel>(id, mesh,
                                                                      config);
      break;
  }
}

template <typename T>
template <template <typename, int> class Model>
void DeformableModel<T>::BuildLinearVolumetricModelHelper(
    DeformableBodyId id, const geometry::VolumeMesh<double>& mesh,
    const fem::DeformableBodyConfig<T>& config) {
  constexpr int kNaturalDimension = 3;
  constexpr int kSpatialDimension = 3;
  constexpr int kQuadratureOrder = 1;
  using QuadratureType =
      fem::internal::SimplexGaussianQuadrature<kNaturalDimension,
                                               kQuadratureOrder>;
  constexpr int kNumQuads = QuadratureType::num_quadrature_points;
  using IsoparametricElementType =
      fem::internal::LinearSimplexElement<T, kNaturalDimension,
                                          kSpatialDimension, kNumQuads>;
  using ConstitutiveModelType = Model<T, kNumQuads>;
  static_assert(
      std::is_base_of_v<
          fem::internal::ConstitutiveModel<
              ConstitutiveModelType, typename ConstitutiveModelType::Traits>,
          ConstitutiveModelType>,
      "The template parameter 'Model' must be derived from "
      "ConstitutiveModel.");
  using FemElementType =
      fem::internal::VolumetricElement<IsoparametricElementType, QuadratureType,
                                       ConstitutiveModelType>;
  using FemModelType = fem::internal::VolumetricModel<FemElementType>;

  const fem::DampingModel<T> damping_model(
      config.mass_damping_coefficient(),
      config.stiffness_damping_coefficient());

  auto fem_model = std::make_unique<FemModelType>();
  ConstitutiveModelType constitutive_model(config.youngs_modulus(),
                                           config.poissons_ratio());
  typename FemModelType::VolumetricBuilder builder(fem_model.get());
  builder.AddLinearTetrahedralElements(mesh, constitutive_model,
                                       config.mass_density(), damping_model);
  builder.Build();

  fem_models_.emplace(id, std::move(fem_model));
}

template <typename T>
void DeformableModel<T>::DoDeclareSystemResources(MultibodyPlant<T>* plant) {
  /* Ensure that the owning plant is the one declaring system resources. */
  DRAKE_DEMAND(plant == plant_);
  /* Declare discrete states. */
  for (const auto& [deformable_id, fem_model] : fem_models_) {
    std::unique_ptr<fem::FemState<T>> default_fem_state =
        fem_model->MakeFemState();
    const int num_dofs = default_fem_state->num_dofs();
    VectorX<T> model_state(num_dofs * 3 /* q, v, and a */);
    model_state.head(num_dofs) = default_fem_state->GetPositions();
    model_state.segment(num_dofs, num_dofs) =
        default_fem_state->GetVelocities();
    model_state.tail(num_dofs) = default_fem_state->GetAccelerations();
    discrete_state_indexes_.emplace(
        deformable_id, this->DeclareDiscreteState(plant, model_state));
  }
}

template <typename T>
void DeformableModel<T>::ThrowUnlessRegistered(const char* source_method,
                                               DeformableBodyId id) const {
  if (fem_models_.find(id) == fem_models_.end()) {
    throw std::logic_error(std::string(source_method) +
                           "(): No deformable body with id " + to_string(id) +
                           " has been registered.");
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

template class drake::multibody::internal::DeformableModel<double>;
