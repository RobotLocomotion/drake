#include "drake/multibody/plant/deformable_model.h"

#include <algorithm>
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
  const geometry::VolumeMesh<double>* mesh_G =
      inspector.GetReferenceMesh(geometry_id);
  DRAKE_DEMAND(mesh_G != nullptr);
  const math::RigidTransform<T>& X_WG = inspector.GetPoseInFrame(geometry_id);
  geometry::VolumeMesh<double> mesh_W = *mesh_G;
  mesh_W.TransformVertices(X_WG);
  VectorX<T> reference_position(3 * mesh_W.num_vertices());
  for (int v = 0; v < mesh_W.num_vertices(); ++v) {
    reference_position.template segment<3>(3 * v) = mesh_W.vertex(v);
  }

  const DeformableBodyId body_id = DeformableBodyId::get_new_id();
  /* Build FEM model for the deformable body. */
  BuildLinearVolumetricModel(body_id, mesh_W, config);

  /* Do the book-keeping. */
  reference_positions_.emplace(body_id, std::move(reference_position));
  body_id_to_geometry_id_.emplace(body_id, geometry_id);
  geometry_id_to_body_id_.emplace(geometry_id, body_id);
  body_ids_.emplace_back(body_id);
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
DeformableBodyId DeformableModel<T>::GetBodyId(
    DeformableBodyIndex index) const {
  this->ThrowIfSystemResourcesNotDeclared(__func__);
  DRAKE_THROW_UNLESS(index.is_valid() && index < num_bodies());
  return body_ids_[index];
}

template <typename T>
DeformableBodyIndex DeformableModel<T>::GetBodyIndex(
    DeformableBodyId id) const {
  this->ThrowIfSystemResourcesNotDeclared(__func__);
  ThrowUnlessRegistered(__func__, id);
  return body_id_to_index_.at(id);
}

template <typename T>
GeometryId DeformableModel<T>::GetGeometryId(DeformableBodyId id) const {
  ThrowUnlessRegistered(__func__, id);
  return body_id_to_geometry_id_.at(id);
}

template <typename T>
DeformableBodyId DeformableModel<T>::GetBodyId(
    geometry::GeometryId geometry_id) const {
  if (geometry_id_to_body_id_.count(geometry_id) == 0) {
    throw std::runtime_error(
        fmt::format("The given GeometryId {} does not correspond to a "
                    "deformable body registered with this model.",
                    geometry_id));
  }
  return geometry_id_to_body_id_.at(geometry_id);
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

  /* Declare the vertex position output port. */
  vertex_positions_port_index_ =
      this->DeclareAbstractOutputPort(
              plant, "vertex_positions",
              []() {
                return AbstractValue::Make<
                    geometry::GeometryConfigurationVector<T>>();
              },
              [this](const systems::Context<T>& context,
                     AbstractValue* output) {
                this->CopyVertexPositions(context, output);
              },
              {systems::System<double>::xd_ticket()})
          .get_index();

  std::sort(body_ids_.begin(), body_ids_.end());
  for (DeformableBodyIndex i(0); i < static_cast<int>(body_ids_.size()); ++i) {
    DeformableBodyId id = body_ids_[i];
    body_id_to_index_[id] = i;
  }
}

template <typename T>
void DeformableModel<T>::CopyVertexPositions(const systems::Context<T>& context,
                                             AbstractValue* output) const {
  auto& output_value =
      output->get_mutable_value<geometry::GeometryConfigurationVector<T>>();
  output_value.clear();
  for (const auto& [body_id, geometry_id] : body_id_to_geometry_id_) {
    const auto& fem_model = GetFemModel(body_id);
    const int num_dofs = fem_model.num_dofs();
    const auto& discrete_state_index = GetDiscreteStateIndex(body_id);
    VectorX<T> vertex_positions =
        context.get_discrete_state(discrete_state_index).value().head(num_dofs);
    output_value.set_value(geometry_id, std::move(vertex_positions));
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

}  // namespace multibody
}  // namespace drake

template class drake::multibody::DeformableModel<double>;
