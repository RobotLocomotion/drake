#include "drake/multibody/plant/deformable_model.h"

#include <algorithm>
#include <utility>

#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/multibody/fem/fem_state.h"
#include "drake/multibody/fem/velocity_newmark_scheme.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/force_density_field.h"

namespace drake {
namespace multibody {

using drake::math::RigidTransformd;
using geometry::FrameId;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::SceneGraph;
using geometry::SourceId;

using fem::DeformableBodyConfig;

template <typename T>
DeformableModel<T>::DeformableModel(MultibodyPlant<T>* plant)
    : PhysicalModel<T>(plant) {
  /* Set the time integrator for advancing deformable states in time to be the
   midpoint rule, i.e., q = q₀ + δt/2 *(v₀ + v).
   We only set the integrator when the plant is discrete. For continuous plants,
   we may create a deformable model for various reasons, but the deformable
   model will always be empty. */
  if (plant->time_step() > 0) {
    integrator_ = std::make_unique<fem::internal::VelocityNewmarkScheme<T>>(
        plant->time_step(), 1.0, 0.5);
  }
}

template <typename T>
DeformableModel<T>::~DeformableModel() = default;

template <typename T>
DeformableBodyId DeformableModel<T>::RegisterDeformableBody(
    std::unique_ptr<geometry::GeometryInstance> geometry_instance,
    ModelInstanceIndex model_instance,
    const fem::DeformableBodyConfig<T>& config, double resolution_hint) {
  this->ThrowIfSystemResourcesDeclared(__func__);
  ThrowIfNotDiscrete(__func__);
  ThrowIfNotDouble(__func__);
  if (!(model_instance < this->plant().num_model_instances())) {
    throw std::logic_error(
        "RegisterDeformableBody(): Invalid model instance specified. A valid "
        "model instance can be obtained by calling "
        "MultibodyPlant::AddModelInstance().");
  }
  if constexpr (std::is_same_v<T, double>) {
    const std::string name = geometry_instance->name();
    if (HasBodyNamed(name, model_instance)) {
      const std::string& model_instance_name =
          this->plant().GetModelInstanceName(model_instance);
      throw std::logic_error(fmt::format(
          "RegisterDeformableBody(): Model instance '{}' already contains a "
          "deformable body named '{}'. Body names must be unique within a "
          "given model.",
          model_instance_name, name));
    }
    /* Register the geometry with SceneGraph. */
    SceneGraph<T>& scene_graph = this->mutable_scene_graph();
    const ScopedName scoped_name(
        this->plant().GetModelInstanceName(model_instance), name);
    geometry_instance->set_name(scoped_name.to_string());
    SourceId source_id = this->plant().get_source_id().value();
    /* All deformable bodies are registered with the world frame at the moment.
     */
    const FrameId world_frame_id = scene_graph.world_frame_id();

    // TODO(xuchenhan-tri): Consider allowing users to opt out of illustration
    // property for the deformable body if that's ever useful.
    /* If the geometry doesn't have illustration properties, add an empty one so
     that it can at least be visualized. */
    if (geometry_instance->illustration_properties() == nullptr) {
      geometry_instance->set_illustration_properties(
          geometry::IllustrationProperties{});
    }
    GeometryId geometry_id = scene_graph.RegisterDeformableGeometry(
        source_id, world_frame_id, std::move(geometry_instance),
        resolution_hint);
    /* Record the reference positions. */
    const geometry::SceneGraphInspector<T>& inspector =
        scene_graph.model_inspector();
    const geometry::VolumeMesh<double>* mesh_G =
        inspector.GetReferenceMesh(geometry_id);
    DRAKE_DEMAND(mesh_G != nullptr);
    const math::RigidTransform<double>& X_WG =
        inspector.GetPoseInFrame(geometry_id);

    const DeformableBodyId body_id = DeformableBodyId::get_new_id();
    const DeformableBodyIndex body_index = deformable_bodies_.next_index();
    DeformableBody<T>& body = deformable_bodies_.Add(
        std::unique_ptr<DeformableBody<T>>(new DeformableBody<T>(
            body_index, body_id, name, geometry_id, model_instance, *mesh_G,
            X_WG, config, integrator_->GetWeights())));
    body.set_parent_tree(&this->internal_tree(), body.index());
    body.set_parallelism(parallelism_);
    geometry_id_to_body_id_.emplace(geometry_id, body_id);
    body_id_to_index_.emplace(body_id, body.index());
    return body_id;
  }
  DRAKE_UNREACHABLE();
}

template <typename T>
DeformableBodyId DeformableModel<T>::RegisterDeformableBody(
    std::unique_ptr<geometry::GeometryInstance> geometry_instance,
    const fem::DeformableBodyConfig<T>& config, double resolution_hint) {
  return RegisterDeformableBody(std::move(geometry_instance),
                                default_model_instance(), config,
                                resolution_hint);
}

template <typename T>
void DeformableModel<T>::SetWallBoundaryCondition(DeformableBodyId id,
                                                  const Vector3<T>& p_WQ,
                                                  const Vector3<T>& n_W) {
  ThrowUnlessRegistered(__func__, id);
  GetBody(id).SetWallBoundaryCondition(p_WQ, n_W);
}

template <typename T>
MultibodyConstraintId DeformableModel<T>::AddFixedConstraint(
    DeformableBodyId body_A_id, const RigidBody<T>& body_B,
    const math::RigidTransform<double>& X_BA, const geometry::Shape& shape_G,
    const math::RigidTransform<double>& X_BG) {
  ThrowIfNotDouble(__func__);
  this->ThrowIfSystemResourcesDeclared(__func__);
  ThrowUnlessRegistered(__func__, body_A_id);
  return GetMutableBody(body_A_id).AddFixedConstraint(body_B, X_BA, shape_G,
                                                      X_BG);
}

template <typename T>
systems::DiscreteStateIndex DeformableModel<T>::GetDiscreteStateIndex(
    DeformableBodyId id) const {
  this->ThrowIfSystemResourcesNotDeclared(__func__);
  ThrowUnlessRegistered(__func__, id);
  return GetBody(id).discrete_state_index();
}

template <typename T>
void DeformableModel<T>::SetPositions(
    systems::Context<T>* context, DeformableBodyId id,
    const Eigen::Ref<const Matrix3X<T>>& q) const {
  ThrowUnlessRegistered(__func__, id);
  GetBody(id).SetPositions(context, q);
}

template <typename T>
void DeformableModel<T>::SetVelocities(
    systems::Context<T>* context, DeformableBodyId id,
    const Eigen::Ref<const Matrix3X<T>>& v) const {
  ThrowUnlessRegistered(__func__, id);
  GetBody(id).SetVelocities(context, v);
}

template <typename T>
void DeformableModel<T>::SetPositionsAndVelocities(
    systems::Context<T>* context, DeformableBodyId id,
    const Eigen::Ref<const Matrix3X<T>>& q,
    const Eigen::Ref<const Matrix3X<T>>& v) const {
  ThrowUnlessRegistered(__func__, id);
  GetBody(id).SetPositionsAndVelocities(context, q, v);
}

template <typename T>
Matrix3X<T> DeformableModel<T>::GetPositions(const systems::Context<T>& context,
                                             DeformableBodyId id) const {
  ThrowUnlessRegistered(__func__, id);
  return GetBody(id).GetPositions(context);
}

template <typename T>
Matrix3X<T> DeformableModel<T>::GetVelocities(
    const systems::Context<T>& context, DeformableBodyId id) const {
  ThrowUnlessRegistered(__func__, id);
  return GetBody(id).GetVelocities(context);
}

template <typename T>
Matrix3X<T> DeformableModel<T>::GetPositionsAndVelocities(
    const systems::Context<T>& context, DeformableBodyId id) const {
  ThrowUnlessRegistered(__func__, id);
  return GetBody(id).GetPositionsAndVelocities(context);
}

template <typename T>
void DeformableModel<T>::AddExternalForce(
    std::unique_ptr<ForceDensityFieldBase<T>> force_density) {
  this->ThrowIfSystemResourcesDeclared(__func__);
  ThrowIfNotDouble(__func__);
  ThrowIfNotDiscrete(__func__);
  force_densities_.push_back(std::move(force_density));
}

template <typename T>
const std::vector<const ForceDensityFieldBase<T>*>&
DeformableModel<T>::GetExternalForces(DeformableBodyId id) const {
  this->ThrowIfSystemResourcesNotDeclared(__func__);
  ThrowUnlessRegistered(__func__, id);
  return GetBody(id).external_forces();
}

template <typename T>
void DeformableModel<T>::Disable(DeformableBodyId id,
                                 systems::Context<T>* context) const {
  ThrowUnlessRegistered(__func__, id);
  GetBody(id).Disable(context);
}

template <typename T>
void DeformableModel<T>::Enable(DeformableBodyId id,
                                systems::Context<T>* context) const {
  ThrowUnlessRegistered(__func__, id);
  GetBody(id).Enable(context);
}

template <typename T>
const fem::FemModel<T>& DeformableModel<T>::GetFemModel(
    DeformableBodyId id) const {
  ThrowUnlessRegistered(__func__, id);
  return GetBody(id).fem_model();
}

template <typename T>
const VectorX<double>& DeformableModel<T>::GetReferencePositions(
    DeformableBodyId id) const {
  ThrowUnlessRegistered(__func__, id);
  return GetBody(id).reference_positions();
}

template <typename T>
DeformableBodyId DeformableModel<T>::GetBodyId(
    DeformableBodyIndex index) const {
  return deformable_bodies_.get_element(index).body_id();
}

template <typename T>
bool DeformableModel<T>::HasBodyNamed(const std::string& name) const {
  const std::vector<DeformableBodyIndex> indices = GetBodyIndicesByName(name);
  if (indices.empty()) return false;
  if (indices.size() > 1) {
    throw std::logic_error(
        fmt::format("HasBodyNamed(): The name {} is not unique.", name));
  }
  return true;
}

template <typename T>
bool DeformableModel<T>::HasBodyNamed(const std::string& name,
                                      ModelInstanceIndex model_instance) const {
  return GetBodyIndexByName(name, model_instance).has_value();
}

template <typename T>
const DeformableBody<T>& DeformableModel<T>::GetBodyByName(
    const std::string& name) const {
  const std::vector<DeformableBodyIndex> indices = GetBodyIndicesByName(name);
  if (indices.empty()) {
    throw std::runtime_error(
        fmt::format("GetBodyByName(): No deformable body with the given name "
                    "{} has been registered.",
                    name));
  }
  if (indices.size() > 1) {
    throw std::runtime_error(
        fmt::format("GetBodyByName(): The name {} is not unique. A model "
                    "instance is required to retrieve the body.",
                    name));
  }
  return deformable_bodies_.get_element(indices[0]);
}

template <typename T>
const DeformableBody<T>& DeformableModel<T>::GetBodyByName(
    const std::string& name, ModelInstanceIndex model_instance) const {
  const std::optional<DeformableBodyIndex> index =
      GetBodyIndexByName(name, model_instance);
  if (!index.has_value()) {
    throw std::runtime_error(
        fmt::format("GetBodyByName(): No deformable body with the given name "
                    "{} in instance {}.",
                    name, model_instance));
  }
  return deformable_bodies_.get_element(index.value());
}

template <typename T>
std::vector<DeformableBodyId> DeformableModel<T>::GetBodyIds(
    ModelInstanceIndex model_instance) const {
  std::vector<DeformableBodyId> result;
  for (const DeformableBody<T>* body : deformable_bodies_.elements()) {
    if (body->model_instance() == model_instance) {
      result.push_back(body->body_id());
    }
  }
  return result;
}

template <typename T>
DeformableBodyIndex DeformableModel<T>::GetBodyIndex(
    DeformableBodyId id) const {
  ThrowUnlessRegistered(__func__, id);
  return body_id_to_index_.at(id);
}

template <typename T>
GeometryId DeformableModel<T>::GetGeometryId(DeformableBodyId id) const {
  ThrowUnlessRegistered(__func__, id);
  return GetBody(id).geometry_id();
}

template <typename T>
DeformableBodyId DeformableModel<T>::GetBodyId(
    geometry::GeometryId geometry_id) const {
  if (!geometry_id_to_body_id_.contains(geometry_id)) {
    throw std::runtime_error(
        fmt::format("The given GeometryId {} does not correspond to a "
                    "deformable body registered with this model.",
                    geometry_id));
  }
  return geometry_id_to_body_id_.at(geometry_id);
}

template <typename T>
void DeformableModel<T>::SetParallelism(Parallelism parallelism) {
  parallelism_ = parallelism;
  const std::vector<DeformableBodyIndex>& body_indices =
      deformable_bodies_.indices();
  for (const DeformableBodyIndex& index : body_indices) {
    deformable_bodies_.get_mutable_element(index).set_parallelism(parallelism);
  }
}

template <typename T>
void DeformableModel<T>::SetDefaultState(const systems::Context<T>& context,
                                         systems::State<T>* state) const {
  if constexpr (!std::is_same_v<T, double>) {
    DRAKE_DEMAND(is_empty());
    return;
  } else {
    for (const DeformableBodyIndex& index : deformable_bodies_.indices()) {
      const DeformableBody<T>& body = deformable_bodies_.get_element(index);
      body.SetDefaultState(context, state);
    }
  }
}

template <typename T>
std::unique_ptr<PhysicalModel<double>> DeformableModel<T>::CloneToDouble(
    MultibodyPlant<double>* plant) const {
  auto result = std::make_unique<DeformableModel<double>>(plant);
  /* If this plant is not double, then it's necessarily empty because we don't
   allow non-empty models yet. In that case, return an empty double model. */
  if constexpr (!std::is_same_v<T, double>) {
    DRAKE_DEMAND(this->is_empty());
  } else {
    /* Here we step through every member field one by one, in the exact order
     they are declared in the header, so that a reader could mindlessly compare
     this function to the private fields, and check that every single field got
     a mention.
     For each field, this function will either:
     1. Copy the field directly.
     2. Place a disclaimer comment why that field does not need to be copied. */

    /* Copy over deformable_bodies_. */
    result->deformable_bodies_.ResizeToMatch(deformable_bodies_);
    for (const DeformableBodyIndex& index : deformable_bodies_.indices()) {
      const DeformableBody<T>& body = deformable_bodies_.get_element(index);
      result->deformable_bodies_.Add(body.CloneToDouble());
    }
    result->geometry_id_to_body_id_ = geometry_id_to_body_id_;
    result->body_id_to_index_ = body_id_to_index_;
    /* Copy over force_densities_. */
    for (const auto& force_density : force_densities_) {
      result->force_densities_.emplace_back(force_density->Clone());
    }
    /* `configuration_output_port_index_` is set in `DeclareSceneGraphPorts()`;
     because callers to `PhysicalModel::CloneToScalar` are required to
     subsequently call `DeclareSceneGraphPorts`. */
    result->parallelism_ = parallelism_;
    result->integrator_ = integrator_->Clone();
  }

  return result;
}

template <typename T>
std::unique_ptr<PhysicalModel<AutoDiffXd>>
DeformableModel<T>::CloneToAutoDiffXd(MultibodyPlant<AutoDiffXd>* plant) const {
  return std::make_unique<DeformableModel<AutoDiffXd>>(plant);
}

template <typename T>
std::unique_ptr<PhysicalModel<symbolic::Expression>>
DeformableModel<T>::CloneToSymbolic(
    MultibodyPlant<symbolic::Expression>* plant) const {
  return std::make_unique<DeformableModel<symbolic::Expression>>(plant);
}

template <typename T>
void DeformableModel<T>::DoDeclareSystemResources() {
  if constexpr (!std::is_same_v<T, double>) {
    /* A non-double DeformableModel is always empty. */
    DRAKE_DEMAND(is_empty());
    return;
  } else {
    if (!is_empty()) {
      if (this->plant().get_discrete_contact_solver() !=
          DiscreteContactSolver::kSap) {
        throw std::runtime_error(
            "DeformableModel is only supported by the SAP contact solver. "
            "Please use `kSap`, `kLagged`, or `kSimilar` as the discrete "
            "contact approximation for the MultibodyPlant containing "
            "deformable bodies.");
      }
      if (!this->plant().is_discrete()) {
        throw std::runtime_error(
            "Deformable body simulation is only supported with discrete time "
            "MultibodyPlant.");
      }
    }

    /* Declare discrete states and parameters. */
    const std::vector<DeformableBodyIndex>& body_indices =
        deformable_bodies_.indices();
    for (const DeformableBodyIndex& index : body_indices) {
      DeformableBody<T>& body = deformable_bodies_.get_mutable_element(index);
      body.DeclareDiscreteState(static_cast<internal::MultibodyTreeSystem<T>*>(
          this->mutable_plant()));
      body.DeclareParameters(static_cast<internal::MultibodyTreeSystem<T>*>(
          this->mutable_plant()));
      body.DeclareCacheEntries(static_cast<internal::MultibodyTreeSystem<T>*>(
          this->mutable_plant()));
      const Vector3<T>& gravity =
          this->plant().gravity_field().gravity_vector();
      body.SetExternalForces(force_densities_, gravity);
    }
    /* Declare cache entries and input ports for force density fields that need
     them. */
    for (std::unique_ptr<ForceDensityFieldBase<T>>& force_density :
         force_densities_) {
      /* We know that the static cast is safe because all concrete force density
       fields are derived from ForceDensityField. */
      static_cast<ForceDensityField<T>*>(force_density.get())
          ->DeclareSystemResources(this->mutable_plant());
    }
  }
}

template <typename T>
void DeformableModel<T>::DoDeclareSceneGraphPorts() {
  /* Declare the deformable body configuration output port. This port copies
   the discrete states of all deformable body configurations and puts them into
   a format that's easier for downstream parsing. */
  configuration_output_port_index_ =
      this->DeclareAbstractOutputPort(
              "deformable_body_configuration",
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
}

template <typename T>
void DeformableModel<T>::CopyVertexPositions(const systems::Context<T>& context,
                                             AbstractValue* output) const {
  auto& output_value =
      output->get_mutable_value<geometry::GeometryConfigurationVector<T>>();
  output_value.clear();
  const std::vector<DeformableBodyIndex>& body_indices =
      deformable_bodies_.indices();
  for (const DeformableBodyIndex& index : body_indices) {
    const DeformableBody<T>& body = deformable_bodies_.get_element(index);
    const GeometryId geometry_id = body.geometry_id();
    const int num_dofs = body.fem_model().num_dofs();
    const auto& discrete_state_index = body.discrete_state_index();
    VectorX<T> vertex_positions =
        context.get_discrete_state(discrete_state_index).value().head(num_dofs);
    output_value.set_value(geometry_id, std::move(vertex_positions));
  }
}

template <typename T>
void DeformableModel<T>::ThrowUnlessRegistered(const char* function_name,
                                               DeformableBodyId id) const {
  if (!body_id_to_index_.contains(id)) {
    throw std::logic_error(
        fmt::format("{}(): No deformable body with id {} has been registered.",
                    function_name, id));
  }
}

template <typename T>
void DeformableModel<T>::ThrowIfNotDouble(const char* function_name) const {
  if (!std::is_same_v<T, double>) {
    throw std::logic_error(
        fmt::format("Calls to {}() with a DeformableModel of type T != double "
                    "are not allowed.",
                    function_name));
  }
}
template <typename T>
void DeformableModel<T>::ThrowIfNotDiscrete(const char* function_name) const {
  if (!this->plant().is_discrete()) {
    throw std::logic_error(
        fmt::format("Calls to {}() with a DeformableModel belonging to a "
                    "continuous MultibodyPlant are not allowed.",
                    function_name));
  }
}

template <typename T>
std::vector<DeformableBodyIndex> DeformableModel<T>::GetBodyIndicesByName(
    const std::string& name) const {
  auto [lower, upper] = deformable_bodies_.names_map().equal_range(name);
  std::vector<DeformableBodyIndex> indices;
  indices.reserve(std::distance(lower, upper));
  for (auto it = lower; it != upper; ++it) {
    indices.push_back(it->second);
  }
  return indices;
}

template <typename T>
std::optional<DeformableBodyIndex> DeformableModel<T>::GetBodyIndexByName(
    const std::string& name, ModelInstanceIndex model_instance) const {
  /* Use the name lookup for its side-effect of throwing on an invalid index. */
  unused(this->plant().GetModelInstanceName(model_instance));
  auto [lower, upper] = deformable_bodies_.names_map().equal_range(name);
  for (auto it = lower; it != upper; ++it) {
    if (GetBody(it->second).model_instance() == model_instance) {
      return it->second;
    }
  }
  return std::nullopt;
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::DeformableModel);
