#include "drake/multibody/tree/deformable_body.h"

#include <utility>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/fem/corotated_model.h"
#include "drake/multibody/fem/dirichlet_boundary_condition.h"
#include "drake/multibody/fem/linear_constitutive_model.h"
#include "drake/multibody/fem/linear_corotated_model.h"
#include "drake/multibody/fem/linear_simplex_element.h"
#include "drake/multibody/fem/neohookean_model.h"
#include "drake/multibody/fem/simplex_gaussian_quadrature.h"
#include "drake/multibody/fem/tet_subdivision_quadrature.h"
#include "drake/multibody/fem/velocity_newmark_scheme.h"
#include "drake/multibody/fem/volumetric_model.h"
#include "drake/multibody/tree/force_density_field.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {

using fem::MaterialModel;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::VolumeMesh;

template <typename T>
ScopedName DeformableBody<T>::scoped_name() const {
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  return ScopedName(
      this->get_parent_tree().GetModelInstanceName(this->model_instance()),
      name_);
}

template <typename T>
void DeformableBody<T>::SetWallBoundaryCondition(const Vector3<T>& p_WQ,
                                                 const Vector3<T>& n_W) const {
  DRAKE_THROW_UNLESS(n_W.norm() > 1e-10);
  const Vector3<T> nhat_W = n_W.normalized();

  const int num_nodes = fem_model_->num_nodes();
  constexpr int kDim = 3;
  auto is_inside_wall = [&p_WQ, &nhat_W](const Vector3<T>& p_WV) {
    const T distance_to_wall = (p_WV - p_WQ).dot(nhat_W);
    return distance_to_wall < 0;
  };

  fem::internal::DirichletBoundaryCondition<T> bc;
  for (int n = 0; n < num_nodes; ++n) {
    const int dof_index = kDim * n;
    const auto p_WV = reference_positions_.template segment<kDim>(dof_index);
    if (is_inside_wall(p_WV)) {
      /* Set this node to be subject to zero Dirichlet BC. */
      bc.AddBoundaryCondition(fem::FemNodeIndex(n),
                              {p_WV, Vector3<T>::Zero(), Vector3<T>::Zero()});
    }
  }
  fem_model_->SetDirichletBoundaryCondition(bc);
}

template <typename T>
MultibodyConstraintId DeformableBody<T>::AddFixedConstraint(
    const RigidBody<T>& body_B, const math::RigidTransform<double>& X_BA,
    const geometry::Shape& shape_G, const math::RigidTransform<double>& X_BG) {
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  if (&this->get_parent_tree().get_body(body_B.index()) != &body_B) {
    throw std::logic_error(fmt::format(
        "AddFixedConstraint(): The rigid body with name {} is not registered "
        "with the MultibodyPlant owning the deformable model.",
        body_B.name()));
  }
  /* X_WG is the pose of this body's reference mesh in the world frame. In the
   scope of this function, we call that the A frame and G is used to denote
   the rigid body's geometry, so we rename X_WG_ to X_WA here to avoid
   confusion. */
  const math::RigidTransformd& X_WA = X_WG_;
  const MultibodyConstraintId constraint_id =
      MultibodyConstraintId::get_new_id();
  /* Create an empty spec first. We will add to it. */
  internal::DeformableRigidFixedConstraintSpec spec{
      id_, body_B.index(), {}, {}, constraint_id};
  geometry::SceneGraph<double> scene_graph;
  geometry::SourceId source_id = scene_graph.RegisterSource("deformable_model");
  /* Register the geometry in deformable reference geometry A frame. */
  const math::RigidTransform<double> X_AG = X_BA.InvertAndCompose(X_BG);
  auto instance =
      std::make_unique<GeometryInstance>(X_AG, shape_G.Clone(), "rigid shape");
  GeometryId geometry_id =
      scene_graph.RegisterAnchoredGeometry(source_id, std::move(instance));
  scene_graph.AssignRole(source_id, geometry_id,
                         geometry::ProximityProperties());
  auto context = scene_graph.CreateDefaultContext();
  auto query =
      scene_graph.get_query_output_port().Eval<geometry::QueryObject<double>>(
          *context);
  const VectorX<double>& p_WPi = reference_positions_;
  for (int vertex_index = 0; vertex_index < fem_model_->num_nodes();
       ++vertex_index) {
    /* The vertex position in the deformable body's geometry frame. */
    const Vector3<double>& p_APi =
        X_WA.inverse() * p_WPi.template segment<3>(vertex_index * 3);
    /* Note that `shape` is also registered in the A frame in the throw-away
     scene graph. */
    const std::vector<geometry::SignedDistanceToPoint<double>>
        signed_distances = query.ComputeSignedDistanceToPoint(p_APi);
    DRAKE_DEMAND(ssize(signed_distances) == 1);
    const double signed_distance = signed_distances[0].distance;
    if (signed_distance <= 0.0) {
      spec.vertices.push_back(vertex_index);
      /* Qi is conincident with Pi. */
      spec.p_BQs.emplace_back(X_BA * p_APi);
    }
  }
  // TODO(xuchenhan-tri): consider adding an option to allow empty constraint.
  if (spec.vertices.size() == 0) {
    throw std::runtime_error(fmt::format(
        "AddFixedConstraint(): No constraint has been added between deformable "
        "body with id {} and rigid body with name {}. Remove the call to "
        "AddFixedConstraint() if this is intended.",
        id_, body_B.name()));
  }
  fixed_constraint_specs_.push_back(std::move(spec));
  return constraint_id;
}

template <typename T>
void DeformableBody<T>::SetPositions(
    systems::Context<T>* context,
    const Eigen::Ref<const Matrix3X<T>>& q) const {
  DRAKE_THROW_UNLESS(context != nullptr);
  this->GetParentTreeSystem().ValidateContext(*context);
  const int num_nodes = fem_model_->num_nodes();
  DRAKE_THROW_UNLESS(q.cols() == num_nodes);
  auto all_finite = [](const Matrix3X<T>& positions) {
    return positions.array().isFinite().all();
  };
  DRAKE_THROW_UNLESS(all_finite(q));

  context->get_mutable_discrete_state(discrete_state_index_)
      .get_mutable_value()
      .head(num_nodes * 3) = Eigen::Map<const VectorX<T>>(q.data(), q.size());
}

template <typename T>
void DeformableBody<T>::SetVelocities(
    systems::Context<T>* context,
    const Eigen::Ref<const Matrix3X<T>>& v) const {
  DRAKE_THROW_UNLESS(context != nullptr);
  this->GetParentTreeSystem().ValidateContext(*context);
  const int num_nodes = fem_model_->num_nodes();
  DRAKE_THROW_UNLESS(v.cols() == num_nodes);
  auto all_finite = [](const Matrix3X<T>& velocities) {
    return velocities.array().isFinite().all();
  };
  DRAKE_THROW_UNLESS(all_finite(v));

  const int num_dofs = num_nodes * 3;
  context->get_mutable_discrete_state(discrete_state_index_)
      .get_mutable_value()
      .segment(num_dofs, num_dofs) =
      Eigen::Map<const VectorX<T>>(v.data(), v.size());
}

template <typename T>
void DeformableBody<T>::SetPositionsAndVelocities(
    systems::Context<T>* context, const Eigen::Ref<const Matrix3X<T>>& q,
    const Eigen::Ref<const Matrix3X<T>>& v) const {
  DRAKE_THROW_UNLESS(context != nullptr);
  this->GetParentTreeSystem().ValidateContext(*context);
  const int num_nodes = fem_model_->num_nodes();
  DRAKE_THROW_UNLESS(q.cols() == num_nodes);
  DRAKE_THROW_UNLESS(v.cols() == num_nodes);
  auto all_finite = [](const auto& mat) {
    return mat.array().isFinite().all();
  };
  DRAKE_THROW_UNLESS(all_finite(q));
  DRAKE_THROW_UNLESS(all_finite(v));

  const int num_dofs = num_nodes * 3;
  Eigen::VectorBlock<VectorX<T>> state_value =
      context->get_mutable_discrete_state(discrete_state_index_)
          .get_mutable_value();
  state_value.head(num_dofs) = Eigen::Map<const VectorX<T>>(q.data(), q.size());
  state_value.segment(num_dofs, num_dofs) =
      Eigen::Map<const VectorX<T>>(v.data(), v.size());
}

template <typename T>
Matrix3X<T> DeformableBody<T>::GetPositions(
    const systems::Context<T>& context) const {
  this->GetParentTreeSystem().ValidateContext(context);

  const int num_nodes = fem_model_->num_nodes();
  const VectorX<T>& q = context.get_discrete_state(discrete_state_index_)
                            .get_value()
                            .head(num_nodes * 3);
  return Eigen::Map<const Matrix3X<T>>(q.data(), 3, num_nodes);
}

template <typename T>
Matrix3X<T> DeformableBody<T>::GetVelocities(
    const systems::Context<T>& context) const {
  this->GetParentTreeSystem().ValidateContext(context);

  const int num_nodes = fem_model_->num_nodes();
  const int num_dofs = num_nodes * 3;
  const VectorX<T>& v = context.get_discrete_state(discrete_state_index_)
                            .get_value()
                            .segment(num_dofs, num_dofs);
  return Eigen::Map<const Matrix3X<T>>(v.data(), 3, num_nodes);
}

template <typename T>
Matrix3X<T> DeformableBody<T>::GetPositionsAndVelocities(
    const systems::Context<T>& context) const {
  this->GetParentTreeSystem().ValidateContext(context);
  const int num_nodes = fem_model_->num_nodes();
  const int num_dofs = num_nodes * 3;
  const VectorX<T>& state_value =
      context.get_discrete_state(discrete_state_index_).get_value();
  const auto& q = state_value.head(num_dofs);
  const auto& v = state_value.segment(num_dofs, num_dofs);
  Matrix3X<T> qv(3, 2 * num_nodes);
  qv.leftCols(num_nodes) =
      Eigen::Map<const Matrix3X<T>>(q.data(), 3, num_nodes);
  qv.rightCols(num_nodes) =
      Eigen::Map<const Matrix3X<T>>(v.data(), 3, num_nodes);
  return qv;
}

template <typename T>
bool DeformableBody<T>::is_enabled(const systems::Context<T>& context) const {
  this->GetParentTreeSystem().ValidateContext(context);
  return context.get_parameters().template get_abstract_parameter<bool>(
      is_enabled_parameter_index_);
}

template <typename T>
void DeformableBody<T>::Disable(systems::Context<T>* context) const {
  DRAKE_THROW_UNLESS(context != nullptr);
  this->GetParentTreeSystem().ValidateContext(*context);
  context->get_mutable_abstract_parameter(is_enabled_parameter_index_)
      .template set_value<bool>(false);
  /* Set both the accelerations and the velocities to zero, noting that the
   dofs are stored in the order of q, v, and then a. */
  context->get_mutable_discrete_state(discrete_state_index_)
      .get_mutable_value()
      .tail(2 * num_dofs())
      .setZero();
}

template <typename T>
void DeformableBody<T>::Enable(systems::Context<T>* context) const {
  DRAKE_THROW_UNLESS(context != nullptr);
  this->GetParentTreeSystem().ValidateContext(*context);
  context->get_mutable_abstract_parameter(is_enabled_parameter_index_)
      .set_value(true);
}

template <typename T>
Vector3<T> DeformableBody<T>::CalcCenterOfMassPositionInWorld(
    const systems::Context<T>& context) const {
  DRAKE_DEMAND(fem_model_ != nullptr);
  const fem::FemState<T>& fem_state =
      this->GetParentTreeSystem()
          .get_cache_entry(fem_state_cache_index_)
          .template Eval<fem::FemState<T>>(context);
  return fem_model_->CalcCenterOfMassPositionInWorld(fem_state);
}

template <typename T>
Vector3<T> DeformableBody<T>::CalcCenterOfMassTranslationalVelocityInWorld(
    const systems::Context<T>& context) const {
  DRAKE_DEMAND(fem_model_ != nullptr);
  const fem::FemState<T>& fem_state =
      this->GetParentTreeSystem()
          .get_cache_entry(fem_state_cache_index_)
          .template Eval<fem::FemState<T>>(context);
  return fem_model_->CalcCenterOfMassTranslationalVelocityInWorld(fem_state);
}

template <typename T>
Vector3<T> DeformableBody<T>::CalcEffectiveAngularVelocity(
    const systems::Context<T>& context) const {
  DRAKE_DEMAND(fem_model_ != nullptr);
  const fem::FemState<T>& fem_state =
      this->GetParentTreeSystem()
          .get_cache_entry(fem_state_cache_index_)
          .template Eval<fem::FemState<T>>(context);
  return fem_model_->CalcEffectiveAngularVelocity(fem_state);
}

template <typename T>
DeformableBody<T>::DeformableBody(
    DeformableBodyIndex index, DeformableBodyId id, std::string name,
    GeometryId geometry_id, ModelInstanceIndex model_instance,
    const VolumeMesh<double>& mesh_G, const math::RigidTransform<double>& X_WG,
    const fem::DeformableBodyConfig<T>& config, const Vector3<double>& weights)
    : MultibodyElement<T>(model_instance, index),
      id_(id),
      name_(std::move(name)),
      geometry_id_(geometry_id),
      mesh_G_(mesh_G),
      X_WG_(X_WG),
      X_WD_(X_WG),
      config_(config) {
  if constexpr (std::is_same_v<T, double>) {
    geometry::VolumeMesh<double> mesh_W = mesh_G;
    mesh_W.TransformVertices(X_WG);
    BuildLinearVolumetricModel(mesh_W, config, weights);
    reference_positions_.resize(3 * mesh_W.num_vertices());
    for (int v = 0; v < mesh_W.num_vertices(); ++v) {
      reference_positions_.template segment<3>(3 * v) = mesh_W.vertex(v);
    }
  } else {
    throw std::runtime_error(
        "DeformableBody<T>::DeformableBody(): T must be double.");
  }
}

template <typename T>
std::unique_ptr<DeformableBody<double>> DeformableBody<T>::CloneToDouble()
    const {
  if constexpr (!std::is_same_v<T, double>) {
    /* A non-double body shouldn't exist in the first place. */
    DRAKE_UNREACHABLE();
  } else {
    auto clone =
        std::unique_ptr<DeformableBody<double>>(new DeformableBody<double>(
            this->index(), id_, name_, geometry_id_, this->model_instance(),
            mesh_G_, X_WG_, config_, fem_model_->tangent_matrix_weights()));
    /* We go through all data member one by one in order, and either copy them
     over or explain why they don't need to be copied. */
    /* id_ is copied in the constructor above. */
    /* name_ is copied in the constructor above. */
    /* geometry_id_ is copied in the constructor above. */
    /* mesh_G_ is copied in the constructor above. */
    /* X_WG_ is copied in the constructor above. */
    /* Copy over X_WD_. */
    clone->X_WD_ = X_WD_;
    /* config_ is copied in the constructor above. */
    /* Copy over reference_positions_. */
    clone->reference_positions_ = reference_positions_;
    /* fem_model_ is constructed in the constructor above. */
    /* discrete_state_index_ is set in DoDeclareDiscreteState() when the
     owning DeformableModel declares system resources. */
    /* is_enabled_parameter_index_ is set in DoDeclareParameters() when the
     owning DeformableModel declares system resources. */
    /* Copy over fixed_constraint_specs_. */
    clone->fixed_constraint_specs_ = fixed_constraint_specs_;
    /* gravity_forces_ and external_forces_ are set when the owning
     DeformableModel declares system resources. */
    return clone;
  }
}

template <typename T>
void DeformableBody<T>::SetExternalForces(
    const std::vector<std::unique_ptr<ForceDensityFieldBase<T>>>&
        external_forces,
    const Vector3<T>& gravity) {
  const T& density = config_.mass_density();
  gravity_force_ = std::make_unique<GravityForceField<T>>(gravity, density);
  external_forces_.clear();
  external_forces_.push_back(gravity_force_.get());
  for (const auto& force : external_forces) {
    external_forces_.push_back(force.get());
  }
}

template <typename T>
template <typename T1>
typename std::enable_if_t<std::is_same_v<T1, double>, void>
DeformableBody<T>::BuildLinearVolumetricModel(
    const VolumeMesh<double>& mesh, const fem::DeformableBodyConfig<T>& config,
    const Vector3<double>& weights) {
  switch (config.material_model()) {
    case MaterialModel::kLinear:
      SelectSubdivisionAndBuildVolumetricModel<
          fem::internal::LinearConstitutiveModel>(mesh, config, weights);
      break;
    case MaterialModel::kCorotated:
      SelectSubdivisionAndBuildVolumetricModel<fem::internal::CorotatedModel>(
          mesh, config, weights);
      break;
    case MaterialModel::kNeoHookean:
      SelectSubdivisionAndBuildVolumetricModel<fem::internal::NeoHookeanModel>(
          mesh, config, weights);
      break;
    case MaterialModel::kLinearCorotated:
      SelectSubdivisionAndBuildVolumetricModel<
          fem::internal::LinearCorotatedModel>(mesh, config, weights);
      break;
  }
}

template <typename T>
template <template <typename> class Model, typename T1>
typename std::enable_if_t<std::is_same_v<T1, double>, void>
DeformableBody<T>::SelectSubdivisionAndBuildVolumetricModel(
    const geometry::VolumeMesh<double>& mesh,
    const fem::DeformableBodyConfig<T>& config,
    const Vector3<double>& weights) {
  switch (config.element_subdivision_count()) {
    case 0:
      BuildLinearVolumetricModelHelper<Model, 0>(mesh, config, weights);
      break;
    case 1:
      BuildLinearVolumetricModelHelper<Model, 1>(mesh, config, weights);
      break;
    case 2:
      BuildLinearVolumetricModelHelper<Model, 2>(mesh, config, weights);
      break;
    case 3:
      BuildLinearVolumetricModelHelper<Model, 3>(mesh, config, weights);
      break;
    case 4:
      BuildLinearVolumetricModelHelper<Model, 4>(mesh, config, weights);
      break;
  }
}

template <typename T>
template <template <typename> class Model, int num_subd, typename T1>
typename std::enable_if_t<std::is_same_v<T1, double>, void>
DeformableBody<T>::BuildLinearVolumetricModelHelper(
    const geometry::VolumeMesh<double>& mesh,
    const fem::DeformableBodyConfig<T>& config,
    const Vector3<double>& weights) {
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
  using ConstitutiveModelType = Model<T>;
  static_assert(
      std::is_base_of_v<
          fem::internal::ConstitutiveModel<
              ConstitutiveModelType, typename ConstitutiveModelType::Traits>,
          ConstitutiveModelType>,
      "The template parameter 'Model' must be derived from "
      "ConstitutiveModel.");
  using SubdQuadratureType =
      std::conditional_t<(num_subd <= 0), QuadratureType,
                         fem::internal::TetSubdivisionQuadrature<num_subd>>;
  using SubdIsoparametricElementType =
      std::conditional_t<(num_subd <= 0), IsoparametricElementType,
                         fem::internal::LinearSimplexElement<
                             T, kNaturalDimension, kSpatialDimension,
                             SubdQuadratureType::num_quadrature_points>>;
  using FemElementType = fem::internal::VolumetricElement<
      IsoparametricElementType, QuadratureType, ConstitutiveModelType,
      SubdIsoparametricElementType, SubdQuadratureType>;
  using FemModelType = fem::internal::VolumetricModel<FemElementType>;

  const fem::DampingModel<T> damping_model(
      config.mass_damping_coefficient(),
      config.stiffness_damping_coefficient());

  auto concrete_fem_model = std::make_unique<FemModelType>(weights);
  ConstitutiveModelType constitutive_model(config.youngs_modulus(),
                                           config.poissons_ratio());
  typename FemModelType::VolumetricBuilder builder(concrete_fem_model.get());
  builder.AddLinearTetrahedralElements(mesh, constitutive_model,
                                       config.mass_density(), damping_model);
  builder.Build();
  fem_model_ = std::move(concrete_fem_model);
}

template <typename T>
void DeformableBody<T>::DoDeclareDiscreteState(
    internal::MultibodyTreeSystem<T>* tree_system) {
  const int num_dofs = fem_model_->num_dofs();
  VectorX<T> model_state = VectorX<T>::Zero(num_dofs * 3 /* q, v, and a */);
  model_state.head(num_dofs) = CalcDefaultPositions();
  discrete_state_index_ = this->DeclareDiscreteState(tree_system, model_state);
}

template <typename T>
void DeformableBody<T>::SetDefaultState(const systems::Context<T>&,
                                        systems::State<T>* state) const {
  state->get_mutable_discrete_state(discrete_state_index_)
      .get_mutable_value()
      .head(fem_model_->num_dofs()) = CalcDefaultPositions();
}

template <typename T>
VectorX<T> DeformableBody<T>::CalcDefaultPositions() const {
  const VectorX<double>& p_WVg = reference_positions_;
  const int num_dofs = fem_model_->num_dofs();
  VectorX<T> p_WVd = VectorX<T>::Zero(num_dofs);
  /* `reference_positions_` stores the list of p_WVg for all vertices V
   in the mesh and we have p_WVg = X_WG * p_GV, where p_GV is the position
   of vertex V in the geometry frame G.
   Now we want the default state to store the poisitions p_WVd, the positions
   of the vertices of the deformable body with the default pose D, which is
   given by p_WVd = X_WD * p_DV. By noting p_DV = p_GV, we get
   p_WVd = X_WD * p_GV = X_WD * X_WG⁻¹ * p_WVd. */
  const math::RigidTransform<double> X_DG = X_WD_ * X_WG_.inverse();
  for (int i = 0; i < num_dofs / 3; ++i) {
    p_WVd.template segment<3>(3 * i) = X_DG * p_WVg.template segment<3>(3 * i);
  }
  return p_WVd;
}

template <typename T>
void DeformableBody<T>::DoDeclareParameters(
    internal::MultibodyTreeSystem<T>* tree_system) {
  is_enabled_parameter_index_ =
      this->DeclareAbstractParameter(tree_system, Value<bool>(true));
}

template <typename T>
void DeformableBody<T>::DoDeclareCacheEntries(
    internal::MultibodyTreeSystem<T>* tree_system) {
  /* Declare cache entry for FemState. */
  DRAKE_DEMAND(fem_model_ != nullptr);
  std::unique_ptr<fem::FemState<T>> model_state = fem_model_->MakeFemState();
  const auto& fem_state_cache_entry = this->DeclareCacheEntry(
      tree_system, fmt::format("fem_state_for_body_{}", id_.get_value()),
      systems::ValueProducer(
          *model_state,
          std::function<void(const systems::Context<T>&, fem::FemState<T>*)>(
              [this](const systems::Context<T>& context,
                     fem::FemState<T>* state) {
                this->CalcFemStateFromDiscreteValues(context, state);
              })),
      {tree_system->xd_ticket()});
  fem_state_cache_index_ = fem_state_cache_entry.cache_index();
}

template <typename T>
void DeformableBody<T>::CalcFemStateFromDiscreteValues(
    const systems::Context<T>& context, fem::FemState<T>* fem_state) const {
  DRAKE_DEMAND(fem_state != nullptr);
  const systems::BasicVector<T>& discrete_vector =
      context.get_discrete_state().get_vector(discrete_state_index_);
  const VectorX<T>& discrete_value = discrete_vector.value();
  DRAKE_DEMAND(discrete_value.size() % 3 == 0);
  const int num_dofs = discrete_value.size() / 3;
  DRAKE_DEMAND(num_dofs == fem_model_->num_dofs());

  fem_state->SetPositions(discrete_value.head(num_dofs));
  fem_state->SetVelocities(discrete_value.segment(num_dofs, num_dofs));
  fem_state->SetAccelerations(discrete_value.tail(num_dofs));
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::DeformableBody);

}  // namespace multibody
}  // namespace drake
