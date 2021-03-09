#include "drake/multibody/fixed_fem/dev/softsim_system.h"

#include "drake/multibody/fixed_fem/dev/corotated_model.h"
#include "drake/multibody/fixed_fem/dev/linear_constitutive_model.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
template <typename T>
SoftsimSystem<T>::SoftsimSystem(double dt) : dt_(dt) {
  DRAKE_DEMAND(dt > 0);
  vertex_positions_port_ =
      this->DeclareAbstractOutputPort("vertex_positions",
                                      &SoftsimSystem::CopyVertexPositionsOut)
          .get_index();
  this->DeclarePeriodicDiscreteUpdateEvent(dt, 0,
                                           &SoftsimSystem::AdvanceOneTimeStep);
}

template <typename T>
BodyIndex SoftsimSystem<T>::RegisterDeformableBody(
    const geometry::VolumeMesh<T>& mesh, std::string name,
    const DeformableBodyConfig<T>& config) {
  /* Throw if name is not unique. */
  for (int i = 0; i < num_bodies(); ++i) {
    if (name == names_[i]) {
      throw std::runtime_error(fmt::format(
          "{}(): A body with name '{}' already exists in the system.", __func__,
          name));
    }
  }
  BodyIndex body_index(num_bodies());
  switch (config.material_model()) {
    case MaterialModel::kLinear:
      RegisterDeformableBodyHelper<LinearConstitutiveModel>(
          mesh, std::move(name), config);
      break;
    case MaterialModel::kCorotated:
      RegisterDeformableBodyHelper<CorotatedModel>(mesh, std::move(name),
                                                   config);
      break;
    default:
      DRAKE_UNREACHABLE();
  }
  return body_index;
}

template <typename T>
void SoftsimSystem<T>::SetRegisteredBodyInWall(int body_id,
                                               const Vector3<T>& p_WQ,
                                               const Vector3<T>& n_W) {
  DRAKE_THROW_UNLESS(body_id < num_bodies());
  const int kDim = 3;
  FemSolver<T>& fem_solver = *fem_solvers_[body_id];
  FemModelBase<T>& fem_model = fem_solver.mutable_model();
  const int num_nodes = fem_model.num_nodes();
  const std::unique_ptr<FemStateBase<T>> fem_state =
      fem_model.MakeFemStateBase();
  const VectorX<T>& initial_positions = fem_state->q();
  auto bc = std::make_unique<DirichletBoundaryCondition<T>>(/* ODE order */ 2);
  for (int n = 0; n < num_nodes; ++n) {
    const Vector3<T>& Q = initial_positions.template segment<kDim>(n * kDim);
    if ((Q - p_WQ).dot(n_W) <= 0) {
      const int dof_index(kDim * n);
      for (int d = 0; d < kDim; ++d) {
        bc->AddBoundaryCondition(DofIndex(dof_index + d),
                                 Vector3<T>(Q(d), 0, 0));
      }
    }
  }
  fem_model.SetDirichletBoundaryCondition(std::move(bc));
}

template <typename T>
void SoftsimSystem<T>::AdvanceOneTimeStep(
    const systems::Context<T>& context,
    systems::DiscreteValues<T>* next_states) const {
  const systems::DiscreteValues<T>& all_discrete_states =
      context.get_discrete_state();
  for (int i = 0; i < num_bodies(); ++i) {
    /* Extract q, qdot and qddot from context. */
    const systems::BasicVector<T>& discrete_state =
        all_discrete_states.get_vector(i);
    const auto& discrete_value = discrete_state.get_value();
    const int num_dofs = discrete_value.size() / 3;
    const auto& q = discrete_value.head(num_dofs);
    const auto& qdot = discrete_value.segment(num_dofs, num_dofs);
    const auto& qddot = discrete_value.tail(num_dofs);
    /* Set up FemState and advance to the next time step. */
    FemStateBase<T>& prev_fem_state = *prev_fem_states_[i];
    FemStateBase<T>& next_fem_state = *next_fem_states_[i];
    prev_fem_state.SetQ(q);
    prev_fem_state.SetQdot(qdot);
    prev_fem_state.SetQddot(qddot);
    // TODO(xuchenhan-tri): FemState needs a SetFrom() method. Setting
    //  DiscreteValues from FemStateBase (and vice-versa) should also be made
    //  more compact.
    next_fem_state.SetQ(q);
    next_fem_state.SetQdot(qdot);
    next_fem_state.SetQddot(qddot);
    fem_solvers_[i]->AdvanceOneTimeStep(prev_fem_state, &next_fem_state);
    /* Copy new state to output variable. */
    systems::BasicVector<T>& next_discrete_state =
        next_states->get_mutable_vector(i);
    Eigen::VectorBlock<VectorX<T>> next_discrete_value =
        next_discrete_state.get_mutable_value();
    next_discrete_value.head(num_dofs) = next_fem_state.q();
    next_discrete_value.segment(num_dofs, num_dofs) = next_fem_state.qdot();
    next_discrete_value.tail(num_dofs) = next_fem_state.qddot();
  }
}

template <typename T>
void SoftsimSystem<T>::CopyVertexPositionsOut(
    const systems::Context<T>& context, std::vector<VectorX<T>>* output) const {
  output->resize(num_bodies());
  const systems::DiscreteValues<T>& all_discrete_states =
      context.get_discrete_state();
  for (int i = 0; i < num_bodies(); ++i) {
    const systems::BasicVector<T>& discrete_state =
        all_discrete_states.get_vector(i);
    const auto& discrete_value = discrete_state.get_value();
    const int num_dofs = discrete_value.size() / 3;
    const auto& q = discrete_value.head(num_dofs);
    (*output)[i] = q;
  }
}
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fixed_fem::SoftsimSystem);
