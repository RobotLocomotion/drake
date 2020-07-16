#include "drake/examples/mass_spring_cloth/cloth_spring_model.h"

#include "drake/common/eigen_types.h"
namespace drake {
namespace examples {
namespace mass_spring_cloth {

template <typename T>
ClothSpringModel<T>::ClothSpringModel(int nx, int ny, T h, double dt)
    : nx_(nx),
      ny_(ny),
      num_points_(nx * ny),
      h_(h),
      dt_(dt),
      left_corner_(0),
      right_corner_(ny_ - 1),
      H_(3 * num_points_, 3 * num_points_) {
  if (dt > 0) {
    // Discrete system.
    // Adding 3*N positions and 3*N velocities.
    this->DeclareDiscreteState(6 * num_points_);
    this->DeclareVectorOutputPort("particle_positions",
                                  systems::BasicVector<T>(3 * num_points_),
                                  &ClothSpringModel::CopyDiscreteStateOut);
    this->DeclarePeriodicDiscreteUpdateEvent(
        dt_, 0., &ClothSpringModel::UpdateDiscreteState);
  } else {
    // Continuous system.
    // Adding 3*N positions and 3*N velocities.
    this->DeclareContinuousState(3 * num_points_, 3 * num_points_, 0);
    // A 6*N dimensional output vector for 3*N positions and 3*N velocities.
    this->DeclareVectorOutputPort(systems::BasicVector<T>(3 * num_points_),
                                  &ClothSpringModel::CopyContinuousStateOut);
  }
  param_index_ = this->DeclareNumericParameter(ClothSpringModelParams<T>());
  BuildConnectingSprings(true);
  // Allocate the H matrix now that we know the sparsity pattern.
  if (dt > 0) {
    // The H matrix is organized into num_points * num_points blocks of 3 * 3
    // sub-matrices. The ij-th 3*3 sub-matrix consists of M_ij (mass matrix) and
    // ∂f_i/∂v_j (damping force derivative). Since we do mass-lumping, M_ij =
    // mass(i) * Identity(3,3) if i==j, and Zero(3,3) otherwise. If there exists
    // a spring connecting points i and j, the blocks
    // ∂f_i/∂v_i,∂f_j/∂v_j,∂f_i/∂v_j,and ∂f_j/∂v_i will contain nonzero entries.
    VectorX<int> num_nonzero_entries = VectorX<int>::Ones(3 * num_points_) * 3;
    for (const Spring& s : springs_) {
      for (int d = 0; d < 3; ++d) {
        num_nonzero_entries[3 * s.particle0 + d] += 3;
        num_nonzero_entries[3 * s.particle1 + d] += 3;
      }
    }
    H_.reserve(num_nonzero_entries);
    H_.setZero();
  }
}

template <typename T>
void ClothSpringModel<T>::InitializePositionAndVelocity(
    systems::Context<T>* context) const {
  if (dt_ > 0) {
    // Extract mutable position and velocity states and initialize.
    auto state_values = context->get_mutable_discrete_state()
                            .get_mutable_vector()
                            .get_mutable_value();
    auto x = state_values.head(3 * nx_ * ny_);
    auto v = state_values.tail(3 * nx_ * ny_);
    InitializePositionAndVelocity(&x, &v);
  } else {
    // Extract mutable position and velocity states and initialize.
    systems::VectorBase<T>& x = context->get_mutable_continuous_state()
                                    .get_mutable_generalized_position();
    systems::VectorBase<T>& v = context->get_mutable_continuous_state()
                                    .get_mutable_generalized_velocity();
    InitializePositionAndVelocity(&x, &v);
  }
}

template <typename T>
void ClothSpringModel<T>::BuildConnectingSprings(bool use_shearing_springs) {
  const int num_stretching_springs = (nx_ - 1) * ny_ + (ny_ - 1) * nx_;
  const int num_shearing_springs =
      use_shearing_springs ? (nx_ - 1) * (ny_ - 1) * 2 : 0;
  springs_.resize(num_stretching_springs + num_shearing_springs);
  int ns = 0;
  // Build springs in the y-direction.
  for (int i = 0; i < nx_; ++i) {
    for (int j = 0; j < ny_ - 1; ++j) {
      const int p = i * ny_ + j;
      springs_[ns++] = Spring{p, p + 1, h_};
    }
  }
  // Build springs in the x-direction.
  for (int i = 0; i < nx_ - 1; ++i) {
    for (int j = 0; j < ny_; ++j) {
      const int p = i * ny_ + j;
      springs_[ns++] = Spring{p, p + ny_, h_};
    }
  }
  if (use_shearing_springs) {
    for (int i = 0; i < nx_ - 1; ++i) {
      for (int j = 0; j < ny_ - 1; ++j) {
        // For each point p, build diagonal and anti-diagonal springs for the
        // cell that p is on the top-left corner of.
        /*
                  | p     | p+1
              ────•───────•────
                  | \   / |
                  |   X   |
                  | /   \ |
              ────•───────•────
                  | p+ny_ | p+ny_+1
         */
        const int p = i * ny_ + j;
        springs_[ns++] = Spring{p, p + ny_ + 1, std::sqrt(2) * h_};
        springs_[ns++] = Spring{p + 1, p + ny_, std::sqrt(2) * h_};
      }
    }
  }
}

template <typename T>
void ClothSpringModel<T>::CopyContinuousStateOut(
    const systems::Context<T>& context, systems::BasicVector<T>* output) const {
  output->SetFrom(context.get_continuous_state().get_generalized_position());
}

template <typename T>
void ClothSpringModel<T>::CopyDiscreteStateOut(
    const systems::Context<T>& context, systems::BasicVector<T>* output) const {
  const systems::BasicVector<T>& discrete_state_vector =
      context.get_discrete_state(0);
  output->SetFromVector(
      discrete_state_vector.get_value().head(3 * num_points_));
}

template <typename T>
void ClothSpringModel<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  const systems::VectorBase<T>& v =
      context.get_continuous_state().get_generalized_velocity();
  systems::VectorBase<T>& xdot =
      derivatives->get_mutable_generalized_position();
  xdot.SetFrom(v);
  systems::VectorBase<T>& vdot =
      derivatives->get_mutable_generalized_velocity();
  // Store spring force in vdot and then divide by mass to get acceleration.
  vdot.SetZero();
  AccumulateContinuousSpringForce(context, &vdot);
  const ClothSpringModelParams<T>& p = get_parameters(context);
  T mass_per_point = p.mass() / static_cast<T>(num_points_);
  // Mass of each mass point must be positive.
  DRAKE_THROW_UNLESS(mass_per_point > 0);
  for (int i = 0; i < vdot.size(); ++i) {
    vdot[i] /= mass_per_point;
  }
  // Apply gravity to acceleration.
  const Vector3<T> gravity{0, 0, p.gravity()};
  for (int i = 0; i < num_points_; ++i) {
    accumulate_particle_state(i, gravity, &vdot);
  }
  ApplyDirichletBoundary(&vdot);
}

template <typename T>
void ClothSpringModel<T>::UpdateDiscreteState(
    const systems::Context<T>& context,
    systems::DiscreteValues<T>* next_states) const {
  const systems::BasicVector<T>& current_state =
      context.get_discrete_state().get_vector();
  const Eigen::VectorBlock<const VectorX<T>>& current_state_values =
      current_state.get_value();
  const auto& x_n = current_state_values.head(3 * num_points_);
  const auto& v_n = current_state_values.tail(3 * num_points_);
  // v_hat is the velocity of the points after adding the effect of the explicit
  // elastic and gravity forces.
  VectorX<T> v_hat(v_n);
  const ClothSpringModelParams<T>& p = get_parameters(context);
  // Add contribution of gravity to v_hat.
  const Vector3<T> gravity_dv{0, 0, p.gravity() * dt_};
  for (int i = 0; i < num_points_; ++i) {
    accumulate_particle_state(i, gravity_dv, &v_hat);
  }
  // Add the contribution of explicit spring elastic force to v_hat.
  VectorX<T> elastic_force = VectorX<T>::Zero(v_hat.size());
  AccumulateElasticForce(p, x_n, &elastic_force);
  T mass_per_point = p.mass() / static_cast<T>(num_points_);
  v_hat += elastic_force * dt_ / mass_per_point;
  ApplyDirichletBoundary(&v_hat);

  // dv is the change in velocity contributed by the implicit damping force.
  // The momentum equation M * dv = f(vⁿ⁺¹) * dt is equivalent to
  // M * dv = (f(v_hat) + ∂f/∂v(v_hat) * dv) * dt,
  // because damping force is linear in v. Moving terms, we end up with
  // (M - ∂f/∂v(v_hat)) * dv = f(v_hat) * dt which we abbreviate as
  // H * dv = f * dt.
  VectorX<T> dv = VectorX<T>::Zero(v_hat.size());
  VectorX<T> damping_force = VectorX<T>::Zero(v_hat.size());
  AccumulateDampingForce(p, x_n, v_hat, &damping_force);
  CalcDiscreteDv(p, x_n, damping_force, &dv);

  // Write v_hat + dv into the velocity at the next time step.
  Eigen::VectorBlock<VectorX<T>> next_state_values =
      next_states->get_mutable_vector().get_mutable_value();
  auto next_x = next_state_values.head(3 * num_points_);
  auto next_v = next_state_values.tail(3 * num_points_);
  next_v = v_hat + dv;
  ApplyDirichletBoundary(&next_v);
  // Update position using velocity at time n+1.
  next_x = x_n + dt_ * next_v;
}

template <typename T>
void ClothSpringModel<T>::AccumulateContinuousSpringForce(
    const systems::Context<T>& context, systems::VectorBase<T>* forces) const {
  const systems::VectorBase<T>& x =
      context.get_continuous_state().get_generalized_position();
  const systems::VectorBase<T>& v =
      context.get_continuous_state().get_generalized_velocity();
  const ClothSpringModelParams<T>& p = get_parameters(context);
  AccumulateElasticForce(p, x, forces);
  AccumulateDampingForce(p, x, v, forces);
}
template class ClothSpringModel<double>;

}  // namespace mass_spring_cloth
}  // namespace examples
}  // namespace drake
