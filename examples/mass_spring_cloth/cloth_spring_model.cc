#include "drake/examples/mass_spring_cloth/cloth_spring_model.h"

#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace mass_spring_cloth {

template <typename T>
ClothSpringModel<T>::ClothSpringModel(int nx, int ny, T h, double dt)
    : nx_(nx),
      ny_(ny),
      num_particles_(nx * ny),
      h_(h),
      dt_(dt),
      bottom_left_corner_(0),
      top_left_corner_(ny_ - 1),
      H_(3 * num_particles_, 3 * num_particles_) {
  systems::BasicVector<T> initial_state = InitializePositionAndVelocity();
  if (dt > 0) {
    // Discrete system.
    // Adding 3*N positions and 3*N velocities.
    this->DeclareDiscreteState(initial_state);
    this->DeclareVectorOutputPort("particle_positions", 3 * num_particles_,
                                  &ClothSpringModel::CopyDiscreteStateOut);
    this->DeclarePeriodicDiscreteUpdateEvent(
        dt_, 0., &ClothSpringModel::UpdateDiscreteState);
  } else {
    // Continuous system.
    // Adding 3*N positions and 3*N velocities.
    this->DeclareContinuousState(initial_state, 3 * num_particles_,
                                 3 * num_particles_, 0);
    // A 3*N dimensional output vector for positions.
    this->DeclareVectorOutputPort(systems::kUseDefaultName, 3 * num_particles_,
                                  &ClothSpringModel::CopyContinuousStateOut);
  }
  param_index_ = this->DeclareNumericParameter(ClothSpringModelParams<T>());
  BuildConnectingSprings(true);
  // As in symbolic assembly, we allocate for the nonzero entries in the H
  // matrix now that we know the sparsity pattern.
  if (dt > 0) {
    // The H matrix is organized into num_particles * num_particles blocks of 3
    // * 3 sub-matrices. The ij-th 3*3 sub-matrix consists of M_ij (mass matrix)
    // and ∂f_i/∂v_j (damping force derivative). M_ij is a diagonal matrix with
    // the mass of each particle on the diagonal. If there exists a spring
    // connecting particles i and j, the blocks
    // ∂f_i/∂v_i,∂f_j/∂v_j,∂f_i/∂v_j,and ∂f_j/∂v_i will contain nonzero entries.
    VectorX<int> num_nonzero_entries =
        VectorX<int>::Ones(3 * num_particles_) * 3;
    for (const Spring& s : springs_) {
      for (int d = 0; d < 3; ++d) {
        num_nonzero_entries[3 * s.particle0 + d] += 3;
        num_nonzero_entries[3 * s.particle1 + d] += 3;
      }
    }
    H_.reserve(num_nonzero_entries);
    H_.setZero();
    // CG is guaranteed to converge when iteration reaches the number of DOFs.
    // It usually takes far fewer iterations to converge.
    set_linear_solve_max_iterations(num_particles_ * 3);
    // Set the default accuracy for the linear solve.
    set_linear_solve_accuracy();
  }
}

template <typename T>
systems::BasicVector<T> ClothSpringModel<T>::InitializePositionAndVelocity()
    const {
  // Each particle has 6 state variables, 3 for positions, 3 for velocities.
  VectorX<T> state_values(6 * num_particles_);
  auto q = state_values.head(3 * num_particles_);
  auto v = state_values.tail(3 * num_particles_);
  for (int i = 0; i < nx_; ++i) {
    for (int j = 0; j < ny_; ++j) {
      int particle_index = i * ny_ + j;
      /* The particles are ordered in the following fashion:
      +y
        ^
        ┊
        ┊ny-1    2ny-1            nx*ny-1
        ●━━━━━━━●━━┄┄┄┄━━●━━━━━━━●
        ┃       ┃        ┃       ┃
        ┃ny-2   ┃2ny-2   ┃       ┃
        ●━━━━━━━●━━┄┄┄┄━━●━━━━━━━●
        ┃       ┃        ┃       ┃
        ┊       ┊        ┊       ┊
        ┊       ┊        ┊       ┊
        ┃       ┃        ┃       ┃
        ┃1      ┃ny+1    ┃       ┃
        ●━━━━━━━●━━┄┄┄┄━━●━━━━━━━●
        ┃       ┃        ┃       ┃
        ┃0      ┃ny      ┃       ┃(nx-1)*ny
      ┄┄●━━━━━━━●━━┄┄┄┄━━●━━━━━━━●┄┄┄┄┄┄┄┄┄> +x
        ┊

        Note that we replaced nx_/ny_ with nx/ny in the diagram above for more
      readability.
      */
      this->set_particle_state(particle_index, {i * h_, j * h_, 0.0}, &q);
      this->set_particle_state(particle_index, {0.0, 0.0, 0.0}, &v);
    }
  }
  return systems::BasicVector<T>(state_values);
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
        // For each particle p, build diagonal and anti-diagonal springs for the
        // cell that p is on the bottom-left corner of.
        /*
                  | p+1   | p+ny+1
              ────•───────•────
                  | \   / |
                  |   X   |
                  | /   \ |
              ────•───────•────
                  | p     | p+ny
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
      discrete_state_vector.value().head(3 * num_particles_));
}

template <typename T>
void ClothSpringModel<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  const systems::VectorBase<T>& v =
      context.get_continuous_state().get_generalized_velocity();
  systems::VectorBase<T>& qdot =
      derivatives->get_mutable_generalized_position();
  qdot.SetFrom(v);
  systems::VectorBase<T>& vdot =
      derivatives->get_mutable_generalized_velocity();
  // Store spring force in vdot and then divide by mass to get acceleration.
  VectorX<T> tmp_vdot = VectorX<T>::Zero(vdot.size());
  AccumulateContinuousSpringForce(context, &tmp_vdot);
  const ClothSpringModelParams<T>& p = GetParameters(context);
  const T mass_per_particle = p.mass() / static_cast<T>(num_particles_);
  // Mass of each particle must be positive.
  DRAKE_THROW_UNLESS(mass_per_particle > 0);
  for (int i = 0; i < vdot.size(); ++i) {
    tmp_vdot[i] /= mass_per_particle;
  }
  // Apply gravity to acceleration.
  const Vector3<T> gravity{0, 0, p.gravity()};
  for (int i = 0; i < num_particles_; ++i) {
    accumulate_particle_state(i, gravity, &tmp_vdot);
  }
  ApplyDirichletBoundary(&tmp_vdot);
  vdot.SetFromVector(tmp_vdot);
}

template <typename T>
void ClothSpringModel<T>::UpdateDiscreteState(
    const systems::Context<T>& context,
    systems::DiscreteValues<T>* next_states) const {
  const VectorX<T>& current_state_values = context.get_discrete_state().value();
  const auto& q_n = current_state_values.head(3 * num_particles_);
  const auto& v_n = current_state_values.tail(3 * num_particles_);
  // v_hat is the velocity of the particles after adding the effect of the
  // explicit elastic and gravity forces.
  VectorX<T> v_hat(v_n);
  const ClothSpringModelParams<T>& p = GetParameters(context);
  // Add contribution of gravity to v_hat.
  const Vector3<T> gravity_dv{0, 0, p.gravity() * dt_};
  for (int i = 0; i < num_particles_; ++i) {
    accumulate_particle_state(i, gravity_dv, &v_hat);
  }
  // Add the contribution of explicit spring elastic force to v_hat.
  VectorX<T> elastic_force = VectorX<T>::Zero(v_hat.size());
  AccumulateElasticForce(p, q_n, &elastic_force);
  T mass_per_particle = p.mass() / static_cast<T>(num_particles_);
  v_hat += elastic_force * dt_ / mass_per_particle;
  ApplyDirichletBoundary(&v_hat);

  // dv is the change in velocity contributed by the implicit damping force.
  // Freezing q to be q_n, the momentum equation
  //
  // M * dv = f(vⁿ⁺¹) * dt
  //
  // is equivalent to
  //
  // M * dv = (f(v_hat) + ∂f/∂v(v_hat) * dv) * dt

  // because the damping force is linear in v. Moving terms, we end up with

  // (M - ∂f/∂v(v_hat)) * dv = f(v_hat) * dt,

  // which we abbreviate as H * dv = f * dt.
  VectorX<T> dv = VectorX<T>::Zero(v_hat.size());
  VectorX<T> damping_force = VectorX<T>::Zero(v_hat.size());
  AccumulateDampingForce(p, q_n, v_hat, &damping_force);
  CalcDiscreteDv(p, q_n, &damping_force, &dv);

  // Write v_hat + dv into the velocity at the next time step.
  Eigen::VectorBlock<VectorX<T>> next_state_values =
      next_states->get_mutable_value();
  auto next_q = next_state_values.head(3 * num_particles_);
  auto next_v = next_state_values.tail(3 * num_particles_);
  next_v = v_hat + dv;
  // Apply BC again to prevent numerical drift.
  ApplyDirichletBoundary(&next_v);
  // Update position using velocity at time n+1.
  next_q = q_n + dt_ * next_v;
}

template <typename T>
void ClothSpringModel<T>::AccumulateContinuousSpringForce(
    const systems::Context<T>& context, EigenPtr<VectorX<T>> forces) const {
  DRAKE_DEMAND(forces != nullptr);
  const VectorX<T> continuous_states =
      context.get_continuous_state().CopyToVector();
  const auto& q = continuous_states.head(num_positions());
  const auto& v = continuous_states.segment(num_positions(), num_velocities());
  const ClothSpringModelParams<T>& p = GetParameters(context);
  AccumulateElasticForce(p, q, forces);
  AccumulateDampingForce(p, q, v, forces);
}

template <typename T>
void ClothSpringModel<T>::AccumulateElasticForce(
    const ClothSpringModelParams<T>& param,
    const Eigen::Ref<const VectorX<T>>& q,
    EigenPtr<VectorX<T>> elastic_force) const {
  DRAKE_DEMAND(elastic_force != nullptr);
  for (const Spring& s : springs_) {
    // Get the positions of the two particles connected by the spring.
    const int p0 = s.particle0;
    const int p1 = s.particle1;
    const Vector3<T> p_WP0 = particle_state(p0, q);
    const Vector3<T> p_WP1 = particle_state(p1, q);
    const Vector3<T> p_P0P1_W = p_WP1 - p_WP0;
    const T spring_length = p_P0P1_W.norm();
    ThrowIfInvalidSpringLength(spring_length, s.rest_length);
    const Vector3<T> n = p_P0P1_W / spring_length;
    // If the n is the unit vector point from P0 to P1,
    // the spring elastic force = k * (current_length - rest_length) * n
    const Vector3<T> f = param.k() * (spring_length - s.rest_length) * n;
    accumulate_particle_state(p0, f, elastic_force);
    accumulate_particle_state(p1, -f, elastic_force);
  }
}

template <typename T>
void ClothSpringModel<T>::AccumulateDampingForce(
    const ClothSpringModelParams<T>& param,
    const Eigen::Ref<const VectorX<T>>& q,
    const Eigen::Ref<const VectorX<T>>& v,
    EigenPtr<VectorX<T>> damping_force) const {
  DRAKE_DEMAND(damping_force != nullptr);
  for (const Spring& s : springs_) {
    // Get the positions and velocities of the two particles connected by
    // the spring.
    const int p0 = s.particle0;
    const int p1 = s.particle1;
    const Vector3<T> p_WP0 = particle_state(p0, q);
    const Vector3<T> p_WP1 = particle_state(p1, q);
    const Vector3<T> v_WP0 = particle_state(p0, v);
    const Vector3<T> v_WP1 = particle_state(p1, v);
    const Vector3<T> p_P0P1_W = p_WP1 - p_WP0;
    const T spring_length = p_P0P1_W.norm();
    ThrowIfInvalidSpringLength(spring_length, s.rest_length);
    const Vector3<T> n = p_P0P1_W / spring_length;
    // If the n is the unit vector point from q0 to q1,
    // the damping force = (damping coefficient * velocity difference)
    // projected in the direction of n.
    const Vector3<T> f = param.d() * (v_WP1 - v_WP0).dot(n) * n;
    accumulate_particle_state(p0, f, damping_force);
    accumulate_particle_state(p1, -f, damping_force);
  }
}
template <typename T>
void ClothSpringModel<T>::CalcDiscreteDv(const ClothSpringModelParams<T>& param,
                                         const VectorX<T>& q, VectorX<T>* f,
                                         VectorX<T>* dv) const {
  DRAKE_DEMAND(q.size() == dv->size());
  DRAKE_DEMAND(q.size() == f->size());
  const T mass_per_particle = param.mass() / static_cast<T>(num_particles_);
  // Here we construct the H matrix. Take extreme care to overwrite the stale
  // data from the previous time step. Initialize the diagonal of the matrix to
  // be mass_per_particle.
  for (int i = 0; i < num_particles_; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        H_.coeffRef(3 * i + j, 3 * i + k) =
            (j == k) ? mass_per_particle : T(0.0);
      }
    }
  }
  // Add in the contribution from damping force differential.
  for (const Spring& s : springs_) {
    // Get the positions of the two particles connected by the spring.
    const int p0 = s.particle0;
    const int p1 = s.particle1;
    const Vector3<T> p_WP0 = particle_state(p0, q);
    const Vector3<T> p_WP1 = particle_state(p1, q);
    const Vector3<T> p_P0P1_W = p_WP1 - p_WP0;
    const T spring_length = p_P0P1_W.norm();
    ThrowIfInvalidSpringLength(spring_length, s.rest_length);
    const Vector3<T> n = p_P0P1_W / spring_length;
    // If the n is the unit vector point from q0 to q1,
    // the spring elastic force = elastic stiffness * (length - restlength) *
    // n, and damping force = (damping coefficient * velocity difference)
    // projected in the direction of n.
    const Matrix3<T> minus_damping_force_derivative =
        param.d() * dt_ * n * n.transpose();
    // H_ is a sparse matrix. If it were a DenseMatrix, these for loops would
    // be equivalent to:
    // H_.block<3,3>(3*p0, 3*p0) += minus_damping_force_derivative;
    // H_.block<3,3>(3*p1, 3*p1) += minus_damping_force_derivative;
    // H_.block<3,3>(3*p0, 3*p1) = -minus_damping_force_derivative;
    // H_.block<3,3>(3*p1, 3*p0) = -minus_damping_force_derivative;
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        // The diagonal blocks get contribution from multiple springs. We can
        // safely add to the diagonal blocks because they were reset earlier.
        H_.coeffRef(3 * p0 + j, 3 * p0 + k) +=
            minus_damping_force_derivative(j, k);
        H_.coeffRef(3 * p1 + j, 3 * p1 + k) +=
            minus_damping_force_derivative(j, k);
        // Do not add in the off-diagonal terms involving fixed particles.
        if (p0 != bottom_left_corner_ && p0 != top_left_corner_ &&
            p1 != bottom_left_corner_ && p1 != top_left_corner_) {
          // Each off-diagonal block only gets contribution from a single
          // spring. It's important to overwrite the value from previous
          // time-step.
          H_.coeffRef(3 * p1 + j, 3 * p0 + k) =
              -minus_damping_force_derivative(j, k);
          H_.coeffRef(3 * p0 + j, 3 * p1 + k) =
              -minus_damping_force_derivative(j, k);
        }
      }
    }
  }
  // Set the blocks in H corresponding to the particles subject to boundary
  // condition to the identity matrix.
  for (int j = 0; j < 3; ++j) {
    for (int k = 0; k < 3; ++k) {
      H_.coeffRef(3 * bottom_left_corner_ + j, 3 * bottom_left_corner_ + k) =
          (j == k) ? 1.0 : 0.0;
      H_.coeffRef(3 * top_left_corner_ + j, 3 * top_left_corner_ + k) =
          (j == k) ? 1.0 : 0.0;
    }
  }
  // Set the right-hand-side entries corresponding to particles subject to
  // boundary conditions to zero.
  set_particle_state(bottom_left_corner_, {0.0, 0.0, 0.0}, f);
  set_particle_state(top_left_corner_, {0.0, 0.0, 0.0}, f);
  cg_.compute(H_);
  (*dv) = cg_.solve((*f) * dt_);
  DRAKE_THROW_UNLESS(cg_.info() == Eigen::ComputationInfo::Success);
}

template <typename T>
void ClothSpringModel<T>::ThrowIfInvalidSpringLength(
    const T& spring_length, const T& rest_length) const {
  constexpr double kRelativeTolerance =
      10 * std::numeric_limits<double>::epsilon();
  constexpr char prefix[] =
      "Two particles are nearly coincident; the simulation reached an "
      "invalid state.";
  constexpr char postfix[] = "Try simulating a less energetic condition.";
  if (spring_length < kRelativeTolerance * rest_length) {
    if (dt_ > 0) {
      throw std::runtime_error(
          fmt::format("{} Current discrete time step ({} s) may be too "
                      "large. Try a smaller value. If that does not work, {}",
                      prefix, dt_, postfix));
    } else {
      throw std::runtime_error(fmt::format("{} {}", prefix, postfix));
    }
  }
}
template class ClothSpringModel<double>;
}  // namespace mass_spring_cloth
}  // namespace examples
}  // namespace drake
