#include "drake/examples/mass_spring_system/mass_spring_system.h"

#include "drake/common/eigen_types.h"
namespace drake {
namespace examples {
namespace mass_spring_system {

template <typename T>
MassSpringSystem<T>::MassSpringSystem(int nx, int ny, T dx, bool is_discrete,
                                      double dt)
    : nx_(nx),
      ny_(ny),
      num_points_(nx * ny),
      dx_(dx),
      dt_(dt),
      left_corner_(0),
      right_corner_(nx_ - 1),
      H_(3 * num_points_, 3 * num_points_) {
  if (is_discrete) {
    // Discrete system.
    // Adding 3*N positions and 3*N velocities.
    this->DeclareDiscreteState(6 * num_points_);
    this->DeclareVectorOutputPort(systems::BasicVector<T>(6 * num_points_),
                                  &MassSpringSystem::CopyDiscreteStateOut);
    this->DeclarePeriodicDiscreteUpdateEvent(
        dt_, 0., &MassSpringSystem::UpdateDiscretePositionAndVelocity);
  } else {
    // Continuous system.
    // Adding 3*N positions and 3*N velocities.
    this->DeclareContinuousState(3 * num_points_, 3 * num_points_, 0);
    // A 6*N dimensional output vector for 3*N positions and 3*N velocities.
    this->DeclareVectorOutputPort(systems::BasicVector<T>(6 * num_points_),
                                  &MassSpringSystem::CopyContinuousStateOut);
  }
  this->DeclareNumericParameter(MassSpringSystemParams<T>());
  BuildRectanglePatch(true);
  // Allocate the H matrix now that we know the sparsity pattern.
  if (is_discrete) {
    // Each node has at least one non-zero entry: the diagnoal entry.
    VectorX<int> num_nonzero_entries = VectorX<int>::Ones(3 * num_points_) * 3;
    for (int i = 0; i < static_cast<int>(springs_.size()); ++i) {
      for (int d = 0; d < 3; ++d) {
        num_nonzero_entries[3 * springs_[i].node0_ + d] += 3;
        num_nonzero_entries[3 * springs_[i].node1_ + d] += 3;
      }
    }
    H_.reserve(num_nonzero_entries);
    H_.setZero();
  }
}

template <typename T>
void MassSpringSystem<T>::BuildRectanglePatch(bool use_shearing_springs) {
  int num_stretching_springs = (nx_ - 1) * ny_ + (ny_ - 1) * nx_;
  int num_shearing_springs =
      use_shearing_springs ? (nx_ - 1) * (ny_ - 1) * 2 : 0;
  springs_.resize(num_stretching_springs + num_shearing_springs);
  int ns = 0;
  // Build springs in the y-direction.
  for (int i = 0; i < nx_; ++i) {
    for (int j = 0; j < ny_ - 1; ++j) {
      int p = i * nx_ + j;
      springs_[ns++] = Spring{p, p + 1, dx_};
    }
  }
  // Build springs in the x-direction.
  for (int i = 0; i < nx_ - 1; ++i) {
    for (int j = 0; j < ny_; ++j) {
      int p = i * nx_ + j;
      springs_[ns++] = Spring{p, p + nx_, dx_};
    }
  }
  if (use_shearing_springs) {
    for (int i = 0; i < nx_ - 1; ++i) {
      for (int j = 0; j < ny_ - 1; ++j) {
        // For each point p, build diagonal springs for the cell that p is on
        // the top-left corner of.
        int p = i * nx_ + j;
        springs_[ns++] = Spring{p, p + nx_ + 1, std::sqrt(2) * dx_};
        springs_[ns++] = Spring{p + 1, p + nx_, std::sqrt(2) * dx_};
      }
    }
  }
}

template <typename T>
void MassSpringSystem<T>::CopyContinuousStateOut(
    const systems::Context<T>& context, systems::BasicVector<T>* output) const {
  // Get current state from context.
  const systems::VectorBase<T>& continuous_state_vector =
      context.get_continuous_state_vector();
  // Write system output.
  output->SetFrom(continuous_state_vector);
}

template <typename T>
void MassSpringSystem<T>::CopyDiscreteStateOut(
    const systems::Context<T>& context, systems::BasicVector<T>* output) const {
  // Get current state from context.
  const systems::BasicVector<T>& discrete_state_vector =
      context.get_discrete_state(0);
  // Write system output.
  output->SetFrom(discrete_state_vector);
}

template <typename T>
void MassSpringSystem<T>::DoCalcTimeDerivatives(
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
  CalcSpringForce(context, &vdot);
  const MassSpringSystemParams<T>& p = get_parameters(context);
  T mass_per_point = p.mass() / static_cast<T>(num_points_);
  // Mass of each mass point must be positive.
  DRAKE_THROW_UNLESS(mass_per_point > 0);
  for (int i = 0; i < vdot.size(); ++i) {
    vdot[i] /= mass_per_point;
  }
  // Apply gravity to acceleration.
  for (int i = 0; i < num_points_; ++i) {
    vdot[3 * i + 2] += p.gravity();
  }

  // Apply Dirichlet BC at two corners.
  ApplyDirichletBoundary(&vdot);
}

template <typename T>
void MassSpringSystem<T>::UpdateDiscretePositionAndVelocity(
    const systems::Context<T>& context,
    systems::DiscreteValues<T>* next_states) const {
  const systems::BasicVector<T>& current_state =
      context.get_discrete_state().get_vector();
  const auto& current_state_values = current_state.get_value();
  const auto& x_n = current_state_values.head(3 * num_points_);
  const auto& v_n = current_state_values.tail(3 * num_points_);
  // v_hat is the velocity of the points after adding in the explicit elastic
  // force and gravity. Add the contribution of explicit gravitational force to
  // v_hat.
  VectorX<T> v_hat(v_n);
  const MassSpringSystemParams<T>& p = get_parameters(context);
  // Add contribution of gravity to v_hat.
  for (int i = 0; i < num_points_; ++i) {
    v_hat[3 * i + 2] += p.gravity() * dt_;
  }
  // Add the contribution of explicit spring elastic force to v_hat.
  VectorX<T> elastic_force = VectorX<T>::Zero(v_hat.size());
  CalcElasticForce(p, x_n, &elastic_force);
  T mass_per_point = p.mass() / static_cast<T>(num_points_);
  v_hat += elastic_force * dt_ / mass_per_point;
  // Apply Dirichlet BC to v_hat.
  ApplyDirichletBoundary(&v_hat);

  // dv is the change in velocity contributed by the implicit damping force.
  // The momentum equation M * dv = f(vⁿ⁺¹) * dt is equivalent to
  // M * dv = (f(v_hat) + ∂f/∂v(v_hat) * dv) * dt,
  // because damping force is linear in v. Moving terms, we end up with
  // (M - ∂f/∂v(v_hat)) * dv = f(v_hat) * dt which we abbreviate as
  // H * dv = f * dt.
  VectorX<T> dv = VectorX<T>::Zero(v_hat.size());
  VectorX<T> damping_force = VectorX<T>::Zero(v_hat.size());
  CalcDampingForce(p, x_n, v_hat, &damping_force);
  CalcDv(p, x_n, damping_force, &dv);

  // Write v_hat + dv into the velocity at the next time step.
  auto next_state_values =
      next_states->get_mutable_vector().get_mutable_value();
  auto x_np1 = next_state_values.head(3 * num_points_);
  auto v_np1 = next_state_values.tail(3 * num_points_);
  v_np1 = v_hat + dv;

  // Apply Dirichlet BC to the final velocity.
  ApplyDirichletBoundary(&v_np1);

  // Update position using velocity at time n+1.
  x_np1 = x_n + dt_ * v_np1;
}

template <typename T>
void MassSpringSystem<T>::CalcSpringForce(
    const systems::Context<T>& context, systems::VectorBase<T>* forces) const {
  // using TV = Eigen::Matrix<T, 3, 1>;
  const systems::VectorBase<T>& x =
      context.get_continuous_state().get_generalized_position();
  const systems::VectorBase<T>& v =
      context.get_continuous_state().get_generalized_velocity();
  const MassSpringSystemParams<T>& p = get_parameters(context);
  CalcElasticForce(p, x, forces);
  CalcDampingForce(p, x, v, forces);
}
template class MassSpringSystem<double>;

}  // namespace mass_spring_system
}  // namespace examples
}  // namespace drake
