#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/examples/mass_spring_system/gen/mass_spring_system_params.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace mass_spring_system {
/// A system of mass points connected by visco-elastic springs.
///
/// The dynamics of the mass points follow @f$ \dot x = v,  m \dot v = f @f$,
/// where @f$ f @f$ is given by the combination of elastic and damping forces
/// from springs and gravity.
///
/// - States/Outputs:
///   - linear positions in @f$ m @f$ units.
///   - linear velocities in @f$ m/s @f$ units.
///
/// @tparam_double_only
template <typename T>
class MassSpringSystem final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MassSpringSystem)

  MassSpringSystem(int nx, int ny, T dx, bool is_discrete = true,
                   double dt = 1.0 / 30.0);

 protected:
  void CopyContinuousStateOut(const systems::Context<T>& context,
                              systems::BasicVector<T>* output) const;

  void CopyDiscreteStateOut(const systems::Context<T>& context,
                            systems::BasicVector<T>* output) const;

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  void UpdateDiscretePositionAndVelocity(
      const systems::Context<T>& context,
      systems::DiscreteValues<T>* next_states) const;

  // Calculates the spring (elastic and damping combined) forces given the
  // context.
  void CalcSpringForce(const systems::Context<T>& context,
                       systems::VectorBase<T>* forces) const;
  const MassSpringSystemParams<T>& get_parameters(
      const systems::Context<T>& context) const {
    return this->template GetNumericParameter<MassSpringSystemParams>(context,
                                                                      0);
  }
  struct Spring {
    int node0_, node1_;  // Indices of the two nodes.
    T rest_length_;
  };

 private:
  int nx_, ny_;     // Number of points in the x and y direction.
  int num_points_;  // Total number of mass points.
  T dx_;            // The distance between neighboring mass points.
  T dt_;            // The time period between discrete updates.
  int left_corner_,
      right_corner_;  // Node index of two corners of the grid.
  mutable Eigen::SparseMatrix<T>
      H_;  // Pre-allocated H matrix to prevent reallocations.
  std::vector<Spring> springs_;
  mutable Eigen::ConjugateGradient<Eigen::SparseMatrix<double>,
                                   Eigen::Lower | Eigen::Upper>
      cg_;

  // Generates the connectivity of the mesh of springs.
  void BuildRectanglePatch(bool use_shearing_springs);

  // Calculates the damping force from springs given the positions of the nodes
  // and add to the output elastic_force. The values contained in elastic_force
  // will not be reset.
  template <class PositionVectorType, class ForceVectorType>
  void CalcElasticForce(const MassSpringSystemParams<T>& param,
                        const PositionVectorType& x,
                        ForceVectorType* elastic_force) const {
    using TV = Eigen::Matrix<T, 3, 1>;
    for (int i = 0; i < static_cast<int>(springs_.size()); ++i) {
      const Spring& s = springs_[i];
      // Get the positions and velocities of the two mass points connected by
      // the spring.
      int n0 = s.node0_, n1 = s.node1_;
      TV x0 = TV(x[3 * n0], x[3 * n0 + 1], x[3 * n0 + 2]);
      TV x1 = TV(x[3 * n1], x[3 * n1 + 1], x[3 * n1 + 2]);
      TV d = x1 - x0;
      TV n = d.normalized();
      // If the n is the unit vector point from x0 to x1,
      // the spring elastic force = k * (current_length - rest_length) * n
      TV f = param.k() * (d.norm() - s.rest_length_) * n;
      for (int j = 0; j < 3; ++j) {
        (*elastic_force)[3 * n0 + j] += f[j];
        (*elastic_force)[3 * n1 + j] -= f[j];
      }
    }
  }

  // Calculate the damping force from springs given the positions and velocities
  // of the nodes and add to the output damping_force. The values contained in
  // damping_force will not be reset.
  template <class PositionVectorType, class VelocityVectorType,
            class ForceVectorType>
  void CalcDampingForce(const MassSpringSystemParams<T>& param,
                        const PositionVectorType& x,
                        const VelocityVectorType& v,
                        ForceVectorType* damping_force) const {
    using TV = Eigen::Matrix<T, 3, 1>;
    for (int i = 0; i < static_cast<int>(springs_.size()); ++i) {
      const Spring& s = springs_[i];
      // Get the positions and velocities of the two mass points connected by
      // the spring.
      int n0 = s.node0_, n1 = s.node1_;
      TV x0 = TV(x[3 * n0], x[3 * n0 + 1], x[3 * n0 + 2]);
      TV x1 = TV(x[3 * n1], x[3 * n1 + 1], x[3 * n1 + 2]);
      TV v0 = TV(v[3 * n0], v[3 * n0 + 1], v[3 * n0 + 2]);
      TV v1 = TV(v[3 * n1], v[3 * n1 + 1], v[3 * n1 + 2]);
      TV d = x1 - x0;
      TV n = d.normalized();
      // If the n is the unit vector point from x0 to x1,
      // the damping force = (damping coefficient * velocity difference)
      // projected in the direction of n.
      TV f = param.d() * (v1 - v0).dot(n) * n;
      for (int j = 0; j < 3; ++j) {
        (*damping_force)[3 * n0 + j] += f[j];
        (*damping_force)[3 * n1 + j] -= f[j];
      }
    }
  }

  // Calculates the change in velocity induced by the implicit damping force.
  template <class PositionVectorType, class VelocityVectorType,
            class ForceVectorType>
  void CalcDv(const MassSpringSystemParams<T>& param,
              const PositionVectorType& x, const ForceVectorType& f,
              VelocityVectorType* dv) const {
    T mass_per_point = param.mass() / static_cast<T>(num_points_);
    // Initialize the diagonal of the matrix to be mass_per_point.
    for (int i = 0; i < num_points_; ++i) {
      for (int j = 0; j < 3; ++j) {
        for (int k = 0; k < 3; ++k) {
          H_.coeffRef(3 * i + j, 3 * i + k) =
              (j == k) ? mass_per_point : static_cast<T>(0.0);
        }
      }
    }
    // Add in the contribution from damping force differential.
    using TV = Eigen::Matrix<T, 3, 1>;
    for (int i = 0; i < static_cast<int>(springs_.size()); ++i) {
      const Spring& s = springs_[i];
      // Get the positions and velocities of the two mass points connected by
      // the spring.
      int n0 = s.node0_, n1 = s.node1_;
      TV x0 = TV(x[3 * n0], x[3 * n0 + 1], x[3 * n0 + 2]);
      TV x1 = TV(x[3 * n1], x[3 * n1 + 1], x[3 * n1 + 2]);
      TV d = x1 - x0;
      TV n = d.normalized();
      // If the n is the unit vector point from x0 to x1,
      // the spring elastic force = elastic stiffness * (length - restlength) *
      // n, and damping force = (damping coefficient * velocity difference)
      // projected in the direction of n.
      Eigen::Matrix<T, 3, 3> damping_force_derivative =
          param.d() * dt_ * n * n.transpose();

      // Equivalent to the unsupported operation:
      // H_.block<3,3>(3*n0, 3*n0) += damping_force_derivative;
      // H_.block<3,3>(3*n1, 3*n1) += damping_force_derivative;
      // H_.block<3,3>(3*n0, 3*n1) = -damping_force_derivative;
      // H_.block<3,3>(3*n1, 3*n0) = -damping_force_derivative;
      for (int j = 0; j < 3; ++j) {
        for (int k = 0; k < 3; ++k) {
          // The diagonal blocks get contribution from multiple springs. We can
          // safely add to the diagonal blocks because they were reset earlier.
          H_.coeffRef(3 * n0 + j, 3 * n0 + k) += damping_force_derivative(j, k);
          H_.coeffRef(3 * n1 + j, 3 * n1 + k) += damping_force_derivative(j, k);
          // Do not add in the off-diagonal terms involving fixed nodes.
          if (n0 != left_corner_ && n0 != right_corner_ && n1 != left_corner_ &&
              n1 != right_corner_) {
            // Each off-diagonal block only gets contribution from a single
            // spring. It's important to overwrite the value from previous
            // time-step.
            H_.coeffRef(3 * n1 + j, 3 * n0 + k) =
                -damping_force_derivative(j, k);
            H_.coeffRef(3 * n0 + j, 3 * n1 + k) =
                -damping_force_derivative(j, k);
          }
        }
      }
    }
    cg_.compute(H_);
    // Solve H * dv = f * dt.
    (*dv) = cg_.solve(f * dt_);
  }

  /// Apply Dirichlet boundary conditions to the two corners of the rectangular
  /// grid.
  template <class VectorType>
  void ApplyDirichletBoundary(VectorType* state) const {
    for (int d = 0; d < 3; ++d) {
      (*state)[3 * left_corner_ + d] = T(0);
      (*state)[3 * right_corner_ + d] = T(0);
    }
  }
};
}  // namespace mass_spring_system
}  // namespace examples
}  // namespace drake
