#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/sorted_pair.h"
#include "drake/multibody/contact_solvers/block_sparse_matrix.h"
#include "drake/multibody/contact_solvers/contact_problem_graph.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
class SapConstraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapConstraint);

  // Default constructor allow us to place SapConstraint objects in STL
  // containers.
  SapConstraint() = default;

  virtual ~SapConstraint() = default;

  SapConstraint(int clique, const MatrixX<T>& J);

  SapConstraint(int clique0, int clique1, const MatrixX<T>& J0,
                const MatrixX<T>& J1);

  int num_cliques() const;

  int num_constrained_dofs() const;

  int clique0() const;
  int clique1() const;

  const MatrixX<T>& clique0_jacobian() const;
  const MatrixX<T>& clique1_jacobian() const;

  virtual VectorX<T> Project(const Eigen::Ref<const VectorX<T>>& y,
                             std::optional<MatrixX<T>> dPdy) const = 0;

  // VectorX<T> CalcConstraintBias(const T& wi);
  // VectorX<T> CalcRegularizationParameters(const T& wi);

 private:
  // For now we limit ourselves to constraints between two cliques only.
  int clique0_{-1};
  int clique1_{-1};
  MatrixX<T> J0_;
  MatrixX<T> J1_;
  int num_constrained_dofs_{0};
};

template <typename T>
class SapFrictionConeConstraint final : public SapConstraint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapFrictionConeConstraint);

  SapFrictionConeConstraint(int clique, const MatrixX<T>& J, const T& mu);

  SapFrictionConeConstraint(int clique0, int clique1, const MatrixX<T>& J0,
                            const MatrixX<T>& J1, const T& mu);

  VectorX<T> Project(const Eigen::Ref<const VectorX<T>>&,
                     std::optional<MatrixX<T>> dPdy) const final;

 private:
  T mu_{0.0};
};

// SAP Problem defined by:
//   - A⋅(v−v*) = Jᵀ⋅γ
//   - Constraints.
template <typename T>
class SapContactProblem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapContactProblem);

  SapContactProblem(std::vector<MatrixX<T>>&& A, VectorX<T>&& v_star);

  SapContactProblem(
      std::vector<MatrixX<T>>&& A, VectorX<T>&& v_star,
      std::vector<std::unique_ptr<SapConstraint<T>>>&& constraints);

  void AddConstraint(std::unique_ptr<SapConstraint<T>> c);

  int num_cliques() const;

  int num_constraints() const;

  int num_velocities() const;

  int num_velocities(int clique) const;

  const SapConstraint<T>& get_constraint(int k) const;

  // Returns the block diagonal dynamics matrix A.
  const std::vector<MatrixX<T>>& dynamics_matrix() const;

  ContactProblemGraph MakeGraph() const;

 private:
  int nv_{0};
  std::vector<MatrixX<T>> A_;
  VectorX<T> v_star_;
  std::vector<std::unique_ptr<SapConstraint<T>>> constraints_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
