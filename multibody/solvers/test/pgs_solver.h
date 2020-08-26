#pragma once

#include <memory>

#include <gtest/gtest.h>

#include "drake/multibody/solvers/contact_solver.h"
#include "drake/multibody/solvers/contact_solver_utils.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

namespace drake {
namespace multibody {
namespace solvers {
namespace test {

struct PgsSolverParameters {
  // Over-relaxation paramter, in (0, 1]
  double relaxation{0.6};
  // Absolute contact velocity tolerance, m/s.
  double abs_tolerance{1.0e-6};
  // Relative contact velocity tolerance, unitless.
  double rel_tolerance{1.0e-4};
  // Maximum number of PGS iterations.
  int max_iterations{100};
};

struct PgsSolverStats {
  int iterations{0};  // Number of PGS iterations.
  double vc_err{0.0};  // Error in the contact velocities, [m/s].
  double gamma_err{0.0};  // Error in the contact impulses, [Ns].
};

template <typename T>
class PgsSolver final : public ContactSolver<T> {
 public:
  class State {    
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(State);
    State() = default;
    void Resize(int nv, int nc) {
      v_.resize(nv);
      gamma_.resize(3 * nc);
    }
    const VectorX<T>& v() const { return v_; }
    VectorX<T>& mutable_v() { return v_; }
    const VectorX<T>& gamma() const { return gamma_; }
    VectorX<T>& mutable_gamma() { return gamma_; }

   private:
    VectorX<T> v_;
    VectorX<T> gamma_;
  };

  PgsSolver() = default;

  virtual ~PgsSolver() = default;

  void SetSystemDynamicsData(const SystemDynamicsData<T>* data) final;

  void SetPointContactData(const PointContactData<T>* data) final;

  int num_contacts() const final { return contact_data_->num_contacts(); };
  int num_velocities() const final { return dynamics_data_->num_velocities(); }

  void set_parameters(PgsSolverParameters& parameters) {
    parameters_ = parameters;
  }

  ContactSolverResult SolveWithGuess(double dt,
                                     const VectorX<T>& v_guess) final;

  const PgsSolverStats& get_iteration_stats() const { return stats_; }

  const VectorX<T>& GetImpulses() const final { return state_.gamma(); }
  const VectorX<T>& GetVelocities() const final { return state_.v(); }
  const VectorX<T>& GetGeneralizedContactImpulses() const final {
    return tau_c_;
  }
  const VectorX<T>& GetContactVelocities() const final { return vc_; }

 private:
  // All this data must remain const after the call to PreProcessData().
  struct PreProcessedData {
    Eigen::SparseMatrix<T> N;
    VectorX<T> vc_star;
    VectorX<T> v_star;
    // Norm of the 3x3 block diagonal block of matrix N, of size nc.
    VectorX<T> Nii_norm;
    // Approximation to the inverse of the diagonal of N, of size nc.
    VectorX<T> Dinv;
    void Resize(int nv, int nc) {
      N.resize(3 * nc, 3 * nc);
      vc_star.resize(3 * nc);
      v_star.resize(nv);
      Nii_norm.resize(nc);
      Dinv.resize(3 * nc);
    }
  };

  // Quick accessors to problem data.
  const LinearOperator<T>& get_Jc() const { return contact_data_->get_Jc(); }
  const LinearOperator<T>& get_Minv() const {
    return dynamics_data_->get_Minv();
  }
  const VectorX<T>& get_v0() const { return dynamics_data_->get_v0(); }
  const VectorX<T>& get_tau() const { return dynamics_data_->get_tau(); }
  const VectorX<T>& get_phi0() const { return contact_data_->get_phi0(); }
  const VectorX<T>& get_mu() const { return contact_data_->get_mu(); }

  void PreProcessData(double dt);
  bool VerifyConvergenceCriteria(const VectorX<T>& vc, const VectorX<T>& vc_kp,
                                 const VectorX<T>& gamma,
                                 const VectorX<T>& gamma_kp, double* vc_err,
                                 double* gamma_err) const;
  Vector3<T> ProjectImpulse(const Eigen::Ref<const Vector3<T>>& vc,
                            const Eigen::Ref<const Vector3<T>>& gamma,
                            const T& mu) const;
  void ProjectAllImpulses(const VectorX<T>& vc, VectorX<T>* gamma_inout) const;

  PgsSolverParameters parameters_;
  const SystemDynamicsData<T>* dynamics_data_{nullptr};
  const PointContactData<T>* contact_data_{nullptr};
  PreProcessedData pre_proc_data_;
  State state_;
  PgsSolverStats stats_;
  // Workspace (many of these could live in the state as "cached" data.)
  VectorX<T> tau_c_;  // Generalized contact impulses.
  VectorX<T> vc_;  // Contact velocities.  
};

}  // namespace test
}  // namespace solvers
}  // namespace multibody
}  // namespace drake

extern template class ::drake::multibody::solvers::test::PgsSolver<double>;
