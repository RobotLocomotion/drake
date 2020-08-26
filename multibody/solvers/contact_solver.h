#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/solvers/point_contact_data.h"
#include "drake/multibody/solvers/system_dynamics_data.h"
#include "drake/multibody/solvers/contact_solver_utils.h"

namespace drake {
namespace multibody {
namespace solvers {

/// The result from ContactSolver::SolveWithGuess() used to report the
/// success or failure of the solver.
enum class ContactSolverResult {
  /// Successful computation.
  kSuccess = 0,

  /// The solver could not find a solution at the specified tolerances.
  kFailure = 1,
};

template <typename T>
class ContactSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactSolver);

  ContactSolver() = default;

  virtual ~ContactSolver() = default;

  virtual void SetSystemDynamicsData(const SystemDynamicsData<T>* data) = 0;

  virtual void SetPointContactData(const PointContactData<T>* data) = 0;

  virtual int num_contacts() const = 0;
  virtual int num_velocities() const = 0;

  // TODO: return a more interesting generic success return code.
  // At this level either kFail or kSuccess, maybe kInvalidData.
  virtual ContactSolverResult SolveWithGuess(double dt,
                                             const VectorX<T>& v_guess) = 0;

  virtual const VectorX<T>& GetImpulses() const = 0;

  virtual const VectorX<T>& GetVelocities() const = 0;

  virtual const VectorX<T>& GetGeneralizedContactImpulses() const = 0;

  virtual const VectorX<T>& GetContactVelocities() const = 0;

  /// Returns a copy to the vector pi of normal impulses, of size
  /// num_contacts(), in Ns.
  void CopyNormalImpulses(VectorX<T>* pi) const {
    DRAKE_DEMAND(pi != nullptr);
    ExtractNormal(GetImpulses(), pi);
  }

  void CopyFrictionImpulses(VectorX<T>* beta) const {
    DRAKE_DEMAND(beta != nullptr);
    ExtractTangent(GetImpulses(), beta);
  }  

  void CopyNormalContactVelocities(VectorX<T>* vn) const {
    DRAKE_DEMAND(vn != nullptr);
    ExtractNormal(GetContactVelocities(), vn);
  }

  void CopyTangentialContactVelocities(VectorX<T>* vt) const {
    DRAKE_DEMAND(vt != nullptr);
    ExtractTangent(GetContactVelocities(), vt);
  }

 protected:
  // Helper method to form the Delassus operator. Most solvers will need to form
  // it whether if used directly, as part of a pre-processing stage or to just
  // determine scales.
  // 
  // Performs multiplication W = G * Mi * JT one column at a time.
  // G of size 3nc x nv
  // Mi of size nv x nv
  // JT of size nv x 3nc
  //
  // Result W of size 3nc x 3nc
  /// @pre An implementation to MultiplyByTranspose() is required on JT.
  void FormDelassusOperatorMatrix(const LinearOperator<T>& G,
                                  const LinearOperator<T>& Mi,
                                  const LinearOperator<T>& JT,
                                  Eigen::SparseMatrix<T>* W) const {
    DRAKE_DEMAND(G.rows() == 3 * num_contacts());
    DRAKE_DEMAND(G.cols() == num_velocities());
    DRAKE_DEMAND(Mi.rows() == num_velocities());
    DRAKE_DEMAND(Mi.cols() == num_velocities());
    DRAKE_DEMAND(JT.rows() == 3 * num_contacts());
    DRAKE_DEMAND(JT.cols() == num_velocities());
    DRAKE_DEMAND(W->rows() == 3 * num_contacts());
    DRAKE_DEMAND(W->cols() == 3 * num_contacts());

    const int nv = num_velocities();
    const int nc = num_contacts();

    Eigen::SparseVector<T> ej(3 * nc);
    // ei.makeCompressed();   // Not available for SparseVector.
    ej.coeffRef(0) = 1.0;  // Effectively allocate one non-zero entry.

    Eigen::SparseVector<T> JTcolj(nv);
    Eigen::SparseVector<T> MiJTcolj(nv);
    Eigen::SparseVector<T> Wcolj(3 * nc);
    // Reserve maximum number of non-zeros.
    JTcolj.reserve(nv);
    MiJTcolj.reserve(nv);
    Wcolj.reserve(3 * nc);

    // Loop over the j-th column.
    for (int j = 0; j < W->cols(); ++j) {
      // By changing the inner index, we change what entry is the non-zero with
      // value 1.0.
      *ej.innerIndexPtr() = j;

      // Reset to nnz = 0. Memory is not freed.
      JTcolj.setZero();
      MiJTcolj.setZero();
      Wcolj.setZero();

      // Apply each operator in sequence.
      JT.MultiplyByTranspose(ej, &JTcolj);  // JTcolj = JT * ej
      Mi.Multiply(JTcolj, &MiJTcolj);
      G.Multiply(MiJTcolj, &Wcolj);
      W->col(j) = Wcolj;
    }
  }
};
}  // namespace solvers
}  // namespace multibody
}  // namespace drake
