#include "drake/multibody/contact_solvers/sap_solver.h"

#include <algorithm>
#include <vector>

#include "fmt/format.h"

#include "drake/multibody/contact_solvers/contact_solver_utils.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
ContactSolverStatus SapSolver<T>::SolveWithGuess(const T&,
                                                 const SystemDynamicsData<T>&,
                                                 const PointContactData<T>&,
                                                 const VectorX<T>&,
                                                 ContactSolverResults<T>*) {
  throw std::logic_error("This method must be implemented.");
  DRAKE_UNREACHABLE();
}

template <typename T>
void SapSolver<T>::CalcDelassusDiagonalApproximation(
    int nc, const std::vector<MatrixX<T>>& At, const BlockSparseMatrix<T>& J,
    VectorX<T>* delassus_diagonal) const {
  DRAKE_DEMAND(delassus_diagonal != nullptr);
  DRAKE_DEMAND(delassus_diagonal->size() == nc);
  const int nt = At.size();  // Number of trees.

  // We compute a factorization of A once so we can re-use it multiple times
  // below.
  std::vector<Eigen::LDLT<MatrixX<T>>> A_ldlt;
  A_ldlt.resize(nt);
  for (int t = 0; t < nt; ++t) {
    const auto& At_local = At[t];
    A_ldlt[t] = At_local.ldlt();
  }

  // We compute a diagonal approximation to the Delassus operator W. We
  // initialize it to zero and progressively add contributions in an O(n) pass.
  std::vector<Matrix3<T>> W(nc, Matrix3<T>::Zero());
  for (auto [p, t, Jpt] : J.get_blocks()) {
    // Verify assumption that this indeed is a contact Jacobian.
    DRAKE_DEMAND(J.row_start(p) % 3 == 0);
    DRAKE_DEMAND(Jpt.rows() % 3 == 0);
    // ic_start is the first contact point of patch p.
    const int ic_start = J.row_start(p) / 3;
    // k-th contact within patch p.
    for (int k = 0; k < Jpt.rows() / 3; ++k) {
      const int ic = ic_start + k;
      const auto& Jkt = Jpt.template middleRows<3>(3 * k);
      // This effectively computes Jₖₜ⋅A⁻¹⋅Jₖₜᵀ.
      W[ic] += Jkt * A_ldlt[t].solve(Jkt.transpose());
    }
  }

  // Compute delassus_diagonal as the rms norm of k-th diagonal block.
  for (int k = 0; k < nc; ++k) {
    (*delassus_diagonal)[k] = W[k].norm() / 3;
  }
}

template <typename T>
typename SapSolver<T>::PreProcessedData SapSolver<T>::PreProcessData(
    const T& time_step, const SystemDynamicsData<T>& dynamics_data,
    const PointContactData<T>& contact_data) const {
  using std::max;
  using std::min;
  using std::sqrt;

  PreProcessedData data(time_step, dynamics_data.num_velocities(),
                        contact_data.num_contacts());

  // Aliases to data.
  const VectorX<T>& mu = contact_data.get_mu();
  const VectorX<T>& phi0 = contact_data.get_phi0();
  const VectorX<T>& stiffness = contact_data.get_stiffness();
  const VectorX<T>& dissipation = contact_data.get_dissipation();

  // Aliases to mutable pre-processed data.
  VectorX<T>& R = data.R;
  VectorX<T>& vhat = data.vhat;
  VectorX<T>& inv_sqrt_A = data.inv_sqrt_A;

  // Store operators as block-sparse matrices.
  dynamics_data.get_A().AssembleMatrix(&data.A);
  contact_data.get_Jc().AssembleMatrix(&data.J);

  // Extract momentum matrix's per-tree diagonal blocks. Compute diagonal
  // scaling inv_sqrt_A.
  data.At.clear();
  data.At.reserve(data.A.num_blocks());
  for (const auto& block : data.A.get_blocks()) {
    const int t1 = std::get<0>(block);
    const int t2 = std::get<1>(block);
    const MatrixX<T>& Aij = std::get<2>(block);
    // We verify the assumption that M is block diagonal.
    DRAKE_DEMAND(t1 == t2);
    // Each block must be square.
    DRAKE_DEMAND(Aij.rows() == Aij.cols());

    const int nt = Aij.rows();  // Number of DOFs in the tree.
    data.At.push_back(Aij);

    const int start = data.A.row_start(t1);
    DRAKE_DEMAND(start == data.A.col_start(t2));
    inv_sqrt_A.segment(start, nt) = Aij.diagonal().cwiseInverse().cwiseSqrt();
  }

  // Computation of a diagonal approximation to the Delassus operator. N.B. This
  // must happen before the computation of the regularization R below.
  const int nc = phi0.size();
  CalcDelassusDiagonalApproximation(nc, data.At, data.J,
                                    &data.delassus_diagonal);

  // We use the Delassus scaling computed above to estimate regularization
  // parameters in the matrix R.
  const VectorX<T>& delassus_diagonal = data.delassus_diagonal;
  const double beta = parameters().beta;
  const double sigma = parameters().sigma;

  // Rigid approximation constant: Rₙ = β²/(4π²)⋅wᵢ when the contact frequency
  // ωₙ is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ = β⋅δt. See
  // [Castro et al., 2021] for details.
  const T beta_factor = beta * beta / (4.0 * M_PI * M_PI);
  for (int ic = 0; ic < nc; ++ic) {
    const int ic3 = 3 * ic;
    const T& k = stiffness(ic);
    const T& c = dissipation(ic);
    DRAKE_DEMAND(k > 0 && c >= 0);
    const T& wi = delassus_diagonal(ic);
    const T taud = c / k;  // Damping time scale.
    const T Rn =
        max(beta_factor * wi, 1.0 / (time_step * k * (time_step + taud)));
    const T Rt = sigma * wi;
    R.template segment<3>(ic3) = Vector3<T>(Rt, Rt, Rn);

    // Stabilization velocity.
    const T vn_hat = -phi0(ic) / (time_step + taud);
    vhat.template segment<3>(ic3) = Vector3<T>(0, 0, vn_hat);
  }

  data.Rinv = R.cwiseInverse();
  data.v_star = dynamics_data.get_v_star();
  data.mu = mu;
  data.A.Multiply(data.v_star, &data.p_star);

  return data;
}

template <typename T>
void SapSolver<T>::PackContactResults(const PreProcessedData& data,
                                      const VectorX<T>& v, const VectorX<T>& vc,
                                      const VectorX<T>& gamma,
                                      ContactSolverResults<T>* results) {
  DRAKE_DEMAND(results != nullptr);
  results->Resize(data.nv, data.nc);
  results->v_next = v;
  ExtractNormal(vc, &results->vn);
  ExtractTangent(vc, &results->vt);
  ExtractNormal(gamma, &results->fn);
  ExtractTangent(gamma, &results->ft);
  // N.B. While contact solver works with impulses, results are reported as
  // forces.
  results->fn /= data.time_step;
  results->ft /= data.time_step;
  data.J.MultiplyByTranspose(gamma, &results->tau_contact);
  results->tau_contact /= data.time_step;
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

template class ::drake::multibody::contact_solvers::internal::SapSolver<double>;
