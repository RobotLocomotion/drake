#include "drake/solvers/moby_lcp_solver.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <Eigen/LU>
#include <Eigen/SparseCore>
#include <Eigen/SparseLU>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace solvers {

namespace {

template <typename Scalar>
bool CheckLemkeTrivial(int n, const Scalar& zero_tol, const VectorX<Scalar>& q,
                       VectorX<Scalar>* z) {
  // see whether trivial solution exists
  if (q.minCoeff() > -zero_tol) {
    z->resize(n);
    z->fill(0);
    return true;
  }

  return false;
}

// AutoDiff-supported linear system solver for performing principle pivoting
// transformations. The matrix is supposed to be a linear basis, but it's
// possible that the basis becomes degenerate (meaning that the matrix becomes
// singular) due to accumulated roundoff error from pivoting. Recovering from
// a degenerate basis is currently an open problem;
// see http://www.optimization-online.org/DB_FILE/2011/03/2948.pdf, for
// example. The caller would ideally terminate at this point, but
// compilation of householderQr().rank() with AutoDiff currently generates
// template errors. Continuing on blindly means that the calling pivoting
// algorithm might continue on for some time.
template <class T>
VectorX<T> LinearSolve(const MatrixX<T>& M, const VectorX<T>& b) {
  // Special case necessary because Eigen doesn't always handle empty matrices
  // properly.
  if (M.rows() == 0) {
    DRAKE_ASSERT(b.size() == 0);
    return VectorX<T>(0);
  }
  return M.householderQr().solve(b);
}

// Linear system solver, specialized for double types. This method is faster
// than the QR factorization necessary for AutoDiff support. It is assumed that
// the matrix is full rank (see notes for generic LinearSolve() above).
template <>
VectorX<double> LinearSolve(
    const MatrixX<double>& M, const VectorX<double>& b) {
  // Special case necessary because Eigen doesn't always handle empty matrices
  // properly.
  if (M.rows() == 0) {
    DRAKE_ASSERT(b.size() == 0);
    return VectorX<double>(0);
  }
  return M.partialPivLu().solve(b);
}

// Utility function for copying part of a matrix (designated by the indices
// in rows and cols) from in to a target matrix, out. This template approach
// allows selecting parts of both sparse and dense matrices for input; only
// a dense matrix is returned.
template <typename Derived, typename T>
void selectSubMat(const Eigen::MatrixBase<Derived>& in,
                  const std::vector<unsigned>& rows,
                  const std::vector<unsigned>& cols, MatrixX<T>* out) {
  const int num_rows = rows.size();
  const int num_cols = cols.size();
  out->resize(num_rows, num_cols);

  for (int i = 0; i < num_rows; i++) {
    const auto row_in = in.row(rows[i]);
    auto row_out = out->row(i);
    for (int j = 0; j < num_cols; j++) {
      row_out(j) = row_in(cols[j]);
    }
  }
}

// TODO(sammy-tri) this could also use a more efficient implementation.
template <typename T>
void selectSubVec(const VectorX<T>& in,
                  const std::vector<unsigned>& rows, VectorX<T>* out) {
  const int num_rows = rows.size();
  out->resize(num_rows);
  for (int i = 0; i < num_rows; i++) {
    (*out)(i) = in(rows[i]);
  }
}

template <typename Derived>
Eigen::SparseVector<double> makeSparseVector(
    const Eigen::MatrixBase<Derived>& in) {
  DRAKE_ASSERT(in.cols() == 1);
  Eigen::SparseVector<double> out(in.rows());
  for (int i = 0; i < in.rows(); i++) {
    if (in(i) != 0.0) {
      out.coeffRef(i) = in(i);
    }
  }
  return out;
}

template <typename Derived>
Eigen::Index minCoeffIdx(const Eigen::MatrixBase<Derived>& in) {
  Eigen::Index idx;
  in.minCoeff(&idx);
  return idx;
}

const double kSqrtEps = std::sqrt(std::numeric_limits<double>::epsilon());
}  // anonymous namespace

template <typename T>
void MobyLCPSolver<T>::SetLoggingEnabled(bool enabled) {
  log_enabled_ = enabled; }

template <typename T>
std::ostream& MobyLCPSolver<T>::Log() const {
  if (log_enabled_) {
    return std::cerr;
  }
  return null_stream_;
}

template <typename T>
void MobyLCPSolver<T>::ClearIndexVectors() const {
  // clear all vectors
  all_.clear();
  tlist_.clear();
  bas_.clear();
  nonbas_.clear();
  j_.clear();
}

template <>
void MobyLCPSolver<Eigen::AutoDiffScalar<Vector1d>>::DoSolve(
    const MathematicalProgram&, const Eigen::VectorXd&,
    const SolverOptions&, MathematicalProgramResult*) const {
  throw std::logic_error(
      "MobyLCPSolver cannot yet be used in a MathematicalProgram "
      "while templatized as an AutoDiff");
}

// TODO(edrumwri): Break the following code out into a special
// MobyLcpMathematicalProgram class.
template <typename T>
void MobyLCPSolver<T>::DoSolve(
    const MathematicalProgram& prog,
    const Eigen::VectorXd& initial_guess,
    const SolverOptions& merged_options,
    MathematicalProgramResult* result) const {
  if (!prog.GetVariableScaling().empty()) {
    static const logging::Warn log_once(
      "MobyLCPSolver doesn't support the feature of variable scaling.");
  }

  // Moby doesn't use initial guess or the solver options.
  unused(initial_guess);
  unused(merged_options);

  // Solve each individual LCP, writing the result back to the decision
  // variables through the binding and returning true iff all LCPs are
  // feasible.
  //
  // If any is infeasible, returns false and does not alter the decision
  // variables.
  //
  // TODO(ggould-tri) This could also be solved by constructing a single large
  // square matrix and vector, and then copying the elements of the individual
  // Ms and qs into the appropriate places.  That would be equivalent to this
  // implementation but might perform better if the solver were to parallelize
  // internally.

  const auto& bindings = prog.linear_complementarity_constraints();
  Eigen::VectorXd x_sol(prog.num_vars());
  for (const auto& binding : bindings) {
    Eigen::VectorXd constraint_solution(binding.GetNumElements());
    const std::shared_ptr<LinearComplementarityConstraint> constraint =
        binding.evaluator();
    bool solved = SolveLcpLemkeRegularized(
        constraint->M(), constraint->q(), &constraint_solution);
    if (!solved) {
      result->set_solution_result(SolutionResult::kUnknownError);
      return;
    }
    for (int i = 0; i < binding.evaluator()->num_vars(); ++i) {
      const int variable_index =
          prog.FindDecisionVariableIndex(binding.variables()(i));
      x_sol(variable_index) = constraint_solution(i);
    }
  }
  result->set_optimal_cost(0.0);
  result->set_x_val(x_sol);
  result->set_solution_result(SolutionResult::kSolutionFound);
}

template <typename T>
bool MobyLCPSolver<T>::SolveLcpFast(const MatrixX<T>& M,
                                    const VectorX<T>& q, VectorX<T>* z,
                                    const T& zero_tol) const {
  using std::abs;

  // Variables that will be reused multiple times, thus hopefully allowing
  // Eigen to keep from freeing/reallocating memory repeatedly.
  VectorX<T> zz, w, qbas;
  MatrixX<T> Mmix, Msub;

  const unsigned N = q.rows();
  const unsigned UINF = std::numeric_limits<unsigned>::max();

  if (M.rows() != N || M.cols() != N)
    throw std::logic_error("M's dimensions do not match that of q.");

  Log() << "MobyLCPSolver::SolveLcpFast() entered" << std::endl;

  // look for trivial solution
  if (N == 0) {
    Log() << "MobyLCPSolver::SolveLcpFast() - empty problem" << std::endl;
    z->resize(0);
    return true;
  }

  // set zero tolerance if necessary
  T mod_zero_tol = zero_tol;
  if (mod_zero_tol < 0)
    mod_zero_tol = ComputeZeroTolerance(M);

  // prepare to setup basic and nonbasic variable indices for z
  nonbas_.clear();
  bas_.clear();

  // see whether to warm-start
  if (z->size() == q.size()) {
    Log() << "MobyLCPSolver::SolveLcpFast() - warm starting activated"
          << std::endl;

    for (unsigned i = 0; i < z->size(); i++) {
      if (abs((*z)[i]) < mod_zero_tol) {
        bas_.push_back(i);
      } else {
        nonbas_.push_back(i);
      }
    }

    if (log_enabled_) {
      std::ostringstream str;
      str << " -- non-basic indices:";
      for (unsigned i = 0; i < nonbas_.size(); i++) str << " " << nonbas_[i];
      Log() << str.str() << std::endl;
    }
  } else {
    // get minimum element of q (really w)
    Eigen::Index minw;
    const T minw_val = q.minCoeff(&minw);
    if (minw_val > -mod_zero_tol) {
      Log() << "MobyLCPSolver::SolveLcpFast() - trivial solution found"
            << std::endl;
      z->resize(N);
      z->fill(0);
      return true;
    }

    // setup basic and nonbasic variable indices
    nonbas_.push_back(minw);
    bas_.resize(N - 1);
    for (unsigned i = 0, j = 0; i < N; i++) {
      if (i != minw) {
        bas_[j++] = i;
      }
    }
  }

  // Loop for the maximum number of pivots.
  const unsigned MAX_PIV = 2 * N;
  for (pivots_ = 1; pivots_ <= MAX_PIV; pivots_++) {
    // select nonbasic indices
    selectSubMat(M, nonbas_, nonbas_, &Msub);
    selectSubMat(M, bas_, nonbas_, &Mmix);
    selectSubVec(q, nonbas_, &zz);
    selectSubVec(q, bas_, &qbas);
    zz *= -1;

    // Solve for nonbasic z.
    zz = LinearSolve(Msub, zz.eval());

    // Eigen doesn't handle empty matrices properly, which causes the code
    // below to abort in the absence of the conditional.
    unsigned minw;
    if (Mmix.rows() == 0) {
      w = VectorX<T>();
      minw = UINF;
    } else {
      w = Mmix * zz;
      w += qbas;
      minw = minCoeffIdx(w);
    }

    // TODO(sammy-tri) this log can't print when minw is UINF.
    // LOG() << "MobyLCPSolver::SolveLcpFast() - minimum w after pivot: "
    // << _w[minw] << std::endl;

    // if w >= 0, check whether any component of z < 0
    if (minw == UINF || w[minw] > -mod_zero_tol) {
      // find the (a) minimum of z
      unsigned minz = (zz.rows() > 0) ? minCoeffIdx(zz) : UINF;
      if (log_enabled_ && zz.rows() > 0) {
        Log() << "MobyLCPSolver::SolveLcpFast() - minimum z after pivot: "
              << zz[minz] << std::endl;
      }
      if (minz < UINF && zz[minz] < -mod_zero_tol) {
        // get the original index and remove it from the nonbasic set
        unsigned idx = nonbas_[minz];
        nonbas_.erase(nonbas_.begin() + minz);

        // move index to basic set and continue looping
        bas_.push_back(idx);
        std::sort(bas_.begin(), bas_.end());
      } else {
        // found the solution
        z->resize(N);
        z->fill(0);

        // set values of z corresponding to _z
        for (unsigned i = 0, j = 0; j < nonbas_.size(); i++, j++) {
          (*z)[nonbas_[j]] = zz[i];
        }

        Log() << "MobyLCPSolver::SolveLcpFast() - solution found!" << std::endl;
        return true;
      }
    } else {
      Log() << "(minimum w too negative)" << std::endl;

      // one or more components of w violating w >= 0
      // move component of w from basic set to nonbasic set
      unsigned idx = bas_[minw];
      bas_.erase(bas_.begin() + minw);
      nonbas_.push_back(idx);
      std::sort(nonbas_.begin(), nonbas_.end());

      // look whether any component of z needs to move to basic set
      unsigned minz = (zz.rows() > 0) ? minCoeffIdx(zz) : UINF;
      if (log_enabled_ && zz.rows() > 0) {
        Log() << "MobyLCPSolver::SolveLcpFast() - minimum z after pivot: "
              << zz[minz] << std::endl;
      }
      if (minz < UINF && zz[minz] < -mod_zero_tol) {
        // move index to basic set and continue looping
        unsigned k = nonbas_[minz];
        Log() << "MobyLCPSolver::SolveLcpFast() - moving index " << k
              << " to basic set" << std::endl;

        nonbas_.erase(nonbas_.begin() + minz);
        bas_.push_back(k);
        std::sort(bas_.begin(), bas_.end());
      }
    }
  }

  Log() << "MobyLCPSolver::SolveLcpFast() - maximum allowable pivots exceeded"
        << std::endl;

  // if we're here, then the maximum number of pivots has been exceeded
  z->setZero(N);
  return false;
}

template <typename T>
bool MobyLCPSolver<T>::SolveLcpFastRegularized(const MatrixX<T>& M,
                                               const VectorX<T>& q,
                                               VectorX<T>* z, int min_exp,
                                               unsigned step_exp, int max_exp,
                                               const T& zero_tol) const {
  Log() << "MobyLCPSolver::SolveLcpFastRegularized() entered" << std::endl;

  // Variables that will be reused multiple times, thus hopefully allowing
  // Eigen to keep from freeing/reallocating memory repeatedly.
  VectorX<T> wx;
  MatrixX<T> MM;

  // look for fast exit
  if (q.size() == 0) {
    z->resize(0);
    return true;
  }

  // copy MM
  MM = M;

  // A discourse on the zero tolerance in the context of regularization:
  // The zero tolerance is used to determine when an element of w or z is
  // effectively zero though its floating point value is negative. The question
  // is whether the regularization process will change the zero tolerance
  // necessary to solve the problem numerically. In such a case, the infinity
  // norm of M would be small while the infinity norm of MM (regularized M)
  // would be large. Consider the case of a symmetric, indefinite matrix with
  // maximum and minimum eigenvalues of a and -a, respectively. The matrix could
  // be made positive definite (and thereby guaranteed to possess a solution to
  // the linear complementarity problem) by adding an identity matrix times
  // (a+ε) to the LCP matrix, where ε > 0 (its magnitude will depend upon the
  // magnitude of a). The infinity norm (and hence the zero tolerance) could
  // then be expected to grow by a factor of approximately two during the
  // regularization process. In other words, recomputing the zero tolerance
  // for each regularization update to the LCP matrix appears wasteful. For
  // this reason, we compute it only once below, but a practical effect is
  // not discernible at this time.

  // Assign value for zero tolerance, if necessary.
  const T mod_zero_tol = (zero_tol > 0) ? zero_tol : ComputeZeroTolerance(M);

  Log() << " zero tolerance: " << mod_zero_tol << std::endl;

  // store the total pivots
  unsigned total_piv = 0;

  // try non-regularized version first
  bool result = SolveLcpFast(MM, q, z, mod_zero_tol);
  if (result) {
    // verify that solution truly is a solution -- check z
    if (z->minCoeff() >= -mod_zero_tol) {
      // check w
      wx = (M * (*z)) + q;
      if (wx.minCoeff() >= -mod_zero_tol) {
        // Check element-wise operation of z*wx.
        wx = z->array() * wx.eval().array();
        const T wx_min = wx.minCoeff();
        const T wx_max = wx.maxCoeff();

        if (wx_min >= -mod_zero_tol && wx_max < mod_zero_tol) {
          Log() << "  solved with no regularization necessary!" << std::endl;
          Log() << "  pivots / total pivots: " << pivots_ << " " << pivots_
                << std::endl;
          Log() << "MobyLCPSolver::SolveLcpFastRegularized() exited"
                << std::endl;

          return true;
        } else {
          Log() << "MobyLCPSolver::SolveLcpFastRegularized() - "
                << "'<w, z> not within tolerance(min value: " << wx_min
                << " max value: " << wx_max << ")" << std::endl;
        }
      } else {
        Log() << "  MobyLCPSolver::SolveLcpFastRegularized() - "
              << "'w' not solved to desired tolerance" << std::endl;
        Log() << "  minimum w: " << wx.minCoeff() << std::endl;
      }
    } else {
      Log() << "  MobyLCPSolver::SolveLcpFastRegularized() - "
            << "'z' not solved to desired tolerance" << std::endl;
      Log() << "  minimum z: " << z->minCoeff() << std::endl;
    }
  } else {
    Log() << "  MobyLCPSolver::SolveLcpFastRegularized() "
          << "- solver failed with zero regularization" << std::endl;
  }

  // update the pivots
  total_piv += pivots_;

  // start the regularization process
  int rf = min_exp;
  while (rf < max_exp) {
    // setup regularization factor
    double lambda =
        std::pow(static_cast<double>(10.0), static_cast<double>(rf));

    Log() << "  trying to solve LCP with regularization factor: " << lambda
          << std::endl;

    // regularize M
    MM = M;
    for (unsigned i = 0; i < M.rows(); i++) {
      MM(i, i) += lambda;
    }

    // try to solve the LCP
    result = SolveLcpFast(MM, q, z, mod_zero_tol);

    // update total pivots
    total_piv += pivots_;

    if (result) {
      // verify that solution truly is a solution -- check z
      if (z->minCoeff() > -mod_zero_tol) {
        // check w
        wx = (MM * (*z)) + q;
        if (wx.minCoeff() > -mod_zero_tol) {
          // Check element-wise operation of z*wx.
          wx = z->array() * wx.eval().array();
          const T wx_min = wx.minCoeff();
          const T wx_max = wx.maxCoeff();

          if (wx_min > -mod_zero_tol && wx_max < mod_zero_tol) {
            Log() << "  solved with regularization factor: " << lambda
                  << std::endl;
            Log() << "  pivots / total pivots: " << pivots_ << " " << total_piv
                  << std::endl;
            Log() << "MobyLCPSolver::SolveLcpFastRegularized() exited"
                  << std::endl;
            pivots_ = total_piv;
            return true;
          } else {
            Log() << "MobyLCPSolver::SolveLcpFastRegularized() - "
                  << "'<w, z> not within tolerance(min value: " << wx_min
                  << " max value: " << wx_max << ")" << std::endl;
          }
        } else {
          Log() << "  MobyLCPSolver::SolveLcpFastRegularized() - "
                << "'w' not solved to desired tolerance" << std::endl;
          Log() << "  minimum w: " << wx.minCoeff() << std::endl;
        }
      } else {
        Log() << "  MobyLCPSolver::SolveLcpFastRegularized() - "
              << "'z' not solved to desired tolerance" << std::endl;
        Log() << "  minimum z: " << z->minCoeff() << std::endl;
      }
    }

    // increase rf
    rf += step_exp;
  }

  Log() << "  unable to solve given any regularization!" << std::endl;
  Log() << "MobyLCPSolver::SolveLcpFastRegularized() exited" << std::endl;

  // store total pivots
  pivots_ = total_piv;

  // still here?  failure...
  return false;
}

// Retrieves the solution computed by Lemke's Algorithm.
// T is irrelevant for this method (necessary only for the member function).
// MatrixType allows both dense and sparse matrices to be used.
// Scalar allows this method to be used for when the T is AutoDiffXd but
// the caller wants to use sparse methods.
// TODO(edrumwri): Address this kludge when calling sparse LCP solves from
//                 MobyLCPSolver<AutoDiffXd> has been prevented.
template <typename T>
template <typename MatrixType, typename Scalar>
void MobyLCPSolver<T>::FinishLemkeSolution(const MatrixType& M,
                                           const VectorX<Scalar>& q,
                                           const VectorX<Scalar>& x,
                                           VectorX<Scalar>* z) const {
  using std::abs;
  using std::max;
  std::vector<unsigned>::iterator iiter;
  int idx;
  for (idx = 0, iiter = bas_.begin(); iiter != bas_.end(); iiter++, idx++) {
    (*z)(*iiter) = x(idx);
  }

  // TODO(sammy-tri) Is there a more efficient way to resize and
  // preserve the data?
  z->conservativeResize(q.size());

  // check to see whether tolerances are satisfied
  if (log_enabled_) {
    VectorX<T> wl = (M * (*z)) + q;
    const T minw = wl.minCoeff();
    const T w_dot_z = abs(wl.dot(*z));
    Log() << "  z: " << z << std::endl;
    Log() << "  w: " << wl << std::endl;
    Log() << "  minimum w: " << minw << std::endl;
    Log() << "  w'z: " << w_dot_z << std::endl;
  }
}

template <typename T>
bool MobyLCPSolver<T>::SolveLcpLemke(const MatrixX<T>& M,
                                     const VectorX<T>& q, VectorX<T>* z,
                                     const T& piv_tol,
                                     const T& zero_tol) const {
  using std::max;

  // Variables that will be reused multiple times, thus hopefully allowing
  // Eigen to keep from freeing/reallocating memory repeatedly.
  VectorX<T> result, dj, dl, x, xj, Be, u, z0;
  MatrixX<T> Bl, t1, t2;

  if (log_enabled_) {
    Log() << "MobyLCPSolver::SolveLcpLemke() entered" << std::endl;
    Log() << "  M: " << std::endl << M;
    Log() << "  q: " << q << std::endl;
  }

  const unsigned n = q.size();
  const unsigned max_iter = std::min(unsigned{1000}, 50 * n);

  if (M.rows() != n || M.cols() != n)
    throw std::logic_error("M's dimensions do not match that of q.");

  // update the pivots
  pivots_ = 0;

  // look for immediate exit
  if (n == 0) {
    z->resize(0);
    return true;
  }

  // come up with a sensible value for zero tolerance if none is given
  T mod_zero_tol = zero_tol;
  if (mod_zero_tol <= 0)
    mod_zero_tol = ComputeZeroTolerance(M);

  if (CheckLemkeTrivial(n, mod_zero_tol, q, z)) {
    Log() << " -- trivial solution found" << std::endl;
    Log() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
    return true;
  }

  // Lemke's algorithm doesn't seem to like warmstarting
  //
  // TODO(sammy-tri) this is not present in the sparse solver, and it
  // causes subtle dead code below.
  z->fill(0);

  // copy z to z0
  z0 = *z;

  ClearIndexVectors();

  // initialize variables
  z->resize(n * 2);
  z->fill(0);
  unsigned t = 2 * n;
  unsigned entering = t;
  unsigned leaving = 0;
  for (unsigned i = 0; i < n; i++) {
    all_.push_back(i);
  }
  unsigned lvindex;
  unsigned idx;
  std::vector<unsigned>::iterator iiter;

  // determine initial basis
  if (z0.size() != n) {
    // setup the nonbasic indices
    for (unsigned i = 0; i < n; i++) nonbas_.push_back(i);
  } else {
    for (unsigned i = 0; i < n; i++) {
      if (z0[i] > 0) {
        bas_.push_back(i);
      } else {
        nonbas_.push_back(i);
      }
    }
  }

  // determine initial values
  if (!bas_.empty()) {
    Log() << "-- initial basis not empty (warmstarting)" << std::endl;

    // start from good initial basis
    Bl.resize(n, n);
    Bl.setIdentity();
    Bl *= -1;

    // select columns of M corresponding to z vars in the basis
    selectSubMat(M, all_, bas_, &t1);

    // select columns of I corresponding to z vars not in the basis
    selectSubMat(Bl, all_, nonbas_, &t2);

    // setup the basis matrix
    Bl.resize(n, t1.cols() + t2.cols());
    Bl.block(0, 0, t1.rows(), t1.cols()) = t1;
    Bl.block(0, t1.cols(), t2.rows(), t2.cols()) = t2;

    // Solve B*x = -q.
    x = LinearSolve(Bl, q);
  } else {
    Log() << "-- using basis of -1 (no warmstarting)" << std::endl;

    // use standard initial basis
    Bl.resize(n, n);
    Bl.setIdentity();
    Bl *= -1;
    x = q;
  }

  // check whether initial basis provides a solution
  if (x.minCoeff() >= 0.0) {
    Log() << " -- initial basis provides a solution!" << std::endl;
    FinishLemkeSolution(M, q, x, z);
    Log() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
    return true;
  }

  // use a new pivot tolerance if necessary
  const T naive_piv_tol = n * max(T(1), M.template lpNorm<Eigen::Infinity>()) *
      std::numeric_limits<double>::epsilon();
  const T mod_piv_tol = (piv_tol > 0) ? piv_tol : naive_piv_tol;

  // determine initial leaving variable
  Eigen::Index min_x;
  const T min_x_val = x.topRows(n).minCoeff(&min_x);
  const T tval = -min_x_val;
  for (size_t i = 0; i < nonbas_.size(); i++) {
    bas_.push_back(nonbas_[i] + n);
  }
  lvindex = min_x;
  iiter = bas_.begin();
  std::advance(iiter, lvindex);
  leaving = *iiter;
  Log() << " -- x: " << x << std::endl;
  Log() << " -- first pivot: leaving index=" << lvindex
        << "  entering index=" << entering << " minimum value: " << tval
        << std::endl;

  // pivot in the artificial variable
  *iiter = t;  // replace w var with _z0 in basic indices
  u.resize(n);
  for (unsigned i = 0; i < n; i++) {
    u[i] = (x[i] < 0) ? 1 : 0;
  }
  Be = (Bl * u) * -1;
  u *= tval;
  x += u;
  x[lvindex] = tval;
  Bl.col(lvindex) = Be;
  Log() << "  new q: " << x << std::endl;

  // main iterations begin here
  for (pivots_ = 0; pivots_ < max_iter; pivots_++) {
    if (log_enabled_) {
      std::ostringstream basic;
      for (unsigned i = 0; i < bas_.size(); i++) {
        basic << " " << bas_[i];
      }
      Log() << "basic variables:" << basic.str() << std::endl;
      Log() << "leaving: " << leaving << " t:" << t << std::endl;
    }

    // check whether done; if not, get new entering variable
    if (leaving == t) {
      Log() << "-- solved LCP successfully!" << std::endl;
      FinishLemkeSolution(M, q, x, z);
      Log() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
      return true;
    } else if (leaving < n) {
      entering = n + leaving;
      Be.resize(n);
      Be.fill(0);
      Be[leaving] = -1;
    } else {
      entering = leaving - n;
      Be = M.col(entering);
    }
    dl = Be;

    // See comments above on the possibility of this solve failing.
    dl = LinearSolve(Bl, dl.eval());

    // ** find new leaving variable
    j_.clear();
    for (unsigned i = 0; i < dl.size(); i++) {
      if (dl[i] > mod_piv_tol) {
        j_.push_back(i);
      }
    }

    // check for no new pivots; ray termination
    if (j_.empty()) {
      Log()
          << "MobyLCPSolver::SolveLcpLemke() - no new pivots (ray termination)"
          << std::endl;
      Log() << "MobyLCPSolver::SolveLcpLemke() exiting" << std::endl;
      z->setZero(n);
      return false;
    }

    if (log_enabled_) {
      std::ostringstream j;
      for (unsigned i = 0; i < j_.size(); i++) j << " " << j_[i];
      Log() << "d: " << dl << std::endl;
      Log() << "j (before min ratio):" << j.str() << std::endl;
    }

    // select elements j from x and d
    selectSubVec(x, j_, &xj);
    selectSubVec(dl, j_, &dj);

    // compute minimal ratios x(j) + EPS_DOUBLE ./ d(j), d > 0
    result.resize(xj.size());
    result.fill(mod_zero_tol);
    result = xj.eval().array() + result.array();
    result = result.eval().array() / dj.array();
    const T theta = result.minCoeff();

    // NOTE: lexicographic ordering is not used here to prevent
    // cycling (see [Cottle 1992], pp. 340-342). Cycling is indirectly prevented
    // by (a) limiting the maximum number of pivots and (b) using
    // regularized solvers, as necessary. In other words, cycling may cause
    // solver to fail when the LCP is theoretically solvable, but wrapping the
    // solver with regularization practically addresses the problem (albeit,
    // at the cost of additional computation).

    // find indices of minimal ratios, d> 0
    //   divide _x(j) ./ d(j) -- remove elements above the minimum ratio
    for (int i = 0; i < result.size(); i++) {
      result(i) = xj(i) / dj(i);
    }

    for (iiter = j_.begin(), idx = 0; iiter != j_.end();) {
      if (result[idx++] <= theta) {
        iiter++;
      } else {
        iiter = j_.erase(iiter);
      }
    }
    if (log_enabled_) {
      std::ostringstream j;
      for (unsigned i = 0; i < j_.size(); i++) {
        j << " " << j_[i];
      }
      Log() << "j (after min ratio):" << j.str() << std::endl;
    }

    // if j is empty, then likely the zero tolerance is too low
    if (j_.empty()) {
      Log() << "zero tolerance too low?" << std::endl;
      Log() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
      z->setZero(n);
      return false;
    }

    // check whether artificial index among these
    tlist_.clear();
    for (size_t i = 0; i < j_.size(); i++) {
      tlist_.push_back(bas_[j_[i]]);
    }
    if (std::find(tlist_.begin(), tlist_.end(), t) != tlist_.end()) {
      iiter = std::find(bas_.begin(), bas_.end(), t);
      lvindex = iiter - bas_.begin();
    } else {
      // several indices pass the minimum ratio test, pick one randomly
      //      lvindex = _j[rand() % _j.size()];
      // NOTE: solver seems *much* more capable of solving when we pick the
      // first
      // element rather than picking a random one
      lvindex = j_[0];
    }

    // set leaving = bas(lvindex)
    iiter = bas_.begin();
    std::advance(iiter, lvindex);
    leaving = *iiter;

    // ** perform pivot
    const T ratio = x[lvindex] / dl[lvindex];
    dl *= ratio;
    x -= dl;
    x[lvindex] = ratio;
    Bl.col(lvindex) = Be;
    *iiter = entering;
    Log() << " -- pivoting: leaving index=" << lvindex
          << "  entering index=" << entering << std::endl;
  }

  Log() << " -- maximum number of iterations exceeded (n=" << n
        << ", max=" << max_iter << ")" << std::endl;
  Log() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
  z->setZero(n);
  return false;
}

template <class T>
bool MobyLCPSolver<T>::SolveLcpLemkeRegularized(const MatrixX<T>& M,
                                                const VectorX<T>& q,
                                                VectorX<T>* z, int min_exp,
                                                unsigned step_exp, int max_exp,
                                                const T& piv_tol,
                                                const T& zero_tol) const {
  // Variables that will be reused multiple times, thus hopefully allowing
  // Eigen to keep from freeing/reallocating memory repeatedly.
  VectorX<T> wx;

  Log() << "MobyLCPSolver::SolveLcpLemkeRegularized() entered" << std::endl;

  // look for fast exit
  if (q.size() == 0) {
    z->resize(0);
    return true;
  }

  // copy MM
  MatrixX<T> MM = M;

  // Assign value for zero tolerance, if necessary. See discussion in
  // SolveLcpFastRegularized() to see why this tolerance is computed here once,
  // rather than for each regularized version of M.
  const T mod_zero_tol = (zero_tol > 0) ? zero_tol : ComputeZeroTolerance(M);

  Log() << " zero tolerance: " << mod_zero_tol << std::endl;

  // store the total pivots
  unsigned total_piv = 0;

  // try non-regularized version first
  bool result = SolveLcpLemke(MM, q, z, piv_tol, mod_zero_tol);
  if (result) {
    // verify that solution truly is a solution -- check z
    if (z->minCoeff() >= -mod_zero_tol) {
      // check w
      wx = (M * (*z)) + q;
      if (wx.minCoeff() >= -mod_zero_tol) {
        // Check element-wise operation of z*wx.
        wx = z->array() * wx.eval().array();

        const T wx_min = wx.minCoeff();
        const T wx_max = wx.maxCoeff();
        if (wx_min >= -mod_zero_tol && wx_max < mod_zero_tol) {
          Log() << "  solved with no regularization necessary!" << std::endl;
          Log() << "MobyLCPSolver::SolveLcpLemkeRegularized() exited"
                << std::endl;

          return true;
        } else {
          Log() << "MobyLCPSolver::SolveLcpLemke() - "
                << "'<w, z> not within tolerance(min value: " << wx_min
                << " max value: " << wx_max << ")" << std::endl;
        }
      } else {
        Log() << "  MobyLCPSolver::SolveLcpLemke() - 'w' not solved to desired "
            "tolerance"
              << std::endl;
        Log() << "  minimum w: " << wx.minCoeff() << std::endl;
      }
    } else {
      Log() << "  MobyLCPSolver::SolveLcpLemke() - 'z' not solved to desired "
          "tolerance"
            << std::endl;
      Log() << "  minimum z: " << z->minCoeff() << std::endl;
    }
  }

  // update the pivots
  total_piv += pivots_;

  // start the regularization process
  int rf = min_exp;
  while (rf < max_exp) {
    // setup regularization factor
    double lambda =
        std::pow(static_cast<double>(10.0), static_cast<double>(rf));

    Log() << "  trying to solve LCP with regularization factor: " << lambda
          << std::endl;

    // regularize M
    MM = M;
    for (unsigned i = 0; i < M.rows(); i++) {
      MM(i, i) += lambda;
    }

    // try to solve the LCP
    result = SolveLcpLemke(MM, q, z, piv_tol, mod_zero_tol);

    // update total pivots
    total_piv += pivots_;

    if (result) {
      // verify that solution truly is a solution -- check z
      if (z->minCoeff() > -mod_zero_tol) {
        // check w
        wx = (MM * (*z)) + q;
        if (wx.minCoeff() > -mod_zero_tol) {
          // Check element-wise operation of z*wx.
          wx = z->array() * wx.eval().array();

          const T wx_min = wx.minCoeff();
          const T wx_max = wx.maxCoeff();
          if (wx_min > -mod_zero_tol && wx_max < mod_zero_tol) {
            Log() << "  solved with regularization factor: " << lambda
                  << std::endl;
            Log() << "MobyLCPSolver::SolveLcpLemkeRegularized() exited"
                  << std::endl;
            pivots_ = total_piv;
            return true;
          } else {
            Log() << "MobyLCPSolver::SolveLcpLemke() - "
                  << "'<w, z> not within tolerance(min value: " << wx_min
                  << " max value: " << wx_max << ")" << std::endl;
          }
        } else {
          Log() << "  MobyLCPSolver::SolveLcpLemke() - 'w' not solved to "
              "desired tolerance"
                << std::endl;
          Log() << "  minimum w: " << wx.minCoeff() << std::endl;
        }
      } else {
        Log() << "  MobyLCPSolver::SolveLcpLemke() - 'z' not solved to desired "
            "tolerance"
              << std::endl;
        Log() << "  minimum z: " << z->minCoeff() << std::endl;
      }
    }

    // increase rf
    rf += step_exp;
  }

  Log() << "  unable to solve given any regularization!" << std::endl;
  Log() << "MobyLCPSolver::SolveLcpLemkeRegularized() exited" << std::endl;

  // store total pivots
  pivots_ = total_piv;

  // still here?  failure...
  return false;
}

template <typename T>
MobyLCPSolver<T>::MobyLCPSolver()
    : SolverBase(&id, &is_available, &is_enabled,
                 &ProgramAttributesSatisfied) {}

template <typename T>
MobyLCPSolver<T>::~MobyLCPSolver() = default;

SolverId MobyLcpSolverId::id() {
  static const never_destroyed<SolverId> singleton{"Moby LCP"};
  return singleton.access();
}

template <typename T>
SolverId MobyLCPSolver<T>::id() {
  return MobyLcpSolverId::id();
}

template <typename T>
bool MobyLCPSolver<T>::is_available() {
  return true;
}

template <typename T>
bool MobyLCPSolver<T>::is_enabled() {
  return true;
}

template <typename T>
bool MobyLCPSolver<T>::ProgramAttributesSatisfied(
    const MathematicalProgram& prog) {
  // This solver currently imposes restrictions that its problem:
  //
  // (1) Contains only linear complementarity constraints,
  // (2) Has no element of any decision variable appear in more than one
  //     constraint, and
  // (3) Has every element of every decision variable in a constraint.
  //
  // Restriction 1 could reasonably be relaxed by reformulating other
  // constraint types that can be expressed as LCPs (eg, convex QLPs),
  // although this would also entail adding an output stage to convert
  // the LCP results back to the desired form.  See eg. @RussTedrake on
  // how to convert a linear equality constraint of n elements to an
  // LCP of 2n elements.
  //
  // There is no obvious way to relax restriction 2.
  //
  // Restriction 3 could reasonably be relaxed to simply let unbound
  // variables sit at 0.
  if (prog.required_capabilities() != ProgramAttributes({
        ProgramAttribute::kLinearComplementarityConstraint})) {
    return false;
  }

  // Check that the available LCPs cover the program and no two LCPs cover the
  // same variable.
  const auto& bindings = prog.linear_complementarity_constraints();
  for (int i = 0; i < static_cast<int>(prog.num_vars()); ++i) {
    int coverings = 0;
    for (const auto& binding : bindings) {
      if (binding.ContainsVariable(prog.decision_variable(i))) {
        coverings++;
      }
    }
    if (coverings != 1) {
      return false;
    }
  }

  return true;
}

// Instantiate templates.
template class MobyLCPSolver<double>;
template class MobyLCPSolver<Eigen::AutoDiffScalar<Vector1d>>;

}  // namespace solvers
}  // namespace drake
