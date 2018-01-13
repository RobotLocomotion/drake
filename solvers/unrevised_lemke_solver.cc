#include "drake/solvers/unrevised_lemke.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include <Eigen/LU>
#include <Eigen/SparseCore>
#include <Eigen/SparseLU>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"

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
void SelectSubMat(const Eigen::MatrixBase<Derived>& in,
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
void SelectSubVec(const VectorX<T>& in,
                  const std::vector<unsigned>& rows, VectorX<T>* out) {
  const int num_rows = rows.size();
  out->resize(num_rows);
  for (int i = 0; i < num_rows; i++) {
    (*out)(i) = in(rows[i]);
  }
}

template <typename Derived>
Eigen::Index GetMinCoeffIndex(const Eigen::MatrixBase<Derived>& in) {
  Eigen::Index idx;
  in.minCoeff(&idx);
  return idx;
}

}  // anonymous namespace

template <typename T>
void UnrevisedLemkeSolver<T>::ClearIndexVectors() const {
  // clear all vectors
  all_.clear();
  tlist_.clear();
  bas_.clear();
  nonbas_.clear();
  j_.clear();
}

template <>
SolutionResult
    UnrevisedLemkeSolver<Eigen::AutoDiffScalar<drake::Vector1d>>::Solve(
// NOLINTNEXTLINE(*)  Don't lint old, non-style-compliant code below.
    MathematicalProgram&) const {
  DRAKE_ABORT_MSG("UnrevisedLemkeSolver cannot yet be used in a "
                  "MathematicalProgram while templatized as an AutoDiff");
  return SolutionResult::kUnknownError;
}

// TODO(edrumwri): Break the following code out into a special
// MobyLcpMathematicalProgram class.
template <typename T>
// NOLINTNEXTLINE(*)  Don't lint old, non-style-compliant code below.
SolutionResult UnrevisedLemkeSolver<T>::Solve(MathematicalProgram& prog) const {
  // TODO(ggould-tri) This solver currently imposes restrictions that its
  // problem:
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

  DRAKE_ASSERT(prog.generic_constraints().empty());
  DRAKE_ASSERT(prog.generic_costs().empty());
  DRAKE_ASSERT(prog.GetAllLinearConstraints().empty());
  DRAKE_ASSERT(prog.bounding_box_constraints().empty());

  const auto& bindings = prog.linear_complementarity_constraints();

  // Assert that the available LCPs cover the program and no two LCPs cover
  // the same variable.
  for (int i = 0; i < static_cast<int>(prog.num_vars()); ++i) {
    int coverings = 0;
    for (const auto& binding : bindings) {
      if (binding.ContainsVariable(prog.decision_variable(i))) {
        coverings++;
      }
    }
    DRAKE_ASSERT(coverings == 1);
  }

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

  // We don't actually indicate different results.
  prog.SetSolverId(UnrevisedLemkeSolverId::id());

  for (const auto& binding : bindings) {
    Eigen::VectorXd constraint_solution(binding.GetNumElements());
    const std::shared_ptr<LinearComplementarityConstraint> constraint =
        binding.constraint();
    bool solved = SolveLcpLemkeRegularized(
        constraint->M(), constraint->q(), &constraint_solution);
    if (!solved) {
      return SolutionResult::kUnknownError;
    }
    prog.SetDecisionVariableValues(binding.variables(), constraint_solution);
    prog.SetOptimalCost(0.0);
  }
  return SolutionResult::kSolutionFound;
}

// Retrieves the solution computed by Lemke's Algorithm.
// T is irrelevant for this method (necessary only for the member function).
// MatrixType allows both dense and sparse matrices to be used.
// Scalar allows this method to be used for when the T is AutoDiffXd but
// the caller wants to use sparse methods.
// TODO(edrumwri): Address this kludge when calling sparse LCP solves from
//                 UnrevisedLemkeSolver<AutoDiffXd> has been prevented.
template <typename T>
template <typename MatrixType, typename Scalar>
void UnrevisedLemkeSolver<T>::FinishLemkeSolution(const MatrixType& M,
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
    log() << "  z: " << z << std::endl;
    log() << "  w: " << wl << std::endl;
    log() << "  minimum w: " << minw << std::endl;
    log() << "  w'z: " << w_dot_z << std::endl;
  }
}

template <typename T>
bool UnrevisedLemkeSolver<T>::SolveLcpLemke(const MatrixX<T>& M,
                                     const VectorX<T>& q, VectorX<T>* z,
                                     const T& piv_tol,
                                     const T& zero_tol) const {
  using std::max;

  // Variables that will be reused multiple times, thus hopefully allowing
  // Eigen to keep from freeing/reallocating memory repeatedly.
  VectorX<T> result, dj, dl, x, xj, Be, u, z0;
  MatrixX<T> Bl, t1, t2;

  if (log_enabled_) {
    log() << "UnrevisedLemkeSolver::SolveLcpLemke() entered" << std::endl;
    log() << "  M: " << std::endl << M;
    log() << "  q: " << q << std::endl;
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
    log() << " -- trivial solution found" << std::endl;
    log() << "UnrevisedLemkeSolver::SolveLcpLemke() exited" << std::endl;
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
    log() << "-- initial basis not empty (warmstarting)" << std::endl;

    // start from good initial basis
    Bl.resize(n, n);
    Bl.setIdentity();
    Bl *= -1;

    // select columns of M corresponding to z vars in the basis
    SelectSubMat(M, all_, bas_, &t1);

    // select columns of I corresponding to z vars not in the basis
    SelectSubMat(Bl, all_, nonbas_, &t2);

    // setup the basis matrix
    Bl.resize(n, t1.cols() + t2.cols());
    Bl.block(0, 0, t1.rows(), t1.cols()) = t1;
    Bl.block(0, t1.cols(), t2.rows(), t2.cols()) = t2;

    // Solve B*x = -q.
    x = LinearSolve(Bl, q);
  } else {
    log() << "-- using basis of -1 (no warmstarting)" << std::endl;

    // use standard initial basis
    Bl.resize(n, n);
    Bl.setIdentity();
    Bl *= -1;
    x = q;
  }

  // check whether initial basis provides a solution
  if (x.minCoeff() >= 0.0) {
    log() << " -- initial basis provides a solution!" << std::endl;
    FinishLemkeSolution(M, q, x, z);
    log() << "UnrevisedLemkeSolver::SolveLcpLemke() exited" << std::endl;
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
  log() << " -- x: " << x << std::endl;
  log() << " -- first pivot: leaving index=" << lvindex
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
  log() << "  new q: " << x << std::endl;

  // main iterations begin here
  for (pivots_ = 0; pivots_ < max_iter; pivots_++) {
    if (log_enabled_) {
      std::ostringstream basic;
      for (unsigned i = 0; i < bas_.size(); i++) {
        basic << " " << bas_[i];
      }
      log() << "basic variables:" << basic.str() << std::endl;
      log() << "leaving: " << leaving << " t:" << t << std::endl;
    }

    // check whether done; if not, get new entering variable
    if (leaving == t) {
      log() << "-- solved LCP successfully!" << std::endl;
      FinishLemkeSolution(M, q, x, z);
      log() << "UnrevisedLemkeSolver::SolveLcpLemke() exited" << std::endl;
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
      log()
          << "UnrevisedLemkeSolver::SolveLcpLemke() - no new pivots (ray termination)"
          << std::endl;
      log() << "UnrevisedLemkeSolver::SolveLcpLemke() exiting" << std::endl;
      z->setZero(n);
      return false;
    }

    if (log_enabled_) {
      std::ostringstream j;
      for (unsigned i = 0; i < j_.size(); i++) j << " " << j_[i];
      log() << "d: " << dl << std::endl;
      log() << "j (before min ratio):" << j.str() << std::endl;
    }

    // select elements j from x and d
    SelectSubVec(x, j_, &xj);
    SelectSubVec(dl, j_, &dj);

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
      log() << "j (after min ratio):" << j.str() << std::endl;
    }

    // if j is empty, then likely the zero tolerance is too low
    if (j_.empty()) {
      log() << "zero tolerance too low?" << std::endl;
      log() << "UnrevisedLemkeSolver::SolveLcpLemke() exited" << std::endl;
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
    log() << " -- pivoting: leaving index=" << lvindex
          << "  entering index=" << entering << std::endl;
  }

  log() << " -- maximum number of iterations exceeded (n=" << n
        << ", max=" << max_iter << ")" << std::endl;
  log() << "UnrevisedLemkeSolver::SolveLcpLemke() exited" << std::endl;
  z->setZero(n);
  return false;
}

template <class T>
bool UnrevisedLemkeSolver<T>::SolveLcpLemkeRegularized(const MatrixX<T>& M,
                                                const VectorX<T>& q,
                                                VectorX<T>* z, int* num_pivots,
                                                int min_exp,
                                                unsigned step_exp, int max_exp,
                                                const T& piv_tol,
                                                const T& zero_tol) const {
  // Variables that will be reused multiple times, thus hopefully allowing
  // Eigen to keep from freeing/reallocating memory repeatedly.
  VectorX<T> wx;

  log() << "UnrevisedLemkeSolver::SolveLcpLemkeRegularized() entered" << std::endl;

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

  SPDLOG_DEBUG(drake::log(), " zero tolerance: {}", mod_zero_tol);

  // store the total pivots
  *num_pivots = 0;

  // try non-regularized version first
  int pivots;
  bool result = SolveLcpLemke(MM, q, z, &pivots, piv_tol, mod_zero_tol);
  *num_pivots += pivots;
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
          SPDLOG_DEBUG(log(), "  solved with no regularization necessary!");
          return true;
        } else {
          SPDLOG_DEBUG(log(), "UnrevisedLemkeSolver::SolveLcpLemke() - " 
              "'<w, z> not within tolerance(min value: {} max value: {})",
              wx_min, wx_max);
        }
      } else {
          SPDLOG_DEBUG(log(), "  UnrevisedLemkeSolver::SolveLcpLemke() - 'w' "
              "not solved to desired tolerance");
          SPDLOG_DEBUG(log(), "  minimum w: {}", wx.minCoeff());
      }
    } else {
      SPDLOG_DEBUG(log(), "  UnrevisedLemkeSolver::SolveLcpLemke() - 'z' "
          "not solved to desired tolerance");
      SPDLOG_DEBUG(log(), "  minimum z: {}", z->minCoeff());
    }
  }

  // start the regularization process
  int rf = min_exp;
  while (rf < max_exp) {
    // setup regularization factor
    double lambda =
        std::pow(static_cast<double>(10.0), static_cast<double>(rf));

    SPDLOG_DEBUG(log(), "  trying to solve LCP with regularization factor {}",
        lambda);

    // regularize M
    MM = M;
    for (int i = 0; i < M.rows(); ++i)
      MM(i, i) += lambda;

    // try to solve the LCP
    result = SolveLcpLemke(MM, q, z, &pivots, piv_tol, mod_zero_tol);
    *num_pivots += pivots;

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
            SPDLOG_DEBUG(log(), "  solved with regularization factor {}",
                lambda);
            SPDLOG_DEBUG(log(), "UnrevisedLemkeSolver::"
                "SolveLcpLemkeRegularized() exited");
            return true;
          } else {
            SPDLOG_DEBUG(log(), "UnrevisedLemkeSolver::SolveLcpLemke() - "
                "'<w, z> not within tolerance(min value: {}, max value: {}",
                wx_min, wx_max);
          }
        } else {
          SPDLOG_DEBUG(log(), "  UnrevisedLemkeSolver::SolveLcpLemke() - "
              "'w' not solved to desired tolerance.");
          SPDLOG_DEBUG(log(), "  minimum w: {}" , wx.minCoeff());
        }
      } else {
          SPDLOG_DEBUG(log(), "  UnrevisedLemkeSolver::SolveLcpLemke() - "
              "'z' not solved to desired tolerance.");
          SPDLOG_DEBUG(log(), "  minimum z: {}" , z->minCoeff());
      }
    }

    // increase rf
    rf += step_exp;
  }

  SPDLOG_DEBUG(log(), "  unable to solve given any regularization!");
  SPDLOG_DEBUG(log(), "UnrevisedLemkeSolver::SolveLcpLemkeRegularized() exited"
      );

  // still here?  failure...
  return false;
}

template <typename T>
SolverId UnrevisedLemkeSolver<T>::solver_id() const {
  return UnrevisedLemkeSolverId::id();
}

SolverId UnrevisedLemkeSolverId::id() {
  static const never_destroyed<SolverId> singleton{"Unrevised Lemke"};
  return singleton.access();
}

// Instantiate templates.
template class UnrevisedLemkeSolver<double>;
template class
    drake::solvers::UnrevisedLemkeSolver<Eigen::AutoDiffScalar<drake::Vector1d>>;

}  // namespace solvers
}  // namespace drake
