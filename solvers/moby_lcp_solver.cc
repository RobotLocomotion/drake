#include "drake/solvers/moby_lcp_solver.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include <Eigen/LU>
#include <fmt/ranges.h>

#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace solvers {
namespace {

using Eigen::MatrixXd;
using Eigen::VectorXd;

bool CheckLemkeTrivial(int n, double zero_tol, const VectorXd& q, VectorXd* z) {
  // see whether trivial solution exists
  if (q.minCoeff() > -zero_tol) {
    z->resize(n);
    z->fill(0);
    return true;
  }

  return false;
}

// Linear system solver (with an extra size check). It is assumed that the
// matrix is full rank (see notes for generic LinearSolve() above).
VectorXd LinearSolve(const MatrixXd& M, const VectorXd& b) {
  // Special case necessary because Eigen doesn't always handle empty matrices
  // properly.
  if (M.rows() == 0) {
    DRAKE_ASSERT(b.size() == 0);
    return VectorXd(0);
  }
  return M.partialPivLu().solve(b);
}

// Utility function for copying part of a matrix (designated by the indices
// in rows and cols) from in to a target matrix, out.
void selectSubMat(const Eigen::MatrixXd& in, const std::vector<unsigned>& rows,
                  const std::vector<unsigned>& cols, MatrixXd* out) {
  const int num_rows = rows.size();
  const int num_cols = cols.size();
  out->resize(num_rows, num_cols);
  if (out->size() == 0) {
    return;
  }
  for (int i = 0; i < num_rows; i++) {
    const auto row_in = in.row(rows[i]);
    auto row_out = out->row(i);
    for (int j = 0; j < num_cols; j++) {
      row_out(j) = row_in(cols[j]);
    }
  }
}

// TODO(sammy-tri) this could also use a more efficient implementation.
void selectSubVec(const VectorXd& in, const std::vector<unsigned>& rows,
                  VectorXd* out) {
  const int num_rows = rows.size();
  out->resize(num_rows);
  for (int i = 0; i < num_rows; i++) {
    (*out)(i) = in(rows[i]);
  }
}

Eigen::Index minCoeffIdx(const VectorXd& in) {
  Eigen::Index idx;
  in.minCoeff(&idx);
  return idx;
}

const double kSqrtEps = std::sqrt(std::numeric_limits<double>::epsilon());
}  // namespace

void MobyLcpSolver::ClearIndexVectors() const {
  // clear all vectors
  all_.clear();
  tlist_.clear();
  bas_.clear();
  nonbas_.clear();
  j_.clear();
}

// TODO(edrumwri): Break the following code out into a special
// MobyLcpMathematicalProgram class.
void MobyLcpSolver::DoSolve(const MathematicalProgram& prog,
                            const Eigen::VectorXd& initial_guess,
                            const SolverOptions& merged_options,
                            MathematicalProgramResult* result) const {
  if (!prog.GetVariableScaling().empty()) {
    static const logging::Warn log_once(
        "MobyLcpSolver doesn't support the feature of variable scaling.");
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
    bool solved = SolveLcpLemkeRegularized(constraint->M(), constraint->q(),
                                           &constraint_solution);
    if (!solved) {
      result->set_solution_result(SolutionResult::kSolverSpecificError);
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

bool MobyLcpSolver::SolveLcpFast(const MatrixXd& M, const VectorXd& q,
                                 VectorXd* z, double zero_tol) const {
  using std::abs;

  // Variables that will be reused multiple times, thus hopefully allowing
  // Eigen to keep from freeing/reallocating memory repeatedly.
  VectorXd zz, w, qbas;
  MatrixXd Mmix, Msub;

  const unsigned N = q.rows();
  const unsigned UINF = std::numeric_limits<unsigned>::max();

  if (M.rows() != N || M.cols() != N)
    throw std::logic_error("M's dimensions do not match that of q.");

  log()->debug("MobyLcpSolver::SolveLcpFast() entered");

  // look for trivial solution
  if (N == 0) {
    log()->debug("MobyLcpSolver::SolveLcpFast() - empty problem");
    z->resize(0);
    return true;
  }

  // set zero tolerance if necessary
  double mod_zero_tol = zero_tol;
  if (mod_zero_tol < 0) mod_zero_tol = ComputeZeroTolerance(M);

  // prepare to setup basic and nonbasic variable indices for z
  nonbas_.clear();
  bas_.clear();

  // see whether to warm-start
  if (z->size() == q.size()) {
    log()->debug("MobyLcpSolver::SolveLcpFast() - warm starting activated");

    for (unsigned i = 0; i < z->size(); i++) {
      if (abs((*z)[i]) < mod_zero_tol) {
        bas_.push_back(i);
      } else {
        nonbas_.push_back(i);
      }
    }

    log()->debug(" -- non-basic indices: {}", fmt::join(nonbas_, " "));
  } else {
    // get minimum element of q (really w)
    Eigen::Index minw;
    const double minw_val = q.minCoeff(&minw);
    if (minw_val > -mod_zero_tol) {
      log()->debug("MobyLcpSolver::SolveLcpFast() - trivial solution found");
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
      w = VectorXd();
      minw = UINF;
    } else {
      w = Mmix * zz;
      w += qbas;
      minw = minCoeffIdx(w);
    }

    // TODO(sammy-tri) this log can't print when minw is UINF.
    // LOG() << "MobyLcpSolver::SolveLcpFast() - minimum w after pivot: "
    // << _w[minw] << std::endl;

    // if w >= 0, check whether any component of z < 0
    if (minw == UINF || w[minw] > -mod_zero_tol) {
      // find the (a) minimum of z
      unsigned minz = (zz.rows() > 0) ? minCoeffIdx(zz) : UINF;
      if (zz.rows() > 0) {
        log()->debug(
            "MobyLcpSolver::SolveLcpFast() - minimum z after pivot: {}",
            zz[minz]);
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

        log()->debug("MobyLcpSolver::SolveLcpFast() - solution found!");
        return true;
      }
    } else {
      log()->debug("(minimum w too negative)");

      // one or more components of w violating w >= 0
      // move component of w from basic set to nonbasic set
      unsigned idx = bas_[minw];
      bas_.erase(bas_.begin() + minw);
      nonbas_.push_back(idx);
      std::sort(nonbas_.begin(), nonbas_.end());

      // look whether any component of z needs to move to basic set
      unsigned minz = (zz.rows() > 0) ? minCoeffIdx(zz) : UINF;
      if (zz.rows() > 0) {
        log()->debug(
            "MobyLcpSolver::SolveLcpFast() - minimum z after pivot: {}",
            zz[minz]);
      }
      if (minz < UINF && zz[minz] < -mod_zero_tol) {
        // move index to basic set and continue looping
        unsigned k = nonbas_[minz];
        log()->debug(
            "MobyLcpSolver::SolveLcpFast() - moving index {} to basic set", k);

        nonbas_.erase(nonbas_.begin() + minz);
        bas_.push_back(k);
        std::sort(bas_.begin(), bas_.end());
      }
    }
  }

  log()->debug(
      "MobyLcpSolver::SolveLcpFast() - maximum allowable pivots exceeded");

  // if we're here, then the maximum number of pivots has been exceeded
  z->setZero(N);
  return false;
}

bool MobyLcpSolver::SolveLcpFastRegularized(const MatrixXd& M,
                                            const VectorXd& q, VectorXd* z,
                                            int min_exp, unsigned step_exp,
                                            int max_exp,
                                            double zero_tol) const {
  log()->debug("MobyLcpSolver::SolveLcpFastRegularized() entered");

  // Variables that will be reused multiple times, thus hopefully allowing
  // Eigen to keep from freeing/reallocating memory repeatedly.
  VectorXd wx;
  MatrixXd MM;

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
  const double mod_zero_tol =
      (zero_tol > 0) ? zero_tol : ComputeZeroTolerance(M);

  log()->debug(" zero tolerance: {}", mod_zero_tol);

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
        const double wx_min = wx.minCoeff();
        const double wx_max = wx.maxCoeff();

        if (wx_min >= -mod_zero_tol && wx_max < mod_zero_tol) {
          log()->debug("  solved with no regularization necessary!");
          log()->debug("  pivots / total pivots: {} {}", pivots_, pivots_);
          log()->debug("MobyLcpSolver::SolveLcpFastRegularized() exited");
          return true;
        } else {
          log()->debug(
              "MobyLcpSolver::SolveLcpFastRegularized() - "
              "'<w, z> not within tolerance(min value: {} max value: {})",
              wx_min, wx_max);
        }
      } else {
        log()->debug(
            "  MobyLcpSolver::SolveLcpFastRegularized() - "
            "'w' not solved to desired tolerance");
        log()->debug("  minimum w: {}", wx.minCoeff());
      }
    } else {
      log()->debug(
          "  MobyLcpSolver::SolveLcpFastRegularized() - "
          "'z' not solved to desired tolerance");
      log()->debug("  minimum z: {}", z->minCoeff());
    }
  } else {
    log()->debug(
        "  MobyLcpSolver::SolveLcpFastRegularized() "
        "- solver failed with zero regularization");
  }

  // update the pivots
  total_piv += pivots_;

  // start the regularization process
  int rf = min_exp;
  while (rf < max_exp) {
    // setup regularization factor
    double lambda =
        std::pow(static_cast<double>(10.0), static_cast<double>(rf));

    log()->debug("  trying to solve LCP with regularization factor: {}",
                 lambda);

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
          const double wx_min = wx.minCoeff();
          const double wx_max = wx.maxCoeff();

          if (wx_min > -mod_zero_tol && wx_max < mod_zero_tol) {
            log()->debug("  solved with regularization factor: {}", lambda);
            log()->debug("  pivots / total pivots: {} {}", pivots_, total_piv);
            log()->debug("MobyLcpSolver::SolveLcpFastRegularized() exited");
            pivots_ = total_piv;
            return true;
          } else {
            log()->debug(
                "MobyLcpSolver::SolveLcpFastRegularized() - "
                "'<w, z> not within tolerance(min value: {} max value: {})",
                wx_min, wx_max);
          }
        } else {
          log()->debug(
              "  MobyLcpSolver::SolveLcpFastRegularized() - "
              "'w' not solved to desired tolerance");
          log()->debug("  minimum w: {}", wx.minCoeff());
        }
      } else {
        log()->debug(
            "  MobyLcpSolver::SolveLcpFastRegularized() - "
            "'z' not solved to desired tolerance");
        log()->debug("  minimum z: {}", z->minCoeff());
      }
    }

    // increase rf
    rf += step_exp;
  }

  log()->debug("  unable to solve given any regularization!");
  log()->debug("MobyLcpSolver::SolveLcpFastRegularized() exited");

  // store total pivots
  pivots_ = total_piv;

  // still here?  failure...
  return false;
}

// Retrieves the solution computed by Lemke's Algorithm.
void MobyLcpSolver::FinishLemkeSolution(const MatrixXd& M, const VectorXd& q,
                                        const VectorXd& x, VectorXd* z) const {
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
  const VectorXd wl = (M * (*z)) + q;
  log()->debug("  z: {}", fmt_eigen(*z));
  log()->debug("  w: {}", fmt_eigen(wl));
  log()->debug("  minimum w: {}", wl.minCoeff());
  log()->debug("  w'z: {}", abs(wl.dot(*z)));
}

bool MobyLcpSolver::SolveLcpLemke(const MatrixXd& M, const VectorXd& q,
                                  VectorXd* z, double piv_tol,
                                  double zero_tol) const {
  using std::max;

  // Variables that will be reused multiple times, thus hopefully allowing
  // Eigen to keep from freeing/reallocating memory repeatedly.
  VectorXd result, dj, dl, x, xj, Be, u, z0;
  MatrixXd Bl, t1, t2;

  log()->debug("MobyLcpSolver::SolveLcpLemke() entered");
  log()->debug("  M: {}", fmt_eigen(M));
  log()->debug("  q: {}", fmt_eigen(q));

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
  double mod_zero_tol = zero_tol;
  if (mod_zero_tol <= 0) mod_zero_tol = ComputeZeroTolerance(M);

  if (CheckLemkeTrivial(n, mod_zero_tol, q, z)) {
    log()->debug(" -- trivial solution found");
    log()->debug("MobyLcpSolver::SolveLcpLemke() exited");
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
    log()->debug("-- initial basis not empty (warmstarting)");

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
    log()->debug("-- using basis of -1 (no warmstarting)");

    // use standard initial basis
    Bl.resize(n, n);
    Bl.setIdentity();
    Bl *= -1;
    x = q;
  }

  // check whether initial basis provides a solution
  if (x.minCoeff() >= 0.0) {
    log()->debug(" -- initial basis provides a solution!");
    FinishLemkeSolution(M, q, x, z);
    log()->debug("MobyLcpSolver::SolveLcpLemke() exited");
    return true;
  }

  // use a new pivot tolerance if necessary
  const double naive_piv_tol = n *
                               max(1.0, M.template lpNorm<Eigen::Infinity>()) *
                               std::numeric_limits<double>::epsilon();
  const double mod_piv_tol = (piv_tol > 0) ? piv_tol : naive_piv_tol;

  // determine initial leaving variable
  Eigen::Index min_x;
  const double min_x_val = x.topRows(n).minCoeff(&min_x);
  const double tval = -min_x_val;
  for (size_t i = 0; i < nonbas_.size(); i++) {
    bas_.push_back(nonbas_[i] + n);
  }
  lvindex = min_x;
  iiter = bas_.begin();
  std::advance(iiter, lvindex);
  leaving = *iiter;
  log()->debug(" -- x: {}", fmt_eigen(x));
  log()->debug(
      " -- first pivot: leaving index={} entering index={} "
      "minimum value: {}",
      lvindex, entering, tval);

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
  log()->debug("  new q: {}", fmt_eigen(x));

  // main iterations begin here
  for (pivots_ = 0; pivots_ < max_iter; pivots_++) {
    log()->debug("basic variables: {}", fmt::join(bas_, " "));
    log()->debug("leaving: {} t: {}", leaving, t);

    // check whether done; if not, get new entering variable
    if (leaving == t) {
      log()->debug("-- solved LCP successfully!");
      FinishLemkeSolution(M, q, x, z);
      log()->debug("MobyLcpSolver::SolveLcpLemke() exited");
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
      log()->debug(
          "MobyLcpSolver::SolveLcpLemke() - no new pivots (ray termination)");
      log()->debug("MobyLcpSolver::SolveLcpLemke() exiting");
      z->setZero(n);
      return false;
    }

    log()->debug("d: {}", fmt_eigen(dl));
    log()->debug("j (before min ratio): {}", fmt::join(j_, " "));

    // select elements j from x and d
    selectSubVec(x, j_, &xj);
    selectSubVec(dl, j_, &dj);

    // compute minimal ratios x(j) + EPS_DOUBLE ./ d(j), d > 0
    result.resize(xj.size());
    result.fill(mod_zero_tol);
    result = xj.eval().array() + result.array();
    result = result.eval().array() / dj.array();
    const double theta = result.minCoeff();

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
    log()->debug("j (after min ratio): {}", fmt::join(j_, " "));

    // if j is empty, then likely the zero tolerance is too low
    if (j_.empty()) {
      log()->debug("zero tolerance too low?");
      log()->debug("MobyLcpSolver::SolveLcpLemke() exited");
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
    const double ratio = x[lvindex] / dl[lvindex];
    dl *= ratio;
    x -= dl;
    x[lvindex] = ratio;
    Bl.col(lvindex) = Be;
    *iiter = entering;
    log()->debug(" -- pivoting: leaving index={}  entering index={}", lvindex,
                 entering);
  }

  log()->debug(" -- maximum number of iterations exceeded (n={}, max={})", n,
               max_iter);
  log()->debug("MobyLcpSolver::SolveLcpLemke() exited");
  z->setZero(n);
  return false;
}

bool MobyLcpSolver::SolveLcpLemkeRegularized(const MatrixXd& M,
                                             const VectorXd& q, VectorXd* z,
                                             int min_exp, unsigned step_exp,
                                             int max_exp, double piv_tol,
                                             double zero_tol) const {
  // Variables that will be reused multiple times, thus hopefully allowing
  // Eigen to keep from freeing/reallocating memory repeatedly.
  VectorXd wx;

  log()->debug("MobyLcpSolver::SolveLcpLemkeRegularized() entered");

  // look for fast exit
  if (q.size() == 0) {
    z->resize(0);
    return true;
  }

  // copy MM
  MatrixXd MM = M;

  // Assign value for zero tolerance, if necessary. See discussion in
  // SolveLcpFastRegularized() to see why this tolerance is computed here once,
  // rather than for each regularized version of M.
  const double mod_zero_tol =
      (zero_tol > 0) ? zero_tol : ComputeZeroTolerance(M);

  log()->debug(" zero tolerance: {}", mod_zero_tol);

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

        const double wx_min = wx.minCoeff();
        const double wx_max = wx.maxCoeff();
        if (wx_min >= -mod_zero_tol && wx_max < mod_zero_tol) {
          log()->debug("  solved with no regularization necessary!");
          log()->debug("MobyLcpSolver::SolveLcpLemkeRegularized() exited");
          return true;
        } else {
          log()->debug(
              "MobyLcpSolver::SolveLcpLemke() - "
              "'<w, z> not within tolerance(min value: {} max value: {})",
              wx_min, wx_max);
        }
      } else {
        log()->debug(
            "  MobyLcpSolver::SolveLcpLemke() - 'w' not solved to desired "
            "tolerance");
        log()->debug("  minimum w: {}", wx.minCoeff());
      }
    } else {
      log()->debug(
          "  MobyLcpSolver::SolveLcpLemke() - 'z' not solved to desired "
          "tolerance");
      log()->debug("  minimum z: {}", z->minCoeff());
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

    log()->debug("  trying to solve LCP with regularization factor: {}",
                 lambda);

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

          const double wx_min = wx.minCoeff();
          const double wx_max = wx.maxCoeff();
          if (wx_min > -mod_zero_tol && wx_max < mod_zero_tol) {
            log()->debug("  solved with regularization factor: {}", lambda);
            log()->debug("MobyLcpSolver::SolveLcpLemkeRegularized() exited");
            pivots_ = total_piv;
            return true;
          } else {
            log()->debug(
                "MobyLcpSolver::SolveLcpLemke() - "
                "'<w, z> not within tolerance(min value: {} max value: {})",
                wx_min, wx_max);
          }
        } else {
          log()->debug(
              "  MobyLcpSolver::SolveLcpLemke() - 'w' not solved to "
              "desired tolerance");
          log()->debug("  minimum w: {}", wx.minCoeff());
        }
      } else {
        log()->debug(
            "  MobyLcpSolver::SolveLcpLemke() - 'z' not solved to desired "
            "tolerance");
        log()->debug("  minimum z: {}", z->minCoeff());
      }
    }

    // increase rf
    rf += step_exp;
  }

  log()->debug("  unable to solve given any regularization!");
  log()->debug("MobyLcpSolver::SolveLcpLemkeRegularized() exited");

  // store total pivots
  pivots_ = total_piv;

  // still here?  failure...
  return false;
}

MobyLcpSolver::MobyLcpSolver()
    : SolverBase(id(), &is_available, &is_enabled,
                 &ProgramAttributesSatisfied) {}

MobyLcpSolver::~MobyLcpSolver() = default;

SolverId MobyLcpSolver::id() {
  static const never_destroyed<SolverId> singleton{"Moby LCP"};
  return singleton.access();
}

bool MobyLcpSolver::is_available() {
  return true;
}

bool MobyLcpSolver::is_enabled() {
  return true;
}

bool MobyLcpSolver::ProgramAttributesSatisfied(
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
  if (prog.required_capabilities() !=
      ProgramAttributes({ProgramAttribute::kLinearComplementarityConstraint})) {
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

}  // namespace solvers
}  // namespace drake
