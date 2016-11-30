// Adapted with permission from code by Evan Drumwright
// (https://github.com/edrumwri).

#include "drake/solvers/moby_lcp_solver.h"

#include <Eigen/LU>
#include <Eigen/SparseCore>
#include <Eigen/SparseLU>

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "drake/common/drake_assert.h"

namespace drake {
namespace solvers {
namespace {
template <typename Derived>
void selectSubMat(const Eigen::MatrixBase<Derived>& in,
                  const std::vector<unsigned>& rows,
                  const std::vector<unsigned>& cols, Eigen::MatrixXd* out) {
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
template <typename Derived>
void selectSubVec(const Eigen::MatrixBase<Derived>& in,
                  const std::vector<unsigned>& rows, Eigen::VectorXd* out) {
  const int num_rows = rows.size();
  out->resize(num_rows);
  for (int i = 0; i < num_rows; i++) {
    (*out)(i) = in(rows[i]);
  }
}

template <typename DerivedA, typename DerivedB, typename F>
void transformVecElements(const Eigen::MatrixBase<DerivedA>& in,
                          Eigen::MatrixBase<DerivedB>* out, F func) {
  DRAKE_ASSERT(in.cols() == 1);
  DRAKE_ASSERT(out->cols() == 1);
  DRAKE_ASSERT(in.rows() == out->rows());
  for (int i = 0; i < in.rows(); i++) {
    (*out)(i) = func(in(i), (*out)(i));
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

const double NEAR_ZERO = std::sqrt(std::numeric_limits<double>::epsilon());
}  // anonymous namespace

// Sole constructor
MobyLCPSolver::MobyLCPSolver() : log_enabled_(false) {}

void MobyLCPSolver::SetLoggingEnabled(bool enabled) { log_enabled_ = enabled; }

std::ostream& MobyLCPSolver::Log() const {
  if (log_enabled_) {
    return std::cerr;
  }
  return null_stream_;
}

void MobyLCPSolver::ClearIndexVectors() const {
  // clear all vectors
  all_.clear();
  tlist_.clear();
  bas_.clear();
  nonbas_.clear();
  j_.clear();
}

SolutionResult MobyLCPSolver::Solve(MathematicalProgram& prog) const {
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
  for (size_t i = 0; i < prog.num_vars(); i++) {
    int coverings = 0;
    for (const auto& binding : bindings) {
      if (binding.Covers(i)) {
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
  Eigen::VectorXd solution(prog.num_vars());

  // We don't actually indicate different results.
  prog.SetSolverResult("MobyLCP", 0);

  for (const auto& binding : bindings) {
    Eigen::VectorXd constraint_solution(binding.GetNumElements());
    const std::shared_ptr<LinearComplementarityConstraint> constraint =
        binding.constraint();
    bool solved = SolveLcpLemkeRegularized(
        constraint->M(), constraint->q(), &constraint_solution);
    if (!solved) {
      return SolutionResult::kUnknownError;
    }
    binding.WriteThrough(constraint_solution, &solution);
  }
  prog.SetDecisionVariableValues(solution);
  return SolutionResult::kSolutionFound;
}

/// Fast pivoting algorithm for denerate, monotone LCPs with few nonzero,
/// nonbasic variables
bool MobyLCPSolver::SolveLcpFast(const Eigen::MatrixXd& M,
                                 const Eigen::VectorXd& q, Eigen::VectorXd* z,
                                 double zero_tol) const {
  const unsigned N = q.rows();
  const unsigned UINF = std::numeric_limits<unsigned>::max();

  Log() << "MobyLCPSolver::SolveLcpFast() entered" << std::endl;

  // look for trivial solution
  if (N == 0) {
    Log() << "MobyLCPSolver::SolveLcpFast() - empty problem" << std::endl;
    z->resize(0);
    return true;
  }

  // set zero tolerance if necessary
  if (zero_tol < 0.0) {
    zero_tol = M.rows() * M.lpNorm<Eigen::Infinity>() *
               std::numeric_limits<double>::epsilon();
  }

  // prepare to setup basic and nonbasic variable indices for z
  nonbas_.clear();
  bas_.clear();

  // see whether to warm-start
  if (z->size() == q.size()) {
    Log() << "MobyLCPSolver::SolveLcpFast() - warm starting activated"
          << std::endl;

    for (unsigned i = 0; i < z->size(); i++) {
      if (std::fabs((*z)[i]) < zero_tol) {
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
    const double minw_val = q.minCoeff(&minw);
    if (minw_val > -zero_tol) {
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

  // loop for maximum number of pivots
  //  const unsigned MAX_PIV = std::max(N*N, (unsigned) 1000);
  const unsigned MAX_PIV = 2 * N;
  for (pivots_ = 0; pivots_ < MAX_PIV; pivots_++) {
    // select nonbasic indices
    selectSubMat(M, nonbas_, nonbas_, &Msub_);
    selectSubMat(M, bas_, nonbas_, &Mmix_);
    selectSubVec(q, nonbas_, &z_);
    selectSubVec(q, bas_, &qbas_);
    // _z.negate();
    z_ = z_ * -1;

    // solve for nonbasic z
    z_ = Msub_.lu().solve(z_);

    // compute w and find minimum value
    w_ = Mmix_ * z_;
    w_ += qbas_;
    unsigned minw = (w_.rows() > 0) ? minCoeffIdx(w_) : UINF;

    // TODO(sammy-tri) this log can't print when minw is UINF.
    // LOG() << "MobyLCPSolver::SolveLcpFast() - minimum w after pivot: "
    // << _w[minw] << std::endl;

    // if w >= 0, check whether any component of z < 0
    if (minw == UINF || w_[minw] > -zero_tol) {
      // find the (a) minimum of z
      unsigned minz = (z_.rows() > 0) ? minCoeffIdx(z_) : UINF;
      if (log_enabled_ && z_.rows() > 0) {
        Log() << "MobyLCPSolver::SolveLcpFast() - minimum z after pivot: "
              << z_[minz] << std::endl;
      }
      if (minz < UINF && z_[minz] < -zero_tol) {
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
          (*z)[nonbas_[j]] = z_[i];
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
      unsigned minz = (z_.rows() > 0) ? minCoeffIdx(z_) : UINF;
      if (log_enabled_ && z_.rows() > 0) {
        Log() << "MobyLCPSolver::SolveLcpFast() - minimum z after pivot: "
              << z_[minz] << std::endl;
      }
      if (minz < UINF && z_[minz] < -zero_tol) {
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
  return false;
}

/// Regularized wrapper around PPM I
bool MobyLCPSolver::SolveLcpFastRegularized(const Eigen::MatrixXd& M,
                                            const Eigen::VectorXd& q,
                                            Eigen::VectorXd* z, int min_exp,
                                            unsigned step_exp, int max_exp,
                                            double zero_tol) const {
  Log() << "MobyLCPSolver::SolveLcpFastRegularized() entered" << std::endl;

  // look for fast exit
  if (q.size() == 0) {
    z->resize(0);
    return true;
  }

  // copy MM
  MM_ = M;

  // assign value for zero tolerance, if necessary
  const double ZERO_TOL =
      (zero_tol > static_cast<double>(0.0))
          ? zero_tol
          : (q.size() * M.lpNorm<Eigen::Infinity>() * NEAR_ZERO);

  Log() << " zero tolerance: " << ZERO_TOL << std::endl;

  // store the total pivots
  unsigned total_piv = 0;

  // try non-regularized version first
  bool result = SolveLcpFast(MM_, q, z, zero_tol);
  if (result) {
    // verify that solution truly is a solution -- check z
    if (z->minCoeff() >= -ZERO_TOL) {
      // check w
      wx_ = (M * (*z)) + q;
      if (wx_.minCoeff() >= -ZERO_TOL) {
        // check z'w
        transformVecElements(*z, &wx_, std::multiplies<double>());
        const double wx_min = wx_.minCoeff();
        const double wx_max = wx_.maxCoeff();

        if (wx_min >= -ZERO_TOL && wx_max < ZERO_TOL) {
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
        Log() << "  minimum w: " << wx_.minCoeff() << std::endl;
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
    MM_ = M;
    for (unsigned i = 0; i < M.rows(); i++) {
      MM_(i, i) += lambda;
    }

    // try to solve the LCP
    result = SolveLcpFast(MM_, q, z, zero_tol);

    // update total pivots
    total_piv += pivots_;

    if (result) {
      // verify that solution truly is a solution -- check z
      if (z->minCoeff() > -ZERO_TOL) {
        // check w
        wx_ = (MM_ * (*z)) + q;
        if (wx_.minCoeff() > -ZERO_TOL) {
          // check z'w
          transformVecElements(*z, &wx_, std::multiplies<double>());
          const double wx_min = wx_.minCoeff();
          const double wx_max = wx_.maxCoeff();

          if (wx_min > -ZERO_TOL && wx_max < ZERO_TOL) {
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
          Log() << "  minimum w: " << wx_.minCoeff() << std::endl;
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

bool MobyLCPSolver::CheckLemkeTrivial(int n, double zero_tol,
                                      const Eigen::VectorXd& q,
                                      Eigen::VectorXd* z) const {
  // see whether trivial solution exists
  if (q.minCoeff() > -zero_tol) {
    z->resize(n);
    z->fill(0);
    return true;
  }

  return false;
}

template <typename MatrixType>
void MobyLCPSolver::FinishLemkeSolution(const MatrixType& M,
                                        const Eigen::VectorXd& q,
                                        Eigen::VectorXd* z) const {
  std::vector<unsigned>::iterator iiter;
  int idx;
  for (idx = 0, iiter = bas_.begin(); iiter != bas_.end(); iiter++, idx++) {
    (*z)(*iiter) = x_(idx);
  }

  // TODO(sammy-tri) Is there a more efficient way to resize and
  // preserve the data?
  z->conservativeResize(q.size());

  // check to see whether tolerances are satisfied
  if (log_enabled_) {
    wl_ = (M * (*z)) + q;
    const double minw = wl_.minCoeff();
    const double w_dot_z = std::fabs(wl_.dot(*z));
    Log() << "  z: " << z << std::endl;
    Log() << "  _w: " << wl_ << std::endl;
    Log() << "  minimum w: " << minw << std::endl;
    Log() << "  w'z: " << w_dot_z << std::endl;
  }
}

/// Lemke's algorithm for solving linear complementarity problems
/**
 * \param z a vector "close" to the solution on input (optional); contains
 *        the solution on output
 */
bool MobyLCPSolver::SolveLcpLemke(const Eigen::MatrixXd& M,
                                  const Eigen::VectorXd& q, Eigen::VectorXd* z,
                                  double piv_tol, double zero_tol) const {
  if (log_enabled_) {
    Log() << "MobyLCPSolver::SolveLcpLemke() entered" << std::endl;
    Log() << "  M: " << std::endl << M;
    Log() << "  q: " << q << std::endl;
  }

  const unsigned n = q.size();
  const unsigned MAXITER = std::min((unsigned)1000, 50 * n);

  // update the pivots
  pivots_ = 0;

  // look for immediate exit
  if (n == 0) {
    z->resize(0);
    return true;
  }

  // come up with a sensible value for zero tolerance if none is given
  if (zero_tol <= static_cast<double>(0.0)) {
    zero_tol =
        M.lpNorm<Eigen::Infinity>() * std::numeric_limits<double>::epsilon();
  }

  if (CheckLemkeTrivial(n, zero_tol, q, z)) {
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
  z0_ = *z;

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
  if (z0_.size() != n) {
    // setup the nonbasic indices
    for (unsigned i = 0; i < n; i++) nonbas_.push_back(i);
  } else {
    for (unsigned i = 0; i < n; i++) {
      if (z0_[i] > 0) {
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
    Bl_.resize(n, n);
    Bl_.setIdentity();
    // _Bl.negate();
    Bl_ *= -1;

    // select columns of M corresponding to z vars in the basis
    selectSubMat(M, all_, bas_, &t1_);

    // select columns of I corresponding to z vars not in the basis
    selectSubMat(Bl_, all_, nonbas_, &t2_);

    // setup the basis matrix
    Bl_.resize(n, t1_.cols() + t2_.cols());
    Bl_.block(0, 0, t1_.rows(), t1_.cols()) = t1_;
    Bl_.block(0, t1_.cols(), t2_.rows(), t2_.cols()) = t2_;

    // solve B*x = -q
    //
    // The original version of this code from Moby could handle the
    // case where the colver failed (though only in this dense
    // implementation).  It's not obvious how to do this from Eigen,
    // so we've (sam.creasey) assumed that the solve succeeds.
    Al_ = Bl_;
    x_ = Al_.lu().solve(q);
  } else {
    Log() << "-- using basis of -1 (no warmstarting)" << std::endl;

    // use standard initial basis
    Bl_.resize(n, n);
    Bl_.setIdentity();
    Bl_ *= -1;
    x_ = q;
  }

  // check whether initial basis provides a solution
  if (x_.minCoeff() >= 0.0) {
    Log() << " -- initial basis provides a solution!" << std::endl;
    FinishLemkeSolution(M, q, z);
    Log() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
    return true;
  }

  // use a new pivot tolerance if necessary
  const double PIV_TOL =
      (piv_tol > static_cast<double>(0.0))
          ? piv_tol
          : (std::numeric_limits<double>::epsilon() * n *
             std::max(static_cast<double>(1.0), M.lpNorm<Eigen::Infinity>()));

  // determine initial leaving variable
  Eigen::Index min_x;
  const double min_x_val = x_.topRows(n).minCoeff(&min_x);
  double tval = -min_x_val;
  for (size_t i = 0; i < nonbas_.size(); i++) {
    bas_.push_back(nonbas_[i] + n);
  }
  lvindex = min_x;
  iiter = bas_.begin();
  std::advance(iiter, lvindex);
  leaving = *iiter;
  Log() << " -- x: " << x_ << std::endl;
  Log() << " -- first pivot: leaving index=" << lvindex
        << "  entering index=" << entering << " minimum value: " << tval
        << std::endl;

  // pivot in the artificial variable
  *iiter = t;  // replace w var with _z0 in basic indices
  u_.resize(n);
  for (unsigned i = 0; i < n; i++) {
    u_[i] = (x_[i] < 0.0) ? 1.0 : 0.0;
  }
  Be_ = (Bl_ * u_) * -1;
  u_ *= tval;
  x_ += u_;
  x_[lvindex] = tval;
  Bl_.col(lvindex) = Be_;
  Log() << "  new q: " << x_ << std::endl;

  // main iterations begin here
  for (pivots_ = 0; pivots_ < MAXITER; pivots_++) {
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
      FinishLemkeSolution(M, q, z);
      Log() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
      return true;
    } else if (leaving < n) {
      entering = n + leaving;
      Be_.resize(n);
      Be_.fill(0);
      Be_[leaving] = -1;
    } else {
      entering = leaving - n;
      Be_ = M.col(entering);
    }
    dl_ = Be_;

    // Again, The original version of this code from Moby could handle
    // the case where the solver failed.  There was also some
    // commented out code related to trying to restart on finding a
    // solution.  For the reasons above, and the fact that it was
    // commented out, this functionality has not been preserved.
    Al_ = Bl_;
    dl_ = Al_.lu().solve(dl_);

    // ** find new leaving variable
    j_.clear();
    for (unsigned i = 0; i < dl_.size(); i++) {
      if (dl_[i] > PIV_TOL) {
        j_.push_back(i);
      }
    }

    // check for no new pivots; ray termination
    if (j_.empty()) {
      Log()
          << "MobyLCPSolver::SolveLcpLemke() - no new pivots (ray termination)"
          << std::endl;
      Log() << "MobyLCPSolver::SolveLcpLemke() exiting" << std::endl;
      return false;
    }

    if (log_enabled_) {
      std::ostringstream j;
      for (unsigned i = 0; i < j_.size(); i++) j << " " << j_[i];
      Log() << "d: " << dl_ << std::endl;
      Log() << "j (before min ratio):" << j.str() << std::endl;
    }

    // select elements j from x and d
    selectSubVec(x_, j_, &xj_);
    selectSubVec(dl_, j_, &dj_);

    // compute minimal ratios x(j) + EPS_DOUBLE ./ d(j), d > 0
    result_.resize(xj_.size());
    result_.fill(zero_tol);
    transformVecElements(xj_, &result_, std::plus<double>());
    transformVecElements(dj_, &result_,
                         [](double a, double b) { return b / a; });
    double theta = result_.minCoeff();

    // NOTE: lexicographic ordering does not appear to be used here to prevent
    // cycling (see [Cottle 1992], pp. 340-342)
    // find indices of minimal ratios, d> 0
    //   divide _x(j) ./ d(j) -- remove elements above the minimum ratio
    for (int i = 0; i < result_.size(); i++) {
      result_(i) = xj_(i) / dj_(i);
    }

    for (iiter = j_.begin(), idx = 0; iiter != j_.end();) {
      if (result_[idx++] <= theta) {
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
    double ratio = x_[lvindex] / dl_[lvindex];
    dl_ *= ratio;
    x_ -= dl_;
    x_[lvindex] = ratio;
    Bl_.col(lvindex) = Be_;
    *iiter = entering;
    Log() << " -- pivoting: leaving index=" << lvindex
          << "  entering index=" << entering << std::endl;
  }

  Log() << " -- maximum number of iterations exceeded (n=" << n
        << ", max=" << MAXITER << ")" << std::endl;
  Log() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
  return false;
}

/// Regularized wrapper around Lemke's algorithm
bool MobyLCPSolver::SolveLcpLemkeRegularized(const Eigen::MatrixXd& M,
                                             const Eigen::VectorXd& q,
                                             Eigen::VectorXd* z, int min_exp,
                                             unsigned step_exp, int max_exp,
                                             double piv_tol,
                                             double zero_tol) const {
  Log() << "MobyLCPSolver::SolveLcpLemkeRegularized() entered" << std::endl;

  // look for fast exit
  if (q.size() == 0) {
    z->resize(0);
    return true;
  }

  // copy MM
  MM_ = M;

  // assign value for zero tolerance, if necessary
  const double ZERO_TOL =
      (zero_tol > static_cast<double>(0.0))
          ? zero_tol
          : (q.size() * M.lpNorm<Eigen::Infinity>() * NEAR_ZERO);

  Log() << " zero tolerance: " << ZERO_TOL << std::endl;

  // store the total pivots
  unsigned total_piv = 0;

  // try non-regularized version first
  bool result = SolveLcpLemke(MM_, q, z, piv_tol, zero_tol);
  if (result) {
    // verify that solution truly is a solution -- check z
    if (z->minCoeff() >= -ZERO_TOL) {
      // check w
      wx_ = (M * (*z)) + q;
      if (wx_.minCoeff() >= -ZERO_TOL) {
        // check z'w
        transformVecElements(*z, &wx_, std::multiplies<double>());
        const double wx_min = wx_.minCoeff();
        const double wx_max = wx_.maxCoeff();
        if (wx_min >= -ZERO_TOL && wx_max < ZERO_TOL) {
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
        Log() << "  minimum w: " << wx_.minCoeff() << std::endl;
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
    MM_ = M;
    for (unsigned i = 0; i < M.rows(); i++) {
      MM_(i, i) += lambda;
    }

    // try to solve the LCP
    result = SolveLcpLemke(MM_, q, z, piv_tol, zero_tol);

    // update total pivots
    total_piv += pivots_;

    if (result) {
      // verify that solution truly is a solution -- check z
      if (z->minCoeff() > -ZERO_TOL) {
        // check w
        wx_ = (MM_ * (*z)) + q;
        if (wx_.minCoeff() > -ZERO_TOL) {
          // check z'w
          transformVecElements(*z, &wx_, std::multiplies<double>());
          const double wx_min = wx_.minCoeff();
          const double wx_max = wx_.maxCoeff();
          if (wx_min > -ZERO_TOL && wx_max < ZERO_TOL) {
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
          Log() << "  minimum w: " << wx_.minCoeff() << std::endl;
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

/// Lemke's algorithm for solving linear complementarity problems using sparse
/// matrices
/**
 * \param z a vector "close" to the solution on input (optional); contains
 *        the solution on output
 */
bool MobyLCPSolver::SolveLcpLemke(const Eigen::SparseMatrix<double>& M,
                                  const Eigen::VectorXd& q, Eigen::VectorXd* z,
                                  double piv_tol, double zero_tol) const {
  if (log_enabled_) {
    Log() << "MobyLCPSolver::SolveLcpLemke() entered" << std::endl;
    Log() << "  M: " << std::endl << M;
    Log() << "  q: " << q << std::endl;
  }

  const unsigned n = q.size();
  const unsigned MAXITER = std::min((unsigned)1000, 50 * n);

  // look for immediate exit
  if (n == 0) {
    z->resize(0);
    return true;
  }

  // come up with a sensible value for zero tolerance if none is given
  if (zero_tol <= static_cast<double>(0.0)) {
    Eigen::MatrixXd dense_M = M;
    zero_tol = dense_M.lpNorm<Eigen::Infinity>() *
               std::numeric_limits<double>::epsilon() * n;
  }

  if (CheckLemkeTrivial(n, zero_tol, q, z)) {
    Log() << " -- trivial solution found" << std::endl;
    Log() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
    return true;
  }

  // copy z to z0
  z0_ = *z;

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
  if (z0_.size() != n) {
    for (unsigned i = 0; i < n; i++) {
      nonbas_.push_back(i);
    }
  } else {
    for (unsigned i = 0; i < n; i++) {
      if (z0_[i] > 0) {
        bas_.push_back(i);
      } else {
        nonbas_.push_back(i);
      }
    }
  }

  // determine initial values
  sBl_ = Eigen::SparseMatrix<double>(n, n);
  if (!bas_.empty()) {
    typedef Eigen::Triplet<double> Triplet;
    std::vector<Triplet> triplet_list;
    for (int i = 0; i < M.outerSize(); i++) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(M, i); it; ++it) {
        int j = it.col();
        std::vector<unsigned>::const_iterator j_it =
            std::find(bas_.begin(), bas_.end(), j);
        if (j_it == bas_.end()) {
          continue;
        } else {
          triplet_list.push_back(Triplet(i, j, it.value()));
        }
      }
    }
    for (size_t i = 0, j = bas_.size(); i < nonbas_.size(); i++, j++) {
      triplet_list.push_back(Triplet(nonbas_[i], j, 1.0));
    }

    sBl_.setFromTriplets(triplet_list.begin(), triplet_list.end());
  } else {
    sBl_.setIdentity();
    sBl_ *= -1;
  }

  // solve B*x = -q
  std::unique_ptr<Eigen::SparseLU<Eigen::SparseMatrix<double>>> solver;
  solver.reset(new Eigen::SparseLU<Eigen::SparseMatrix<double>>);
  solver->analyzePattern(sBl_);
  solver->factorize(sBl_);
  x_ = solver->solve(q);
  x_ *= -1;

  // check whether initial basis provides a solution
  if (x_.minCoeff() >= 0.0) {
    Log() << " -- initial basis provides a solution!" << std::endl;
    FinishLemkeSolution(M, q, z);
    Log() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
    return true;
  }

  // determine initial leaving variable
  Eigen::Index min_x;
  const double min_x_val = x_.topRows(n).minCoeff(&min_x);
  double tval = -min_x_val;
  for (size_t i = 0; i < nonbas_.size(); i++) {
    bas_.push_back(nonbas_[i] + n);
  }
  lvindex = min_x;
  iiter = bas_.begin();
  std::advance(iiter, lvindex);
  leaving = *iiter;

  // pivot in the artificial variable
  *iiter = t;  // replace w var with _z0 in basic indices
  u_.resize(n);
  for (unsigned i = 0; i < n; i++) {
    u_[i] = (x_[i] < 0.0) ? 1.0 : 0.0;
  }
  Be_ = (sBl_ * u_) * -1;
  u_ *= tval;
  x_ += u_;
  x_[lvindex] = tval;
  sBl_.col(lvindex) = makeSparseVector(Be_);
  Log() << "  new q: " << x_ << std::endl;

  // main iterations begin here
  for (pivots_ = 0; pivots_ < MAXITER; pivots_++) {
    // check whether done; if not, get new entering variable
    if (leaving == t) {
      Log() << "-- solved LCP successfully!" << std::endl;
      FinishLemkeSolution(M, q, z);
      Log() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
      return true;
    } else if (leaving < n) {
      entering = n + leaving;
      Be_.resize(n);
      Be_.fill(0);
      Be_[leaving] = -1;
    } else {
      entering = leaving - n;
      Be_ = M.col(entering);
    }
    solver.reset(new Eigen::SparseLU<Eigen::SparseMatrix<double>>);
    solver->analyzePattern(sBl_);
    solver->factorize(sBl_);
    dl_ = solver->solve(Be_);

    // use a new pivot tolerance if necessary
    const double PIV_TOL = (piv_tol > static_cast<double>(0.0))
                               ? piv_tol
                               : (std::numeric_limits<double>::epsilon() * n *
                                  std::max(static_cast<double>(1.0),
                                           Be_.lpNorm<Eigen::Infinity>()));

    // ** find new leaving variable
    j_.clear();
    for (unsigned i = 0; i < dl_.size(); i++) {
      if (dl_[i] > PIV_TOL) {
        j_.push_back(i);
      }
    }
    // check for no new pivots; ray termination
    if (j_.empty()) {
      Log()
          << "MobyLCPSolver::SolveLcpLemke() - no new pivots (ray termination)"
          << std::endl;
      Log() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
      return false;
    }

    Log() << " -- column of M': " << dl_ << std::endl;

    // select elements j from x and d
    selectSubVec(x_, j_, &xj_);
    selectSubVec(dl_, j_, &dj_);

    // compute minimal ratios x(j) + EPS_DOUBLE ./ d(j), d > 0
    result_.resize(xj_.size());
    result_.fill(zero_tol);
    transformVecElements(xj_, &result_, std::plus<double>());
    transformVecElements(dj_, &result_,
                         [](double a, double b) { return b / a; });
    double theta = result_.minCoeff();

    // NOTE: lexicographic ordering does not appear to be used here to prevent
    // cycling (see [Cottle 1992], pp. 340-342)
    // find indices of minimal ratios, d> 0
    //   divide _x(j) ./ d(j) -- remove elements above the minimum ratio
    for (int i = 0; i < result_.size(); i++) {
      result_(i) = xj_(i) / dj_(i);
    }
    for (iiter = j_.begin(), idx = 0; iiter != j_.end();) {
      if (result_[idx++] <= theta) {
        iiter++;
      } else {
        iiter = j_.erase(iiter);
      }
    }
    // if j is empty, then likely the zero tolerance is too low
    if (j_.empty()) {
      Log() << "zero tolerance too low?" << std::endl;
      Log() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
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
      // first element rather than picking a random one
      lvindex = j_[0];
    }

    // set leaving = bas(lvindex)
    iiter = bas_.begin();
    std::advance(iiter, lvindex);
    leaving = *iiter;

    // ** perform pivot
    double ratio = x_[lvindex] / dl_[lvindex];
    dl_ *= ratio;
    x_ -= dl_;
    x_[lvindex] = ratio;
    sBl_.col(lvindex) = makeSparseVector(Be_);
    *iiter = entering;
    Log() << " -- pivoting: leaving index=" << lvindex
          << "  entering index=" << entering << std::endl;
  }

  Log() << " -- maximum number of iterations exceeded" << std::endl;
  Log() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
  return false;
}

/// Regularized wrapper around Lemke's algorithm for srpase matrices
bool MobyLCPSolver::SolveLcpLemkeRegularized(
    const Eigen::SparseMatrix<double>& M, const Eigen::VectorXd& q,
    Eigen::VectorXd* z, int min_exp, unsigned step_exp, int max_exp,
    double piv_tol, double zero_tol) const {
  Log() << "MobyLCPSolver::SolveLcpLemkeRegularized() entered" << std::endl;

  // look for fast exit
  if (q.size() == 0) {
    z->resize(0);
    return true;
  }

  // copy MM
  MMs_ = M;

  // assign value for zero tolerance, if necessary
  const double ZERO_TOL =
      (zero_tol > static_cast<double>(0.0)) ? zero_tol : q.size() * NEAR_ZERO;

  // try non-regularized version first
  bool result = SolveLcpLemke(MMs_, q, z, piv_tol, zero_tol);
  if (result) {
    // verify that solution truly is a solution -- check z
    if (z->minCoeff() >= -ZERO_TOL) {
      // check w
      wx_ = (M * (*z)) + q;
      if (wx_.minCoeff() >= -ZERO_TOL) {
        // check z'w
        transformVecElements(*z, &wx_, std::multiplies<double>());
        const double wx_min = wx_.minCoeff();
        const double wx_max = wx_.maxCoeff();
        if (wx_min >= -ZERO_TOL && wx_max < ZERO_TOL) {
          Log() << "  solved with no regularization necessary!" << std::endl;
          Log() << "MobyLCPSolver::SolveLcpLemkeRegularized() exited"
                << std::endl;

          return true;
        } else {
          Log() << "MobyLCPSolver::SolveLcpLemke() - "
                << "'<w, z> not within tolerance(min value: " << wx_min
                << " max value: " << wx_max << ")"
                << " tol " << ZERO_TOL << std::endl;
        }
      }
    }
  }

  eye_.resize(M.rows(), M.cols());
  eye_.setIdentity();

  // start the regularization process
  int rf = min_exp;
  while (rf < max_exp) {
    // setup regularization factor
    double lambda =
        std::pow(static_cast<double>(10.0), static_cast<double>(rf));
    (diag_lambda_ = eye_) *= lambda;

    // regularize M
    (MMx_ = MMs_) += diag_lambda_;

    // try to solve the LCP
    if ((result = SolveLcpLemke(MMx_, q, z, piv_tol, zero_tol))) {
      // verify that solution truly is a solution -- check z
      if (z->minCoeff() > -ZERO_TOL) {
        // check w
        wx_ = (MMx_ * (*z)) + q;
        if (wx_.minCoeff() > -ZERO_TOL) {
          // check z'w
          transformVecElements(*z, &wx_, std::multiplies<double>());
          if (wx_.minCoeff() > -ZERO_TOL && wx_.maxCoeff() < ZERO_TOL) {
            Log() << "  solved with regularization factor: " << lambda
                  << std::endl;
            Log() << "MobyLCPSolver::SolveLcpLemkeRegularized() exited"
                  << std::endl;

            return true;
          }
        }
      }
    }

    // increase rf
    rf += step_exp;
  }

  Log() << "  unable to solve given any regularization!" << std::endl;
  Log() << "MobyLCPSolver::SolveLcpLemkeRegularized() exited" << std::endl;

  // still here?  failure...
  return false;
}

}  // namespace solvers
}  // namespace drake
