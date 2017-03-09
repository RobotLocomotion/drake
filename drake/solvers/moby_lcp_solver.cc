#include "drake/solvers/moby_lcp_solver.h"
#include "drake/solvers/moby_lcp_solver-inl.h"
#include <unsupported/Eigen/AutoDiff>

namespace drake {
namespace solvers {

namespace {

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

const double sqrt_eps = std::sqrt(std::numeric_limits<double>::epsilon());
}  // anonymous namespace

// Sole constructor
template <typename T>
MobyLCPSolver<T>::MobyLCPSolver()
    : MathematicalProgramSolverInterface(SolverType::kMobyLCP),
      log_enabled_(false) {}

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
SolutionResult MobyLCPSolver<Eigen::AutoDiffScalar<drake::Vector1d>>::Solve(
// NOLINTNEXTLINE(*)  Don't lint old, non-style-compliant code below.
    MathematicalProgram& prog) const {
  DRAKE_ABORT_MSG("MobyLCPSolver cannot yet be used in a MathematicalProgram "
                  "while templatized as an AutoDiff");
  return SolutionResult::kUnknownError;
}

template <typename T>
// NOLINTNEXTLINE(*)  Don't lint old, non-style-compliant code below.
SolutionResult MobyLCPSolver<T>::Solve(MathematicalProgram& prog) const {
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
  prog.SetSolverResult(solver_type(), 0);

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
  }
  return SolutionResult::kSolutionFound;
}

/// Fast pivoting algorithm for LCPs of the form M = PAPᵀ, q = Pb, where b ∈ ℝᵐ,
/// P ∈ ℝⁿˣᵐ, and A ∈ ℝᵐˣᵐ (where A is positive definite). Therefore, q is in
/// the range of P and M is positive semi-definite. An LCP of this form is
/// also guaranteed to have a solution [Cottle 1992].
///
/// This particular implementation focuses on the case where the solution
/// requires few nonzero nonbasic variables, meaning that few 'z' variables need
/// be nonzero to find a solution to Mz + q = w. This algorithm, which is based
/// off of Dantzig's Principle Pivoting Method I [Cottle 1992] is described in
/// [Drumwright 2015]. This algorithm is able to use "warm" starting-
/// a solution to a "nearby" LCP can be used to find the solution to a given LCP
/// more quickly.
///
/// Although this solver is theoretically guaranteed to give a solution to
/// the LCPs described above, accumulated floating point error from pivoting
/// operations could cause the solver to fail. Additionally, the solver can be
/// applied with some success to problems outside of its guaranteed matrix
/// class. For these reasons, the solver returns a flag indicating
/// success/failure.
/// @param[in] M the LCP matrix.
/// @param[in] q the LCP vector.
/// @param[in,out] z the solution to the LCP on return (if the solver succeeds).
///                If the length of z is equal to the length of q, the solver
///                will attempt to use z's value as a starting solution.
/// @param[in] zero_tol The tolerance for testing against zero. If the tolerance
///            is negative (default) the solver will determine a generally
///            reasonable tolerance.
/// @throws std::logic_error if M is non-square or M's dimensions do not
///         equal q's dimension.
/// @returns `true` if the solver succeeded and `false` otherwise.
///
/// [Cottle 1992]      R. Cottle, J.-S. Pang, and R. Stone. The Linear
///                    Complementarity Problem. Academic Press, 1992.
/// [Drumwright 2015]  E. Drumwright. Rapidly computable viscous friction and
///                    no-slip rigid contact models. arXiv: 1504.00719v1. 2015.
template <typename T>
bool MobyLCPSolver<T>::SolveLcpFast(const MatrixX<T>& M,
                                    const VectorX<T>& q, VectorX<T>* z,
                                    T zero_tol) const {
  using std::abs;

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
  if (zero_tol < 0) {
    zero_tol = M.rows() * M.template lpNorm<Eigen::Infinity>() *
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
      if (abs((*z)[i]) < zero_tol) {
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

    // Solve for nonbasic z. If the QR factorization reveals that the matrix
    // is singular, the basis (Msub_) has become degenerate. Recovering from
    // a degenerate basis for pivoting algorithms is currently an open problem.
    // See http://www.optimization-online.org/DB_FILE/2011/03/2948.pdf for
    // example. The algorithm would ideally terminate at this point, but
    // compilation with AutoDiff currently generates template errors.
    z_ = Msub_.householderQr().solve(z_.eval());

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

/// Regularized version of the fast pivoting algorithm for LCPs of the form
/// M = PAPᵀ, q = Pb, where b ∈ ℝᵐ, P ∈ ℝⁿˣᵐ, and A ∈ ℝᵐˣᵐ (where A is positive
/// definite). Therefore, q is in the range of P and M is positive
/// semi-definite. Please see SolveLcpFast() for more documentation about the
/// particular algorithm.
///
/// This implementation wraps that algorithm with a Tikhonov-type
/// regularization approach. Specifically, this implementation repeatedly
/// attempts to solve the LCP:<pre>
/// (M + Iα)z + q = w
/// z ≥ 0
/// w ≥ 0
/// zᵀw = 0
/// </pre>
/// where I is the identity matrix and α ≪ 1, using geometrically increasing
/// values of α, until the LCP is solved. Cottle et al. describe how, for
/// sufficiently large α, the LCP will always be solvable [Cottle 1992], p. 493.
///
/// Although this solver is theoretically guaranteed to give a solution to
/// the LCPs described above, accumulated floating point error from pivoting
/// operations could cause the solver to fail. Additionally, the solver can be
/// applied with some success to problems outside of its guaranteed matrix
/// class. For these reasons, the solver returns a flag indicating
/// success/failure.
/// @param[in] M the LCP matrix.
/// @param[in] q the LCP vector.
/// @param[in,out] z the solution to the LCP on return (if the solver succeeds).
///                If the length of z is equal to the length of q, the solver
///                will attempt to use z's value as a starting solution.
/// @param[in] min_exp The minimum exponent for computing α over [10ᵝ, 10ᵞ] in
///                    steps of 10ᵟ, where β is the minimum exponent, γ is the
///                    maximum exponent, and δ is the stepping exponent.
/// @param[in] step_exp The stepping exponent for computing α over [10ᵝ, 10ᵞ] in
///                     steps of 10ᵟ, where β is the minimum exponent, γ is the
///                     maximum exponent, and δ is the stepping exponent.
/// @param[in] max_exp The maximum exponent for computing α over [10ᵝ, 10ᵞ] in
///                    steps of 10ᵟ, where β is the minimum exponent, γ is the
///                    maximum exponent, and δ is the stepping exponent.
/// @param[in] zero_tol The tolerance for testing against zero. If the tolerance
///            is negative (default) the solver will determine a generally
///            reasonable tolerance.
/// @throws std::logic_error if M is non-square or M's dimensions do not
///         equal q's dimension.
/// @returns `true` if the solver succeeded and `false` if the solver did not
///          find a solution for α = 10ᵞ.
/// @sa SolveLcpFast()
///
/// [Cottle, 1992]     R. Cottle, J.-S. Pang, and R. Stone. The Linear
///                    Complementarity Problem. Academic Press, 1992.
template <typename T>
bool MobyLCPSolver<T>::SolveLcpFastRegularized(const MatrixX<T>& M,
                                               const VectorX<T>& q,
                                               VectorX<T>* z, int min_exp,
                                               unsigned step_exp, int max_exp,
                                               T zero_tol) const {
  Log() << "MobyLCPSolver::SolveLcpFastRegularized() entered" << std::endl;

  // look for fast exit
  if (q.size() == 0) {
    z->resize(0);
    return true;
  }

  // copy MM
  MM_ = M;

  // assign value for zero tolerance, if necessary
  const T naive_tol = q.size() * M.template lpNorm<Eigen::Infinity>() *
      sqrt_eps;
  const T ZERO_TOL = (zero_tol > 0) ? zero_tol : naive_tol;

  Log() << " zero tolerance: " << ZERO_TOL << std::endl;

  // store the total pivots
  unsigned total_piv = 0;

  // try non-regularized version first
  bool result = SolveLcpFast(MM_, q, z, ZERO_TOL);
  if (result) {
    // verify that solution truly is a solution -- check z
    if (z->minCoeff() >= -ZERO_TOL) {
      // check w
      wx_ = (M * (*z)) + q;
      if (wx_.minCoeff() >= -ZERO_TOL) {
        // Check element-wise operation of z*wx_.
        wx_ = z->array() * wx_.eval().array();
        const T wx_min = wx_.minCoeff();
        const T wx_max = wx_.maxCoeff();

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
    result = SolveLcpFast(MM_, q, z, ZERO_TOL);

    // update total pivots
    total_piv += pivots_;

    if (result) {
      // verify that solution truly is a solution -- check z
      if (z->minCoeff() > -ZERO_TOL) {
        // check w
        wx_ = (MM_ * (*z)) + q;
        if (wx_.minCoeff() > -ZERO_TOL) {
          // Check element-wise operation of z*wx_.
          wx_ = z->array() * wx_.eval().array();
          const T wx_min = wx_.minCoeff();
          const T wx_max = wx_.maxCoeff();

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

template <typename T>
template <typename Scalar>
bool MobyLCPSolver<T>::CheckLemkeTrivial(int n, const Scalar& zero_tol,
                                         const VectorX<Scalar>& q,
                                         VectorX<Scalar>* z) const {
  // see whether trivial solution exists
  if (q.minCoeff() > -zero_tol) {
    z->resize(n);
    z->fill(0);
    return true;
  }

  return false;
}

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
    const auto wl = (M * (*z)) + q;
    const T minw = wl.minCoeff();
    const T w_dot_z = abs(wl.dot(*z));
    Log() << "  z: " << z << std::endl;
    Log() << "  w: " << wl << std::endl;
    Log() << "  minimum w: " << minw << std::endl;
    Log() << "  w'z: " << w_dot_z << std::endl;
  }
}

/// Lemke's Algorithm for solving LCPs in the matrix class E, which contains
/// all strictly semimonotone matrices, all P-matrices, and all strictly
/// copositive matrices. Lemke's Algorithm is described in [Cottle 1992],
/// Section 4.4. This implementation was adapted from the LEMKE Library
/// [LEMKE] for Matlab; this particular implementation fixes a bug
/// in LEMKE that could occur when multiple indices passed the minimum ratio
/// test.
///
/// Although this solver is theoretically guaranteed to give a solution to
/// the LCPs described above, accumulated floating point error from pivoting
/// operations could cause the solver to fail. Additionally, the solver can be
/// applied with some success to problems outside of its guaranteed matrix
/// classes. For these reasons, the solver returns a flag indicating
/// success/failure.
/// @param[in] M the LCP matrix.
/// @param[in] q the LCP vector.
/// @param[in,out] z the solution to the LCP on return (if the solver succeeds).
///                If the length of z is equal to the length of q, the solver
///                will attempt to use z's value as a starting solution. **This
///                warmstarting is generally not recommended**: it has a
///                predisposition to lead to a failing pivoting sequence.
/// @param[in] zero_tol The tolerance for testing against zero. If the tolerance
///            is negative (default) the solver will determine a generally
///            reasonable tolerance.
/// @param[in] piv_tol The tolerance for testing against zero, specifically
///            used for the purpose of finding variables for pivoting. If the
///            tolerance is negative (default) the solver will determine a
///            generally reasonable tolerance.
/// @returns `true` if the solver **believes** it has computed a solution
///          (which it determines by the ability to "pivot out" the "artificial"
///          variable (see [Cottle 1992]) and `false` otherwise.
/// @warning The caller should verify that the algorithm has solved the LCP to
///          the desired tolerances on returns indicating success.
/// @throws std::logic_error if M is not square or the dimensions of M do not
///         match the length of q.
///
/// [Cottle 1992]      R. Cottle, J.-S. Pang, and R. Stone. The Linear
///                    Complementarity Problem. Academic Press, 1992.
/// [LEMKE]            P. Fackler and M. Miranda. LEMKE.
///                    http://people.sc.fsu.edu/~burkardt/m\_src/lemke/lemke.m
template <typename T>
bool MobyLCPSolver<T>::SolveLcpLemke(const MatrixX<T>& M,
                                     const VectorX<T>& q, VectorX<T>* z,
                                     T piv_tol, T zero_tol) const {
  using std::max;

  if (log_enabled_) {
    Log() << "MobyLCPSolver::SolveLcpLemke() entered" << std::endl;
    Log() << "  M: " << std::endl << M;
    Log() << "  q: " << q << std::endl;
  }

  const unsigned n = q.size();
  const unsigned max_iter = std::min((unsigned)1000, 50 * n);

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
  if (zero_tol <= 0) {
    zero_tol =
        M.template lpNorm<Eigen::Infinity>() *
            std::numeric_limits<double>::epsilon();
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
    // Solve for nonbasic z. If the QR factorization reveals that the matrix
    // is singular, the basis (Msub_) has become degenerate. Recovering from
    // a degenerate basis for Lemke's Algorithm is currently an open problem.
    // See http://www.optimization-online.org/DB_FILE/2011/03/2948.pdf, for
    // example. The algorithm would ideally terminate at this point, but
    // compilation of householderQr().rank() with AutoDiff currently generates
    // template errors. Leaving this as-is means that the algorithm might
    // continue on for some time and could conceivably even indicate success on
    // return, though return does not guarantee the solution is correct to
    // desired tolerances anyway (see function documentation).
    x_ = Bl_.householderQr().solve(q);
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
    FinishLemkeSolution(M, q, x_, z);
    Log() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
    return true;
  }

  // use a new pivot tolerance if necessary
  const T naive_piv_tol = n * max(T(1), M.template lpNorm<Eigen::Infinity>()) *
      std::numeric_limits<double>::epsilon();
  const T PIV_TOL = (piv_tol > 0) ? piv_tol : naive_piv_tol;

  // determine initial leaving variable
  Eigen::Index min_x;
  const T min_x_val = x_.topRows(n).minCoeff(&min_x);
  const T tval = -min_x_val;
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
    u_[i] = (x_[i] < 0) ? 1 : 0;
  }
  Be_ = (Bl_ * u_) * -1;
  u_ *= tval;
  x_ += u_;
  x_[lvindex] = tval;
  Bl_.col(lvindex) = Be_;
  Log() << "  new q: " << x_ << std::endl;

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
      FinishLemkeSolution(M, q, x_, z);
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

    // See comments above on the possibility of this solve failing.
    dl_ = Bl_.householderQr().solve(dl_.eval());

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
    result_ = xj_.eval().array() + result_.array();
    result_ = result_.eval().array() / dj_.array();
    const T theta = result_.minCoeff();

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
    const T ratio = x_[lvindex] / dl_[lvindex];
    dl_ *= ratio;
    x_ -= dl_;
    x_[lvindex] = ratio;
    Bl_.col(lvindex) = Be_;
    *iiter = entering;
    Log() << " -- pivoting: leaving index=" << lvindex
          << "  entering index=" << entering << std::endl;
  }

  Log() << " -- maximum number of iterations exceeded (n=" << n
        << ", max=" << max_iter << ")" << std::endl;
  Log() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
  return false;
}

/// Lemke's Algorithm for solving LCPs in the matrix class E, which contains
/// all strictly semimonotone matrices, all P-matrices, and all strictly
/// copositive matrices. Lemke's Algorithm is described in [Cottle 1992],
/// Section 4.4.
///
/// This implementation wraps that algorithm with a Tikhonov-type
/// regularization approach. Specifically, this implementation repeatedly
/// attempts to solve the LCP:<pre>
/// (M + Iα)z + q = w
/// z ≥ 0
/// w ≥ 0
/// zᵀw = 0
/// </pre>
/// where I is the identity matrix and α ≪ 1, using geometrically increasing
/// values of α, until the LCP is solved. See SolveLcpFastRegularized() for
/// description of the regularization process and the function parameters,
/// which are identical. See SolveLcpLemke() for a description of Lemke's
/// Algorithm. See SolveLcpFastRegularized() for a description of all
/// calling parameters other than @p z, which apply equally well to this
/// function.
/// @param[in,out] z the solution to the LCP on return (if the solver succeeds).
///                If the length of z is equal to the length of q, the solver
///                will attempt to use z's value as a starting solution. **This
///                warmstarting is generally not recommended**: it has a
///                predisposition to lead to a failing pivoting sequence.
///
/// @sa SolveLcpFastRegularized()
/// @sa SolveLcpLemke()
///
/// [Cottle 1992]      R. Cottle, J.-S. Pang, and R. Stone. The Linear
///                    Complementarity Problem. Academic Press, 1992.
template <class T>
bool MobyLCPSolver<T>::SolveLcpLemkeRegularized(const MatrixX<T>& M,
                                                const VectorX<T>& q,
                                                VectorX<T>* z, int min_exp,
                                                unsigned step_exp, int max_exp,
                                                T piv_tol,
                                                T zero_tol) const {
  Log() << "MobyLCPSolver::SolveLcpLemkeRegularized() entered" << std::endl;

  // look for fast exit
  if (q.size() == 0) {
    z->resize(0);
    return true;
  }

  // copy MM
  MM_ = M;

  // assign value for zero tolerance, if necessary
  T naive_tol = q.size() * M.template lpNorm<Eigen::Infinity>() * sqrt_eps;
  const T ZERO_TOL = (zero_tol > 0) ? zero_tol : naive_tol;

  Log() << " zero tolerance: " << ZERO_TOL << std::endl;

  // store the total pivots
  unsigned total_piv = 0;

  // try non-regularized version first
  bool result = SolveLcpLemke(MM_, q, z, piv_tol, ZERO_TOL);
  if (result) {
    // verify that solution truly is a solution -- check z
    if (z->minCoeff() >= -ZERO_TOL) {
      // check w
      wx_ = (M * (*z)) + q;
      if (wx_.minCoeff() >= -ZERO_TOL) {
        // Check element-wise operation of z*wx_.
        wx_ = z->array() * wx_.eval().array();

        const T wx_min = wx_.minCoeff();
        const T wx_max = wx_.maxCoeff();
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
    result = SolveLcpLemke(MM_, q, z, piv_tol, ZERO_TOL);

    // update total pivots
    total_piv += pivots_;

    if (result) {
      // verify that solution truly is a solution -- check z
      if (z->minCoeff() > -ZERO_TOL) {
        // check w
        wx_ = (MM_ * (*z)) + q;
        if (wx_.minCoeff() > -ZERO_TOL) {
          // Check element-wise operation of z*wx_.
          wx_ = z->array() * wx_.eval().array();

          const T wx_min = wx_.minCoeff();
          const T wx_max = wx_.maxCoeff();
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

/// Lemke's Algorithm for solving LCPs in the matrix class E, which contains
/// all strictly semimonotone matrices, all P-matrices, and all strictly
/// copositive matrices, for the special case of sparse matrices. See
/// the non-sparse version of SolveLcpLemke() for descriptions of the calling
/// and return parameters.
template <typename T>
bool MobyLCPSolver<T>::SolveLcpLemke(const Eigen::SparseMatrix<double>& M,
                                     const Eigen::VectorXd& q,
                                     Eigen::VectorXd* z,
                                     double piv_tol, double zero_tol) const {
  Eigen::VectorXd x, dl, result, z0, xj, dj, u, Be;

  if (log_enabled_) {
    Log() << "MobyLCPSolver::SolveLcpLemke() entered" << std::endl;
    Log() << "  M: " << std::endl << M;
    Log() << "  q: " << q << std::endl;
  }

  const unsigned n = q.size();
  const unsigned max_iter = std::min((unsigned)1000, 50 * n);

  if (M.rows() != n || M.cols() != n)
    throw std::logic_error("M's dimensions do not match that of q.");

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
    for (unsigned i = 0; i < n; i++) {
      nonbas_.push_back(i);
    }
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
  if (solver->info() != Eigen::ComputationInfo::Success) {
    // Initial basis was singular (and a poor choice). Try a known good one.
    ClearIndexVectors();
    for (unsigned i = 0; i < n; i++)
      nonbas_.push_back(i);
    sBl_.setIdentity();
    sBl_ *= -1;
    solver->analyzePattern(sBl_);
    solver->factorize(sBl_);
    DRAKE_DEMAND(solver->info() == Eigen::ComputationInfo::Success);
  }
  x = solver->solve(q);
  x *= -1;

  // check whether initial basis provides a solution
  if (x.minCoeff() >= 0.0) {
    Log() << " -- initial basis provides a solution!" << std::endl;
    FinishLemkeSolution(M, q, x, z);
    Log() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
    return true;
  }

  // determine initial leaving variable
  Eigen::Index min_x;
  const double min_x_val = x.topRows(n).minCoeff(&min_x);
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
  u.resize(n);
  for (unsigned i = 0; i < n; i++) {
    u[i] = (x[i] < 0.0) ? 1.0 : 0.0;
  }
  Be = (sBl_ * u) * -1;
  u *= tval;
  x += u;
  x[lvindex] = tval;
  sBl_.col(lvindex) = makeSparseVector(Be);
  Log() << "  new q: " << x << std::endl;

  // main iterations begin here
  for (pivots_ = 0; pivots_ < max_iter; pivots_++) {
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
    solver.reset(new Eigen::SparseLU<Eigen::SparseMatrix<double>>);
    solver->analyzePattern(sBl_);
    solver->factorize(sBl_);
    if (solver->info() != Eigen::ComputationInfo::Success)
      return false;
    dl = solver->solve(Be);

    // use a new pivot tolerance if necessary
    const double PIV_TOL = (piv_tol > static_cast<double>(0.0)) ? piv_tol : (
        std::numeric_limits<double>::epsilon() * n *
            std::max(1.0, Be.lpNorm<Eigen::Infinity>()));

    // ** find new leaving variable
    j_.clear();
    for (unsigned i = 0; i < dl.size(); i++) {
      if (dl[i] > PIV_TOL) {
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

    Log() << " -- column of M': " << dl << std::endl;

    // select elements j from x and d
    selectSubVec(x, j_, &xj);
    selectSubVec(dl, j_, &dj);

    // compute minimal ratios x(j) + EPS_DOUBLE ./ d(j), d > 0
    result.resize(xj.size());
    result.fill(zero_tol);
    result = xj.eval().array() + result.array();
    result = result.eval().array() / dj.array();
    double theta = result.minCoeff();

    // NOTE: lexicographic ordering does not appear to be used here to prevent
    // cycling (see [Cottle 1992], pp. 340-342)
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
    double ratio = x[lvindex] / dl[lvindex];
    dl *= ratio;
    x -= dl;
    x[lvindex] = ratio;
    sBl_.col(lvindex) = makeSparseVector(Be);
    *iiter = entering;
    Log() << " -- pivoting: leaving index=" << lvindex
          << "  entering index=" << entering << std::endl;
  }

  Log() << " -- maximum number of iterations exceeded" << std::endl;
  Log() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
  return false;
}

/// Regularized wrapper around Lemke's Algorithm for solving LCPs in the matrix
/// class E. See the non-sparse version of SolveLcpLemkeRegularized() for
/// descriptions of the calling and return parameters.
template <typename T>
bool MobyLCPSolver<T>::SolveLcpLemkeRegularized(
    const Eigen::SparseMatrix<double>& M, const Eigen::VectorXd& q,
    Eigen::VectorXd* z, int min_exp, unsigned step_exp, int max_exp,
    double piv_tol, double zero_tol) const {
  Eigen::VectorXd x, wx;

  Log() << "MobyLCPSolver::SolveLcpLemkeRegularized() entered" << std::endl;

  // look for fast exit
  if (q.size() == 0) {
    z->resize(0);
    return true;
  }

  // copy MM
  MMs_ = M;

  // assign value for zero tolerance, if necessary
  const double ZERO_TOL = (zero_tol > 0) ? zero_tol : q.size() * sqrt_eps;

  // try non-regularized version first
  bool result = SolveLcpLemke(MMs_, q, z, piv_tol, ZERO_TOL);
  if (result) {
    // verify that solution truly is a solution -- check z
    if (z->minCoeff() >= -ZERO_TOL) {
      // check w
      wx = (M * (*z)) + q;
      if (wx.minCoeff() >= -ZERO_TOL) {
        // Check element-wise operation of z*wx.
        wx = z->array() * wx.eval().array();
        const double wx_min = wx.minCoeff();
        const double wx_max = wx.maxCoeff();
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
        wx = (MMx_ * (*z)) + q;
        if (wx.minCoeff() > -ZERO_TOL) {
          // Check element-wise operation of z*wx_.
          wx_ = z->array() * wx_.eval().array();
          if (wx.minCoeff() > -ZERO_TOL && wx.maxCoeff() < ZERO_TOL) {
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

// TODO(edrumwri): Move all code above to moby_lcp_solver-inl.h once it has
// been appropriately reviewed.
template class MobyLCPSolver<double>;

template class
    drake::solvers::MobyLCPSolver<Eigen::AutoDiffScalar<drake::Vector1d>>;

}  // namespace solvers
}  // namespace drake
