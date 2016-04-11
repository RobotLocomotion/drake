// Adapted with permission from code by Evan Drumwright
// (https://github.com/edrumwri).

#include "MobyLCP.h"

#include <Eigen/LU>
#include <Eigen/SparseCore>
#include <Eigen/SparseLU>

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "Optimization.h"

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
  assert(in.cols() == 1);
  assert(out->cols() == 1);
  assert(in.rows() == out->rows());
  for (int i = 0; i < in.rows(); i++) {
    (*out)(i) = func(in(i), (*out)(i));
  }
}

template <typename Derived>
Eigen::SparseVector<double> makeSparseVector(
    const Eigen::MatrixBase<Derived>& in) {
  assert(in.cols() == 1);
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
}

namespace Drake {

// Sole constructor
MobyLCPSolver::MobyLCPSolver() : log_enabled_(false) {}

void MobyLCPSolver::SetLoggingEnabled(bool enabled) { log_enabled_ = enabled; }

std::ostream& MobyLCPSolver::LOG() const {
  if (log_enabled_) {
    return std::cerr;
  }
  return null_stream_;
}

void MobyLCPSolver::ClearIndexVectors() const {
  // clear all vectors
  _all.clear();
  _tlist.clear();
  _bas.clear();
  _nonbas.clear();
  _j.clear();
}

bool MobyLCPSolver::Solve(OptimizationProblem& prog) const {
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

  assert(prog.generic_constraints().empty());
  assert(prog.generic_objectives().empty());
  assert(prog.GetAllLinearConstraints().empty());
  assert(prog.bounding_box_constraints().empty());

  const auto& bindings = prog.linear_complementarity_constraints();

  // Assert that the available LCPs cover the program and no two LCPs cover
  // the same variable.
  for (int i = 0; i < prog.num_vars(); i++) {
    bool coverings = 0;
    for (const auto& binding : bindings) {
      if (binding.covers(i)) {
        coverings++;
      }
    }
    assert(coverings == 1);
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
  for (const auto& binding : bindings) {
    Eigen::VectorXd constraint_solution(binding.GetNumElements());
    const std::shared_ptr<LinearComplementarityConstraint> constraint =
        binding.constraint();
    bool solved = SolveLcpLemkeRegularized(
        constraint->M(), constraint->q(), &constraint_solution);
    if (!solved) {
      return false;
    }
    binding.WriteThrough(constraint_solution, &solution);
  }
  prog.SetDecisionVariableValues(solution);
  return true;
}

/// Fast pivoting algorithm for denerate, monotone LCPs with few nonzero,
/// nonbasic variables
bool MobyLCPSolver::SolveLcpFast(const Eigen::MatrixXd& M,
                                 const Eigen::VectorXd& q, Eigen::VectorXd* z,
                                 double zero_tol) const {
  const unsigned N = q.rows();
  const unsigned UINF = std::numeric_limits<unsigned>::max();

  LOG() << "MobyLCPSolver::SolveLcpFast() entered" << std::endl;

  // look for trivial solution
  if (N == 0) {
    LOG() << "MobyLCPSolver::SolveLcpFast() - empty problem" << std::endl;
    z->resize(0);
    return true;
  }

  // set zero tolerance if necessary
  if (zero_tol < 0.0) {
    zero_tol = M.rows() * M.lpNorm<Eigen::Infinity>() *
               std::numeric_limits<double>::epsilon();
  }

  // prepare to setup basic and nonbasic variable indices for z
  _nonbas.clear();
  _bas.clear();

  // see whether to warm-start
  if (z->size() == q.size()) {
    LOG() << "MobyLCPSolver::SolveLcpFast() - warm starting activated"
          << std::endl;

    for (unsigned i = 0; i < z->size(); i++) {
      if (std::fabs((*z)[i]) < zero_tol) {
        _bas.push_back(i);
      } else {
        _nonbas.push_back(i);
      }
    }

    if (log_enabled_) {
      std::ostringstream str;
      str << " -- non-basic indices:";
      for (unsigned i = 0; i < _nonbas.size(); i++) str << " " << _nonbas[i];
      LOG() << str.str() << std::endl;
    }
  } else {
    // get minimum element of q (really w)
    Eigen::Index minw;
    const double minw_val = q.minCoeff(&minw);
    if (minw_val > -zero_tol) {
      LOG() << "MobyLCPSolver::SolveLcpFast() - trivial solution found"
            << std::endl;
      z->resize(N);
      z->fill(0);
      return true;
    }

    // setup basic and nonbasic variable indices
    _nonbas.push_back(minw);
    _bas.resize(N - 1);
    for (unsigned i = 0, j = 0; i < N; i++) {
      if (i != minw) {
        _bas[j++] = i;
      }
    }
  }

  // loop for maximum number of pivots
  //  const unsigned MAX_PIV = std::max(N*N, (unsigned) 1000);
  const unsigned MAX_PIV = 2 * N;
  for (pivots = 0; pivots < MAX_PIV; pivots++) {
    // select nonbasic indices
    selectSubMat(M, _nonbas, _nonbas, &_Msub);
    selectSubMat(M, _bas, _nonbas, &_Mmix);
    selectSubVec(q, _nonbas, &_z);
    selectSubVec(q, _bas, &_qbas);
    // _z.negate();
    _z = _z * -1;

    // solve for nonbasic z
    _z = _Msub.lu().solve(_z);

    // compute w and find minimum value
    _w = _Mmix * _z;
    _w += _qbas;
    unsigned minw = (_w.rows() > 0) ? minCoeffIdx(_w) : UINF;

    // TODO(sammy-tri) this log can't print when minw is UINF.
    // LOG() << "MobyLCPSolver::SolveLcpFast() - minimum w after pivot: "
    // << _w[minw] << std::endl;

    // if w >= 0, check whether any component of z < 0
    if (minw == UINF || _w[minw] > -zero_tol) {
      // find the (a) minimum of z
      unsigned minz = (_z.rows() > 0) ? minCoeffIdx(_z) : UINF;
      if (log_enabled_ && _z.rows() > 0) {
        LOG() << "MobyLCPSolver::SolveLcpFast() - minimum z after pivot: "
              << _z[minz] << std::endl;
      }
      if (minz < UINF && _z[minz] < -zero_tol) {
        // get the original index and remove it from the nonbasic set
        unsigned idx = _nonbas[minz];
        _nonbas.erase(_nonbas.begin() + minz);

        // move index to basic set and continue looping
        _bas.push_back(idx);
        std::sort(_bas.begin(), _bas.end());
      } else {
        // found the solution
        z->resize(N);
        z->fill(0);

        // set values of z corresponding to _z
        for (unsigned i = 0, j = 0; j < _nonbas.size(); i++, j++) {
          (*z)[_nonbas[j]] = _z[i];
        }

        LOG() << "MobyLCPSolver::SolveLcpFast() - solution found!" << std::endl;
        return true;
      }
    } else {
      LOG() << "(minimum w too negative)" << std::endl;

      // one or more components of w violating w >= 0
      // move component of w from basic set to nonbasic set
      unsigned idx = _bas[minw];
      _bas.erase(_bas.begin() + minw);
      _nonbas.push_back(idx);
      std::sort(_nonbas.begin(), _nonbas.end());

      // look whether any component of z needs to move to basic set
      unsigned minz = (_z.rows() > 0) ? minCoeffIdx(_z) : UINF;
      if (log_enabled_ && _z.rows() > 0) {
        LOG() << "MobyLCPSolver::SolveLcpFast() - minimum z after pivot: "
              << _z[minz] << std::endl;
      }
      if (minz < UINF && _z[minz] < -zero_tol) {
        // move index to basic set and continue looping
        unsigned idx = _nonbas[minz];
        LOG() << "MobyLCPSolver::SolveLcpFast() - moving index " << idx
              << " to basic set" << std::endl;

        _nonbas.erase(_nonbas.begin() + minz);
        _bas.push_back(idx);
        std::sort(_bas.begin(), _bas.end());
      }
    }
  }

  LOG() << "MobyLCPSolver::SolveLcpFast() - maximum allowable pivots exceeded"
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
  LOG() << "MobyLCPSolver::SolveLcpFastRegularized() entered" << std::endl;

  // look for fast exit
  if (q.size() == 0) {
    z->resize(0);
    return true;
  }

  // copy MM
  _MM = M;

  // assign value for zero tolerance, if necessary
  const double ZERO_TOL =
      (zero_tol > static_cast<double>(0.0))
          ? zero_tol
          : (q.size() * M.lpNorm<Eigen::Infinity>() * NEAR_ZERO);

  LOG() << " zero tolerance: " << ZERO_TOL << std::endl;

  // store the total pivots
  unsigned total_piv = 0;

  // try non-regularized version first
  bool result = SolveLcpFast(_MM, q, z, zero_tol);
  if (result) {
    // verify that solution truly is a solution -- check z
    if (z->minCoeff() >= -ZERO_TOL) {
      // check w
      _wx = (M * (*z)) + q;
      if (_wx.minCoeff() >= -ZERO_TOL) {
        // check z'w
        transformVecElements(*z, &_wx, std::multiplies<double>());
        const double wx_min = _wx.minCoeff();
        const double wx_max = _wx.maxCoeff();

        if (wx_min >= -ZERO_TOL && wx_max < ZERO_TOL) {
          LOG() << "  solved with no regularization necessary!" << std::endl;
          LOG() << "  pivots / total pivots: " << pivots << " " << pivots
                << std::endl;
          LOG() << "MobyLCPSolver::SolveLcpFastRegularized() exited"
                << std::endl;

          return true;
        } else {
          LOG() << "MobyLCPSolver::SolveLcpFastRegularized() - "
                << "'<w, z> not within tolerance(min value: " << wx_min
                << " max value: " << wx_max << ")" << std::endl;
        }
      } else {
        LOG() << "  MobyLCPSolver::SolveLcpFastRegularized() - "
              << "'w' not solved to desired tolerance" << std::endl;
        LOG() << "  minimum w: " << _wx.minCoeff() << std::endl;
      }
    } else {
      LOG() << "  MobyLCPSolver::SolveLcpFastRegularized() - "
            << "'z' not solved to desired tolerance" << std::endl;
      LOG() << "  minimum z: " << z->minCoeff() << std::endl;
    }
  } else {
    LOG() << "  MobyLCPSolver::SolveLcpFastRegularized() "
          << "- solver failed with zero regularization" << std::endl;
  }

  // update the pivots
  total_piv += pivots;

  // start the regularization process
  int rf = min_exp;
  while (rf < max_exp) {
    // setup regularization factor
    double lambda =
        std::pow(static_cast<double>(10.0), static_cast<double>(rf));

    LOG() << "  trying to solve LCP with regularization factor: " << lambda
          << std::endl;

    // regularize M
    _MM = M;
    for (unsigned i = 0; i < M.rows(); i++) {
      _MM(i, i) += lambda;
    }

    // try to solve the LCP
    result = SolveLcpFast(_MM, q, z, zero_tol);

    // update total pivots
    total_piv += pivots;

    if (result) {
      // verify that solution truly is a solution -- check z
      if (z->minCoeff() > -ZERO_TOL) {
        // check w
        _wx = (_MM * (*z)) + q;
        if (_wx.minCoeff() > -ZERO_TOL) {
          // check z'w
          transformVecElements(*z, &_wx, std::multiplies<double>());
          const double wx_min = _wx.minCoeff();
          const double wx_max = _wx.maxCoeff();

          if (wx_min > -ZERO_TOL && wx_max < ZERO_TOL) {
            LOG() << "  solved with regularization factor: " << lambda
                  << std::endl;
            LOG() << "  pivots / total pivots: " << pivots << " " << total_piv
                  << std::endl;
            LOG() << "MobyLCPSolver::SolveLcpFastRegularized() exited"
                  << std::endl;
            pivots = total_piv;
            return true;
          } else {
            LOG() << "MobyLCPSolver::SolveLcpFastRegularized() - "
                  << "'<w, z> not within tolerance(min value: " << wx_min
                  << " max value: " << wx_max << ")" << std::endl;
          }
        } else {
          LOG() << "  MobyLCPSolver::SolveLcpFastRegularized() - "
                << "'w' not solved to desired tolerance" << std::endl;
          LOG() << "  minimum w: " << _wx.minCoeff() << std::endl;
        }
      } else {
        LOG() << "  MobyLCPSolver::SolveLcpFastRegularized() - "
              << "'z' not solved to desired tolerance" << std::endl;
        LOG() << "  minimum z: " << z->minCoeff() << std::endl;
      }
    }

    // increase rf
    rf += step_exp;
  }

  LOG() << "  unable to solve given any regularization!" << std::endl;
  LOG() << "MobyLCPSolver::SolveLcpFastRegularized() exited" << std::endl;

  // store total pivots
  pivots = total_piv;

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
  for (idx = 0, iiter = _bas.begin(); iiter != _bas.end(); iiter++, idx++) {
    (*z)(*iiter) = _x(idx);
  }

  // TODO(sammy-tri) Is there a more efficient way to resize and
  // preserve the data?
  z->conservativeResize(q.size());

  // check to see whether tolerances are satisfied
  if (log_enabled_) {
    _wl = (M * (*z)) + q;
    const double minw = _wl.minCoeff();
    const double w_dot_z = std::fabs(_wl.dot(*z));
    LOG() << "  z: " << z << std::endl;
    LOG() << "  _w: " << _wl << std::endl;
    LOG() << "  minimum w: " << minw << std::endl;
    LOG() << "  w'z: " << w_dot_z << std::endl;
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
    LOG() << "MobyLCPSolver::SolveLcpLemke() entered" << std::endl;
    LOG() << "  M: " << std::endl << M;
    LOG() << "  q: " << q << std::endl;
  }

  const unsigned n = q.size();
  const unsigned MAXITER = std::min((unsigned)1000, 50 * n);

  // update the pivots
  pivots = 0;

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
    LOG() << " -- trivial solution found" << std::endl;
    LOG() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
    return true;
  }

  // Lemke's algorithm doesn't seem to like warmstarting
  //
  // TODO(sammy-tri) this is not present in the sparse solver, and it
  // causes subtle dead code below.
  z->fill(0);

  // copy z to z0
  _z0 = *z;

  ClearIndexVectors();

  // initialize variables
  z->resize(n * 2);
  z->fill(0);
  unsigned t = 2 * n;
  unsigned entering = t;
  unsigned leaving = 0;
  for (unsigned i = 0; i < n; i++) {
    _all.push_back(i);
  }
  unsigned lvindex;
  unsigned idx;
  std::vector<unsigned>::iterator iiter;

  // determine initial basis
  if (_z0.size() != n) {
    // setup the nonbasic indices
    for (unsigned i = 0; i < n; i++) _nonbas.push_back(i);
  } else {
    for (unsigned i = 0; i < n; i++) {
      if (_z0[i] > 0) {
        _bas.push_back(i);
      } else {
        _nonbas.push_back(i);
      }
    }
  }

  // determine initial values
  if (!_bas.empty()) {
    LOG() << "-- initial basis not empty (warmstarting)" << std::endl;

    // start from good initial basis
    _Bl.resize(n, n);
    _Bl.setIdentity();
    // _Bl.negate();
    _Bl *= -1;

    // select columns of M corresponding to z vars in the basis
    selectSubMat(M, _all, _bas, &_t1);

    // select columns of I corresponding to z vars not in the basis
    selectSubMat(_Bl, _all, _nonbas, &_t2);

    // setup the basis matrix
    _Bl.resize(n, _t1.cols() + _t2.cols());
    _Bl.block(0, 0, _t1.rows(), _t1.cols()) = _t1;
    _Bl.block(0, _t1.cols(), _t2.rows(), _t2.cols()) = _t2;

    // solve B*x = -q
    //
    // The original version of this code from Moby could handle the
    // case where the colver failed (though only in this dense
    // implementation).  It's not obvious how to do this from Eigen,
    // so we've (sam.creasey) assumed that the solve succeeds.
    _Al = _Bl;
    _x = _Al.lu().solve(q);
  } else {
    LOG() << "-- using basis of -1 (no warmstarting)" << std::endl;

    // use standard initial basis
    _Bl.resize(n, n);
    _Bl.setIdentity();
    _Bl *= -1;
    _x = q;
  }

  // check whether initial basis provides a solution
  if (_x.minCoeff() >= 0.0) {
    LOG() << " -- initial basis provides a solution!" << std::endl;
    FinishLemkeSolution(M, q, z);
    LOG() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
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
  const double min_x_val = _x.topRows(n).minCoeff(&min_x);
  double tval = -min_x_val;
  for (int i = 0; i < _nonbas.size(); i++) {
    _bas.push_back(_nonbas[i] + n);
  }
  lvindex = min_x;
  iiter = _bas.begin();
  std::advance(iiter, lvindex);
  leaving = *iiter;
  LOG() << " -- x: " << _x << std::endl;
  LOG() << " -- first pivot: leaving index=" << lvindex
        << "  entering index=" << entering << " minimum value: " << tval
        << std::endl;

  // pivot in the artificial variable
  *iiter = t;  // replace w var with _z0 in basic indices
  _u.resize(n);
  for (unsigned i = 0; i < n; i++) {
    _u[i] = (_x[i] < 0.0) ? 1.0 : 0.0;
  }
  _Be = (_Bl * _u) * -1;
  _u *= tval;
  _x += _u;
  _x[lvindex] = tval;
  _Bl.col(lvindex) = _Be;
  LOG() << "  new q: " << _x << std::endl;

  // main iterations begin here
  for (pivots = 0; pivots < MAXITER; pivots++) {
    if (log_enabled_) {
      std::ostringstream basic;
      for (unsigned i = 0; i < _bas.size(); i++) {
        basic << " " << _bas[i];
      }
      LOG() << "basic variables:" << basic.str() << std::endl;
      LOG() << "leaving: " << leaving << " t:" << t << std::endl;
    }

    // check whether done; if not, get new entering variable
    if (leaving == t) {
      LOG() << "-- solved LCP successfully!" << std::endl;
      FinishLemkeSolution(M, q, z);
      LOG() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
      return true;
    } else if (leaving < n) {
      entering = n + leaving;
      _Be.resize(n);
      _Be.fill(0);
      _Be[leaving] = -1;
    } else {
      entering = leaving - n;
      _Be = M.col(entering);
    }
    _dl = _Be;

    // Again, The original version of this code from Moby could handle
    // the case where the solver failed.  There was also some
    // commented out code related to trying to restart on finding a
    // solution.  For the reasons above, and the fact that it was
    // commented out, this functionality has not been preserved.
    _Al = _Bl;
    _dl = _Al.lu().solve(_dl);

    // ** find new leaving variable
    _j.clear();
    for (unsigned i = 0; i < _dl.size(); i++) {
      if (_dl[i] > PIV_TOL) {
        _j.push_back(i);
      }
    }

    // check for no new pivots; ray termination
    if (_j.empty()) {
      LOG()
          << "MobyLCPSolver::SolveLcpLemke() - no new pivots (ray termination)"
          << std::endl;
      LOG() << "MobyLCPSolver::SolveLcpLemke() exiting" << std::endl;
      return false;
    }

    if (log_enabled_) {
      std::ostringstream j;
      for (unsigned i = 0; i < _j.size(); i++) j << " " << _j[i];
      LOG() << "d: " << _dl << std::endl;
      LOG() << "j (before min ratio):" << j.str() << std::endl;
    }

    // select elements j from x and d
    selectSubVec(_x, _j, &_xj);
    selectSubVec(_dl, _j, &_dj);

    // compute minimal ratios x(j) + EPS_DOUBLE ./ d(j), d > 0
    _result.resize(_xj.size());
    _result.fill(zero_tol);
    transformVecElements(_xj, &_result, std::plus<double>());
    transformVecElements(_dj, &_result,
                         [](double a, double b) { return b / a; });
    double theta = _result.minCoeff();

    // NOTE: lexicographic ordering does not appear to be used here to prevent
    // cycling (see [Cottle 1992], pp. 340-342)
    // find indices of minimal ratios, d> 0
    //   divide _x(j) ./ d(j) -- remove elements above the minimum ratio
    for (int i = 0; i < _result.size(); i++) {
      _result(i) = _xj(i) / _dj(i);
    }

    for (iiter = _j.begin(), idx = 0; iiter != _j.end();) {
      if (_result[idx++] <= theta) {
        iiter++;
      } else {
        iiter = _j.erase(iiter);
      }
    }
    if (log_enabled_) {
      std::ostringstream j;
      for (unsigned i = 0; i < _j.size(); i++) {
        j << " " << _j[i];
      }
      LOG() << "j (after min ratio):" << j.str() << std::endl;
    }

    // if j is empty, then likely the zero tolerance is too low
    if (_j.empty()) {
      LOG() << "zero tolerance too low?" << std::endl;
      LOG() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
      return false;
    }

    // check whether artificial index among these
    _tlist.clear();
    for (int i = 0; i < _j.size(); i++) {
      _tlist.push_back(_bas[_j[i]]);
    }
    if (std::find(_tlist.begin(), _tlist.end(), t) != _tlist.end()) {
      iiter = std::find(_bas.begin(), _bas.end(), t);
      lvindex = iiter - _bas.begin();
    } else {
      // several indices pass the minimum ratio test, pick one randomly
      //      lvindex = _j[rand() % _j.size()];
      // NOTE: solver seems *much* more capable of solving when we pick the
      // first
      // element rather than picking a random one
      lvindex = _j[0];
    }

    // set leaving = bas(lvindex)
    iiter = _bas.begin();
    std::advance(iiter, lvindex);
    leaving = *iiter;

    // ** perform pivot
    double ratio = _x[lvindex] / _dl[lvindex];
    _dl *= ratio;
    _x -= _dl;
    _x[lvindex] = ratio;
    _Bl.col(lvindex) = _Be;
    *iiter = entering;
    LOG() << " -- pivoting: leaving index=" << lvindex
          << "  entering index=" << entering << std::endl;
  }

  LOG() << " -- maximum number of iterations exceeded (n=" << n
        << ", max=" << MAXITER << ")" << std::endl;
  LOG() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
  return false;
}

/// Regularized wrapper around Lemke's algorithm
bool MobyLCPSolver::SolveLcpLemkeRegularized(const Eigen::MatrixXd& M,
                                             const Eigen::VectorXd& q,
                                             Eigen::VectorXd* z, int min_exp,
                                             unsigned step_exp, int max_exp,
                                             double piv_tol,
                                             double zero_tol) const {
  LOG() << "MobyLCPSolver::SolveLcpLemkeRegularized() entered" << std::endl;

  // look for fast exit
  if (q.size() == 0) {
    z->resize(0);
    return true;
  }

  // copy MM
  _MM = M;

  // assign value for zero tolerance, if necessary
  const double ZERO_TOL =
      (zero_tol > static_cast<double>(0.0))
          ? zero_tol
          : (q.size() * M.lpNorm<Eigen::Infinity>() * NEAR_ZERO);

  LOG() << " zero tolerance: " << ZERO_TOL << std::endl;

  // store the total pivots
  unsigned total_piv = 0;

  // try non-regularized version first
  bool result = SolveLcpLemke(_MM, q, z, piv_tol, zero_tol);
  if (result) {
    // verify that solution truly is a solution -- check z
    if (z->minCoeff() >= -ZERO_TOL) {
      // check w
      _wx = (M * (*z)) + q;
      if (_wx.minCoeff() >= -ZERO_TOL) {
        // check z'w
        transformVecElements(*z, &_wx, std::multiplies<double>());
        const double wx_min = _wx.minCoeff();
        const double wx_max = _wx.maxCoeff();
        if (wx_min >= -ZERO_TOL && wx_max < ZERO_TOL) {
          LOG() << "  solved with no regularization necessary!" << std::endl;
          LOG() << "MobyLCPSolver::SolveLcpLemkeRegularized() exited"
                << std::endl;

          return true;
        } else {
          LOG() << "MobyLCPSolver::SolveLcpLemke() - "
                << "'<w, z> not within tolerance(min value: " << wx_min
                << " max value: " << wx_max << ")" << std::endl;
        }
      } else {
        LOG() << "  MobyLCPSolver::SolveLcpLemke() - 'w' not solved to desired "
                 "tolerance"
              << std::endl;
        LOG() << "  minimum w: " << _wx.minCoeff() << std::endl;
      }
    } else {
      LOG() << "  MobyLCPSolver::SolveLcpLemke() - 'z' not solved to desired "
               "tolerance"
            << std::endl;
      LOG() << "  minimum z: " << z->minCoeff() << std::endl;
    }
  }

  // update the pivots
  total_piv += pivots;

  // start the regularization process
  int rf = min_exp;
  while (rf < max_exp) {
    // setup regularization factor
    double lambda =
        std::pow(static_cast<double>(10.0), static_cast<double>(rf));

    LOG() << "  trying to solve LCP with regularization factor: " << lambda
          << std::endl;

    // regularize M
    _MM = M;
    for (unsigned i = 0; i < M.rows(); i++) {
      _MM(i, i) += lambda;
    }

    // try to solve the LCP
    result = SolveLcpLemke(_MM, q, z, piv_tol, zero_tol);

    // update total pivots
    total_piv += pivots;

    if (result) {
      // verify that solution truly is a solution -- check z
      if (z->minCoeff() > -ZERO_TOL) {
        // check w
        _wx = (_MM * (*z)) + q;
        if (_wx.minCoeff() > -ZERO_TOL) {
          // check z'w
          transformVecElements(*z, &_wx, std::multiplies<double>());
          const double wx_min = _wx.minCoeff();
          const double wx_max = _wx.maxCoeff();
          if (wx_min > -ZERO_TOL && wx_max < ZERO_TOL) {
            LOG() << "  solved with regularization factor: " << lambda
                  << std::endl;
            LOG() << "MobyLCPSolver::SolveLcpLemkeRegularized() exited"
                  << std::endl;
            pivots = total_piv;
            return true;
          } else {
            LOG() << "MobyLCPSolver::SolveLcpLemke() - "
                  << "'<w, z> not within tolerance(min value: " << wx_min
                  << " max value: " << wx_max << ")" << std::endl;
          }
        } else {
          LOG() << "  MobyLCPSolver::SolveLcpLemke() - 'w' not solved to "
                   "desired tolerance"
                << std::endl;
          LOG() << "  minimum w: " << _wx.minCoeff() << std::endl;
        }
      } else {
        LOG() << "  MobyLCPSolver::SolveLcpLemke() - 'z' not solved to desired "
                 "tolerance"
              << std::endl;
        LOG() << "  minimum z: " << z->minCoeff() << std::endl;
      }
    }

    // increase rf
    rf += step_exp;
  }

  LOG() << "  unable to solve given any regularization!" << std::endl;
  LOG() << "MobyLCPSolver::SolveLcpLemkeRegularized() exited" << std::endl;

  // store total pivots
  pivots = total_piv;

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
    LOG() << "MobyLCPSolver::SolveLcpLemke() entered" << std::endl;
    LOG() << "  M: " << std::endl << M;
    LOG() << "  q: " << q << std::endl;
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
    LOG() << " -- trivial solution found" << std::endl;
    LOG() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
    return true;
  }

  // copy z to z0
  _z0 = *z;

  ClearIndexVectors();

  // initialize variables
  z->resize(n * 2);
  z->fill(0);
  unsigned t = 2 * n;
  unsigned entering = t;
  unsigned leaving = 0;
  for (unsigned i = 0; i < n; i++) {
    _all.push_back(i);
  }
  unsigned lvindex;
  unsigned idx;
  std::vector<unsigned>::iterator iiter;

  // determine initial basis
  if (_z0.size() != n) {
    for (unsigned i = 0; i < n; i++) {
      _nonbas.push_back(i);
    }
  } else {
    for (unsigned i = 0; i < n; i++) {
      if (_z0[i] > 0) {
        _bas.push_back(i);
      } else {
        _nonbas.push_back(i);
      }
    }
  }

  // determine initial values
  _sBl = Eigen::SparseMatrix<double>(n, n);
  if (!_bas.empty()) {
    typedef Eigen::Triplet<double> Triplet;
    std::vector<Triplet> triplet_list;
    for (int i = 0; i < M.outerSize(); i++) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(M, i); it; ++it) {
        int j = it.col();
        std::vector<unsigned>::const_iterator j_it =
            std::find(_bas.begin(), _bas.end(), j);
        if (j_it == _bas.end()) {
          continue;
        } else {
          triplet_list.push_back(Triplet(i, j, it.value()));
        }
      }
    }
    for (int i = 0, j = _bas.size(); i < _nonbas.size(); i++, j++) {
      triplet_list.push_back(Triplet(_nonbas[i], j, 1.0));
    }

    _sBl.setFromTriplets(triplet_list.begin(), triplet_list.end());
  } else {
    _sBl.setIdentity();
    _sBl *= -1;
  }

  // solve B*x = -q
  std::unique_ptr<Eigen::SparseLU<Eigen::SparseMatrix<double>>> solver;
  solver.reset(new Eigen::SparseLU<Eigen::SparseMatrix<double>>);
  solver->analyzePattern(_sBl);
  solver->factorize(_sBl);
  _x = solver->solve(q);
  _x *= -1;

  // check whether initial basis provides a solution
  if (_x.minCoeff() >= 0.0) {
    LOG() << " -- initial basis provides a solution!" << std::endl;
    FinishLemkeSolution(M, q, z);
    LOG() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
    return true;
  }

  // determine initial leaving variable
  Eigen::Index min_x;
  const double min_x_val = _x.topRows(n).minCoeff(&min_x);
  double tval = -min_x_val;
  for (int i = 0; i < _nonbas.size(); i++) {
    _bas.push_back(_nonbas[i] + n);
  }
  lvindex = min_x;
  iiter = _bas.begin();
  std::advance(iiter, lvindex);
  leaving = *iiter;

  // pivot in the artificial variable
  *iiter = t;  // replace w var with _z0 in basic indices
  _u.resize(n);
  for (unsigned i = 0; i < n; i++) {
    _u[i] = (_x[i] < 0.0) ? 1.0 : 0.0;
  }
  _Be = (_sBl * _u) * -1;
  _u *= tval;
  _x += _u;
  _x[lvindex] = tval;
  _sBl.col(lvindex) = makeSparseVector(_Be);
  LOG() << "  new q: " << _x << std::endl;

  // main iterations begin here
  for (pivots = 0; pivots < MAXITER; pivots++) {
    // check whether done; if not, get new entering variable
    if (leaving == t) {
      LOG() << "-- solved LCP successfully!" << std::endl;
      FinishLemkeSolution(M, q, z);
      LOG() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
      return true;
    } else if (leaving < n) {
      entering = n + leaving;
      _Be.resize(n);
      _Be.fill(0);
      _Be[leaving] = -1;
    } else {
      entering = leaving - n;
      _Be = M.col(entering);
    }
    solver.reset(new Eigen::SparseLU<Eigen::SparseMatrix<double>>);
    solver->analyzePattern(_sBl);
    solver->factorize(_sBl);
    _dl = solver->solve(_Be);

    // use a new pivot tolerance if necessary
    const double PIV_TOL = (piv_tol > static_cast<double>(0.0))
                               ? piv_tol
                               : (std::numeric_limits<double>::epsilon() * n *
                                  std::max(static_cast<double>(1.0),
                                           _Be.lpNorm<Eigen::Infinity>()));

    // ** find new leaving variable
    _j.clear();
    for (unsigned i = 0; i < _dl.size(); i++) {
      if (_dl[i] > PIV_TOL) {
        _j.push_back(i);
      }
    }
    // check for no new pivots; ray termination
    if (_j.empty()) {
      LOG()
          << "MobyLCPSolver::SolveLcpLemke() - no new pivots (ray termination)"
          << std::endl;
      LOG() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
      return false;
    }

    LOG() << " -- column of M': " << _dl << std::endl;

    // select elements j from x and d
    selectSubVec(_x, _j, &_xj);
    selectSubVec(_dl, _j, &_dj);

    // compute minimal ratios x(j) + EPS_DOUBLE ./ d(j), d > 0
    _result.resize(_xj.size());
    _result.fill(zero_tol);
    transformVecElements(_xj, &_result, std::plus<double>());
    transformVecElements(_dj, &_result,
                         [](double a, double b) { return b / a; });
    double theta = _result.minCoeff();

    // NOTE: lexicographic ordering does not appear to be used here to prevent
    // cycling (see [Cottle 1992], pp. 340-342)
    // find indices of minimal ratios, d> 0
    //   divide _x(j) ./ d(j) -- remove elements above the minimum ratio
    for (int i = 0; i < _result.size(); i++) {
      _result(i) = _xj(i) / _dj(i);
    }
    for (iiter = _j.begin(), idx = 0; iiter != _j.end();) {
      if (_result[idx++] <= theta) {
        iiter++;
      } else {
        iiter = _j.erase(iiter);
      }
    }
    // if j is empty, then likely the zero tolerance is too low
    if (_j.empty()) {
      LOG() << "zero tolerance too low?" << std::endl;
      LOG() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
      return false;
    }

    // check whether artificial index among these
    _tlist.clear();
    for (int i = 0; i < _j.size(); i++) {
      _tlist.push_back(_bas[_j[i]]);
    }
    if (std::find(_tlist.begin(), _tlist.end(), t) != _tlist.end()) {
      iiter = std::find(_bas.begin(), _bas.end(), t);
      lvindex = iiter - _bas.begin();
    } else {
      // several indices pass the minimum ratio test, pick one randomly
      //      lvindex = _j[rand() % _j.size()];

      // NOTE: solver seems *much* more capable of solving when we pick the
      // first element rather than picking a random one
      lvindex = _j[0];
    }

    // set leaving = bas(lvindex)
    iiter = _bas.begin();
    std::advance(iiter, lvindex);
    leaving = *iiter;

    // ** perform pivot
    double ratio = _x[lvindex] / _dl[lvindex];
    _dl *= ratio;
    _x -= _dl;
    _x[lvindex] = ratio;
    _sBl.col(lvindex) = makeSparseVector(_Be);
    *iiter = entering;
    LOG() << " -- pivoting: leaving index=" << lvindex
          << "  entering index=" << entering << std::endl;
  }

  LOG() << " -- maximum number of iterations exceeded" << std::endl;
  LOG() << "MobyLCPSolver::SolveLcpLemke() exited" << std::endl;
  return false;
}

/// Regularized wrapper around Lemke's algorithm for srpase matrices
bool MobyLCPSolver::SolveLcpLemkeRegularized(
    const Eigen::SparseMatrix<double>& M, const Eigen::VectorXd& q,
    Eigen::VectorXd* z, int min_exp, unsigned step_exp, int max_exp,
    double piv_tol, double zero_tol) const {
  LOG() << "MobyLCPSolver::SolveLcpLemkeRegularized() entered" << std::endl;

  // look for fast exit
  if (q.size() == 0) {
    z->resize(0);
    return true;
  }

  // copy MM
  _MMs = M;

  // assign value for zero tolerance, if necessary
  const double ZERO_TOL =
      (zero_tol > static_cast<double>(0.0)) ? zero_tol : q.size() * NEAR_ZERO;

  // try non-regularized version first
  bool result = SolveLcpLemke(_MMs, q, z, piv_tol, zero_tol);
  if (result) {
    // verify that solution truly is a solution -- check z
    if (z->minCoeff() >= -ZERO_TOL) {
      // check w
      _wx = (M * (*z)) + q;
      if (_wx.minCoeff() >= -ZERO_TOL) {
        // check z'w
        transformVecElements(*z, &_wx, std::multiplies<double>());
        const double wx_min = _wx.minCoeff();
        const double wx_max = _wx.maxCoeff();
        if (wx_min >= -ZERO_TOL && wx_max < ZERO_TOL) {
          LOG() << "  solved with no regularization necessary!" << std::endl;
          LOG() << "MobyLCPSolver::SolveLcpLemkeRegularized() exited"
                << std::endl;

          return true;
        } else {
          LOG() << "MobyLCPSolver::SolveLcpLemke() - "
                << "'<w, z> not within tolerance(min value: " << wx_min
                << " max value: " << wx_max << ")"
                << " tol " << ZERO_TOL << std::endl;
        }
      }
    }
  }

  _eye.resize(M.rows(), M.cols());
  _eye.setIdentity();

  // start the regularization process
  int rf = min_exp;
  while (rf < max_exp) {
    // setup regularization factor
    double lambda =
        std::pow(static_cast<double>(10.0), static_cast<double>(rf));
    (_diag_lambda = _eye) *= lambda;

    // regularize M
    (_MMx = _MMs) += _diag_lambda;

    // try to solve the LCP
    if ((result = SolveLcpLemke(_MMx, q, z, piv_tol, zero_tol))) {
      // verify that solution truly is a solution -- check z
      if (z->minCoeff() > -ZERO_TOL) {
        // check w
        _wx = (_MMx * (*z)) + q;
        if (_wx.minCoeff() > -ZERO_TOL) {
          // check z'w
          transformVecElements(*z, &_wx, std::multiplies<double>());
          if (_wx.minCoeff() > -ZERO_TOL && _wx.maxCoeff() < ZERO_TOL) {
            LOG() << "  solved with regularization factor: " << lambda
                  << std::endl;
            LOG() << "MobyLCPSolver::SolveLcpLemkeRegularized() exited"
                  << std::endl;

            return true;
          }
        }
      }
    }

    // increase rf
    rf += step_exp;
  }

  LOG() << "  unable to solve given any regularization!" << std::endl;
  LOG() << "MobyLCPSolver::SolveLcpLemkeRegularized() exited" << std::endl;

  // still here?  failure...
  return false;
}
}
