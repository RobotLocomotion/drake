// Adapted with permission from code by Evan Drumwright
// (https://github.com/edrumwri).

#include <Eigen/LU>
#include <Eigen/SparseCore>
#include <Eigen/SparseLU>

#include <algorithm>
#include <cstring>
#include <cmath>
#include <cfloat>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <sstream>
#include <utility>
#include <vector>

#include "MobyLCP.h"

using std::pair;
using std::map;
using std::make_pair;

namespace {
// TODO sammy these helpers probably have some faster, more Eigen way
// to do this than I've come up with.  This implementation, anyway, is
// definitely slow.  It's an attempt at rewriting some Ravelin
// functionality.  Also, the only reason that the out paramater is a
// MatrixXd is so that this function can resize it.  Runtime
// performance may be improved by allowing the caller to create a
// fixed-size matrix and pass that in instead.
template <typename Derived>
void selectSubMat(const Eigen::MatrixBase<Derived>& in,
                  const std::vector<unsigned>& rows,
                  const std::vector<unsigned>& cols,
                  Eigen::MatrixXd* out) {
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

// TODO sammy this could also use a more efficient implementation.
template <typename Derived>
void selectSubVec(const Eigen::MatrixBase<Derived>& in,
                  const std::vector<unsigned>& rows,
                  Eigen::VectorXd* out) {
  const int num_rows = rows.size();
  out->resize(num_rows);
  for (int i = 0; i < num_rows; i++) {
    (*out)(i) = in(rows[i]);
  }
}

template <typename DerivedA, typename DerivedB, typename F>
void transformVecElements(const Eigen::MatrixBase<DerivedA>& in,
                          Eigen::MatrixBase<DerivedB>* out,
                          F func) {
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

/// Get the minimum index of vector v; if there are multiple minima (within
/// zero_tol), returns one randomly
unsigned rand_min(const Eigen::VectorXd& v, double zero_tol) {
  std::vector<unsigned> minima;
  Eigen::Index minv;
  const unsigned minv_val = v.minCoeff(&minv);
  minima.push_back(minv);
  for (unsigned i=0; i< v.rows(); i++) {
    if (i != minv && v[i] < minv_val + zero_tol) {
      minima.push_back(i);
    }
  }
  return minima[rand() % minima.size()];
}

const double NEAR_ZERO = std::sqrt(std::numeric_limits<double>::epsilon());
}

namespace Drake {

// TODO DEFECT ggould replace this with a proper logging hookup
#define LOG std::cerr
#define LOGGING(x) (true)
#define LOG_OPT 0

// Sole constructor
LCP::LCP() {
}

/// Fast pivoting algorithm for denerate, monotone LCPs with few nonzero,
/// nonbasic variables
bool LCP::lcp_fast(const MatrixNd& M, const VectorNd& q, VectorNd* z,
                   double zero_tol) {
  const unsigned N = q.rows();
  const unsigned UINF = std::numeric_limits<unsigned>::max();

  LOG << "LCP::lcp_fast() entered" << std::endl;

  // look for trivial solution
  if (N == 0) {
    LOG << "LCP::lcp_fast() - empty problem" << std::endl;
    z->resize(0);
    return true;
  }

  // set zero tolerance if necessary
  if (zero_tol < 0.0) {
    zero_tol = M.rows() * M.lpNorm<Eigen::Infinity>() 
        * std::numeric_limits<double>::epsilon();
  }

  // prepare to setup basic and nonbasic variable indices for z
  _nonbas.clear();
  _bas.clear();

  // see whether to warm-start
  if (z->size() == q.size()) {
    LOG << "LCP::lcp_fast() - warm starting activated"
                      << std::endl;

    for (unsigned i=0; i< z->size(); i++) {
      if (std::fabs((*z)[i]) < zero_tol) {
        _bas.push_back(i);
      } else {
        _nonbas.push_back(i);
      }
    }

    if (LOGGING(LOG_OPT)) {
      std::ostringstream str;
      str << " -- non-basic indices:";
      for (unsigned i=0; i< _nonbas.size(); i++)
        str << " " << _nonbas[i];
      LOG << str.str() << std::endl;
    }
  } else {
    // get minimum element of q (really w)
    Eigen::Index minw;
    const double minw_val = q.minCoeff(&minw);
    if (minw_val > -zero_tol) {
      LOG << "LCP::lcp_fast() - trivial solution found"
                        << std::endl;
      z->resize(N);
      z->fill(0);
      return true;
    }

    // setup basic and nonbasic variable indices
    _nonbas.push_back(minw);
    _bas.resize(N-1);
    for (unsigned i=0, j=0; i< N; i++) {
      if (i != minw) {
        _bas[j++] = i;
      }
    }
  }

  // loop for maximum number of pivots
  //  const unsigned MAX_PIV = std::max(N*N, (unsigned) 1000);
  const unsigned MAX_PIV = 2*N;
  for (pivots=0; pivots < MAX_PIV; pivots++) {
    // select nonbasic indices
    selectSubMat(M, _nonbas, _nonbas, &_Msub);
    selectSubMat(M, _bas, _nonbas, &_Mmix);
    selectSubVec(q, _nonbas, &_z);
    selectSubVec(q, _bas, &_qbas);
    //_z.negate();
    _z = _z * -1;

    // solve for nonbasic z
    _z = _Msub.lu().solve(_z);

    // compute w and find minimum value
    _w = _Mmix * _z;
    _w += _qbas;
    unsigned minw = (_w.rows() > 0) ? rand_min(_w, zero_tol) : UINF;

    // TODO sammy this log can't print when minw is UINF.
    // LOG << "LCP::lcp_fast() - minimum w after pivot: "
    // << _w[minw] << std::endl;
    
    // if w >= 0, check whether any component of z < 0
    if (minw == UINF || _w[minw] > -zero_tol) {
      // find the (a) minimum of z
      unsigned minz = (_z.rows() > 0) ? rand_min(_z, zero_tol) : UINF;
      if (LOGGING(LOG_OPT) && _z.rows() > 0) {
        LOG << "LCP::lcp_fast() - minimum z after pivot: "
                          << _z[minz] << std::endl;
      }
      if (minz < UINF && _z[minz] < -zero_tol) {
        // get the original index and remove it from the nonbasic set
        unsigned idx = _nonbas[minz];
        _nonbas.erase(_nonbas.begin()+minz);

        // move index to basic set and continue looping
        _bas.push_back(idx);
        std::sort(_bas.begin(), _bas.end());
      } else {
        // found the solution
        z->resize(N);
        z->fill(0);

        // set values of z corresponding to _z
        for (unsigned i=0, j=0; j < _nonbas.size(); i++, j++) {
          (*z)[_nonbas[j]] = _z[i];
        }

        LOG << "LCP::lcp_fast() - solution found!" << std::endl;
        return true;
      }
    } else {
      LOG << "(minimum w too negative)" << std::endl;

      // one or more components of w violating w >= 0
      // move component of w from basic set to nonbasic set
      unsigned idx = _bas[minw];
      _bas.erase(_bas.begin()+minw);
      _nonbas.push_back(idx);
      std::sort(_nonbas.begin(), _nonbas.end());

      // look whether any component of z needs to move to basic set
      unsigned minz = (_z.rows() > 0) ? rand_min(_z, zero_tol) : UINF;
      if (LOGGING(LOG_OPT) && _z.rows() > 0) {
        LOG << "LCP::lcp_fast() - minimum z after pivot: "
                          << _z[minz] << std::endl;
      }
      if (minz < UINF && _z[minz] < -zero_tol) {
        // move index to basic set and continue looping
        unsigned idx = _nonbas[minz];
        LOG << "LCP::lcp_fast() - moving index " << idx
                          << " to basic set" << std::endl;

        _nonbas.erase(_nonbas.begin()+minz);
        _bas.push_back(idx);
        std::sort(_bas.begin(), _bas.end());
      }
    }
  }

  LOG << "LCP::lcp_fast() - maximum allowable pivots exceeded" << std::endl;

  // if we're here, then the maximum number of pivots has been exceeded
  return false;
}

/// Regularized wrapper around PPM I
bool LCP::lcp_fast_regularized(
    const MatrixNd& M, const VectorNd& q, VectorNd* z,
    int min_exp, unsigned step_exp, int max_exp,
    double piv_tol, double zero_tol) {
  LOG << "LCP::lcp_fast_regularized() entered" << std::endl;

  // look for fast exit
  if (q.size() == 0) {
    z->resize(0);
    return true;
  }

  // copy MM
  _MM = M;

  // assign value for zero tolerance, if necessary
  const double ZERO_TOL = (zero_tol > static_cast<double>(0.0))
      ? zero_tol
      : (q.size() * M.lpNorm<Eigen::Infinity>() * NEAR_ZERO);

  LOG << " zero tolerance: " << ZERO_TOL << std::endl;

  // store the total pivots
  unsigned total_piv = 0;

  // try non-regularized version first
  bool result = lcp_fast(_MM, q, z, zero_tol);
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
          LOG << "  solved with no regularization necessary!"
                            << std::endl;
          LOG << "  pivots / total pivots: " << pivots <<
              " " << pivots << std::endl;
          LOG << "LCP::lcp_fast_regularized() exited" << std::endl;

          return true;
        } else {
          LOG << "LCP::lcp_fast_regularized() - "
              <<"'<w, z> not within tolerance(min value: " << wx_min
              << " max value: " << wx_max << ")"
              << std::endl;
        }
      } else {
        LOG << "  LCP::lcp_fast_regularized() - "
            << "'w' not solved to desired tolerance" << std::endl;
        LOG << "  minimum w: " << _wx.minCoeff()
            << std::endl;
      }
    } else {
      LOG << "  LCP::lcp_fast_regularized() - "
          <<"'z' not solved to desired tolerance" << std::endl;
      LOG << "  minimum z: " << z->minCoeff()
          << std::endl;
    }
  } else {
    LOG << "  LCP::lcp_fast_regularized() "
        << "- solver failed with zero regularization" << std::endl;
  }

  // update the pivots
  total_piv += pivots;

  // start the regularization process
  int rf = min_exp;
  while (rf < max_exp) {
    // setup regularization factor
    double lambda = std::pow(static_cast<double>(10.0),
                             static_cast<double>(rf));

    LOG << "  trying to solve LCP with regularization factor: " << lambda
        << std::endl;

    // regularize M
    _MM = M;
    for (unsigned i=0; i< M.rows(); i++) {
      _MM(i, i) += lambda;
    }

    // try to solve the LCP
    result = lcp_fast(_MM, q, z, zero_tol);

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
            LOG << "  solved with regularization factor: " << lambda
                << std::endl;
            LOG << "  pivots / total pivots: " << pivots
                              << " " << total_piv << std::endl;
            LOG << "LCP::lcp_fast_regularized() exited"
                              << std::endl;
            pivots = total_piv;
            return true;
          } else {
            LOG << "LCP::lcp_fast_regularized() - "
                << "'<w, z> not within tolerance(min value: " << wx_min
                << " max value: " << wx_max << ")" << std::endl;
          }
        } else {
          LOG << "  LCP::lcp_fast_regularized() - "
              << "'w' not solved to desired tolerance" << std::endl;
          LOG << "  minimum w: "
              << _wx.minCoeff()
              << std::endl;
        }
      } else {
        LOG << "  LCP::lcp_fast_regularized() - "
            <<"'z' not solved to desired tolerance" << std::endl;
        LOG << "  minimum z: " << z->minCoeff()
            << std::endl;
      }
    }

    // increase rf
    rf += step_exp;
  }

  LOG << "  unable to solve given any regularization!"
                    << std::endl;
  LOG << "LCP::lcp_fast_regularized() exited" << std::endl;

  // store total pivots
  pivots = total_piv;

  // still here?  failure...
  return false;
}

/// Lemke's algorithm for solving linear complementarity problems
/**
 * \param z a vector "close" to the solution on input (optional); contains
 *        the solution on output
 */
bool LCP::lcp_lemke(const MatrixNd& M, const VectorNd& q, VectorNd* z,
                    double piv_tol, double zero_tol) {
  const unsigned n = q.size();
  const unsigned MAXITER = std::min((unsigned) 1000, 50*n);

  // indicate whether we've restarted
  bool restarted = false;

  // update the pivots
  pivots = 0;

  // look for immediate exit
  if (n == 0) {
    z->resize(0);
    return true;
  }

  // Lemke's algorithm doesn't seem to like warmstarting
  z->fill(0);

  // copy z to z0
  _z0 = *z;

  // come up with a sensible value for zero tolerance if none is given
  if (zero_tol <= static_cast<double>(0.0)) {
    zero_tol = M.lpNorm<Eigen::Infinity>() 
        * std::numeric_limits<double>::epsilon();
  }

  LOG << "LCP::lcp_lemke() entered" << std::endl;
  LOG << "  M: " << std::endl << M;
  LOG << "  q: " << q << std::endl;
  LOG << "  zero_tol: " << zero_tol << std::endl;

  // see whether trivial solution exists
  if (q.minCoeff() > -zero_tol) {
    LOG << " -- trivial solution found" << std::endl;
    LOG << "LCP::lcp_lemke() exited" << std::endl;
    z->resize(n);
    z->fill(0);
    return true;
  }

restart:  // solver restarts from here when basis becomes bad

  // clear all vectors
  _all.clear();
  _tlist.clear();
  _bas.clear();
  _nonbas.clear();
  _j.clear();

  // initialize variables
  z->resize(n*2);
  z->fill(0);
  unsigned t = 2*n;
  unsigned entering = t;
  unsigned leaving = 0;
  _all.clear();
  for (unsigned i=0; i< n; i++) {
    _all.push_back(i);
  }
  unsigned lvindex;
  unsigned idx;
  std::vector<unsigned>::iterator iiter;
  _tlist.clear();

  // determine initial basis
  _bas.clear();
  _nonbas.clear();
  if (_z0.size() != n) {
    // setup the nonbasic indices
    for (unsigned i=0; i< n; i++)
      _nonbas.push_back(i);

    // set the restart basis to random
    _restart_z0 = Eigen::VectorXd::Random(n);
  } else {
    // setup the initial basis
    for (unsigned i=0; i< n; i++) {
      if (_z0[i] > 0) {
        _bas.push_back(i);
      } else {
        _nonbas.push_back(i);
      }
    }

    // setup the restart basis
    if (!restarted) {
      _restart_z0.resize(n);
      _restart_z0.fill(0);
    } else {
      LOG << "-- setting restart basis to random" << std::endl;

      // we've already restarted once, set the restart basis to random
      _restart_z0 = Eigen::VectorXd::Random(n);
    }
  }

  // determine initial values
  if (!_bas.empty()) {
    LOG << "-- initial basis not empty (warmstarting)" << std::endl;
    
    // start from good initial basis
    _Bl.resize(n, n);
    _Bl.setIdentity();
    //_Bl.negate();
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
#if 0
    try {
#endif
    _Al = _Bl;
    _x = _Al.lu().solve(q);
#if 0
    // TODO sammy how to we detect if Eigen failed to solve?
    } catch (SingularException e) {
      // initial basis was no good, set it up as if we have no basis
      _bas.clear();
      _nonbas.clear();
      for (unsigned i=0; i< n; i++) {
        _nonbas.push_back(i);
      }

      // set B to -1 and solve x correspondingly
      _Bl.set_identity(n);
      _Bl.negate();
      _x = q;

      // set next initial basis to random
      _restart_z0.resize(n);
      for (unsigned i=0; i< n; i++) {
        _restart_z0[i] = (rand_r() % 2 == 0) ? 0.0 : 1.0;
      }
    }
#endif
  } else {
    LOG << "-- using basis of -1 (no warmstarting)" << std::endl;

    // use standard initial basis
    _Bl.resize(n, n);
    _Bl.setIdentity();
    //_Bl.negate();
    _Bl *= -1;
    _x = q;
  }

/*
  unsigned basis_count = std::numeric_limits<unsigned>::max();
  while (true)
  {
    try
    {
      _Al = _Bl;
      _x = q;
      _LA.solve_fast(_Al, _x);
      break;
    }
    catch (SingularException e)
    {
      // if initial basis didn't work, prepare to iterate through all 2^n bases
      if (basis_count == std::numeric_limits<unsigned>::max())
        basis_count = 0;

      // cycle through all bases until we find one that works
      set_basis(n, basis_count++, _bas, _nonbas);

      // select columns of M corresponding to z vars in the basis
      M.select(_all.begin(), _all.end(), _bas.begin(), _bas.end(), _t1);

      // select columns of I corresponding to z vars not in the basis
      _Bl.select(_all.begin(), _all.end(), _nonbas.begin(), _nonbas.end(), _t2);

      // setup the basis matrix
      _Bl.resize(n, _t1.columns() + _t2.columns());
      _Bl.set_sub_mat(0,0,_t1);
      _Bl.set_sub_mat(0,_t1.columns(),_t2);
    }
  }
  _x.negate();
*/

  // check whether initial basis provides a solution
  if (_x.minCoeff() >= 0.0) {
      // if (std::find_if(_x.begin(), _x.end(),
      // std::bind2nd(std::less<double>(), 0.0))
      // == _x.end()) {

    for (idx = 0, iiter = _bas.begin(); iiter != _bas.end(); iiter++, idx++) {
      (*z)(*iiter) = _x(idx);
    }

    // TODO sammy is there a more efficient way to resize and preserve the data?
    z->conservativeResize(n);

    // check to see whether tolerances are satisfied
    LOG << " -- initial basis provides a solution!" << std::endl;
    if (LOGGING(LOG_OPT)) {
      _wl = (M * (*z)) + q;
      double minw = _wl.minCoeff();
      double w_dot_z = std::fabs(_wl.dot(*z));
      LOG << "  z: " << z << std::endl;
      LOG << "  _w: " << _wl << std::endl;
      LOG << "  minimum w: " << minw << std::endl;
      LOG << "  w'z: " << w_dot_z << std::endl;
    }
    LOG << "LCP::lcp_lemke() exited" << std::endl;

    return true;
  }

  // use a new pivot tolerance if necessary
  const double PIV_TOL = (piv_tol > static_cast<double>(0.0))
      ? piv_tol
      : (std::numeric_limits<double>::epsilon()
         * n * std::max(static_cast<double>(1.0), 
                        M.lpNorm<Eigen::Infinity>()));

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
  LOG << " -- x: " << _x << std::endl;
  LOG << " -- first pivot: leaving index=" << lvindex
      << "  entering index=" << entering
      << " minimum value: " << tval << std::endl;

  // pivot in the artificial variable
  *iiter = t;    // replace w var with _z0 in basic indices
  _u.resize(n);
  for (unsigned i=0; i< n; i++) {
    _u[i] = (_x[i] < 0.0) ? 1.0 : 0.0;
  }
  _Be = (_Bl * _u) * -1;
  _u *= tval;
  _x += _u;
  _x[lvindex] = tval;
  _Bl.col(lvindex) = _Be;
  LOG << "  new q: " << _x << std::endl;

  // main iterations begin here
  for (pivots=0; pivots< MAXITER; pivots++) {
    if (LOGGING(LOG_OPT)) {
      std::ostringstream basic;
      for (unsigned i=0; i< _bas.size(); i++) {
        basic << " " << _bas[i];
      }
      LOG << "basic variables:" << basic.str() << std::endl;
      LOG << "leaving: " << leaving << " t:" << t << std::endl;
    }

    // check whether done; if not, get new entering variable
    if (leaving == t) {
      LOG << "-- solved LCP successfully!" << std::endl;
      unsigned idx;
      for (idx = 0, iiter = _bas.begin(); iiter != _bas.end(); iiter++, idx++) {
        (*z)[*iiter] = _x[idx];
      }
      z->conservativeResize(n);

      // verify tolerances
      if (LOGGING(LOG_OPT)) {
      _wl = (M * (*z)) + q;
        double minw = _wl.minCoeff();
        double w_dot_z = std::fabs(_wl.dot(*z));
        LOG << "  found solution!" << std::endl;
        LOG << "  minimum w: " << minw << std::endl;
        LOG << "  w'z: " << w_dot_z << std::endl;
        LOG << "  n: " << n
                          << " number of pivots: " << pivots << std::endl;
      }
      LOG << "LCP::lcp_lemke() exited" << std::endl;

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

    // TODO sammy again, I'm not sure how tell that the Eigen solver failed.
    _Al = _Bl;
    _dl = _Al.lu().solve(_dl);
#if 0
    try {
      _Al = _Bl;
      _dl = _Al.lu().solve(_dl);
    } catch (SingularException e) {
      LOG << " -- warning: linear system solver failed (basis became singular)"
          << std::endl;
      LOG << " -- LCP::lcp_lemke() exiting" << std::endl;

      // log failure
      #ifndef NDEBUG
//      log_failure(M, q);
      #endif

      return false;
/*
      LOG << " -- warning: linear system solver failed; restarting with new basis" << std::endl;

      // set the bases
      _z0 = _restart_z0;

      // setup the restart basis
      _restart_z0.resize(n);
      for (unsigned i=0; i< n; i++)
        _restart_z0[i] = (rand() % 2 == 0) ? 1.0 : 0.0;

      // indicate we've restarted
      restarted = true;

      // restart
      goto restart;
*/
/*
// least squares solver
      try
      {
        // use slower SVD pseudo-inverse
        _Al = _Bl;
        _dl = _Be;
        _LA.solve_LS_fast1(_Al, _dl);
      }
      catch (NumericalException e)
      {
        _Al = _Bl;
        _LA.solve_LS_fast2(_Al, _dl);
      }
*/
    }
#endif

    // ** find new leaving variable
    _j.clear();
    for (unsigned i=0; i< _dl.size(); i++) {
      if (_dl[i] > PIV_TOL) {
        _j.push_back(i);
      }
    }

    // check for no new pivots; ray termination
    if (_j.empty()) {
      LOG << "LCP::lcp_lemke() - no new pivots (ray termination)" << std::endl;
      LOG << "LCP::lcp_lemke() exiting" << std::endl;

      // log failure
      #ifndef NDEBUG
//      log_failure(M, q);
      #endif

      return false;
    }

    if (LOGGING(LOG_OPT)) {
      std::ostringstream j;
      for (unsigned i=0; i< _j.size(); i++)
        j << " " << _j[i];
      LOG << "d: " << _dl << std::endl;
      LOG << "j (before min ratio):" << j.str() << std::endl;
    }

    // select elements j from x and d
    selectSubVec(_x, _j, &_xj);
    selectSubVec(_dl, _j, &_dj);

    // compute minimal ratios x(j) + EPS_DOUBLE ./ d(j), d > 0
    _result.resize(_xj.size());
    _result.fill(zero_tol);
    transformVecElements(_xj, &_result, std::plus<double>());
    transformVecElements(_dj, &_result, [](double a, double b) { 
      return b / a;});
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
    if (LOGGING(LOG_OPT)) {
      std::ostringstream j;
      for (unsigned i=0; i< _j.size(); i++) {
        j << " " << _j[i];
      }
      LOG << "j (after min ratio):" << j.str() << std::endl;
    }

    // if j is empty, then likely the zero tolerance is too low
    if (_j.empty()) {
      LOG << "zero tolerance too low?" << std::endl;
      LOG << "LCP::lcp_lemke() exited" << std::endl;
      z->conservativeResize(n);

      // log failure
      #ifndef NDEBUG
//      log_failure(M, q);
      #endif

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
// NOTE: solver seems *much* more capable of solving when we pick the first
// element rather than picking a random one
      lvindex = _j[0];
    }

    // set leaving = bas(lvindex)
    iiter = _bas.begin();
    std::advance(iiter, lvindex);
    leaving = *iiter;

    // ** perform pivot
    double ratio = _x[lvindex]/_dl[lvindex];
    _dl *= ratio;
    _x -= _dl;
    _x[lvindex] = ratio;
    _Bl.col(lvindex) = _Be;
    *iiter = entering;
    LOG << " -- pivoting: leaving index=" << lvindex
                      << "  entering index=" << entering << std::endl;
  }

  LOG << " -- maximum number of iterations exceeded (n=" << n
                    << ", max=" << MAXITER << ")" << std::endl;
  LOG << "LCP::lcp_lemke() exited" << std::endl;

  // max iterations exceeded
  z->conservativeResize(n);
  // log failure
  #ifndef NDEBUG
  //  log_failure(M, q);
  #endif

  return false;
}

/// Regularized wrapper around Lemke's algorithm
bool LCP::lcp_lemke_regularized(
    const MatrixNd& M, const VectorNd& q, VectorNd* z,
    int min_exp, unsigned step_exp, int max_exp,
    double piv_tol, double zero_tol) {
  LOG << "LCP::lcp_lemke_regularized() entered" << std::endl;

  // look for fast exit
  if (q.size() == 0) {
    z->resize(0);
    return true;
  }

  // copy MM
  _MM = M;

  // assign value for zero tolerance, if necessary
  const double ZERO_TOL = (zero_tol > static_cast<double>(0.0))
      ? zero_tol
      : (q.size() * M.lpNorm<Eigen::Infinity>() * NEAR_ZERO);

  LOG << " zero tolerance: " << ZERO_TOL << std::endl;

  // store the total pivots
  unsigned total_piv = 0;

  // try non-regularized version first
  bool result = lcp_lemke(_MM, q, z, piv_tol, zero_tol);
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
          LOG << "  solved with no regularization necessary!" << std::endl;
          LOG << "LCP::lcp_lemke_regularized() exited" << std::endl;

          return true;
        } else {
          LOG << "LCP::lcp_lemke() - "
              << "'<w, z> not within tolerance(min value: " << wx_min
              << " max value: " << wx_max << ")" << std::endl;
        }
      } else {
        LOG << "  LCP::lcp_lemke() - 'w' not solved to desired tolerance"
            << std::endl;
        LOG << "  minimum w: " << _wx.minCoeff()
            << std::endl;
      }
    } else {
      LOG << "  LCP::lcp_lemke() - 'z' not solved to desired tolerance"
          << std::endl;
      LOG << "  minimum z: " << z->minCoeff()
          << std::endl;
    }
  }

  // update the pivots
  total_piv += pivots;

  // start the regularization process
  int rf = min_exp;
  while (rf < max_exp) {
    // setup regularization factor
    double lambda = std::pow(static_cast<double>(10.0),
                             static_cast<double>(rf));

    LOG << "  trying to solve LCP with regularization factor: " << lambda
        << std::endl;

    // regularize M
    _MM = M;
    for (unsigned i=0; i< M.rows(); i++) {
      _MM(i, i) += lambda;
    }

    // try to solve the LCP
    result = lcp_lemke(_MM, q, z, piv_tol, zero_tol);

    // update total pivots
    total_piv += pivots;

    if (result) {
      // verify that solution truly is a solution -- check z
      if (z->minCoeff() > -ZERO_TOL) {
        // check w
        _wx = (_MM * (*z)) + q;
        if (_wx.minCoeff() > - ZERO_TOL) {
          // check z'w
          transformVecElements(*z, &_wx, std::multiplies<double>());
          const double wx_min = _wx.minCoeff();
          const double wx_max = _wx.maxCoeff();
          if (wx_min > -ZERO_TOL && wx_max < ZERO_TOL) {
            LOG << "  solved with regularization factor: " << lambda
                << std::endl;
            LOG << "LCP::lcp_lemke_regularized() exited" << std::endl;
            pivots = total_piv;
            return true;
          } else {
            LOG << "LCP::lcp_lemke() - "
                << "'<w, z> not within tolerance(min value: " << wx_min
                << " max value: " << wx_max << ")" << std::endl;
          }
        } else {
          LOG << "  LCP::lcp_lemke() - 'w' not solved to desired tolerance"
              << std::endl;
          LOG << "  minimum w: " << _wx.minCoeff()
              << std::endl;
        }
      } else {
        LOG << "  LCP::lcp_lemke() - 'z' not solved to desired tolerance"
            << std::endl;
        LOG << "  minimum z: " << z->minCoeff()
            << std::endl;
      }
    }

    // increase rf
    rf += step_exp;
  }

  LOG << "  unable to solve given any regularization!"
                    << std::endl;
  LOG << "LCP::lcp_lemke_regularized() exited"
                    << std::endl;

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
bool LCP::lcp_lemke(const SparseMatrixNd& M, const VectorNd& q, VectorNd* z,
                    double piv_tol, double zero_tol) {
  const unsigned n = q.size();
  const unsigned MAXITER = std::min((unsigned) 1000, 50*n);

  // look for immediate exit
  if (n == 0) {
    z->resize(0);
    return true;
  }

  // clear all vectors
  _all.clear();
  _tlist.clear();
  _bas.clear();
  _nonbas.clear();
  _j.clear();

  // copy z to z0
  _z0 = *z;

  // come up with a sensible value for zero tolerance if none is given
  // come up with a sensible value for zero tolerance if none is given
  if (zero_tol <= static_cast<double>(0.0)) {
    Eigen::MatrixXd dense_M = M;
    zero_tol = dense_M.lpNorm<Eigen::Infinity>() 
        * std::numeric_limits<double>::epsilon() * n;
  }

  LOG << "LCP::lcp_lemke() entered" << std::endl;
  LOG << "  M: " << std::endl << M;
  LOG << "  q: " << q << std::endl;

  // see whether trivial solution exists
  if (q.minCoeff() > -zero_tol) {
    LOG << " -- trivial solution found" << std::endl;
    LOG << "LCP::lcp_lemke() exited" << std::endl;
    z->resize(n);
    z->fill(0);
    return true;
  }

  // initialize variables
  z->resize(n*2);
  z->fill(0);
  unsigned t = 2*n;
  unsigned entering = t;
  unsigned leaving = 0;
  _all.clear();
  for (unsigned i=0; i< n; i++) {
    _all.push_back(i);
  }
  unsigned lvindex;
  unsigned idx;
  std::vector<unsigned>::iterator iiter;
  _tlist.clear();

  // determine initial basis
  _bas.clear();
  _nonbas.clear();
  if (_z0.size() != n) {
    for (unsigned i=0; i< n; i++) {
      _nonbas.push_back(i);
    }
  } else {
    for (unsigned i=0; i< n; i++) {
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
    std::vector<Triplet> tripletList;
    for (int i=0; i< M.outerSize(); i++) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(M, i); it; ++it) {
        int j = it.col();
        std::vector<unsigned>::const_iterator j_it =
            std::find(_bas.begin(), _bas.end(), j);
        if (j_it == _bas.end()) {
          continue;
        } else {
          tripletList.push_back(Triplet(i, j, it.value()));
        }
      }
    }
    for (int i = 0, j = _bas.size(); i < _nonbas.size(); i++, j++) {
      tripletList.push_back(Triplet(_nonbas[i], j, 1.0));
    }

    _sBl.setFromTriplets(tripletList.begin(), tripletList.end());
  } else {
    _sBl.setIdentity();
    //_sBl.negate();
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
    for (idx = 0, iiter = _bas.begin(); iiter != _bas.end(); iiter++, idx++) {
      (*z)[*iiter] = _x[idx];
    }
    z->conservativeResize(n);

    // check to see whether tolerances are satisfied
    LOG << " -- initial basis provides a solution!" << std::endl;
    if (LOGGING(LOG_OPT)) {
      _wl = (M * (*z)) + q;
      double minw = _wl.minCoeff();
      double w_dot_z = std::fabs(_wl.dot(*z));
      LOG << "  z: " << z << std::endl;
      LOG << "  w: " << _wl << std::endl;
      LOG << "  minimum w: " << minw << std::endl;
      LOG << "  w'z: " << w_dot_z << std::endl;
    }
    LOG << "LCP::lcp_lemke() exited" << std::endl;

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
  *iiter = t;    // replace w var with _z0 in basic indices
  _u.resize(n);
  for (unsigned i=0; i< n; i++) {
    _u[i] = (_x[i] < 0.0) ? 1.0 : 0.0;
  }
  _Be = (_sBl * _u) * -1;
  _u *= tval;
  _x += _u;
  _x[lvindex] = tval;
  _sBl.col(lvindex) = makeSparseVector(_Be);
  LOG << "  new q: " << _x << std::endl;

  // main iterations begin here
  for (unsigned iter=0; iter < MAXITER; iter++) {
    // check whether done; if not, get new entering variable
    if (leaving == t) {
      LOG << "-- solved LCP successfully!" << std::endl;
      unsigned idx;
      for (idx = 0, iiter = _bas.begin(); iiter != _bas.end(); iiter++, idx++) {
        (*z)[*iiter] = _x[idx];
      }
      z->conservativeResize(n);

      // verify tolerances
      if (LOGGING(LOG_OPT)) {
        _wl = (M * (*z)) + q;
        double minw = _wl.minCoeff();
        double w_dot_z = std::fabs(_wl.dot(*z));
        LOG << "  found solution!" << std::endl;
        LOG << "  minimum w: " << minw << std::endl;
        LOG << "  w'z: " << w_dot_z << std::endl;
      }
      LOG << "LCP::lcp_lemke() exited" << std::endl;

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
        : (std::numeric_limits<double>::epsilon()
           * n * std::max(static_cast<double>(1.0), 
                          _Be.lpNorm<Eigen::Infinity>()));

    // ** find new leaving variable
    _j.clear();
    for (unsigned i=0; i< _dl.size(); i++) {
      if (_dl[i] > PIV_TOL) {
        _j.push_back(i);
      }
    }
    // check for no new pivots; ray termination
    if (_j.empty()) {
      LOG << "LCP::lcp_lemke() - no new pivots (ray termination)" << std::endl;
      LOG << "LCP::lcp_lemke() exited" << std::endl;

      z->conservativeResize(n);
      return false;
    }

    LOG << " -- column of M': " << _dl << std::endl;

    // select elements j from x and d
    selectSubVec(_x, _j, &_xj);
    selectSubVec(_dl, _j, &_dj);

    // compute minimal ratios x(j) + EPS_DOUBLE ./ d(j), d > 0
    _result.resize(_xj.size());
    _result.fill(zero_tol);
    transformVecElements(_xj, &_result, std::plus<double>());
    transformVecElements(_dj, &_result, [](double a, double b) { 
        return b / a;});
    double theta = _result.minCoeff();

    // NOTE: lexicographic ordering does not appear to be used here to prevent
    // cycling (see [Cottle 1992], pp. 340-342)
    // find indices of minimal ratios, d> 0
    //   divide _x(j) ./ d(j) -- remove elements above the minimum ratio
    for (int i = 0; i < _result.size(); i++) {
      _result(i) = _xj(i) / _dj(i);
    }
    for (iiter = _j.begin(), idx = 0; iiter != _j.end(); ) {
      if (_result[idx++] <= theta) {
        iiter++;
      } else {
        iiter = _j.erase(iiter);
      }
    }
    // if j is empty, then likely the zero tolerance is too low
    if (_j.empty()) {
      LOG << "zero tolerance too low?" << std::endl;
      LOG << "LCP::lcp_lemke() exited" << std::endl;
      z->conservativeResize(n);
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
    double ratio = _x[lvindex]/_dl[lvindex];
    _dl *= ratio;
    _x -= _dl;
    _x[lvindex] = ratio;
    _sBl.col(lvindex) = makeSparseVector(_Be);
    *iiter = entering;
    LOG << " -- pivoting: leaving index=" << lvindex
        << "  entering index=" << entering << std::endl;
  }

  LOG << " -- maximum number of iterations exceeded" << std::endl;
  LOG << "LCP::lcp_lemke() exited" << std::endl;

  // max iterations exceeded
  z->conservativeResize(n);

  return false;
}

#if 0 
/// Sets a basis
void LCP::set_basis(unsigned n, unsigned count,
                    vector<unsigned>& bas, vector<unsigned>& nbas) {
  // clear bas and nbas
  bas.clear();
  nbas.clear();

  uint64 countL = count;
//  unsigned long long n2 = 1L << ((unsigned long long) n-1);
  uint64 n2 = 1;
  for (unsigned i=0; i< n; i++)
    n2 *= 2;

  for (unsigned i=0; i< n; i++) {
    if (countL / n2 > 0)
      bas.push_back(i);
    else
      nbas.push_back(i);
    countL = countL % n2;
    n2 = n2 >> 1L;
    assert(n2 != 0);
  }
}

/// Logs LCP solver failure
void LCP::log_failure(const MatrixNd& M, const VectorNd& q) {
  // generate a unique filename
  std::ostringstream fname;
  fname << "lemke.Mq.";
  for (unsigned i=0; i< 8; i++) {
    fname << rand_r() % 10;
  }
  fname << ".fail";

  // open the file
  std::ofstream out(fname.str().c_str());

  // write the matrix
  for (unsigned i=0; i< M.rows(); i++) {
    for (unsigned j=0; j< M.cols(); j++) {
      out << M(i, j) << " ";
    }
    out << std::endl;
  }

  for (unsigned j=0; j< M.cols(); j++) {
    out << q[j] << " ";
  }

  out << std::endl;
  out.close();
}
#endif

/// Regularized wrapper around Lemke's algorithm for srpase matrices
bool LCP::lcp_lemke_regularized(
  const SparseMatrixNd& M, const VectorNd& q, VectorNd* z,
      int min_exp, unsigned step_exp, int max_exp,
      double piv_tol, double zero_tol) {
  LOG << "LCP::lcp_lemke_regularized() entered" << std::endl;

  // look for fast exit
  if (q.size() == 0) {
    z->resize(0);
    return true;
  }

  // copy MM
  _MMs = M;

  // assign value for zero tolerance, if necessary
  const double ZERO_TOL = (zero_tol > static_cast<double>(0.0))
      ? zero_tol
      : q.size() * std::numeric_limits<double>::epsilon();

  // try non-regularized version first
  bool result = lcp_lemke(_MMs, q, z, piv_tol, zero_tol);
  if (result) {
    // verify that solution truly is a solution -- check z
    if (z->minCoeff() >= -ZERO_TOL) {
      // check w
      _wx = (M * (*z)) + q;
      if (_wx.minCoeff() >= -ZERO_TOL) {
        // check z'w
        transformVecElements(*z, &_wx, std::multiplies<double>());
        if (_wx.minCoeff() >= -ZERO_TOL && _wx.maxCoeff() < ZERO_TOL) {
          LOG << "  solved with no regularization necessary!" << std::endl;
          LOG << "LCP::lcp_lemke_regularized() exited" << std::endl;
          return true;
        }
      }
    }
  }

  // add a zero sparse diagonal matrix to _MMs
  _eye.setIdentity();
  _zero.setZero();
  _MMs += _zero;

  // start the regularization process
  int rf = min_exp;
  while (rf < max_exp) {
    // setup regularization factor
    double lambda = std::pow(static_cast<double>(10.0),
                             static_cast<double>(rf));
    (_diag_lambda = _eye) *= lambda;

    // regularize M
    (_MMx = _MMs) += _diag_lambda;

    // try to solve the LCP
    if ((result = lcp_lemke(_MMx, q, z, piv_tol, zero_tol))) {
      // verify that solution truly is a solution -- check z
      if (z->minCoeff() > -ZERO_TOL) {
        // check w
        _wx = (_MMx * (*z)) + q;
        if (_wx.minCoeff() > -ZERO_TOL) {
          // check z'w
          transformVecElements(*z, &_wx, std::multiplies<double>());
          if (_wx.minCoeff() > -ZERO_TOL && _wx.maxCoeff() < ZERO_TOL) {
            LOG << "  solved with regularization factor: " << lambda
                << std::endl;
            LOG << "LCP::lcp_lemke_regularized() exited" << std::endl;

            return true;
          }
        }
      }
    }

    // increase rf
    rf += step_exp;
  }

  LOG << "  unable to solve given any regularization!" << std::endl;
  LOG << "LCP::lcp_lemke_regularized() exited" << std::endl;

  // still here?  failure...
  return false;
}
#if 0

// picks (randomly) the minimum element from a vector that has potentially
// multiple minima
static RowIteratord_const rand_min2(const VectorNd& v) {
  const double EPS = std::sqrt(std::numeric_limits<double>::epsilon());
  std::vector<unsigned> idx;

  double minimum = *std::min_element(v.row_iterator_begin(),
                                     v.row_iterator_end());
  for (RowIteratord_const i = v.row_iterator_begin();
       i != v.row_iterator_end();
       i++) {
    if (*i - EPS <= minimum) {
      idx.push_back(i-v.row_iterator_begin());
    }
  }

  // pick one at random
  assert(!idx.empty());
  unsigned elm = idx[rand() % idx.size()];
  return v.row_iterator_begin()+elm;
}

/// Fast pivoting algorithm for frictionless contact
bool LCP::fast_pivoting(const MatrixNd& M, const VectorNd& q, VectorNd& z,
                        double eps) {
  const unsigned N = q.size();
  const unsigned MAX_PIVOTS = N*3;
  RowIteratord_const minw, minz;

  // look for degenerate problem
  if (N == 0) {
    z.resize(0);
    return true;
  }

  // compute minimum indices
  minw = std::min_element(q.row_iterator_begin(), q.row_iterator_end());

  // look for easy solution
  if (*minw > -eps) {
    z.set_zero(N);
    return true;
  }

  // setup the basic variable and non-basic variable indices
  _bas.clear();
  _nonbas.clear();
  for (unsigned i=0; i< N; i++) {
    if (i != minw - q.row_iterator_begin()) {
      _bas.push_back(i);
    }
  }
  _nonbas.push_back(minw-q.row_iterator_begin());

  // start the pivoting algorithm
  for (unsigned i=0; i< MAX_PIVOTS; i++) {
    // solve for nonbasic z
    M.select_square(_nonbas.begin(), _nonbas.end(), _M);
    q.select(_nonbas.begin(), _nonbas.end(), _qprime);
    _qprime.negate();

    // compute z subset
    try {
      _LA.solve_fast(_M, _qprime);
    } catch (SingularException e) {
      M.select_square(_nonbas.begin(), _nonbas.end(), _M);
      try {
        _LA.solve_LS_fast(_M, _qprime, LinAlgd::eSVD1, -1.0);
      } catch (NumericalException e) {
        try {
          M.select_square(_nonbas.begin(), _nonbas.end(), _M);
          _LA.solve_LS_fast(_M, _qprime, LinAlgd::eSVD2, -1.0);
        } catch (NumericalException e) {
          return false;
        }
      }
    }

    // setup proposed z
    z.set_zero(N);
    for (unsigned j=0; j< _nonbas.size(); j++) {
      z[_nonbas[j]] = _qprime[j];
    }

    // compute w
    M.mult(z, _w) += q;

    // recompute minimum indices
    minw = rand_min2(_w);
    minz = rand_min2(z);

    // see whether this has solved the problem
    if (*minw > -eps) {
      // check whether any component of z < 0
      if (*minz < -eps) {
        // move the element to the basic set
        unsigned idx = minz-z.row_iterator_begin();
        _nonbas.erase(std::find(_nonbas.begin(), _nonbas.end(), idx));
        _bas.insert(std::lower_bound(_bas.begin(), _bas.end(), idx), idx);
      } else {
        return true;
      }
    } else {
      // move mimimum component of w to nonbasic set
      unsigned idx = minw-_w.row_iterator_begin();
      _bas.erase(std::find(_bas.begin(), _bas.end(), idx));
      _nonbas.insert(std::lower_bound(_nonbas.begin(), _nonbas.end(), idx),
                     idx);

      // look whether a component of z needs to move to basic set
      if (*minz < -eps) {
        // move the element to the basic set
        unsigned idx = minz-z.row_iterator_begin();
        _nonbas.erase(std::find(_nonbas.begin(), _nonbas.end(), idx));
        _bas.insert(std::lower_bound(_bas.begin(), _bas.end(), idx), idx);
      }
    }
  }

  // if we're here, the maximum number of pivots was exceeded
  std::cerr << "LCP::fast_pivoting() warning- "
            << "maximum number of pivots exceeded (" << MAX_PIVOTS << ")"
            << std::endl;
  return false;
};
#endif
}
