#include "EigenQuadProg.h"
#include <iostream>

void Eigen::QP::resize(int nVar, int nEq, int nIneq)
{
  _nVar = nVar;
  _nEq = nEq;
  _nIneq = nIneq;

  _R = MatrixXd(_nVar, _nVar);
  _J = MatrixXd(_nVar, _nVar);

  _s = VectorXd(_nIneq+_nEq);
  _z = VectorXd(_nVar);
  _r = VectorXd(_nIneq + _nEq);
  _d = VectorXd(_nVar);
  _np = VectorXd(_nVar);
  _u = VectorXd(_nIneq + _nEq);
  _x_old = VectorXd(_nVar);
  _u_old = VectorXd(_nIneq + _nEq);

  _A = VectorXi(_nIneq+_nEq);
  _A_old = VectorXi(_nIneq+_nEq);
  _iai = VectorXi(_nIneq+_nEq);
  _iaexcl = VectorXi(_nIneq+_nEq);

  _chol = LLT<MatrixXd,Lower>(_nVar);
}

bool Eigen::QP::add_constraint(MatrixXd& _R, MatrixXd& _J, VectorXd& _d, int& iq, double& R_norm)
{
  int n=_J.rows();
#ifdef TRACE_SOLVER
  std::cerr << "Add constraint " << iq << '/';
#endif
  int j, k;
  //int i, j, k;
  double cc, ss, h, t1, t2, xny;

  /* we have to find the Givens rotation which will reduce the element
     _d(j) to zero.
     if it is already zero we don't have to do anything, except of
     decreasing j */  
  for (j = n - 1; j >= iq + 1; j--)
  {
    /* The Givens rotation is done with the matrix (cc cs, cs -cc).
       If cc is one, then element (j) of _d is zero compared with element
       (j - 1). Hence we don't have to do anything. 
       If cc is zero, then we just have to switch column (j) and column (j - 1) 
       of _J. Since we only switch columns in _J, we have to be careful how we
       update _d depending on the sign of gs.
       Otherwise we have to apply the Givens rotation to these columns.
       The i - 1 element of _d has to be updated to h. */
    cc = _d(j - 1);
    ss = _d(j);
    h = distance(cc, ss);
    if (h == 0.0)
      continue;
    _d(j) = 0.0;
    ss = ss / h;
    cc = cc / h;
    if (cc < 0.0)
    {
      cc = -cc;
      ss = -ss;
      _d(j - 1) = -h;
    }
    else
      _d(j - 1) = h;
    xny = ss / (1.0 + cc);
    for (k = 0; k < n; k++)
    {
      t1 = _J(k,j - 1);
      t2 = _J(k,j);
      _J(k,j - 1) = t1 * cc + t2 * ss;
      _J(k,j) = xny * (t1 + _J(k,j - 1)) - t2;
    }
  }
  /* update the number of constraints added*/
  iq++;
  /* To update _R we have to put the iq components of the _d vector
     into column iq - 1 of _R
     */
  _R.col(iq-1).head(iq) = _d.head(iq);
#ifdef TRACE_SOLVER
  std::cerr << iq << std::endl;
#endif

  if (std::abs(_d(iq - 1)) <= std::numeric_limits<double>::epsilon() * R_norm)
    // problem degenerate
    return false;
  R_norm = std::max<double>(R_norm, std::abs(_d(iq - 1)));
  return true;  
}

void Eigen::QP::delete_constraint(MatrixXd& _R, MatrixXd& _J, VectorXi& _A, VectorXd& _u,  int p, int& iq, int l)
{
  int n = _R.rows();
#ifdef TRACE_SOLVER
  std::cerr << "Delete constraint " << l << ' ' << iq;
#endif
  int i, j, k, qq;
  double cc, ss, h, xny, t1, t2;

  /* Find the index qq for active constraint l to be removed */
  for (i = p; i < iq; i++)
    if (_A(i) == l)
    {
      qq = i;
      break;
    }

  /* remove the constraint from the active set and the duals */
  for (i = qq; i < iq - 1; i++)
  {
    _A(i) = _A(i + 1);
    _u(i) = _u(i + 1);
    _R.col(i) = _R.col(i+1);
  }

  _A(iq - 1) = _A(iq);
  _u(iq - 1) = _u(iq);
  _A(iq) = 0; 
  _u(iq) = 0.0;
  for (j = 0; j < iq; j++)
    _R(j,iq - 1) = 0.0;
  /* constraint has been fully removed */
  iq--;
#ifdef TRACE_SOLVER
  std::cerr << '/' << iq << std::endl;
#endif 

  if (iq == 0)
    return;

  for (j = qq; j < iq; j++)
  {
    cc = _R(j,j);
    ss = _R(j + 1,j);
    h = distance(cc, ss);
    if (h == 0.0)
      continue;
    cc = cc / h;
    ss = ss / h;
    _R(j + 1,j) = 0.0;
    if (cc < 0.0)
    {
      _R(j,j) = -h;
      cc = -cc;
      ss = -ss;
    }
    else
      _R(j,j) = h;

    xny = ss / (1.0 + cc);
    for (k = j + 1; k < iq; k++)
    {
      t1 = _R(j,k);
      t2 = _R(j + 1,k);
      _R(j,k) = t1 * cc + t2 * ss;
      _R(j + 1,k) = xny * (t1 + _R(j,k)) - t2;
    }
    for (k = 0; k < n; k++)
    {
      t1 = _J(k,j);
      t2 = _J(k,j + 1);
      _J(k,j) = t1 * cc + t2 * ss;
      _J(k,j + 1) = xny * (_J(k,j) + t1) - t2;
    }
  }  
}

double Eigen::QP::solve_quadprog2(LLT<MatrixXd,Lower> &chol,  double c1, const VectorXd & g0,  
    const MatrixXd & CE, const VectorXd & ce0,  
    const MatrixXd & CI, const VectorXd & ci0, 
    VectorXd& x) 
{
  int i, k, l; /* indices */
  //int i, j, k, l; /* indices */
  int ip, me, mi;
  //int n=g0.size();   
  int p=CE.cols(); 
  int m=CI.cols();
  /*
     MatrixXd _R(g0.size(),g0.size()), _J(g0.size(),g0.size());
     VectorXd _s(m+p), _z(n), _r(m + p), _d(n),  _np(n), _u(m + p);
     VectorXd _x_old(n), _u_old(m + p);
     VectorXi _A(m + p), _A_old(m + p), _iai(m + p), _iaexcl(m+p);
     */

  double f_value, psi, c2, sum, ss, R_norm;
  const double inf = std::numeric_limits<double>::infinity();
  double t, t1, t2; /* t is the step length, which is the minimum of the partial step length t1 
                     * and the full step length t2 */
  int q;
  int iq, iter = 0;

  me = p; /* number of equality constraints */
  mi = m; /* number of inequality constraints */
  q = 0;  /* size of the active set _A (containing the indices of the active constraints) */
  q++;
  q--;

  /*
   * Preprocessing phase
   */



  /* initialize the matrix _R */
  _d.setZero();
  _R.setZero();
  R_norm = 1.0; /* this variable will hold the norm of the matrix _R */

  /* compute the inverse of the factorized matrix G^-1, this is the initial value for H */
  // _J = L^-T
  _J.setIdentity();
  _J = chol.matrixU().solve(_J);
  c2 = _J.trace();
#ifdef TRACE_SOLVER
  print_matrix("_J", _J, n);
#endif

  /* c1 * c2 is an estimate for cond(G) */

  /* 
   * Find the unconstrained minimizer of the quadratic form 0.5 * x G x + g0 x 
   * this is a feasible point in the dual space
   * x = G^-1 * g0
   */
  x = chol.solve(g0);
  x = -x;
  /* and compute the current solution value */ 
  f_value = 0.5 * g0.dot(x);
#ifdef TRACE_SOLVER
  std::cerr << "Unconstrained solution: " << f_value << std::endl;
  print_vector("x", x, n);
#endif

  /* Add equality constraints to the working set _A */
  iq = 0;
  for (i = 0; i < me; i++)
  {
    if (CE.row(i).isZero())
      continue;

    _np = CE.col(i);
    compute_d(_d, _J, _np);
    update_z(_z, _J, _d,  iq);
    update_r(_R, _r, _d,  iq);
#ifdef TRACE_SOLVER
    print_matrix("_R", _R, iq);
    print_vector("_z", _z, n);
    print_vector("_r", _r, iq);
    print_vector("_d", _d, n);
#endif

    /* compute full step length t2: i.e., the minimum step in primal space _s.t. the contraint 
       becomes feasible */
    t2 = 0.0;
    if (std::abs(_z.dot(_z)) > std::numeric_limits<double>::epsilon()) // i.e. _z != 0
      t2 = (-_np.dot(x) - ce0(i)) / _z.dot(_np);

    x += t2 * _z;

    /* set _u = _u+ */
    _u(iq) = t2;
    _u.head(iq) -= t2 * _r.head(iq);

    /* compute the new solution value */
    f_value += 0.5 * (t2 * t2) * _z.dot(_np);
    _A(i) = -i - 1;

    if (!add_constraint(_R, _J, _d, iq, R_norm))
    {
      // FIXME: it should raise an error
      // Equality constraints are linearly dependent
      return f_value;
    }
  }

  /* set _iai = K \ _A */
  for (i = 0; i < mi; i++)
    _iai(i) = i;

l1:	
  iter++;
#ifdef TRACE_SOLVER
  print_vector("x", x, n);
#endif
  /* step 1: choose a violated constraint */
  for (i = me; i < iq; i++)
  {
    ip = _A(i);
    _iai(ip) = -1;
  }

  /* compute _s(x) = ci^T * x + ci0 for all elements of K \ _A */
  ss = 0.0;
  psi = 0.0; /* this value will contain the sum of all infeasibilities */
  ip = 0; /* ip will be the index of the chosen violated constraint */
  for (i = 0; i < mi; i++)
  {
    _iaexcl(i) = 1;
    sum = CI.col(i).dot(x) + ci0(i);
    _s(i) = sum;
    psi += std::min(0.0, sum);
  }
#ifdef TRACE_SOLVER
  print_vector("_s", _s, mi);
#endif


  if (std::abs(psi) <= mi * std::numeric_limits<double>::epsilon() * c1 * c2* 100.0)
  {
    /* numerically there are not infeasibilities anymore */
    q = iq;
    return f_value;
  }

  /* save old values for _u, x and _A */
  _u_old.head(iq) = _u.head(iq);
  _A_old.head(iq) = _A.head(iq);
  _x_old = x;

l2: /* Step 2: check for feasibility and determine a new S-pair */
  for (i = 0; i < mi; i++)
  {
    if (_s(i) < ss && _iai(i) != -1 && _iaexcl(i))
    {
      ss = _s(i);
      ip = i;
    }
  }
  if (ss >= 0.0)
  {
    q = iq;
    return f_value;
  }

  /* set _np = n(ip) */
  _np = CI.col(ip);
  /* set _u = (_u 0)^T */
  _u(iq) = 0.0;
  /* add ip to the active set _A */
  _A(iq) = ip;

#ifdef TRACE_SOLVER
  std::cerr << "Trying with constraint " << ip << std::endl;
  print_vector("_np", _np, n);
#endif

l2a:
  /* Step 2a: determine step direction */
  /* compute _z = H _np: the step direction in the primal space (through _J, see the paper) */
  compute_d(_d, _J, _np);
  update_z(_z, _J, _d, iq);
  /* compute N* _np (if q > 0): the negative of the step direction in the dual space */
  update_r(_R, _r, _d, iq);
#ifdef TRACE_SOLVER
  std::cerr << "Step direction _z" << std::endl;
  print_vector("_z", _z, n);
  print_vector("_r", _r, iq + 1);
  print_vector("_u", _u, iq + 1);
  print_vector("_d", _d, n);
  print_ivector("_A", _A, iq + 1);
#endif

  /* Step 2b: compute step length */
  l = 0;
  /* Compute t1: partial step length (maximum step in dual space without violating dual feasibility */
  t1 = inf; /* +inf */
  /* find the index l _s.t. it reaches the minimum of _u+(x) / _r */
  for (k = me; k < iq; k++)
  {
    double tmp;
    if (_r(k) > 0.0 && ((tmp = _u(k) / _r(k)) < t1) )
    {
      t1 = tmp;
      l = _A(k);
    }
  }
  /* Compute t2: full step length (minimum step in primal space such that the constraint ip becomes feasible */
  if (std::abs(_z.dot(_z))  > std::numeric_limits<double>::epsilon()) // i.e. _z != 0
    t2 = -_s(ip) / _z.dot(_np);
  else
    t2 = inf; /* +inf */

  /* the step is chosen as the minimum of t1 and t2 */
  t = std::min(t1, t2);
#ifdef TRACE_SOLVER
  std::cerr << "Step sizes: " << t << " (t1 = " << t1 << ", t2 = " << t2 << ") ";
#endif

  /* Step 2c: determine new S-pair and take step: */

  /* case (i): no step in primal or dual space */
  if (t >= inf)
  {
    /* QPP is infeasible */
    // FIXME: unbounded to raise
    q = iq;
    return inf;
  }
  /* case (ii): step in dual space */
  if (t2 >= inf)
  {
    /* set _u = _u +  t * [-_r 1) and drop constraint l from the active set _A */
    _u.head(iq) -= t * _r.head(iq);
    _u(iq) += t;
    _iai(l) = l;
    delete_constraint(_R, _J, _A, _u, p, iq, l);
#ifdef TRACE_SOLVER
    std::cerr << " in dual space: " 
      << f_value << std::endl;
    print_vector("x", x, n);
    print_vector("_z", _z, n);
    print_ivector("_A", _A, iq + 1);
#endif
    goto l2a;
  }

  /* case (iii): step in primal and dual space */

  x += t * _z;
  /* update the solution value */
  f_value += t * _z.dot(_np) * (0.5 * t + _u(iq));

  _u.head(iq) -= t * _r.head(iq);
  _u(iq) += t;
#ifdef TRACE_SOLVER
  std::cerr << " in both spaces: " 
    << f_value << std::endl;
  print_vector("x", x, n);
  print_vector("_u", _u, iq + 1);
  print_vector("_r", _r, iq + 1);
  print_ivector("_A", _A, iq + 1);
#endif

  if (t == t2)
  {
#ifdef TRACE_SOLVER
    std::cerr << "Full step has taken " << t << std::endl;
    print_vector("x", x, n);
#endif
    /* full step has taken */
    /* add constraint ip to the active set*/
    if (!add_constraint(_R, _J, _d, iq, R_norm))
    {
      _iaexcl(ip) = 0;
      delete_constraint(_R, _J, _A, _u, p, iq, ip);
#ifdef TRACE_SOLVER
      print_matrix("_R", _R, n);
      print_ivector("_A", _A, iq);
#endif
      for (i = 0; i < m; i++)
        _iai(i) = i;
      for (i = 0; i < iq; i++)
      {
        _A(i) = _A_old(i);
        _iai(_A(i)) = -1;
        _u(i) = _u_old(i);
      }
      x = _x_old;
      goto l2; /* go to step 2 */
    }    
    else
      _iai(ip) = -1;
#ifdef TRACE_SOLVER
    print_matrix("_R", _R, n);
    print_ivector("_A", _A, iq);
#endif
    goto l1;
  }

  /* a patial step has taken */
#ifdef TRACE_SOLVER
  std::cerr << "Partial step has taken " << t << std::endl;
  print_vector("x", x, n);
#endif
  /* drop constraint l */
  _iai(l) = l;
  delete_constraint(_R, _J, _A, _u, p, iq, l);
#ifdef TRACE_SOLVER
  print_matrix("_R", _R, n);
  print_ivector("_A", _A, iq);
#endif

  _s(ip) = CI.col(ip).dot(x) + ci0(ip);

#ifdef TRACE_SOLVER
  print_vector("_s", _s, mi);
#endif
  goto l2a;
}

double Eigen::QP::solve_quadprog(MatrixXd & G,  const VectorXd & g0,  
    const MatrixXd & CE, const VectorXd & ce0,  
    const MatrixXd & CI, const VectorXd & ci0, 
    VectorXd& x)
{
  //LLT<MatrixXd,Lower> chol(G.cols());
  double c1;

  /* compute the trace of the original matrix G */
  c1 = G.trace();

  /* decompose the matrix G in the form LL^T */
  _chol.compute(G);

  if (_chol.info() != Success) {    
    //std::cerr << "G " << G << "\nnot PSD.\n";
    return std::numeric_limits<double>::infinity();
  }

  return solve_quadprog2(_chol, c1, g0, CE, ce0, CI, ci0, x);
}
