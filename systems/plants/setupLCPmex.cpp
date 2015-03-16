#include "mex.h"
#include <iostream>
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"
#include <sstream>

using namespace Eigen;
using namespace std;

template <typename DerivedA, typename DerivedB, typename DerivedC, typename DerivedD> 
inline void manipulatorDynamics(RigidBodyManipulator *model, MatrixBase<DerivedA> const & q, MatrixBase<DerivedA> const & v, MatrixBase<DerivedB> &H, MatrixBase<DerivedC> &C, MatrixBase<DerivedD> &B)
{
  const int nq = model->num_positions;
  H.resize(nq,nq); 
  C = VectorXd::Zero(nq);
  B = model->B;
  model->HandC(q, v, (MatrixXd*)nullptr, H, C, (MatrixXd*)nullptr, (MatrixXd*)nullptr, (MatrixXd*)nullptr);
}

template <typename Derived>
inline void getThresholdIndices(MatrixBase<Derived> const &values, const double threshold, vector<int> & indices)
{
  indices.clear();
  const int n = values.size();
  for (int x = 0; x < n; x++) {
    if (values[x] < threshold) {
      indices.push_back(x);
    }
  }
}

inline void filterByIndices(vector<int> const &indices, VectorXd const & v, VectorXd & filtered)
{
  const int n = indices.size();
  filtered = VectorXd::Zero(n);
  for (int x = 0; x < n; x++) {
    filtered[x] = v[indices[x]];
  }
}

//builds a filtered matrix containing only the rows specified by indices
inline void filterByIndices(vector<int> const &indices, MatrixXd const & M, MatrixXd &filtered)
{
  const int n = indices.size();
  filtered = MatrixXd::Zero(n, M.cols());
  for (int x = 0; x < n; x++) {
    filtered.row(x) = M.row(indices[x]);
  }
}

vector<int> operator<(VectorXd const & v, double c)
{
  vector<int> filter_indices; 
  getThresholdIndices(v, c, filter_indices);
  return filter_indices;
}

std::string matrix_size(MatrixXd const & m)
{
  stringstream ss;
  ss << m.rows() << "x" << m.cols();
  return ss.str();
}

//[M, w, Mqdn, wqdn] = setupLCPmex(mex_model_ptr, q, qd, u, phiC, n, D, h, z_inactive_guess_tol)
void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) { 
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  const int nq = model->num_positions;
  const int nv = model->num_velocities;
  const int numContactPairs = mxGetNumberOfElements(prhs[4]);
  const int mC = mxGetNumberOfElements(prhs[6]);
  const double z_inactive_guess_tol = static_cast<double>(mxGetScalar(prhs[8]));
  const double h = static_cast<double>(mxGetScalar(prhs[7]));
  const Map<VectorXd> q(mxGetPr(prhs[1]),nq);
  const Map<VectorXd> v(mxGetPr(prhs[2]),nv);
  const Map<VectorXd> u(mxGetPr(prhs[3]),model->B.cols());
  
  const Map<VectorXd> phiC(mxGetPr(prhs[4]),numContactPairs);
  const Map<MatrixXd> n(mxGetPr(prhs[5]), numContactPairs, nq);

  VectorXd C, phiL, phiL_possible, phiC_possible;
  MatrixXd H, B, JL, JL_possible, n_possible;

  manipulatorDynamics(model, q, v, H, C, B);
  auto phiP = model->positionConstraints<double>(1);
  model->jointLimitConstraints(q, phiL, JL);

  
  vector<int> possible_contact_indices = (phiC + h*n*v) < z_inactive_guess_tol;
  vector<int> possible_limit_indices = (phiL + h*JL*v) < z_inactive_guess_tol;
  
  const int nC = possible_contact_indices.size();
  const int nL = possible_limit_indices.size();
  const int nP = phiP.value().size();

  const int LCP_size = nL+nP+(mC+2)*nC;

  filterByIndices(possible_limit_indices, phiL, phiL_possible);
  filterByIndices(possible_contact_indices, phiC, phiC_possible);

  filterByIndices(possible_limit_indices, JL, JL_possible);
  filterByIndices(possible_contact_indices, n, n_possible);

  MatrixXd D_possible(mC*nC, nq);

  for (int i = 0; i < mC ; i++) {
    Map<MatrixXd> D_i(mxGetPr(mxGetCell(prhs[6], i)), numContactPairs , nq);
    MatrixXd D_i_possible;
    filterByIndices(possible_contact_indices, D_i, D_i_possible);
    D_possible.block(nC*i, 0, nC, nq) = D_i_possible;
  }


  MatrixXd Hinv = H.inverse();
  MatrixXd J(LCP_size, nq);
  J << JL_possible, phiP.gradient().value(), n_possible, D_possible, MatrixXd::Zero(nC, nq);

  plhs[0] = mxCreateDoubleMatrix(LCP_size, LCP_size, mxREAL);
  plhs[1] = mxCreateDoubleMatrix(LCP_size, 1, mxREAL);
  plhs[2] = mxCreateDoubleMatrix(nq, LCP_size, mxREAL);
  plhs[3] = mxCreateDoubleMatrix(nq, 1, mxREAL);

  Map<MatrixXd> M(mxGetPr(plhs[0]), LCP_size, LCP_size);
  Map<VectorXd> w(mxGetPr(plhs[1]), LCP_size);
  Map<MatrixXd> Mqdn(mxGetPr(plhs[2]), nq, LCP_size);
  Map<VectorXd> wqdn(mxGetPr(plhs[3]), nq);

  Mqdn = Hinv*J.transpose();
  wqdn = v + h*Hinv*(B*u - C);

  M << h*JL_possible*Mqdn,
       h*phiP.gradient().value()*Mqdn,
       h*n_possible*Mqdn,
       D_possible*Mqdn,
       MatrixXd::Zero(nC, LCP_size);
  
  for (int i=0; i < mC ; i++) {
    M.block(nL+nP+nC+nC*i, LCP_size - nC, nC, nC) = MatrixXd::Identity(nC, nC);
    M.block(nL+nP+nC+mC*nC, nL+nP+nC+nC*i, nC, nC) = -1.0*MatrixXd::Identity(nC, nC);
  }

  double mu = 1.0; //TODO: pull this from contactConstraints
  M.block(nL+nP+nC+mC*nC, nP+nP, nC, nC) = mu*MatrixXd::Identity(nC, nC);

  w << phiL_possible + h*JL_possible*wqdn,
       phiP.value() + h *phiP.gradient().value()*wqdn,
       phiC_possible + h*n_possible*wqdn,
       D_possible*wqdn,
       VectorXd::Zero(nC);


  mxArray* mxlb = mxCreateDoubleMatrix(LCP_size, 1, mxREAL);
  mxArray* mxub = mxCreateDoubleMatrix(LCP_size, 1, mxREAL);
  
  //experimental
  Map<VectorXd> lb(mxGetPr(mxlb), LCP_size);
  Map<VectorXd> ub(mxGetPr(mxub), LCP_size);
  lb = VectorXd::Zero(LCP_size);
  ub = 1e20*VectorXd::Ones(LCP_size);

  mxArray *lhs[1];
  mxArray *rhs[] = {plhs[0], plhs[1], mxlb, mxub};

  mexCallMATLABsafe(1, lhs, 4, rhs, "pathlcp");
  Map<VectorXd> z(mxGetPr(lhs[0]), LCP_size);

  cout << "z: " << endl;
  cout << z << endl;
}