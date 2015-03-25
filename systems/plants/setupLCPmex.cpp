#include "mex.h"
#include <iostream>
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"
#include <sstream>

using namespace Eigen;
using namespace std;

#define BIG 1e20

inline void getInclusionIndices(vector<bool> const & inclusion, vector<size_t> & indices, bool get_true_indices)
{
  const size_t n = inclusion.size();
  indices.clear();
  for (size_t x = 0; x < n; x++) {
    if (inclusion[x] == get_true_indices) {
      indices.push_back(x);
    }
  }
}

template <typename Derived>
inline void getThresholdInclusion(MatrixBase<Derived> const & values, const double threshold, vector<bool> & below_threshold)
{
  const size_t n = values.size();
  below_threshold.clear();
  for (size_t x = 0; x < n; x++) {
    below_threshold.push_back(values[x] < threshold);
  }
}

//counts number of inclusions
inline size_t getNumTrue(vector<bool> const & bools)
{
  size_t count = 0;
  const size_t n = bools.size();
  for (size_t x = 0; x < n; x++) {
    if (bools[x]) {
      count++;
    }
  }
  return count;
}

//splits a vector into two based on inclusion mapping
inline void partitionVector(vector<bool> const & indices, VectorXd const & v, VectorXd & included, VectorXd & excluded)
{
  const size_t n = indices.size();
  const size_t count = getNumTrue(indices);

  included = VectorXd::Zero(count);
  excluded = VectorXd::Zero(n - count);
  
  size_t inclusionIndex = 0;
  size_t exclusionIndex = 0;

  for (size_t x = 0; x < n; x++) {
    if (indices[x]) {
      included[inclusionIndex++] = v[x];
    } else {
      excluded[exclusionIndex++] = v[x];
    }
  }
}

//splits a matrix into two based on a row inclusion mapping
inline void partitionMatrix(vector<bool> const & indices, MatrixXd const & M, MatrixXd & included, MatrixXd & excluded)
{   
  const size_t n = indices.size();
  const size_t count = getNumTrue(indices);
  const size_t cols = M.cols();

  included = MatrixXd::Zero(count, cols);
  excluded = MatrixXd::Zero(n-count, cols);

  size_t inclusionIndex = 0;
  size_t exclusionIndex = 0;

  for (size_t x = 0; x < n; x++) {
    if (indices[x]) {
      included.row(inclusionIndex++) = M.row(x);
    } else {
      excluded.row(exclusionIndex++) = M.row(x);
    }
  }
}

//builds a filtered matrix containing only the rows specified by indices
inline void filterByIndices(vector<size_t> const &indices, MatrixXd const & M, MatrixXd & filtered)
{
  const size_t n = indices.size();
  filtered = MatrixXd::Zero(n, M.cols());
  for (size_t x = 0; x < n; x++) {
    filtered.row(x) = M.row(indices[x]);
  }
}

//[z, Mqdn, wqdn] = setupLCPmex(mex_model_ptr, q, qd, u, phiC, n, D, h, z_inactive_guess_tol)
void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) { 
  
  if (nlhs != 3 || nrhs != 9) {
    mexErrMsgIdAndTxt("Drake:setupLCPmex:InvalidUsage","Usage: [z, Mqdn, wqdn] = setupLCPmex(mex_model_ptr, q, qd, u, phiC, n, D, h, z_inactive_guess_tol)");
  }

  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  const int nq = model->num_positions;
  const int nv = model->num_velocities;
  
  //input mappings
  const mxArray* q_array = prhs[1];
  const mxArray* v_array = prhs[2];
  const mxArray* u_array = prhs[3];
  const mxArray* phiC_array = prhs[4];
  const mxArray* n_array = prhs[5];
  const mxArray* D_array = prhs[6];
  const mxArray* h_array = prhs[7];
  const mxArray* inactive_guess_array = prhs[8];

  const size_t numContactPairs = mxGetNumberOfElements(phiC_array);
  const size_t mC = mxGetNumberOfElements(D_array);
  const double z_inactive_guess_tol = static_cast<double>(mxGetScalar(inactive_guess_array));
  const double h = static_cast<double>(mxGetScalar(h_array));
  const Map<VectorXd> q(mxGetPr(q_array),nq);
  const Map<VectorXd> v(mxGetPr(v_array),nv);
  const Map<VectorXd> u(mxGetPr(u_array),model->B.cols());
  const Map<VectorXd> phiC(mxGetPr(phiC_array),numContactPairs);
  const Map<MatrixXd> n(mxGetPr(n_array), numContactPairs, nq);

  VectorXd C, phiL, phiP, phiL_possible, phiC_possible, phiL_check, phiC_check;
  MatrixXd H, B, JP, JL, JL_possible, n_possible, JL_check, n_check;
  
  H.resize(nv, nv); 
  C = VectorXd::Zero(nv);
  B = model->B;
  model->HandC(q, v, (MatrixXd*)nullptr, H, C, (MatrixXd*)nullptr, (MatrixXd*)nullptr, (MatrixXd*)nullptr);

  model->positionConstraints(phiP, JP);
  model->jointLimitConstraints(q, phiL, JL);
  
  const size_t nP = phiP.size();
  
  plhs[2] = mxCreateDoubleMatrix(nq, 1, mxREAL);
  Map<VectorXd> wqdn(mxGetPr(plhs[2]), nq);

  LLT<MatrixXd> H_cholesky(H); // compute the Cholesky decomposition of H
  wqdn = H_cholesky.solve(B * u - C);
  wqdn *= h;
  wqdn += v;

  //use forward euler step in joint space as
  //initial guess for active constraints
  vector<bool> possible_contact;
  vector<bool> possible_jointlimit;
  getThresholdInclusion(phiC + h * n * v, z_inactive_guess_tol, possible_contact);
  getThresholdInclusion(phiL + h * JL * v, z_inactive_guess_tol, possible_jointlimit);

  while (true) {
  //continue from here if our inactive guess fails
    const size_t nC = getNumTrue(possible_contact);
    const size_t nL = getNumTrue(possible_jointlimit);
    const size_t lcp_size = nL + nP + (mC + 2) * nC;

    plhs[0] = mxCreateDoubleMatrix(lcp_size, 1, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(nq, lcp_size, mxREAL);

    Map<VectorXd> z(mxGetPr(plhs[0]), lcp_size);
    Map<MatrixXd> Mqdn(mxGetPr(plhs[1]), nq, lcp_size);
    
    if (lcp_size == 0) {
      return;
    }

    vector<size_t> possible_contact_indices;
    getInclusionIndices(possible_contact, possible_contact_indices, true);

    partitionVector(possible_contact, phiC, phiC_possible, phiC_check);
    partitionVector(possible_jointlimit, phiL, phiL_possible, phiL_check);
    partitionMatrix(possible_contact, n, n_possible, n_check);
    partitionMatrix(possible_jointlimit, JL, JL_possible, JL_check);
    
    MatrixXd D_possible(mC * nC, nq);
    for (size_t i = 0; i < mC ; i++) {
      Map<MatrixXd> D_i(mxGetPr(mxGetCell(D_array, i)), numContactPairs , nq);
      MatrixXd D_i_possible, D_i_exclude;
      filterByIndices(possible_contact_indices, D_i, D_i_possible);
      D_possible.block(nC * i, 0, nC, nq) = D_i_possible;
    }

    MatrixXd J(lcp_size, nq);
    J << JL_possible, JP, n_possible, D_possible, MatrixXd::Zero(nC, nq);

    Mqdn = H_cholesky.solve(J.transpose());
   
    //solve LCP problem 
    //TODO: call fastQP first
    //TODO: call path from C++ (currently only 32-bit C libraries available)
    mxArray* mxM = mxCreateDoubleMatrix(lcp_size, lcp_size, mxREAL);
    mxArray* mxw = mxCreateDoubleMatrix(lcp_size, 1, mxREAL);
    mxArray* mxlb = mxCreateDoubleMatrix(lcp_size, 1, mxREAL);
    mxArray* mxub = mxCreateDoubleMatrix(lcp_size, 1, mxREAL);  
    Map<VectorXd> lb(mxGetPr(mxlb), lcp_size);
    Map<VectorXd> ub(mxGetPr(mxub), lcp_size);
    lb << VectorXd::Zero(nL),
          -BIG * VectorXd::Ones(nP),
          VectorXd::Zero(nC + mC * nC + nC);
    ub = BIG * VectorXd::Ones(lcp_size);
    
    Map<MatrixXd> M(mxGetPr(mxM),lcp_size, lcp_size);
    Map<VectorXd> w(mxGetPr(mxw), lcp_size);

    //build LCP matrix
    M << h * JL_possible*Mqdn,
         h * JP * Mqdn,
         h * n_possible * Mqdn,
         D_possible * Mqdn,
         MatrixXd::Zero(nC, lcp_size);
    

    if (nC > 0) {
      for (size_t i = 0; i < mC ; i++) {
        M.block(nL + nP + nC + nC * i, nL + nP + nC + mC * nC, nC, nC) = MatrixXd::Identity(nC, nC);
        M.block(nL + nP + nC + mC*nC, nL + nP + nC + nC * i, nC, nC) = -MatrixXd::Identity(nC, nC);
      }
      double mu = 1.0; //TODO: pull this from contactConstraints
      M.block(nL + nP + nC + mC * nC, nL + nP, nC, nC) = mu * MatrixXd::Identity(nC, nC);
    }

    //build LCP vector
    w << phiL_possible + h * JL_possible * wqdn,
         phiP + h * JP * wqdn,
         phiC_possible + h * n_possible * wqdn,
         D_possible * wqdn,
         VectorXd::Zero(nC);

    mxArray *lhs[1];
    mxArray *rhs[] = {mxM, mxw, mxlb, mxub};
    
    //call solver
    mexCallMATLAB(1, lhs, 4, rhs, "pathlcp");
    Map<VectorXd> z_path(mxGetPr(lhs[0]), lcp_size);
    z = z_path;

    //clean up
    mxDestroyArray(lhs[0]);
    mxDestroyArray(mxlb);
    mxDestroyArray(mxub);
    mxDestroyArray(mxM);
    mxDestroyArray(mxw);
    
    VectorXd qdn = Mqdn * z + wqdn;

    vector<size_t> impossible_contact_indices, impossible_limit_indices;
    getInclusionIndices(possible_contact, impossible_contact_indices, false);
    getInclusionIndices(possible_jointlimit, impossible_limit_indices, false);

    vector<bool> penetrating_joints, penetrating_contacts;
    getThresholdInclusion(phiL_check + h * JL_check * qdn, 0.0, penetrating_joints);
    getThresholdInclusion(phiC_check + h * n_check * qdn, 0.0, penetrating_contacts);

    const size_t num_penetrating_joints = getNumTrue(penetrating_joints);
    const size_t num_penetrating_contacts = getNumTrue(penetrating_contacts);
    const size_t penetrations = num_penetrating_joints + num_penetrating_contacts;

    //check nonpenetration assumptions
    if (penetrations > 0) {
      //revise joint limit active set
      for (size_t i = 0; i < impossible_limit_indices.size(); i++) {
        if (penetrating_joints[i]) {
          possible_jointlimit[impossible_limit_indices[i]] = true;
        }
      }

      //revise contact constraint active set
      for (size_t i = 0; i < impossible_contact_indices.size(); i++) {
        if (penetrating_contacts[i]) {
          possible_contact[impossible_contact_indices[i]] = true;
        }
      }
      //throw away our old solution and try again
      mxDestroyArray(plhs[0]);
      mxDestroyArray(plhs[1]);
      continue;
    }

    //our initial guess was correct. we're done
    break;
  }

}
