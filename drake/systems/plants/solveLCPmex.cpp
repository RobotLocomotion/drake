#include "mex.h"
#include <iostream>
#include "drakeMexUtil.h"
#include "MexWrapper.h"
#include "RigidBodyManipulator.h"
#include "math.h"
#include "fastQP.h"
#include <sstream>

using namespace Eigen;
using namespace std;

#define BIG 1e20
#define SMALL 1e-8

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

inline bool anyTrue(vector<bool> const & bools)
{ 
  const size_t n = bools.size();
  for (size_t x = 0; x < n; x++) {
    if (bools[x]) {
      return true;
    }
  }  
  return false;
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

//filters a vector by index
inline void filterByIndices(vector<size_t> const &indices, VectorXd const & v, VectorXd & filtered)
{
  const size_t n = indices.size();
  filtered = VectorXd::Zero(n);
  for (size_t x = 0; x < n; x++) {
    filtered[x] = v[indices[x]];
  }
}

template <typename DerivedM, typename Derivedw, typename Derivedlb, typename Derivedz>
bool callFastQP(MatrixBase<DerivedM> const & M, MatrixBase<Derivedw> const & w, MatrixBase<Derivedlb> const & lb, vector<bool> & z_inactive, const size_t checkLimit,  MatrixBase<Derivedz> & z) 
{
  const size_t num_inactive_z = getNumTrue(z_inactive);
  if (num_inactive_z == 0) { 
    return false;
  }

  vector<size_t> z_inactive_indices, z_active_indices;
  vector<MatrixXd*> list_pQ;
  set<int> active;
  MatrixXd Aeq, Ain, Qdiag, M_temp, M_check;
  VectorXd fqp, bin, beq, zqp, w_check;
  getInclusionIndices(z_inactive, z_inactive_indices, true);
  filterByIndices(z_inactive_indices, M, M_temp);
  filterByIndices(z_inactive_indices, M_temp.transpose(), Aeq);
  filterByIndices(z_inactive_indices, -w, beq);
  filterByIndices(z_inactive_indices, -lb, bin);
  Ain = -MatrixXd::Identity(num_inactive_z, num_inactive_z);
  Qdiag = MatrixXd::Identity(num_inactive_z, num_inactive_z);
  fqp = VectorXd::Zero(num_inactive_z);
  zqp = VectorXd::Zero(num_inactive_z);
  list_pQ.push_back(&Qdiag);

  int info = fastQP(list_pQ, fqp, Aeq.transpose(), beq, Ain, bin, active, zqp);
  size_t zqp_index = 0;
  if (info < 0) {
    return false;
  } else { 
    for (size_t i = 0; i < num_inactive_z; i++) {
      z[z_inactive_indices[i]] = zqp[zqp_index++];
    }
  }

  //make sure fastQP actually produced a solution
  vector<bool> violations, ineq_violations;
  z_inactive.resize(checkLimit);
  getInclusionIndices(z_inactive, z_active_indices, false);
  filterByIndices(z_active_indices, M, M_temp); // keep active rows
  filterByIndices(z_inactive_indices, M_temp.transpose(), M_check);  //and inactive columns
  filterByIndices(z_active_indices, w, w_check);
  getThresholdInclusion((M_check.transpose() * zqp + w_check).eval(), -SMALL, violations);
  //check equality constraints
  if (anyTrue(violations)) { 
    return false;
  }
 
  getThresholdInclusion((Ain * zqp - bin).eval(), -SMALL, ineq_violations);
  getThresholdInclusion((beq - Aeq.transpose() * zqp).eval(), -SMALL, violations);
  //check inequality constraints
  for (size_t i = 0; i < num_inactive_z; i++) { 
    if (ineq_violations[i] && violations[i]) { 
      return false;
    }
  }

  //check complementarity constraints
  getThresholdInclusion((-(zqp.transpose()*(Aeq*zqp - beq)).cwiseAbs()).eval(), -SMALL, violations);
  if(anyTrue(violations)) {
    return false;
  }
  
  return true;
}

//[z, Mvn, wvn] = setupLCPmex(mex_model_ptr, cache_ptr, u, phiC, n, D, h, z_inactive_guess_tol)
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) { 
  
  if (nlhs != 5 || nrhs != 13) {
    mexErrMsgIdAndTxt("Drake:setupLCPmex:InvalidUsage","Usage: [z, Mvn, wvn, zqp] = setupLCPmex(mex_model_ptr, cache_ptr, u, phiC, n, D, h, z_inactive_guess_tol, z_cached, H, C, B)");
  }
  static unique_ptr<MexWrapper> lcp_mex = unique_ptr<MexWrapper>(new MexWrapper(PATHLCP_MEXFILE));

  int arg_num = 0;
  RigidBodyManipulator *model = static_cast<RigidBodyManipulator*>(getDrakeMexPointer(prhs[arg_num++]));
  KinematicsCache<double>* cache = static_cast<KinematicsCache<double>*>(getDrakeMexPointer(prhs[arg_num++]));
  cache->checkCachedKinematicsSettings(false, true, true, "solveLCPmex");

  const int nq = model->num_positions;
  const int nv = model->num_velocities;
  
  //input mappings
  const mxArray* u_array = prhs[arg_num++];
  const mxArray* phiC_array = prhs[arg_num++];
  const mxArray* n_array = prhs[arg_num++];
  const mxArray* D_array = prhs[arg_num++];
  const mxArray* h_array = prhs[arg_num++];
  const mxArray* inactive_guess_array = prhs[arg_num++];
  const mxArray* z_cached_array = prhs[arg_num++];
  const mxArray* H_array = prhs[arg_num++];
  const mxArray* C_array = prhs[arg_num++];
  const mxArray* B_array = prhs[arg_num++];
  const mxArray* enable_fastqp_array = prhs[arg_num++];

  const size_t num_z_cached = mxGetNumberOfElements(z_cached_array);
  const size_t num_contact_pairs = mxGetNumberOfElements(phiC_array);
  const size_t mC = mxGetNumberOfElements(D_array);
  const double z_inactive_guess_tol = mxGetScalar(inactive_guess_array);
  const double h = mxGetScalar(h_array);
  const auto& q = cache->getQ();
  const auto& v = cache->getV();
  const Map<VectorXd> u(mxGetPrSafe(u_array), mxGetNumberOfElements(u_array));
  const Map<VectorXd> phiC(mxGetPrSafe(phiC_array), num_contact_pairs);
  const Map<MatrixXd> n(mxGetPrSafe(n_array), num_contact_pairs, nq);
  const Map<VectorXd> z_cached(mxGetPrSafe(z_cached_array), num_z_cached);
  const Map<MatrixXd> H(mxGetPrSafe(H_array), nv, nv);
  const Map<VectorXd> C(mxGetPrSafe(C_array), nv);
  const Map<MatrixXd> B(mxGetPrSafe(B_array), mxGetM(B_array), mxGetN(B_array));
 
  const bool enable_fastqp = mxIsLogicalScalarTrue(enable_fastqp_array);

  VectorXd phiL, phiL_possible, phiC_possible, phiL_check, phiC_check;
  MatrixXd JL, JL_possible, n_possible, JL_check, n_check;

  auto phiPgrad = model->positionConstraints<double>(*cache,1);
  auto phiP = phiPgrad.value();
  auto JP = phiPgrad.gradient().value();
  model->jointLimitConstraints(q, phiL, JL);  
  
  const size_t nP = phiP.size();
  
  //Convert jacobians to velocity mappings
  const MatrixXd n_velocity = model->transformPositionDotMappingToVelocityMapping(*cache,n);
  const MatrixXd JL_velocity = model->transformPositionDotMappingToVelocityMapping(*cache,JL);
  const auto JP_velocity = model->transformPositionDotMappingToVelocityMapping(*cache,JP);
  
  plhs[2] = mxCreateDoubleMatrix(nv, 1, mxREAL);
  Map<VectorXd> wvn(mxGetPrSafe(plhs[2]), nv);

  LLT<MatrixXd> H_cholesky(H); // compute the Cholesky decomposition of H
  wvn = H_cholesky.solve(B * u - C);
  wvn *= h;
  wvn += v;

  //use forward euler step in joint space as
  //initial guess for active constraints
  vector<bool> possible_contact;
  vector<bool> possible_jointlimit;
  vector<bool> z_inactive;

  getThresholdInclusion((phiC + h * n_velocity * v).eval(), z_inactive_guess_tol, possible_contact);
  getThresholdInclusion((phiL + h * JL_velocity * v).eval(), z_inactive_guess_tol, possible_jointlimit);

  while (true) {
  //continue from here if our inactive guess fails
    const size_t nC = getNumTrue(possible_contact);
    const size_t nL = getNumTrue(possible_jointlimit);
    const size_t lcp_size = nL + nP + (mC + 2) * nC;

    plhs[0] = mxCreateDoubleMatrix(lcp_size, 1, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(nv, lcp_size, mxREAL);

    Map<VectorXd> z(mxGetPrSafe(plhs[0]), lcp_size);
    Map<MatrixXd> Mvn(mxGetPrSafe(plhs[1]), nv, lcp_size);

    if (lcp_size == 0) {
      plhs[3] = mxCreateDoubleMatrix(1, possible_contact.size(), mxREAL);
      plhs[4] = mxCreateDoubleMatrix(1, possible_jointlimit.size(), mxREAL);
      return;
    } 
    z = VectorXd::Zero(lcp_size);

    vector<size_t> possible_contact_indices;
    getInclusionIndices(possible_contact, possible_contact_indices, true);

    partitionVector(possible_contact, phiC, phiC_possible, phiC_check);
    partitionVector(possible_jointlimit, phiL, phiL_possible, phiL_check);
    partitionMatrix(possible_contact, n_velocity, n_possible, n_check);
    partitionMatrix(possible_jointlimit, JL_velocity, JL_possible, JL_check);
    
    MatrixXd D_possible(mC * nC, nv);
    for (size_t i = 0; i < mC ; i++) {
      Map<MatrixXd> D_i(mxGetPrSafe(mxGetCell(D_array, i)), num_contact_pairs , nq);
      MatrixXd D_i_possible, D_i_exclude;
      filterByIndices(possible_contact_indices, D_i, D_i_possible);
      D_possible.block(nC * i, 0, nC, nv) = model->transformPositionDotMappingToVelocityMapping(*cache,D_i_possible);
    }

    // J in velocity coordinates
    MatrixXd J(lcp_size, nv);
    J << JL_possible, JP_velocity, n_possible, D_possible, MatrixXd::Zero(nC, nv);
    Mvn = H_cholesky.solve(J.transpose());
    
    //solve LCP problem 
    //TODO: call path from C++ (currently only 32-bit C libraries available)
    mxArray* mxw = mxCreateDoubleMatrix(lcp_size, 1, mxREAL);
    mxArray* mxlb = mxCreateDoubleMatrix(lcp_size, 1, mxREAL);
    mxArray* mxub = mxCreateDoubleMatrix(lcp_size, 1, mxREAL);  
    
    Map<VectorXd> lb(mxGetPrSafe(mxlb), lcp_size);
    Map<VectorXd> ub(mxGetPrSafe(mxub), lcp_size);
    lb << VectorXd::Zero(nL),
          -BIG * VectorXd::Ones(nP),
          VectorXd::Zero(nC + mC * nC + nC);
    ub = BIG * VectorXd::Ones(lcp_size);

    MatrixXd M(lcp_size, lcp_size);
    Map<VectorXd> w(mxGetPrSafe(mxw), lcp_size);

    //build LCP matrix
    M << h * JL_possible * Mvn,
         h * JP_velocity * Mvn,
         h * n_possible * Mvn,
         D_possible * Mvn,
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
    w << phiL_possible + h *  JL_possible * wvn,
         phiP + h *  JP_velocity * wvn,
         phiC_possible + h * n_possible * wvn,
         D_possible * wvn,
         VectorXd::Zero(nC);

    //try fastQP first
    bool qp_failed = true;
    
    if (enable_fastqp) {
      if (num_z_cached != lcp_size) {
        z_inactive.clear();
        for (int i = 0; i < lcp_size; i++) {
          z_inactive.push_back(true);
        }
      } else {
        getThresholdInclusion((lb - z_cached).eval(), -SMALL, z_inactive);
      }
      qp_failed = !callFastQP(M, w, lb, z_inactive, nL+nP+nC, z);
    }
    
    int nnz;
    mxArray* mxM_sparse = eigenToMatlabSparse(M, nnz);
    mxArray* mxnnzJ = mxCreateDoubleScalar(static_cast<double>(nnz));
    mxArray* mxn = mxCreateDoubleScalar(static_cast<double>(lcp_size));
    mxArray* mxz = mxCreateDoubleMatrix(lcp_size, 1, mxREAL);
    mxArray *lhs[2];
    mxArray *rhs[] = {mxn, mxnnzJ, mxz, mxlb, mxub, mxM_sparse, mxw};

    Map<VectorXd> z_path(mxGetPr(mxz), lcp_size);
    z_path = VectorXd::Zero(lcp_size);
    //fall back to pathlcp
    if(qp_failed) {
      lcp_mex->mexFunction(2, lhs, 7, const_cast<const mxArray**>(rhs));
      z = z_path;
      mxDestroyArray(lhs[0]);
      mxDestroyArray(lhs[1]);
    }

    mxDestroyArray(mxz);
    mxDestroyArray(mxn);
    mxDestroyArray(mxnnzJ);
    mxDestroyArray(mxub);
    mxDestroyArray(mxlb);
    mxDestroyArray(mxw);
    mxDestroyArray(mxM_sparse);

    VectorXd vn = Mvn * z + wvn;

    vector<size_t> impossible_contact_indices, impossible_limit_indices;
    getInclusionIndices(possible_contact, impossible_contact_indices, false);
    getInclusionIndices(possible_jointlimit, impossible_limit_indices, false);

    vector<bool> penetrating_joints, penetrating_contacts;
    getThresholdInclusion((phiL_check + h * JL_check * vn).eval(), 0.0, penetrating_joints);
    getThresholdInclusion((phiC_check + h * n_check * vn).eval(), 0.0, penetrating_contacts);

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

  plhs[3] = mxCreateDoubleMatrix(1, possible_contact.size(), mxREAL);
  plhs[4] = mxCreateDoubleMatrix(1, possible_jointlimit.size(), mxREAL);

  Map<VectorXd> possible_contact_map(mxGetPrSafe(plhs[3]), possible_contact.size());
  Map<VectorXd> possible_jointlimit_map(mxGetPrSafe(plhs[4]), possible_jointlimit.size());

  for (size_t i = 0; i < possible_contact.size(); i++) {
    possible_contact_map[i] = static_cast<double>(possible_contact[i]);
  }

  for (size_t i = 0; i < possible_jointlimit.size(); i++) {
    possible_jointlimit_map[i] = static_cast<double>(possible_jointlimit[i]);
  }

}
