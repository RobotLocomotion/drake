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

inline void getInclusionIndices(vector<bool> const & inclusion, vector<int> & indices, bool getTrueIndices)
{
  const int n = inclusion.size();
  indices.clear();
  for (int x = 0; x < n; x++) {
    if(inclusion[x] == getTrueIndices){
      indices.push_back(x);
    }
  }
}

template <typename Derived>
inline void getThresholdInclusion(MatrixBase<Derived> const &values, const double threshold, vector<bool> & belowThreshold)
{
  const int n = values.size();
  belowThreshold.clear();
  for (int x = 0; x < n; x++) {
    belowThreshold.push_back(values[x]<threshold);
  }
}

//counts number of inclusions
inline int getNumTrue(vector<bool> const & bools)
{
  int count = 0;
  const int n = bools.size();
  for (int x = 0; x < n; x++) {
    if (bools[x]) {
      count++;
    }
  }
  return count;
}

//splits a vector into two based on inclusion mapping
inline void partitionVector(vector<bool> const &indices, VectorXd const & v, VectorXd & included, VectorXd & excluded)
{
  const int n = indices.size();
  const int count = getNumTrue(indices);

  included = VectorXd::Zero(count);
  excluded = VectorXd::Zero(n-count);
  
  int inclusionIndex = 0;
  int exclusionIndex = 0;

  for (int x = 0; x < n; x++) {
    if (indices[x]) {
      included[inclusionIndex++] = v[x];
    } else {
      excluded[exclusionIndex++] = v[x];
    }
  }
}

//splits a matrix into two based on a row inclusion mapping
inline void partitionMatrix(vector<bool> const &indices, MatrixXd const & M, MatrixXd & included, MatrixXd & excluded)
{   
  const int n = indices.size();
  const int count = getNumTrue(indices);
  const int cols = M.cols();

  included = MatrixXd::Zero(count, cols);
  excluded = MatrixXd::Zero(n-count, cols);

  int inclusionIndex = 0;
  int exclusionIndex = 0;

  for (int x = 0; x < n; x++) {
    if (indices[x]) {
      included.row(inclusionIndex++) = M.row(x);
    } else {
      excluded.row(exclusionIndex++) = M.row(x);
    }
  }
}

//builds a filtered matrix containing only the rows specified by indices
inline void filterByIndices(vector<int> const &indices, MatrixXd const & M, MatrixXd & filtered)
{
  const int n = indices.size();
  filtered = MatrixXd::Zero(n, M.cols());
  for (int x = 0; x < n; x++) {
    filtered.row(x) = M.row(indices[x]);
  }
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

  VectorXd C, phiL, phiP, phiL_possible, phiC_possible, phiL_check, phiC_check;
  MatrixXd H, B, JP, JL, JL_possible, n_possible, JL_check, n_check;

  manipulatorDynamics(model, q, v, H, C, B);

  model->positionConstraints(phiP, JP);
  model->jointLimitConstraints(q, phiL, JL);
  MatrixXd Hinv = H.inverse();
  const int nP = phiP.size();
  
  plhs[2] = mxCreateDoubleMatrix(nq, 1, mxREAL);
  Map<VectorXd> wqdn(mxGetPr(plhs[2]), nq);
  wqdn = v + h*Hinv*(B*u - C);

  //use forward euler step in joint space as
  //initial guess for active constraints
  vector<bool> possible_contact;
  vector<bool> possible_jointlimit;
  getThresholdInclusion(phiC + h*n*v, z_inactive_guess_tol, possible_contact);
  getThresholdInclusion(phiL + h*JL*v, z_inactive_guess_tol, possible_jointlimit);

  while (true) {
  //continue from here if our inactive guess fails
    const int nC = getNumTrue(possible_contact);
    const int nL = getNumTrue(possible_jointlimit);
    const int LCP_size = nL+nP+(mC+2)*nC;
    
    vector<int> possible_contact_indices;
    getInclusionIndices(possible_contact, possible_contact_indices, true);

    partitionVector(possible_contact, phiC, phiC_possible, phiC_check);
    partitionVector(possible_jointlimit, phiL, phiL_possible, phiL_check);
    partitionMatrix(possible_contact, n, n_possible, n_check);
    partitionMatrix(possible_jointlimit, JL, JL_possible, JL_check);
    
    MatrixXd D_possible(mC*nC, nq);
    for (int i = 0; i < mC ; i++) {
      Map<MatrixXd> D_i(mxGetPr(mxGetCell(prhs[6], i)), numContactPairs , nq);
      MatrixXd D_i_possible, D_i_exclude;
      filterByIndices(possible_contact_indices, D_i, D_i_possible);
      D_possible.block(nC*i, 0, nC, nq) = D_i_possible;
    }


    MatrixXd J(LCP_size, nq);
    J << JL_possible, JP, n_possible, D_possible, MatrixXd::Zero(nC, nq);

    plhs[0] = mxCreateDoubleMatrix(LCP_size, 1, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(nq, LCP_size, mxREAL);

    Map<VectorXd> z(mxGetPr(plhs[0]), LCP_size);
    Map<MatrixXd> Mqdn(mxGetPr(plhs[1]), nq, LCP_size);
    Mqdn = Hinv*J.transpose();
    
    //solve LCP problem 
    //TODO: call fastQP first
    //TODO: call path from C++ (currently only 32-bit C libraries available)
    mxArray* mxM = mxCreateDoubleMatrix(LCP_size, LCP_size, mxREAL);
    mxArray* mxw = mxCreateDoubleMatrix(LCP_size, 1, mxREAL);
    mxArray* mxlb = mxCreateDoubleMatrix(LCP_size, 1, mxREAL);
    mxArray* mxub = mxCreateDoubleMatrix(LCP_size, 1, mxREAL);  
    Map<VectorXd> lb(mxGetPr(mxlb), LCP_size);
    Map<VectorXd> ub(mxGetPr(mxub), LCP_size);
    lb << VectorXd::Zero(nL),
          -1e20*VectorXd::Ones(nP),
          VectorXd::Zero(nC+mC*nC+nC);
    ub = 1e20*VectorXd::Ones(LCP_size);
    
    Map<MatrixXd> M(mxGetPr(mxM),LCP_size, LCP_size);
    Map<VectorXd> w(mxGetPr(mxw), LCP_size);

    //build LCP matrix
    M << h*JL_possible*Mqdn,
         h*JP*Mqdn,
         h*n_possible*Mqdn,
         D_possible*Mqdn,
         MatrixXd::Zero(nC, LCP_size);
    

    if (nC > 0) {
      for (int i=0; i < mC ; i++) {
        M.block(nL+nP+nC+nC*i, nL+nP+nC+mC*nC, nC, nC) = MatrixXd::Identity(nC, nC);
        M.block(nL+nP+nC+mC*nC, nL+nP+nC+nC*i, nC, nC) = -1.0*MatrixXd::Identity(nC, nC);
      }
      double mu = 1.0; //TODO: pull this from contactConstraints
      M.block(nL+nP+nC+mC*nC, nL+nP, nC, nC) = mu*MatrixXd::Identity(nC, nC);
    }

    //build LCP vector
    w << phiL_possible + h*JL_possible*wqdn,
         phiP + h*JP*wqdn,
         phiC_possible + h*n_possible*wqdn,
         D_possible*wqdn,
         VectorXd::Zero(nC);

    mxArray *lhs[1];
    mxArray *rhs[] = {mxM, mxw, mxlb, mxub};
    
    //call solver
    mexCallMATLAB(1, lhs, 4, rhs, "pathlcp");
    Map<VectorXd> z_path(mxGetPr(lhs[0]), LCP_size);
    z = z_path;

    //clean up
    mxDestroyArray(lhs[0]);
    mxDestroyArray(mxlb);
    mxDestroyArray(mxub);
    mxDestroyArray(mxM);
    mxDestroyArray(mxw);
    
    VectorXd qdn = Mqdn*z + wqdn;

    vector<int> impossible_contact_indices, impossible_limit_indices;
    getInclusionIndices(possible_contact, impossible_contact_indices, false);
    getInclusionIndices(possible_jointlimit, impossible_limit_indices, false);

    vector<bool> penetrating_joints, penetrating_contacts;
    getThresholdInclusion(phiL_check + h*JL_check*qdn, 0.0, penetrating_joints);
    getThresholdInclusion(phiC_check + h*n_check*qdn, 0.0, penetrating_contacts);

    const int pJoints = getNumTrue(penetrating_joints);
    const int pContacts = getNumTrue(penetrating_contacts);
    const int penetrations = pJoints + pContacts;

    //check nonpenetration assumptions
    if (penetrations > 0) {
      //revise joint limit active set
      for (int x = 0; x < impossible_limit_indices.size(); x++) {
        if (penetrating_joints[x]) {
          possible_jointlimit[impossible_limit_indices[x]] = true;
        }
      }

      //revise contact constraint active set
      for (int x = 0; x < impossible_contact_indices.size(); x++) {
        if (penetrating_contacts[x]) {
          possible_contact[impossible_contact_indices[x]] = true;
        }
      }
      //form new LCP
      continue;
    }

    //our initial guess was correct. we're done
    break;
  }

}
