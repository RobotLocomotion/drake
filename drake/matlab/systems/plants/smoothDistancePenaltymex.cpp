#include <mex.h>

#include <cmath>
#include <iostream>
#include "drake/matlab/util/drakeMexUtil.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "rigidBodyTreeMexConversions.h"

using namespace Eigen;
using namespace std;

// convert Matlab cell array of strings into a C++ vector of strings
vector<string> get_strings(const mxArray* rhs) {
  int num = static_cast<int>(mxGetNumberOfElements(rhs));
  vector<string> strings(num);
  for (int i = 0; i < num; i++) {
    const mxArray* ptr = mxGetCell(rhs, i);
    int buflen = static_cast<int>(mxGetN(ptr) * sizeof(mxChar)) + 1;
    char* str = (char*)mxMalloc(buflen);
    mxGetString(ptr, str, buflen);
    strings[i] = string(str);
    mxFree(str);
  }
  return strings;
}

void smoothDistancePenalty(double& c, MatrixXd& dc, RigidBodyTree* robot,
                           const KinematicsCache<double>& cache,
                           const double min_distance, const VectorXd& dist,
                           const MatrixXd& normal, const MatrixXd& xA,
                           const MatrixXd& xB, const vector<int>& idxA,
                           const vector<int>& idxB) {
  VectorXd scaled_dist, pairwise_costs;
  MatrixXd ddist_dq, dscaled_dist_ddist, dpairwise_costs_dscaled_dist;

  int num_pts = static_cast<int>(xA.cols());
  ddist_dq = MatrixXd::Zero(num_pts, robot->get_num_positions());

  // Scale distance
  int nd = static_cast<int>(dist.size());
  double recip_min_dist = 1 / min_distance;
  scaled_dist = recip_min_dist * dist - VectorXd::Ones(nd, 1);
  dscaled_dist_ddist = recip_min_dist * MatrixXd::Identity(nd, nd);
  // Compute penalties
  pairwise_costs = VectorXd::Zero(nd, 1);
  dpairwise_costs_dscaled_dist = MatrixXd::Zero(nd, nd);
  for (int i = 0; i < nd; ++i) {
    if (scaled_dist(i) < 0) {
      double exp_recip_scaled_dist = exp(1 / scaled_dist(i));
      pairwise_costs(i) = -scaled_dist(i) * exp_recip_scaled_dist;
      dpairwise_costs_dscaled_dist(i, i) =
          exp_recip_scaled_dist * (1 / scaled_dist(i) - 1);
    }
  }

  // Compute Jacobian of closest distance vector
  std::vector<std::vector<int> > orig_idx_of_pt_on_bodyA(robot->bodies.size());
  std::vector<std::vector<int> > orig_idx_of_pt_on_bodyB(robot->bodies.size());
  for (int k = 0; k < num_pts; ++k) {
    // DEBUG
    // std::cout << "MinDistanceConstraint::eval: First loop: " << k <<
    // std::endl;
    // std::cout << "pairwise_costs.size() = " << pairwise_costs.size() <<
    // std::endl;
    // std::cout << "pairwise_costs.size() = " << pairwise_costs.size() <<
    // std::endl;
    // END_DEBUG
    if (pairwise_costs(k) > 0) {
      orig_idx_of_pt_on_bodyA.at(idxA.at(k)).push_back(k);
      orig_idx_of_pt_on_bodyB.at(idxB.at(k)).push_back(k);
    }
  }
  for (int k = 0; k < robot->bodies.size(); ++k) {
    // DEBUG
    // std::cout << "MinDistanceConstraint::eval: Second loop: " << k <<
    // std::endl;
    // END_DEBUG
    int l = 0;
    int numA = static_cast<int>(orig_idx_of_pt_on_bodyA.at(k).size());
    int numB = static_cast<int>(orig_idx_of_pt_on_bodyB.at(k).size());
    if (numA + numB == 0) {
      continue;
    }
    Matrix3Xd x_k(3, numA + numB);
    for (; l < numA; ++l) {
      // DEBUG
      // std::cout << "MinDistanceConstraint::eval: Third loop: " << l <<
      // std::endl;
      // END_DEBUG
      x_k.col(l) = xA.col(orig_idx_of_pt_on_bodyA.at(k).at(l));
    }
    for (; l < numA + numB; ++l) {
      // DEBUG
      // std::cout << "MinDistanceConstraint::eval: Fourth loop: " << l <<
      // std::endl;
      // END_DEBUG
      x_k.col(l) = xB.col(orig_idx_of_pt_on_bodyB.at(k).at(l - numA));
    }
    auto J_k = robot->transformPointsJacobian(cache, x_k, k, 0, true);
    l = 0;
    for (; l < numA; ++l) {
      // DEBUG
      // std::cout << "MinDistanceConstraint::eval: Fifth loop: " << l <<
      // std::endl;
      // END_DEBUG
      ddist_dq.row(orig_idx_of_pt_on_bodyA.at(k).at(l)) +=
          normal.col(orig_idx_of_pt_on_bodyA.at(k).at(l)).transpose() *
          J_k.block(3 * l, 0, 3, robot->get_num_positions());
    }
    for (; l < numA + numB; ++l) {
      // DEBUG
      // std::cout << "MinDistanceConstraint::eval: Sixth loop: " << l <<
      // std::endl;
      // END_DEBUG
      ddist_dq.row(orig_idx_of_pt_on_bodyB.at(k).at(l - numA)) +=
          -normal.col(orig_idx_of_pt_on_bodyB.at(k).at(l - numA)).transpose() *
          J_k.block(3 * l, 0, 3, robot->get_num_positions());
    }
  }
  MatrixXd dcost_dscaled_dist(dpairwise_costs_dscaled_dist.colwise().sum());
  c = pairwise_costs.sum();
  dc = dcost_dscaled_dist * dscaled_dist_ddist * ddist_dq;
}

/*
 * mex interface for evaluating a smooth-penalty on violations of a
 * minimum-distance constraint. For each eligible pair of collision geometries,
 * a penalty is computed according to
 *
 * \f[
 * c =
 * \begin{cases}
 *   -de^{\frac{1}{d}}, & d <   0  \\
 *   0,                & d \ge 0.
 * \end{cases}
 * \f]
 *
 * where $d$ is the normalized violation of the minimum distance, $d_{min}$
 *
 * \f[
 * d = \frac{(\phi - d_{min})}{d_{min}}
 * \f]
 *
 * for a signed distance of $\phi$ between the geometries. These pairwise costs
 * are summed to yield a single penalty which is zero if and only if all
 * eligible pairs of collision geometries are separated by at least $d_{min}$.
 *
 * MATLAB signature:
 *
 * [penalty, dpenalty] = ...
 *    smoothDistancePenaltymex( mex_model_ptr, min_distance,
 *allow_multiple_contacts,
 *                        active_collision_options);
 */

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  if (nrhs < 4) {
    mexErrMsgIdAndTxt("Drake:smoothDistancePenaltymex:NotEnoughInputs",
                      "Usage smoothDistancePenaltymex(model_ptr, cache_ptr, "
                      "min_distance, active_collision_options)");
  }

  int arg_num = 0;
  RigidBodyTree* model =
      static_cast<RigidBodyTree*>(getDrakeMexPointer(prhs[arg_num++]));
  KinematicsCache<double>& cache =
      fromMex(prhs[arg_num++], static_cast<KinematicsCache<double>*>(nullptr));

  // Get the minimum allowable distance
  double min_distance = mxGetScalar(prhs[arg_num++]);

  // Parse `active_collision_options`
  vector<int> active_bodies_idx;
  set<string> active_group_names;
  // First get the list of body indices for which to compute distances
  const mxArray* active_collision_options = prhs[arg_num++];
  const mxArray* body_idx = mxGetField(active_collision_options, 0, "body_idx");
  if (body_idx != NULL) {
    int n_active_bodies = static_cast<int>(mxGetNumberOfElements(body_idx));
    active_bodies_idx.resize(n_active_bodies);
    if (mxGetClassID(body_idx) == mxINT32_CLASS) {
      memcpy(active_bodies_idx.data(), (int*)mxGetData(body_idx),
             sizeof(int) * n_active_bodies);
    } else if (mxGetClassID(body_idx) == mxDOUBLE_CLASS) {
      double* ptr = mxGetPrSafe(body_idx);
      for (int i = 0; i < n_active_bodies; i++)
        active_bodies_idx[i] = static_cast<int>(ptr[i]);
    } else {
      mexErrMsgIdAndTxt("Drake:smoothDistancePenaltymex:WrongInputClass",
                        "active_collision_options.body_idx must be an int32 or "
                        "a double array");
    }
    transform(active_bodies_idx.begin(), active_bodies_idx.end(),
              active_bodies_idx.begin(), [](int i) { return --i; });
  }

  // Then get the group names for which to compute distances
  const mxArray* collision_groups =
      mxGetField(active_collision_options, 0, "collision_groups");
  if (collision_groups != NULL) {
    int num = static_cast<int>(mxGetNumberOfElements(collision_groups));
    for (int i = 0; i < num; i++) {
      const mxArray* ptr = mxGetCell(collision_groups, i);
      int buflen = static_cast<int>(mxGetN(ptr) * sizeof(mxChar)) + 1;
      char* str = (char*)mxMalloc(buflen);
      mxGetString(ptr, str, buflen);
      active_group_names.insert(str);
      mxFree(str);
    }
  }

  double penalty;
  vector<int> bodyA_idx, bodyB_idx;
  Matrix3Xd ptsA, ptsB, normals;
  MatrixXd dpenalty;
  VectorXd dist;
  if (active_bodies_idx.size() > 0) {
    if (active_group_names.size() > 0) {
      model->collisionDetect(cache, dist, normals, ptsA, ptsB, bodyA_idx,
                             bodyB_idx, active_bodies_idx, active_group_names);
    } else {
      model->collisionDetect(cache, dist, normals, ptsA, ptsB, bodyA_idx,
                             bodyB_idx, active_bodies_idx);
    }
  } else {
    if (active_group_names.size() > 0) {
      model->collisionDetect(cache, dist, normals, ptsA, ptsB, bodyA_idx,
                             bodyB_idx, active_group_names);
    } else {
      model->collisionDetect(cache, dist, normals, ptsA, ptsB, bodyA_idx,
                             bodyB_idx);
    }
  }

  smoothDistancePenalty(penalty, dpenalty, model, cache, min_distance, dist,
                        normals, ptsA, ptsB, bodyA_idx, bodyB_idx);

  vector<int32_T> idxA(bodyA_idx.size());
  transform(bodyA_idx.begin(), bodyA_idx.end(), idxA.begin(),
            [](int i) { return ++i; });
  vector<int32_T> idxB(bodyB_idx.size());
  transform(bodyB_idx.begin(), bodyB_idx.end(), idxB.begin(),
            [](int i) { return ++i; });

  if (nlhs > 0) {
    plhs[0] = mxCreateDoubleScalar(penalty);
  }
  if (nlhs > 1) {
    plhs[1] = mxCreateDoubleMatrix(static_cast<int>(dpenalty.rows()),
                                   static_cast<int>(dpenalty.cols()), mxREAL);
    memcpy(mxGetPrSafe(plhs[1]), dpenalty.data(),
           sizeof(double) * dpenalty.size());
  }
}
