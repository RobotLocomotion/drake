#pragma once

#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/eigen_stl_types.h"
#include "drake/multibody/rigid_body_tree.h"

// The minimum set of required methods is here: rigidBodyTreeMexFunctions.h

class MatlabRigidBodyTree {
 public:
  // In constructModelmex.cpp use RigidBodyTree<double> as the IR that then
  // is used by MatlabRigidBodyTree to get constructed.
  // Also take ownership of the <double> version so you don't need to clone it.
  MatlabRigidBodyTree(std::unique_ptr<RigidBodyTree<double>> tree);

  ~MatlabRigidBodyTree();

  template <typename Scalar>
  Eigen::Matrix<Scalar, drake::kSpaceDimension, 1>
  centerOfMassJacobianDotTimesV(
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      KinematicsCache<Scalar>& cache,
      const std::set<int>& model_instance_id_set =
      default_model_instance_id_set) const;

 private:
  std::unique_ptr<RigidBodyTree<double>> tree_double_;
  //std::unique_ptr<RigidBodyTree<AutoDiffXd>> tree_dynamic_autodiff_;

  // Use a map? so that I don't need to create a lot of overloaded methods?
  //std::map<some_sort_of_type_id, void*>
};
