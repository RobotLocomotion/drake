#pragma once

#include <set>

#include "drake/common/eigen_autodiff_types.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rbt_with_alternates/object_with_alternates.h"

namespace drake {

const std::set<int> default_model_instance_id_set = {0};

template<typename T>
class RigidBodyTreeWithAlternates :
    public ObjectWithAlternates<RigidBodyTreeWithAlternates, T> {  // CRTP!
  using Super = ObjectWithAlternates<drake::RigidBodyTreeWithAlternates, T>;

 public:
  // Construct and take over ownership of the given RigidBodyTree.
  RigidBodyTreeWithAlternates(std::unique_ptr<RigidBodyTree<T>> tree) :
      tree_(std::move(tree)) {}

  // Construct an alternate MySystem<T> with same structure as the given one.
  // This is also the copy constructor for the T=double fundamental.
  explicit RigidBodyTreeWithAlternates(
      const RigidBodyTreeWithAlternates<double> &fundamental)
      : Super(fundamental), tree_(new RigidBodyTree<T>(*fundamental.tree_)) {}

  // For demo purposes, report the name of the scalar type here.
  const char *type() const { return typeid(T).name(); }

  // Return interesting concrete System-specific stuff.
  const RigidBodyTree<T> &get_rigid_body_tree() const { return *tree_; }

  template<typename Scalar>
  Vector3<Scalar> centerOfMass(
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      KinematicsCache<Scalar> &cache,
      const std::set<int> &model_instance_id_set = {0}) const {
    // TODO(amcastro-tri): replace <double> by <Scalar> once
    // RigidBodyTree<T>::RigidBodyTree(const RigidBodyTree<double>&) is
    // implemented.
    this->template get_alternate<double>().get_rigid_body_tree().
        centerOfMass(cache, model_instance_id_set);
  }

  template <typename Scalar>
  Eigen::Matrix<Scalar, drake::kSpaceDimension, 1>
  centerOfMassJacobianDotTimesV(
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      KinematicsCache<Scalar>& cache,
      const std::set<int>& model_instance_id_set =
      default_model_instance_id_set) const {
    // TODO(amcastro-tri): replace <double> by <Scalar> once
    // RigidBodyTree<T>::RigidBodyTree(const RigidBodyTree<double>&) is
    // implemented.
    this->template get_alternate<double>().get_rigid_body_tree().
        centerOfMassJacobianDotTimesV(cache, model_instance_id_set);
  };

  template <typename Scalar>
  Eigen::Matrix<Scalar, drake::kSpaceDimension, Eigen::Dynamic>
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  centerOfMassJacobian(KinematicsCache<Scalar>& cache,
                       const std::set<int>& model_instance_id_set =
                       default_model_instance_id_set,
                       bool in_terms_of_qdot = false) const {
    // TODO(amcastro-tri): replace <double> by <Scalar> once
    // RigidBodyTree<T>::RigidBodyTree(const RigidBodyTree<double>&) is
    // implemented.
    this->template get_alternate<double>().get_rigid_body_tree().
        centerOfMassJacobian(cache, model_instance_id_set, in_terms_of_qdot);
  };

  template <typename Scalar>
  drake::TwistVector<Scalar> centroidalMomentumMatrixDotTimesV(
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      KinematicsCache<Scalar>& cache,
      const std::set<int>& model_instance_id_set =
      default_model_instance_id_set) const {
    // TODO(amcastro-tri): replace <double> by <Scalar> once
    // RigidBodyTree<T>::RigidBodyTree(const RigidBodyTree<double>&) is
    // implemented.
    this->template get_alternate<double>().get_rigid_body_tree().
        centroidalMomentumMatrixDotTimesV(cache, model_instance_id_set);
  }

  template <typename Scalar>
  drake::TwistMatrix<Scalar> centroidalMomentumMatrix(
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      KinematicsCache<Scalar>& cache,
      const std::set<int>& model_instance_id_set =
      default_model_instance_id_set,
      bool in_terms_of_qdot = false) const {
    // TODO(amcastro-tri): replace <double> by <Scalar> once
    // RigidBodyTree<T>::RigidBodyTree(const RigidBodyTree<double>&) is
    // implemented.
    this->template get_alternate<double>().get_rigid_body_tree().
        centroidalMomentumMatrix(cache, model_instance_id_set, in_terms_of_qdot);
  }

  


 private:
  // Let all other instantiations of this class be friends with this one.
  template<typename TT>
  friend
  class RigidBodyTreeWithAlternates;

  std::unique_ptr<RigidBodyTree<T>> tree_;
};

}  // namespace drake