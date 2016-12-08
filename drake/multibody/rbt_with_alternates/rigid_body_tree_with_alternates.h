#pragma once

#include <set>
#include <string>

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/nice_type_name.h"
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
  std::string type() const { return NiceTypeName::Get<T>(); }

  // Return interesting concrete System-specific stuff.
  const RigidBodyTree<T> &get_rigid_body_tree() const { return *tree_; }

  int get_num_positions() const { return tree_->get_num_positions(); }

  int get_num_velocities() const { return tree_->get_num_velocities(); }

  KinematicsCache<T> CreateKinematicsCache() const {
    return tree_->CreateKinematicsCache();
  }

  template<typename Scalar>
  KinematicsCache<Scalar> CreateKinematicsCacheWithType() const {
    // TODO(amcastro-tri): replace <double> by <Scalar> once
    // RigidBodyTree<T>::RigidBodyTree(const RigidBodyTree<double>&) is
    // implemented.
    // This should also work.
    //return this->template get_alternate<double>().get_rigid_body_tree().
    //    CreateKinematicsCache();
    return tree_->CreateKinematicsCacheWithType<Scalar>();
  }

  template<typename Scalar>
  Vector3<Scalar> centerOfMass(
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      KinematicsCache<Scalar> &cache,
      const std::set<int> &model_instance_id_set = {0}) const {
    // TODO(amcastro-tri): replace <double> by <Scalar> once
    // RigidBodyTree<T>::RigidBodyTree(const RigidBodyTree<double>&) is
    // implemented.
    return this->template get_alternate<double>().get_rigid_body_tree().
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
    return this->template get_alternate<double>().get_rigid_body_tree().
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
    return this->template get_alternate<double>().get_rigid_body_tree().
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
    return this->template get_alternate<double>().get_rigid_body_tree().
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
    return this->template get_alternate<double>().get_rigid_body_tree().
        centroidalMomentumMatrix(cache, model_instance_id_set, in_terms_of_qdot);
  }

  template <typename Scalar>
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void doKinematics(KinematicsCache<Scalar>& cache,
                    bool compute_JdotV = false) const {
    // TODO(amcastro-tri): replace <double> by <Scalar> once
    // RigidBodyTree<T>::RigidBodyTree(const RigidBodyTree<double>&) is
    // implemented.
    this->template get_alternate<double>().get_rigid_body_tree().
        doKinematics(cache, compute_JdotV);
  }

  KinematicPath findKinematicPath(int start_body_or_frame_idx,
                                  int end_body_or_frame_idx) const {
    // Any alternate should return the same result.
    return this->template get_alternate<double>().get_rigid_body_tree().
        findKinematicPath(start_body_or_frame_idx, end_body_or_frame_idx);
  }

  template <typename Scalar, typename DerivedPoints>
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> transformPointsJacobianDotTimesV(
      const KinematicsCache<Scalar>& cache,
      const Eigen::MatrixBase<DerivedPoints>& points,
      int from_body_or_frame_ind, int to_body_or_frame_ind) const {
    // TODO(amcastro-tri): replace <double> by <Scalar> once
    // RigidBodyTree<T>::RigidBodyTree(const RigidBodyTree<double>&) is
    // implemented.
    return this->template get_alternate<double>().get_rigid_body_tree().
        transformPointsJacobianDotTimesV(cache, points,
                                         from_body_or_frame_ind,
                                         to_body_or_frame_ind);
  };

  template <typename Scalar, typename DerivedPoints>
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> transformPointsJacobian(
      const KinematicsCache<Scalar>& cache,
      const Eigen::MatrixBase<DerivedPoints>& points,
      int from_body_or_frame_ind, int to_body_or_frame_ind,
      bool in_terms_of_qdot) const {
    // TODO(amcastro-tri): replace <double> by <Scalar> once
    // RigidBodyTree<T>::RigidBodyTree(const RigidBodyTree<double>&) is
    // implemented.
    return this->template get_alternate<double>().get_rigid_body_tree().
        transformPointsJacobian(cache, points,
                                from_body_or_frame_ind, to_body_or_frame_ind,
                                in_terms_of_qdot);
  };

  bool collisionDetect(
      const KinematicsCache<double>& cache,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::VectorXd& phi,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& normal,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& xA,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& xB,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<int>& bodyA_idx,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<int>& bodyB_idx,
      const std::vector<int>& bodies_idx,
      const std::set<std::string>& active_element_groups,
      bool use_margins = true) {
    return tree_->collisionDetect(cache, phi, normal, xA, xB, bodyA_idx,
                                  bodyB_idx, bodies_idx,
                                  active_element_groups, use_margins);
  }

  bool collisionDetect(
      const KinematicsCache<double>& cache,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::VectorXd& phi,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& normal,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& xA,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& xB,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<int>& bodyA_idx,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<int>& bodyB_idx,
      const std::vector<int>& bodies_idx,
      bool use_margins = true) {
    return tree_->collisionDetect(cache, phi, normal, xA, xB, bodyA_idx,
                                  bodyB_idx, bodies_idx, use_margins);
  }

  bool collisionDetect(
      const KinematicsCache<double>& cache,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::VectorXd& phi,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& normal,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& xA,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& xB,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<int>& bodyA_idx,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<int>& bodyB_idx,
      const std::set<std::string>& active_element_groups,
      bool use_margins = true) {
    return tree_->collisionDetect(cache, phi, normal, xA, xB,
                                  bodyA_idx, bodyB_idx,
                                  active_element_groups, use_margins);
  }

  bool collisionDetect(
      const KinematicsCache<double>& cache,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::VectorXd& phi,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& normal,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& xA,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& xB,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<int>& bodyA_idx,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<int>& bodyB_idx,
      bool use_margins = true) {
    return tree_->collisionDetect(cache, phi, normal, xA, xB,
                                  bodyA_idx, bodyB_idx, use_margins);
  }

  void potentialCollisions(
      const KinematicsCache<double>& cache,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::VectorXd& phi,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& normal,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& xA,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& xB,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<int>& bodyA_idx,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<int>& bodyB_idx,
      bool use_margins = true) {
    return tree_->potentialCollisions(cache, phi, normal, xA, xB,
                                      bodyA_idx, bodyB_idx, use_margins);
  }

 private:
  // Let all other instantiations of this class be friends with this one.
  template<typename TT>
  friend
  class RigidBodyTreeWithAlternates;

  std::unique_ptr<RigidBodyTree<T>> tree_;
};

}  // namespace drake