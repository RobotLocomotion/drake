#pragma once

#include <memory>
#include <random>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"
#include "drake/math/gradient.h"
#include "drake/multibody/joints/floating_base_types.h"

// TODO(liang.Fok) Document these method once #3608 is resolved.
#define POSITION_AND_VELOCITY_DEPENDENT_METHODS(Scalar)                      \
                                                                             \
  virtual Eigen::Transform<Scalar, 3, Eigen::Isometry> jointTransform(       \
      const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& q)   \
      const = 0;                                                             \
                                                                             \
  virtual void motionSubspace(                                               \
      const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& q,   \
      Eigen::Matrix<Scalar, drake::kTwistSize, Eigen::Dynamic, 0,            \
                    drake::kTwistSize, MAX_NUM_VELOCITIES>& motion_subspace, \
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>*                 \
          dmotion_subspace = nullptr) const = 0;                             \
                                                                             \
  virtual void motionSubspaceDotTimesV(                                      \
      const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& q,   \
      const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& v,   \
      Eigen::Matrix<Scalar, 6, 1>& motion_subspace_dot_times_v,              \
      drake::math::Gradient<Eigen::Matrix<Scalar, 6, 1>,                     \
                            Eigen::Dynamic>::type*                           \
          dmotion_subspace_dot_times_vdq = nullptr,                          \
      drake::math::Gradient<Eigen::Matrix<Scalar, 6, 1>, Eigen::Dynamic>::   \
          type* dmotion_subspace_dot_times_vdv = nullptr) const = 0;         \
                                                                             \
  virtual void qdot2v(                                                       \
      const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& q,   \
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0,               \
                    MAX_NUM_VELOCITIES, MAX_NUM_POSITIONS>& qdot_to_v,       \
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>* dqdot_to_v)     \
      const = 0;                                                             \
                                                                             \
  virtual void v2qdot(                                                       \
      const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& q,   \
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0,               \
                    MAX_NUM_POSITIONS, MAX_NUM_VELOCITIES>& v_to_qdot,       \
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>* dv_to_qdot)     \
      const = 0;                                                             \
                                                                             \
  virtual Eigen::Matrix<Scalar, Eigen::Dynamic, 1> frictionTorque(           \
      const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& v)   \
      const = 0;                                                             \
                                                                             \
  virtual Eigen::Matrix<Scalar, Eigen::Dynamic, 1> SpringTorque(             \
      const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& q)   \
      const = 0;

/**
 * A joint defines a spatial relationship between two rigid bodies.
 */
class DrakeJoint {
 public:
  /**
   * Defines the maximum number of position states a joint can have.
   */
  static const int MAX_NUM_POSITIONS = 7;

  /**
   * Defines the maximum number of velocity states a joint can have.
   */
  static const int MAX_NUM_VELOCITIES = 6;

  /**
   * A constructor for use by concrete joints to define the joint's name, fixed
   * frame, and number of degrees of freedom.
   *
   * @param[in] name The joint's name. This can be anything; it does not have
   * to be unique.
   *
   * @param[in] transform_to_parent_body Defines the configuration of the
   * joint's fixed frame relative to the joint's parent body's frame.
   *
   * @param[in] num_positions The number of position states of the joint.
   *
   * @param[in] num_velocities The number of velocity states of the joint.
   */
  DrakeJoint(const std::string& name,
             const Eigen::Isometry3d& transform_to_parent_body,
             int num_positions, int num_velocities);

  /**
   * The destructor.
   */
  virtual ~DrakeJoint();

  /**
   * Returns a clone of this DrakeJoint.
   */
  std::unique_ptr<DrakeJoint> Clone() const;

  /**
   * Returns the transform `X_PF` giving the pose of the joint's "fixed" frame
   * `F` in its parent body frame `P`. Frame `F` is the joint frame that is
   * fixed to the parent body; thus `X_PF` is not configuration dependent.
   *
   * To clarify the sense of the returned transform `X_PF`, consider the
   * location of a point `Q` somewhere in space. Let `p_PQ` be point `Q`
   * measured and expressed in frame `P` and `p_FQ` be point `Q` measured and
   * expressed in frame `F`. Then `p_PQ` is given by:
   *
   * <pre>
   * p_PQ = X_PF * p_FQ
   * </pre>
   */
  const Eigen::Isometry3d& get_transform_to_parent_body() const {
    return transform_to_parent_body_;
  }

  /**
   * Returns the number of position states of this joint.
   */
  int get_num_positions() const { return num_positions_; }

  /**
   * Returns the number of velocity states of this joint.
   */
  int get_num_velocities() const { return num_velocities_; }

  /**
   * Returns the name of this joint.
   */
  const std::string& get_name() const { return name_; }

  /**
   * Returns the name of a particular position degree of freedom of this joint.
   *
   * @param[in] index The index of the position degree of freedom. This value
   * must be between 0 and get_num_positions() - 1.
   */
  virtual std::string get_position_name(int index) const = 0;

  /**
   * Returns the name of a particular velocity degree of freedom of this joint.
   *
   * @param[in] index The index of the velocity degree of freedom. This value
   * must be between 0 and get_num_velocities() - 1.
   */
  virtual std::string get_velocity_name(int index) const;

  virtual bool is_floating() const { return false; }

  /**
   * Returns `true` if this joint is a FixedJoint.
   */
  bool is_fixed() const { return num_positions_ == 0; }

  virtual Eigen::VectorXd zeroConfiguration() const = 0;

  virtual Eigen::VectorXd randomConfiguration(
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::default_random_engine& generator) const = 0;

  virtual const Eigen::VectorXd& getJointLimitMin() const;

  virtual const Eigen::VectorXd& getJointLimitMax() const;

  virtual const Eigen::VectorXd& get_joint_limit_stiffness() const;

  virtual const Eigen::VectorXd& get_joint_limit_dissipation() const;

  POSITION_AND_VELOCITY_DEPENDENT_METHODS(double)

  POSITION_AND_VELOCITY_DEPENDENT_METHODS(
      Eigen::AutoDiffScalar<Eigen::VectorXd>)

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  const std::string name_;
  Eigen::VectorXd joint_limit_min_;
  Eigen::VectorXd joint_limit_max_;
  Eigen::VectorXd joint_limit_stiffness_;
  Eigen::VectorXd joint_limit_dissipation_;

 protected:
  /// Allows descendent classes to perform the actual clone operation.
  virtual std::unique_ptr<DrakeJoint> DoClone() const = 0;

  /// Initializes the private member variables within the provided `clone`.
  void InitializeClone(DrakeJoint* clone) const;

  /// Initializes any additional members within @p clone that could not be set
  /// during construction.
  virtual void DoInitializeClone(DrakeJoint* clone) const = 0;

 private:
  const Eigen::Isometry3d transform_to_parent_body_;
  const int num_positions_;
  const int num_velocities_;
};
