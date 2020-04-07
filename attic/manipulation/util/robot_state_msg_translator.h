#pragma once

#include <string>
#include <unordered_map>
#include <utility>

#include "bot_core/robot_state_t.hpp"

#include "drake/common/drake_deprecated.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace manipulation {

/**
 * This is a utility class for converting bot_core::robot_state_t message to
 * and from various eigen vectors that correspond to the generalized position,
 * velocity and actuator torque. The bot_core::robot_state_t has three major
 * components:
 * <pre>
 *   kinematics: generalized position and velocity including the floating base.
 *   actuator torques: joint torque for actuated joints.
 *   force torque sensor: foot / wrist mounted force torque sensor readings.
 * </pre>
 * The pose in the message represents the transformation from the floating
 * base frame B to the world frame W (X_WB), and the velocity is its spatial
 * velocity V_WB.
 * For the non-floating joints in bot_core::robot_state_t, the convention is
 * that they (name, position, velocity, effort) are stored in equal length
 * vectors, and the numerical values are identified by the joint name with
 * the same index. This joint name is also used to compare against position
 * names in a RigidBodyTree to establish correspondence between the data
 * layouts in the message and the RigidBodyTree. Note that this class does
 * not require one to one joint matching between the message and the
 * RigidBodyTree. This class only translates information for joints that
 * are shared by the message the RigidBodyTree. This design choice is made
 * to facilitate multiple models (rigid body tree) with different complexity
 * of the same robot communicating on the same Lcm channel. For example, a
 * central message publisher generates a Lcm message with the most
 * comprehensive information, and the individual subscribers can pay attention
 * to whatever subset of interest using reduced models.
 */
class DRAKE_DEPRECATED("2020-05-01",
    "The attic/manipulation/util package is being removed.")
RobotStateLcmMessageTranslator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RobotStateLcmMessageTranslator)

  /**
   * Constructs a translator using the model given in @p robot. A const
   * reference of @p robot is saved, and thus its life span must be longer
   * than this. The following assumptions must be met by @p robot, otherwise
   * the constructor throws a std::logic_error:
   * <pre>
   *   1. There is at least 1 non-world rigid body.
   *   2. The first non-world rigid body has to be attached to the world with
   *   either a fixed joint or a floating joint.
   *   3. If the first non-world rigid body is attached with a fixed joints,
   *   there cannot be another floating joint in the tree.
   *   4. There is at most 1 floating joint in the tree.
   *   5. There can be at most 1 rigid body attached to the world with a fixed
   *   joint.
   *   6. If there is a floating joint, its position and velocity index have
   *   to start from 0.
   *   7. All the other joints have to be 1 degree of freedom or fixed.
   * </pre>
   */
  explicit RobotStateLcmMessageTranslator(const RigidBodyTree<double>& robot);

  /**
   * Initializes @p msg based on the rigid body tree passed to the constructor.
   * It resizes all the vectors for the non-floating joints in the order of
   * the rigid body tree's generalized coordinate. All numerical values are
   * initialized to zero. It is highly recommended to initialize
   * bot_core::robot_state_t with this method before passing it to the rest of
   * the API in this class.
   */
  void InitializeMessage(bot_core::robot_state_t* msg) const;

  /**
   * Decodes only the kinematic values (q, v) from @p msg to @p q and @p v.
   * @p q and @p v are assumed to match the rigid body tree's generalized
   * position and velocity size. Note that the values in @p q and @p v that
   * correspond to the floating base will always be set to the decoded values
   * in @p msg. However, for the non-floating joints, only the ones that are
   * present in the message will be decoded and set. For example, suppose the
   * rigid body tree has 2 non-floating joints: j1, and j2. If @p msg only
   * contains j1, then j2's corresponding position and velocity in @p q and
   * @p v will not be set. Conversely, if @p msg contains extra joints than
   * in the rigid body tree, their information are simply ignored.
   *
   * @param[in] msg Lcm message to be decoded.
   * @param[out] q Vector to hold the decoded generalized position.
   * @param[out] v Vector to hold the decoded generalized velocity.
   */
  void DecodeMessageKinematics(const bot_core::robot_state_t& msg,
                               Eigen::Ref<VectorX<double>> q,
                               Eigen::Ref<VectorX<double>> v) const;

  /**
   * Encodes only the generalized position @p q and velocity @p v to @p msg.
   * The information in @p msg that correspond to the floating base will
   * always be set according to @p q and @p v, or identity / zero if there
   * is no floating base. Only joints whose names are presented in @p msg
   * and in the rigid body tree will be set. Suppose @p msg lists 2
   * non-floating joints: j1 and j2, and the rigid body tree only has j1, then
   * j2's corresponding position and velocity will not be set in @p msg.
   * Conversely, if the rigid body tree has joints that are not in @p msg,
   * their information will not be encoded.
   *
   * @param[in] q Generalized position that corresponds to the rigid body tree
   * passed to the constructor.
   * @param[in] v Generalized velocity that corresponds to the rigid body tree
   * passed to the constructor.
   * @param[in,out] msg Lcm message to be encoded.
   */
  void EncodeMessageKinematics(const Eigen::Ref<const VectorX<double>>& q,
                               const Eigen::Ref<const VectorX<double>>& v,
                               bot_core::robot_state_t* msg) const;

  /**
   * Decodes only the non-floating joint torque part from @p msg to @p torque.
   * @p torque is assumed to have the same size as the rigid body tree's number
   * of actuators, and it is assumed to be in the same order, which is not
   * necessarily in the same order as its generalized position or velocity.
   * Only the actuated joints in the rigid body tree will be decoded (if they
   * are present in @p msg).
   */
  void DecodeMessageTorque(const bot_core::robot_state_t& msg,
                           Eigen::Ref<VectorX<double>> torque) const;

  /**
   * Encodes only the actuated non-floating joint torque to @p msg.
   * @p torque needs to match the rigid body tree's actuator size, and it is
   * assumed to be in the same order as the rigid body tree's actuators, which
   * is not necessarily in the same order as its generalized position or
   * velocity. Only the actuated joints listed in @p msg will be encoded.
   */
  void EncodeMessageTorque(const Eigen::Ref<const VectorX<double>>& torque,
                           bot_core::robot_state_t* msg) const;

  /**
   * Returns a const reference to the rigid body tree.
   */
  const RigidBodyTree<double>& get_robot() const { return robot_; }

  /**
   * Checks @p msg non-floating joint parts have consistent dimensions.
   */
  static bool CheckMessageVectorSize(const bot_core::robot_state_t& msg);

 private:
  // Check that robot_state_t can unambiguously represent the RigidBodyTree's
  // state. This method is intended to check preconditions inside a
  // constructor's intitializer list. Returns @p robot if the checks pass.
  // Throws std::logic_error when @p robot is not compatible with robot_state_t.
  static const RigidBodyTree<double>& CheckTreeIsRobotStateLcmTypeCompatible(
      const RigidBodyTree<double>& robot);

  const RigidBodyTree<double>& robot_;
  const RigidBody<double>& root_body_;
  const bool is_floating_base_;

  // Maps from non-floating joint's position name to various indices.
  std::unordered_map<std::string, int> joint_name_to_q_index_;
  std::unordered_map<std::string, int> joint_name_to_v_index_;
  std::unordered_map<std::string, int> joint_name_to_actuator_index_;
};

}  // namespace manipulation
}  // namespace drake
