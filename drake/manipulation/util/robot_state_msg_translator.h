#pragma once

#include <string>
#include <unordered_map>
#include <utility>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/util/drakeGeometryUtil.h"
#include "drake/util/lcmUtil.h"

#include "lcmtypes/bot_core/robot_state_t.hpp"

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
 * For the non-floating joints in bot_core::robot_state_t, the convention is
 * that they (name, position, velocity, effort) are stored in equal length
 * vectors, and the numerical values are identified by the joint name with
 * the same index. This joint name is also used to compare against position
 * names in a RigidBodyTree to establish correspondence between the data
 * layouts in the message and the RigidBodyTree. Note that this class does
 * not require one to one joint matching between the message and the
 * RigidBodyTree. This class only translates information for joints that
 * are shared by the message the RigidBodyTree.
 */
class RobotStateLcmMessageTranslator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RobotStateLcmMessageTranslator)

  /**
   * Constructs a translator using the model given in @p robot. A const
   * reference of @p robot is saved, and thus its life span must be longer
   * that this. The following assumptions must be met by @p robot, otherwise
   * the constructor aborts:
   * <pre>
   *   1. There are at least one non-world rigid body.
   *   2. There can be at most 1 floating base.
   *   3. If there is a floating base, it has to be the first non-world body.
   *   4. The floating joint's position and velocity index has to start from 0.
   *   5. All the other joints has to be 1 degree of freedom or fixed.
   * </pre>
   */
  explicit RobotStateLcmMessageTranslator(const RigidBodyTree<double>& robot);

  /**
   * Initializes @p msg based on the rigid body tree passed to the constructor.
   * Sets all numerical values to zero.
   */
  void InitializeMessage(bot_core::robot_state_t* msg) const;

  /**
   * Decodes only the kinematic values (q, v) from @p msg to @p q and @p v.
   * @p q and @p v are resized to match the rigid body tree's generalized
   * position and velocity size. Note that the values in @p p and @p q that
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
                               VectorX<double>* q, VectorX<double>* v) const;

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
   * @param[in, out] msg Lcm message to be encoded.
   */
  void EncodeMessageKinematics(const VectorX<double>& q,
                               const VectorX<double>& v,
                               bot_core::robot_state_t* msg) const;

  /**
   * Decodes only the non-floating joint torque part from @p msg to @p torque.
   * @p torque is resized to the rigid body tree's actuator size, and it is
   * assumed to be in the same order as the rigid body tree's actuators, which
   * is not necessarily in the same order as its generalized position or
   * velocity. Only the actuated joints in the rigid body tree will be decoded
   * (if they are present in @p msg).
   */
  void DecodeMessageTorque(const bot_core::robot_state_t& msg,
                           VectorX<double>* torque) const;

  /**
   * Encodes only the actuated non-floating joint torque to @p msg.
   * @p torque needs to match the rigid body tree's actuator size, and it is
   * assumed to be in the same order as the rigid body tree's actuators, which
   * is not necessarily in the same order as its generalized position or
   * velocity. Only the actuated joints listed in @p msg will be encoded.
   */
  void EncodeMessageTorque(const VectorX<double>& torque,
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
  /// Check that robot_state_t can unambiguously represent the RigidBodyTree's
  /// state. This method is intended to check preconditions inside a
  /// constructor's
  /// intitializer list.
  /// Aborts when the robot is not compatible with robot_state_t.
  /// @param robot a RigidBodyTree.
  /// @return the same RigidBodyTree that was passed in.
  static const RigidBodyTree<double>& CheckTreeIsRobotStateLcmTypeCompatible(
      const RigidBodyTree<double>& robot);

  const RigidBodyTree<double>& robot_;

  // Pointer to the floating base. nullptr if there is no floating base.
  const RigidBody<double>* const floating_base_;

  // Maps from non-floating joint's position name to various indices.
  std::unordered_map<std::string, int> joint_name_to_q_index_;
  std::unordered_map<std::string, int> joint_name_to_v_index_;
  std::unordered_map<std::string, int> joint_name_to_actuator_index_;
};

}  // namespace manipulation
}  // namespace drake
