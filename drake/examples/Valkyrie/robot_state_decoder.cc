#include "drake/examples/Valkyrie/robot_state_decoder.h"

#include <utility>

#include "lcmtypes/bot_core/robot_state_t.hpp"

#include "drake/examples/Valkyrie/robot_state_lcmtype_util.h"
#include "drake/util/drakeGeometryUtil.h"
#include "drake/util/lcmUtil.h"

namespace drake {
namespace systems {

using std::make_unique;
using std::move;
using std::unique_ptr;
using std::map;
using std::string;

using Eigen::VectorXd;
using Eigen::Matrix;
using Eigen::Dynamic;
using Eigen::Isometry3d;

using bot_core::robot_state_t;

using drake::math::rotmat2rpy;
using drake::math::rotmat2quat;
using drake::math::Gradient;

RobotStateDecoder::RobotStateDecoder(const RigidBodyTree<double>& tree)
    : tree_(CheckTreeIsRobotStateLcmTypeCompatible(tree)),
      floating_body_(tree.bodies[1]->getJoint().is_floating()
                         ? tree.bodies[1].get()
                         : nullptr),
      robot_state_message_port_index_(DeclareAbstractInputPort().get_index()),
      kinematics_cache_port_index_(DeclareAbstractOutputPort().get_index()),
      joint_name_to_body_(CreateJointNameToBodyMap(tree)) {
  set_name("RobotStateDecoder");
}

void RobotStateDecoder::DoCalcOutput(const Context<double>& context,
                                     SystemOutput<double>* output) const {
  // Input: robot_state_t message.
  const auto& message =
      EvalAbstractInput(context, robot_state_message_port_index_)
          ->GetValue<robot_state_t>();

  // Output: KinematicsCache.
  auto& kinematics_cache = output->GetMutableData(kinematics_cache_port_index_)
                               ->GetMutableValue<KinematicsCache<double>>();

  // TODO(tkoolen): don't allocate anew.
  VectorXd q(tree_.get_num_positions());
  VectorXd v(tree_.get_num_velocities());

  // Uninitialized, have not got a valid lcm message yet.
  // TODO(siyuan.feng): Need better way to check for this.
  bool valid_lcm_msg = message.joint_name.size() != 0;

  // Floating joint.
  if (floating_body_) {
    auto floating_body_to_world = DecodePose(message.pose);
    auto floating_body_twist_in_in_world_aligned_body =
        DecodeTwist(message.twist);
    const DrakeJoint& floating_joint = floating_body_->getJoint();
    int position_start = floating_body_->get_position_start_index();
    int velocity_start = floating_body_->get_velocity_start_index();

    // TODO(tkoolen): kind of a crude check:
    if (floating_joint.get_num_positions() == kRpySize + kSpaceDimension) {
      // RPY-parameterized floating joint.
      if (!valid_lcm_msg) {
        q.setZero();
        v.setZero();
      } else {
        // Translation.
        q.segment<kSpaceDimension>(position_start) =
          floating_body_to_world.translation();

        // Orientation.
        auto rpy = rotmat2rpy(floating_body_to_world.linear());
        q.segment<kRpySize>(position_start + kSpaceDimension) = rpy;

        // Translational velocity.
        auto translationdot =
          floating_body_twist_in_in_world_aligned_body.tail<kSpaceDimension>();
        v.segment<kSpaceDimension>(velocity_start) = translationdot;

        // Rotational velocity.
        Matrix<double, kRpySize, kSpaceDimension> phi;
        typename Gradient<decltype(phi), Dynamic>::type* dphi = nullptr;
        typename Gradient<decltype(phi), Dynamic, 2>::type* ddphi = nullptr;
        angularvel2rpydotMatrix(rpy, phi, dphi, ddphi);
        auto angular_velocity_world =
          floating_body_twist_in_in_world_aligned_body.head<kSpaceDimension>();
        auto rpydot = (phi * angular_velocity_world).eval();
        v.segment<kRpySize>(velocity_start + kSpaceDimension) = rpydot;
      }
    } else if (floating_joint.get_num_positions() ==
               kQuaternionSize + kSpaceDimension) {
      // Quaternion-parameterized floating joint.
      if (!valid_lcm_msg) {
        q.setZero();
        v.setZero();
        // Set quaternion
        q.segment<kQuaternionSize>(position_start + kSpaceDimension) =
            rotmat2quat(Matrix3<double>::Identity());
      } else {
        // Translation.
        q.segment<kSpaceDimension>(position_start) =
          floating_body_to_world.translation();

        // Orientation.
        auto quat = rotmat2quat(floating_body_to_world.linear());
        q.segment<kQuaternionSize>(position_start + kSpaceDimension) = quat;

        // Twist.
        // Transform twist from world-aligned body frame to body frame.
        Isometry3d world_aligned_body_to_body;
        world_aligned_body_to_body.linear() =
          floating_body_to_world.linear().transpose();
        world_aligned_body_to_body.translation().setZero();
        world_aligned_body_to_body.makeAffine();
        TwistVector<double> floating_body_twist_in_body =
          transformSpatialMotion(world_aligned_body_to_body,
              floating_body_twist_in_in_world_aligned_body);
        v.segment<kTwistSize>(velocity_start) = floating_body_twist_in_body;
      }
    } else {
      DRAKE_ABORT();
    }
  }

  // Non-floating joints.
  for (size_t i = 0; i < message.joint_name.size(); i++) {
    const auto& name = message.joint_name[i];
    const auto& it = joint_name_to_body_.find(name);
    if (it != joint_name_to_body_.end()) {
      const RigidBody<double>& body = *(it->second);
      double position = message.joint_position[i];
      double velocity = message.joint_velocity[i];
      q[body.get_position_start_index()] = position;
      v[body.get_velocity_start_index()] = velocity;
    }
  }

  if (!q.allFinite()) {
    for (int i = 0; i < tree_.get_num_positions(); i++) {
      std::cout << tree_.get_position_name(i) << ": " << q[i] << std::endl;
    }
    throw std::runtime_error("invalid q");
  }
  if (!v.allFinite()) {
    for (int i = 0; i < tree_.get_num_velocities(); i++) {
      std::cout << tree_.get_velocity_name(i) << ": " << v[i] << std::endl;
    }
    throw std::runtime_error("invalid v");
  }

  kinematics_cache.initialize(q, v);
  tree_.doKinematics(kinematics_cache, true);
}

std::unique_ptr<AbstractValue> RobotStateDecoder::AllocateOutputAbstract(
    const OutputPortDescriptor<double>& output) const {
  return std::make_unique<Value<KinematicsCache<double>>>(
      tree_.CreateKinematicsCache());
}

std::map<std::string, const RigidBody<double>*>
RobotStateDecoder::CreateJointNameToBodyMap(const RigidBodyTree<double>& tree) {
  map<string, const RigidBody<double>*> ret;
  for (const auto& body : tree.bodies) {
    if (body->has_parent_body()) {
      const auto& joint = body->getJoint();
      if (!joint.is_fixed() && !joint.is_floating()) {
        // To match usage of robot_state_t throughout OpenHumanoids code, use
        // position coordinate name as joint name.
        int position_index = body->get_position_start_index();
        ret[tree_.get_position_name(position_index)] = body.get();
      }
    }
  }
  return ret;
}

}  // namespace systems
}  // namespace drake
