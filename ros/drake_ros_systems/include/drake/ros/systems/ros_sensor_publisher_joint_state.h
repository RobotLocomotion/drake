#pragma once

#include <stdexcept>

#include <Eigen/Dense>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#include "drake/systems/System.h"
#include "drake/systems/plants/KinematicsCache.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/vector.h"

using drake::NullVector;
using drake::RigidBodySensor;
using drake::RigidBodySystem;
using drake::RigidBodyDepthSensor;

using Eigen::VectorXd;

namespace drake {
namespace ros {
namespace systems {

/**
 * Holds the objects and data used to extract and publish joint state
 * information for a particular model instance.
 */
struct ModelStateStruct {
  std::string model_instance_name;

  // The ROS topic publisher for publishing the model instance's joint state
  ::ros::Publisher publisher;

  // This message holds the model instance's joint sate and is periodically
  // published.
  std::unique_ptr<sensor_msgs::JointState> message;

  // The rigid bodies belonging to this struct's model instance.
  std::vector<const RigidBody*> rigid_body_list;
};

/**
 * @brief A system that takes the system state as the input and publishes
 * joint state information onto ROS topics.
 *
 * It publishes a joint state information for each robot within the rigid body
 * system. The joint state messages for each robot are published onto different
 * ROS topics that are distinguished by robot name.
 *
 * @concept{system_concept}
 *
 * The resulting system has no internal state; the publish command is throttled
 * by kMinTransmitPeriod_.
 *
 * For convenience, the input is passed directly through as an output. This
 * enables other systems to be cascaded after this system.
 */
template <template <typename> class RobotStateVector>
class SensorPublisherJointState {
 private:
  // Specifies that the odometry messages should be transmitted with a minimum
  // period of 0.05 seconds.
  static constexpr double kMinTransmitPeriod_ = 0.05;

 public:
  template <typename ScalarType>
  using StateVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = RobotStateVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = RobotStateVector<ScalarType>;

  /**
   * The constructor.
   *
   * @param[in] rigid_body_system The rigid body system whose output contains
   * the joint state information.
   *
   * @param[in] model_instance_name_table A mapping from model instance IDs to
   * model instance names. The instance names are used to name space the ROS
   * topics on which the `sensor_msgs::JointState` messages are published. This
   * is necessary since multiple model instances may be simultaneously
   * simulated.
   */
  explicit SensorPublisherJointState(
      std::shared_ptr<RigidBodySystem> rigid_body_system,
      const std::map<int, std::string>& model_instance_name_table)
      : rigid_body_system_(rigid_body_system) {
    // Initializes the time stamp of the previous transmission to be zero.
    previous_send_time_.sec = 0;
    previous_send_time_.nsec = 0;

    // Obtains a reference to the world link in the rigid body tree.
    const RigidBody& world = rigid_body_system->getRigidBodyTree()->world();

    // Creates a ROS topic publisher for each robot in the rigid body system.
    // A robot is defined by any link that's connected to the world via a
    // non-fixed joint.
    for (const auto& entry : model_instance_name_table) {
      // Obtains the current model instance's ID, instance name, and list of
      // rigid bodies.
      int model_instance_id = entry.first;
      std::string model_instance_name = entry.second;
      std::vector<const RigidBody*> rigid_body_list =
          rigid_body_system->getRigidBodyTree()->FindModelInstanceBodies(
              model_instance_id);

      if (ShouldPublishState(rigid_body_list)) {
        model_structs_.push_back(
            CreateJointStateStruct(model_instance_name, rigid_body_list));
      }
    }
  }

  /**
   * Creates and returns a `ModelStateStruct` for a model instance named
   * @p model_instance_name. The `ModelStateStruct` contains a ROS topic
   * publisher for publishing the model instance's joint state, a message for
   * holding the jont state, and a list of rigid bodies that belong to the
   * model instance.
   *
   * @param[in] model_instance_name The model instance's name.
   *
   * @param[in] rigid_body_list A list of pointers to `RigidBody` objects that
   * are part of the model instance specified by @p model_instance_id.
   *
   * @returns The newly created `ModelStateStruct` for the model instance
   * specified by @p model_instance_id.
   */
  std::unique_ptr<ModelStateStruct> CreateJointStateStruct(
      const std::string& model_instance_name,
      std::vector<const RigidBody*>& rigid_body_list) {

    // Instantiates a ModelStateStruct for the current model instance.
    std::unique_ptr<ModelStateStruct> model_state_info(new ModelStateStruct());

    model_state_info->model_instance_name = model_instance_name;
    model_state_info->rigid_body_list = rigid_body_list;

    // Instantiates a ROS node handle through which we can interact with ROS.
    // For more information, see:
    // http://wiki.ros.org/roscpp/Overview/NodeHandles
    ::ros::NodeHandle node_handle;

    // Instantiates a ROS topic publisher for publishing the joint state of the
    // model instance.
    model_state_info->publisher =
        node_handle.advertise<sensor_msgs::JointState>(
            model_instance_name + "/joint_state", 1);

    // Instantiates a message for holding the joint state of the model instance.
    model_state_info->message.reset(new sensor_msgs::JointState());

    // The message contains information in joint space. Thus it is unclear
    // what "frame" should be specified here. Perhaps the "frame" should be
    // "joint"? For now, let's just use the name given to the world.
    // TODO(liang.fok) Figure out what's the appropriate frame name to use.
    model_state_info->message->header.frame_id = RigidBodyTree::kWorldName;

    // Iterates through the rigid bodies that belong to the model instance.
    // For each rigid body, allocate space within the message.
    for (auto const& rigid_body : rigid_body_list) {
      const DrakeJoint& joint = rigid_body->getJoint();

      if (joint.getNumPositions() > 0) {
        if (joint.isFloating()) {
          // Floating joints have unequal numbers of position (4) vs.
          // velocity (3) state. Thus, handle them in a special manner.
          model_state_info->message->name.push_back("floating_x");
          model_state_info->message->name.push_back("floating_y");
          model_state_info->message->name.push_back("floating_z");
          model_state_info->message->name.push_back("floating_roll");
          model_state_info->message->name.push_back("floating_pitch");
          model_state_info->message->name.push_back("floating_yaw");
        } else {
          // Verifies that the joint has the same number of position and
          // velocity DOFs. Throws an exception if this is not true.
          if (joint.getNumPositions() != joint.getNumVelocities()) {
            throw std::runtime_error(
                "SensorPublisherJointState: ERROR: Joint \"" + joint.getName() +
                "\" in model instance \"" + model_instance_name +
                "\" has a different number of positions (" +
                std::to_string(joint.getNumPositions()) +
                ") and velocities (" +
                std::to_string(joint.getNumVelocities())
                + ").");
          }

          // Adds the names of the joint's DOFs to the message.
          for (int ii = 0; ii < joint.getNumPositions(); ++ii) {
            model_state_info->message->name.push_back(
                joint.getPositionName(ii));
          }
        }

        // Resizes the position, velocity, and effort vectors in the message.
        // Then initialize them to have zero state.
        int num_states = model_state_info->message->name.size();

        model_state_info->message->position.resize(num_states);
        model_state_info->message->velocity.resize(num_states);
        model_state_info->message->effort.resize(num_states);

        for (int ii = 0; ii < num_states; ++ii) {
          model_state_info->message->position[ii] = 0;
          model_state_info->message->velocity[ii] = 0;
          model_state_info->message->effort[ii] = 0;
        }
      }
    }

    return std::move(model_state_info);
  }

  StateVector<double> dynamics(const double& t, const StateVector<double>& x,
                               const InputVector<double>& u) const {
    return StateVector<double>();
  }

  OutputVector<double> output(const double& t, const StateVector<double>& x,
                              const InputVector<double>& u) {
    // Aborts if insufficient time has passed since the last transmission. This
    // is to avoid flooding the ROS topics.
    ::ros::Time current_time = ::ros::Time::now();
    if ((current_time - previous_send_time_).toSec() < kMinTransmitPeriod_)
      return u;

    previous_send_time_ = current_time;

    const std::shared_ptr<RigidBodyTree>& rigid_body_tree =
        rigid_body_system_->getRigidBodyTree();

    // The input vector `u` contains the entire system's state. The following
    // line of code converts it into an Eigen vector.
    auto system_state_vector = drake::toEigen(u);

    // The following code extracts the position and velocity vectors from
    // system_state_vector and computes the kinematic properties of the system.
    // These kinematic properties are stored in `cache`, which is then used to
    // compute the 6-DoF position and velocity states of floating joints.
    auto position_state_vector =
        system_state_vector.head(rigid_body_tree->number_of_positions());

    auto velocity_state_vector =
        system_state_vector.segment(rigid_body_tree->number_of_positions(),
                                    rigid_body_tree->number_of_velocities());

    KinematicsCache<double> kinematics_cache = rigid_body_tree->doKinematics(
        position_state_vector, velocity_state_vector);

    for (auto& current_model_info : model_structs_) {
      UpdateAndPublishJointStateMessage(system_state_vector, kinematics_cache,
          current_model_info.get());
    }

    // int q_index = 0;
    // int v_index = 0;

    // // Resets the message_index variable in each of the ModelStateStruct
    // // objects in the model_structs_ map. This is so we can keep track of where
    // // in the joint state message we are saving.
    // for (auto const& map_entry : model_structs_) {
    //   ModelStateStruct* robot_struct = map_entry.second.get();
    //   robot_struct->message_index = 0;
    // }

    // Obtains a reference to the world link in the rigid body tree.
    // const RigidBody& world = rigid_body_tree->world();

    // // Saves the joint state information
    // for (auto const& rigid_body : rigid_body_tree->bodies) {
    //   // Skips the current rigid body if it does not have a parent. Note that
    //   // this includes the world.
    //   if (!rigid_body->hasParent()) continue;

    //   const DrakeJoint& joint = rigid_body->getJoint();

    //   // Skips the current rigid body if is connected to another rigid body
    //   // via a fixed joint.
    //   if (joint.getNumPositions() == 0) continue;

    //   // Defines the key that can be used to obtain the ModelStateStruct
    //   // object for the current robot. The key is simply the model name since
    //   // there should only be one ModelStateStruct per robot.
    //   const std::string& key = rigid_body->get_model_name();

    //   // Verifies that a ModelStateStruct for the current robot
    //   // exists in the model_structs_ map.
    //   auto robot_struct_in_map = model_structs_.find(key);
    //   if (robot_struct_in_map == model_structs_.end()) {
    //     throw std::runtime_error(
    //         "ERROR: SensorPublisherJointState: Unable to find"
    //         "robot struct using key " +
    //         key);
    //   }

    //   ModelStateStruct* robot_struct = robot_struct_in_map->second.get();
    // }


    return u;  // Passes the output through to the next system in the cascade.
  }

  bool isTimeVarying() const { return true; }
  bool isDirectFeedthrough() const { return true; }

 private:
  /* Determines whether joint state messages should be published for the
   * model instance that owns the rigid bodies in @p rigid_body_list. It should
   * if the model instance contains one or more joints that isn't fixed.
   */
  bool ShouldPublishState(
      const std::vector<const RigidBody*>& rigid_body_list) {
    bool result = false;
    for (const auto& rigid_body : rigid_body_list) {
      if (!rigid_body->getJoint().isFixed()) {
        result = true;
      }
    }
    return result;
  }

  /*
   * Updates the joint state message of a particular model instance, then
   * publishes it.
   *
   * @param[in] system_state_vector The current generalized state of the system.
   *
   * @param[in] kinematics_cache The cache of the system's kinematic properties.
   *
   * @param[out] current_model_info The structure that contains the publisher
   * and message for the current model instance.
   */
  template <typename ScalarType, int Rows>
  void UpdateAndPublishJointStateMessage(
      const Eigen::Matrix<ScalarType, Rows, 1>& system_state_vector,
      const KinematicsCache<double>& kinematics_cache,
      ModelStateStruct* current_model_info) {
    // Defines an index into the joint state message.
    int joint_state_index = 0;

    // Obtains a pointer to the rigid body tree.
    const std::shared_ptr<RigidBodyTree>& rigid_body_tree =
        rigid_body_system_->getRigidBodyTree();

    // Iterates through each of the current model instance's rigid bodies.
    for (auto& rigid_body : current_model_info->rigid_body_list) {
      const DrakeJoint& joint = rigid_body->getJoint();
      if (joint.getNumPositions() > 0) {
        if (joint.isFloating()) {

          // The generalized position state of floating joints need to be
          // converted from quaternion values into roll/pitch/yaw values. The
          // following code does this.
          auto transform = rigid_body_tree->relativeTransform(
              kinematics_cache,
              rigid_body_tree->FindBodyIndex(
                  rigid_body->get_parent()->get_name(),
                  rigid_body->get_parent()->get_model_instance_id()),
              rigid_body_tree->FindBodyIndex(
                  rigid_body->get_name(),
                  rigid_body->get_model_instance_id()));
          auto translation = transform.translation();
          auto rpy = drake::math::rotmat2rpy(transform.linear());

          // Saves the floating joint's position state.
          {
            int index = joint_state_index;

            current_model_info->message->position[index++] =
                translation(0);
            current_model_info->message->position[index++] =
                translation(1);
            current_model_info->message->position[index++] =
                translation(2);

            current_model_info->message->position[index++] = rpy(0);
            current_model_info->message->position[index++] = rpy(1);
            current_model_info->message->position[index++] = rpy(2);
          }

          // Saves the floating joint's velocity state. The velocity state can
          // be determined directly from the system state vector.
          {
            int index = joint_state_index;

            int system_state_velocity_index =
                rigid_body->get_velocity_start_index();

            for (size_t ii = 0; ii < joint.getNumVelocities(); ++ii) {
              current_model_info->message->velocity[index++] =
                  system_state_vector[system_state_velocity_index++];
            }
          }

          // Updates joint_state_index to point to the position in the joint
          // state message after the portion that stores this floating joint's
          // state.
          joint_state_index += joint.getNumVelocities();
        } else {
          // Verifies that the joint has the same number of position versus
          // velocity DOFs. Throws an exception if this is not true.
          if (joint.getNumPositions() != joint.getNumVelocities()) {
            throw std::runtime_error(
                "ERROR: Joint \"" + joint.getName() +
                "\" in model instance \"" +
                current_model_info->model_instance_name +
                "\" has a different number of positions and velocities.");
          }

          int system_state_position_index =
            rigid_body->get_position_start_index();
          int system_state_velocity_index =
            rigid_body->get_velocity_start_index();

          // Adds the names of the DOFs that belong to the joint to the
          // message.
          for (int ii = 0; ii < joint.getNumPositions(); ++ii) {
            current_model_info->message->position[joint_state_index] =
                system_state_vector[system_state_position_index++];
            current_model_info->message->velocity[joint_state_index] =
                system_state_vector[system_state_velocity_index++];
            ++joint_state_index;
          }
        }
      }
    }

    // Publishes the joint state message for the current model instance.
    current_model_info->message->header.stamp = ::ros::Time::now();
    current_model_info->publisher.publish(*(current_model_info->message.get()));
  }

  // A local reference to the rigid body system.
  std::shared_ptr<RigidBodySystem> rigid_body_system_;

  // Maintains a set of ModelStateStruct structs, one for each model instance.
  std::vector<std::unique_ptr<ModelStateStruct>> model_structs_;

  // The previous time the LIDAR messages were sent.
  ::ros::Time previous_send_time_;
};

}  // namespace systems
}  // namespace ros
}  // namespace drake
