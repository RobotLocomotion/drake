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

    // Instantiates a ROS node handle through which we can interact with ROS.
    // For more information, see:
    // http://wiki.ros.org/roscpp/Overview/NodeHandles
    ::ros::NodeHandle node_handle;

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
        // Instantiates a ModelStateStruct for the current model instance.
        std::unique_ptr<ModelStateStruct> model_struct(
            new ModelStateStruct());

        if (model_structs_.find(model_instance_name) != model_structs_.end()) {
          throw std::runtime_error("SensorPublisherJointState: ERROR: "
              "Duplicate model instance named \"" + model_instance_name +
              " detected.");
        }

        model_struct->rigid_body_list = rigid_body_list;

        const std::string topic_name = model_instance_name + "/joint_state";
        model_struct->publisher =
            node_handle.advertise<sensor_msgs::JointState>(topic_name, 1);

        model_struct->message.reset(new sensor_msgs::JointState());

        // TODO(liang.fok) Verify this is correct.
        model_struct->message->header.frame_id = RigidBodyTree::kWorldName;

        InitJointStateStruct(robot_name, rigid_body_system->getRigidBodyTree(),
                             model_struct.get());

        model_structs_[model_instance_name] = std::move(model_struct);
      }

    }
  }

  // /**
  //  * Initializes a ModelStateStruct for a particular robot.
  //  *
  //  * @param[in] robot_name The name of the robot.
  //  * @param[in] tree The rigid body tree containing the information needed to
  //  * initialize the joint state message.
  //  * @param[out] robot_struct The struct to initialize.
  //  */
  // void InitJointStateStruct(const std::string& robot_name,
  //                           const std::shared_ptr<RigidBodyTree>& tree,
  //                           ModelStateStruct* robot_struct) {
  //   if (robot_struct == nullptr) {
  //     throw std::runtime_error(
  //         "ERROR: InitJointStateMessage: robot_struct "
  //         "parameter is nullptr!");
  //   }

  //   // robot_struct->num_positions_ = 0;
  //   // robot_struct->num_velocities_ = 0;

  //   // Iterates through the rigid bodies in the rigid body tree searching for
  //   // those that belong to the specified robot. Updates the
  //   // ModelStateStruct using the robot's rigid bodies.
  //   for (auto const& rigid_body : tree->bodies) {
  //     if (rigid_body->get_model_name() == robot_name) {
  //       const DrakeJoint& joint = rigid_body->getJoint();

  //       if (joint.getNumPositions() > 0) {
  //         // robot_struct->num_positions_ += joint.getNumPositions();
  //         // robot_struct->num_velocities_ += joint.getNumVelocities();

  //         if (joint.isFloating()) {
  //           robot_struct->message->name.push_back("floating_x");
  //           robot_struct->message->name.push_back("floating_y");
  //           robot_struct->message->name.push_back("floating_z");
  //           robot_struct->message->name.push_back("floating_roll");
  //           robot_struct->message->name.push_back("floating_pitch");
  //           robot_struct->message->name.push_back("floating_yaw");
  //         } else {
  //           // Verifies that the joint has the same number of position versus
  //           // velocity DOFs. Throws an exception if this is not true.
  //           if (joint.getNumPositions() != joint.getNumVelocities()) {
  //             throw std::runtime_error(
  //                 "ERROR: Joint \"" + joint.getName() + "\" in robot \"" +
  //                 robot_name +
  //                 "\" has a different number of positions and velocities.");
  //           }

  //           // Adds the names of the DOFs that belong to the joint to the
  //           // message.
  //           for (int ii = 0; ii < joint.getNumPositions(); ii++) {
  //             robot_struct->message->name.push_back(joint.getPositionName(ii));
  //           }
  //         }
  //       }

  //       // Resizes the vectors in the message and initialize them to have zero
  //       // state.
  //       int num_states = robot_struct->message->name.size();

  //       robot_struct->message->position.resize(num_states);
  //       robot_struct->message->velocity.resize(num_states);
  //       robot_struct->message->effort.resize(num_states);

  //       for (int ii = 0; ii < num_states; ii++) {
  //         robot_struct->message->position[ii] = 0;
  //         robot_struct->message->velocity[ii] = 0;
  //         robot_struct->message->effort[ii] = 0;
  //       }
  //     }
  //   }
  // }

  StateVector<double> dynamics(const double& t, const StateVector<double>& x,
                               const InputVector<double>& u) const {
    return StateVector<double>();
  }

  OutputVector<double> output(const double& t, const StateVector<double>& x,
                              const InputVector<double>& u) {
    // // Aborts if insufficient time has passed since the last transmission. This
    // // is to avoid flooding the ROS topics.
    // ::ros::Time current_time = ::ros::Time::now();
    // if ((current_time - previous_send_time_).toSec() < kMinTransmitPeriod_)
    //   return u;

    // previous_send_time_ = current_time;

    // const std::shared_ptr<RigidBodyTree>& rigid_body_tree =
    //     rigid_body_system_->getRigidBodyTree();

    // // The input vector u contains the entire system's state. The following
    // // The following code extracts the position and velocity values from it
    // // and computes the kinematic properties of the system.
    // auto uvec = drake::toEigen(u);
    // auto q = uvec.head(rigid_body_tree->number_of_positions());    // position
    // auto v = uvec.segment(rigid_body_tree->number_of_positions(),  // velocity
    //                       rigid_body_tree->number_of_velocities());
    // KinematicsCache<double> cache = rigid_body_tree->doKinematics(q, v);

    // int q_index = 0;
    // int v_index = 0;

    // // Resets the message_index variable in each of the ModelStateStruct
    // // objects in the model_structs_ map. This is so we can keep track of where
    // // in the joint state message we are saving.
    // for (auto const& map_entry : model_structs_) {
    //   ModelStateStruct* robot_struct = map_entry.second.get();
    //   robot_struct->message_index = 0;
    // }

    // // Obtains a reference to the world link in the rigid body tree.
    // // const RigidBody& world = rigid_body_tree->world();

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

    //   if (joint.getNumPositions() > 0) {
    //     if (joint.isFloating()) {
    //       auto transform = rigid_body_tree->relativeTransform(
    //           cache,
    //           rigid_body_tree->FindBodyIndex(
    //               rigid_body->get_parent()->get_name()),
    //           rigid_body_tree->FindBodyIndex(rigid_body->get_name()));
    //       auto translation = transform.translation();
    //       auto rpy = drake::math::rotmat2rpy(transform.linear());

    //       size_t index = robot_struct->message_index;

    //       robot_struct->message->position[index++] = translation(0);
    //       robot_struct->message->position[index++] = translation(1);
    //       robot_struct->message->position[index++] = translation(2);

    //       robot_struct->message->position[index++] = rpy(0);
    //       robot_struct->message->position[index++] = rpy(1);
    //       robot_struct->message->position[index++] = rpy(2);

    //       q_index += joint.getNumPositions();
    //       index = robot_struct->message_index;

    //       for (size_t ii = 0; ii < joint.getNumVelocities(); ii++) {
    //         robot_struct->message->velocity[index++] = v[v_index++];
    //       }

    //       robot_struct->message_index = index;
    //     } else {
    //       // Verifies that the joint has the same number of position versus
    //       // velocity DOFs. Throws an exception if this is not true.
    //       if (joint.getNumPositions() != joint.getNumVelocities()) {
    //         throw std::runtime_error(
    //             "ERROR: Joint \"" + joint.getName() + "\" in robot \"" +
    //             robot_struct->robot_name +
    //             "\" has a different number of positions and velocities.");
    //       }

    //       // Adds the names of the DOFs that belong to the joint to the
    //       // message.
    //       for (int ii = 0; ii < joint.getNumPositions(); ii++) {
    //         size_t index = robot_struct->message_index++;
    //         robot_struct->message->position[index] = q[q_index++];
    //         robot_struct->message->velocity[index] = v[v_index++];
    //       }
    //     }
    //   }
    // }

    // // Publishes the joint state messages.
    // for (auto const& map_entry : model_structs_) {
    //   ModelStateStruct* robot_struct = map_entry.second.get();
    //   robot_struct->message->header.stamp = current_time;
    //   robot_struct->publisher.publish(*(robot_struct->message.get()));
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
  // A local reference to the rigid body system.
  std::shared_ptr<RigidBodySystem> rigid_body_system_;

  // Maintains a set of ModelStateStruct structs, one for each robot.
  // The key is the robot's name.
  std::map<std::string, std::unique_ptr<ModelStateStruct>> model_structs_;

  // The previous time the LIDAR messages were sent.
  ::ros::Time previous_send_time_;
};

}  // namespace systems
}  // namespace ros
}  // namespace drake
