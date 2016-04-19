#ifndef DRAKE_SYSTEMS_PLANTS_BOT_VISUALIZER_ROS_H_
#define DRAKE_SYSTEMS_PLANTS_BOT_VISUALIZER_ROS_H_

#include <Eigen/Dense>

#include "ros/ros.h"

#include "drake/systems/System.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/rost_viewer_load_robot.h"
#include "drake/rost_viewer_draw.h"
// #include "drake/util/drakeGeometryUtil.h"

using Drake::NullVector;

namespace drake {
namespace systems {
namespace plants {

/** BotVisualizerROS<RobotStateVector>
 * @brief A system which takes the robot state as input and publishes a ROS draw
 * command to the drake visualizer
 * @concept{system_concept}
 *
 * The resulting system has no internal state; the publish command is executed
 * on every call to the output method.
 *
 * For convenience, the input is passed directly through as an output.
 */

template <template <typename> class RobotStateVector>
class BotVisualizerROS {
 public:
  template <typename ScalarType>
  using StateVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = RobotStateVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = RobotStateVector<ScalarType>;

  BotVisualizerROS(std::shared_ptr<RigidBodyTree> tree) : tree_(tree) {
    init();
  }

  BotVisualizerROS(const std::string &urdf_filename,
                   const DrakeJoint::FloatingBaseType floating_base_type)
      : tree_(new RigidBodyTree(urdf_filename, floating_base_type)) {
    init();
  }

  void init() {
    // Instantiates the ROS topic publishers.
    drake_viewer_load_robot_publisher_ =
      nh_.advertise<drake::rost_viewer_load_robot>("drake_viewer_load_robot",
        1000);

    drake_viewer_draw_publisher_ =
      nh_.advertise<drake::rost_viewer_draw>("drake_viewer_draw",
        1000);

    // Publishes the load robot message.
    publishLoadRobot();

    std::vector<float> position = {0, 0, 0}, quaternion = {0, 0, 0, 1};

    // Defines a 2D array for draw_msg_.position. The array has dimensions
    // [num_links][3].
    draw_msg_.position.layout.dim.push_back(std_msgs::MultiArrayDimension());
    draw_msg_.position.layout.dim.push_back(std_msgs::MultiArrayDimension());
    draw_msg_.position.layout.dim[0].label = "link";
    draw_msg_.position.layout.dim[1].label = "xyz";
    draw_msg_.position.layout.dim[0].size = tree_->bodies.size();
    draw_msg_.position.layout.dim[1].size = 3;
    draw_msg_.position.layout.dim[0].stride = tree_->bodies.size() * 3;
    draw_msg_.position.layout.dim[1].stride = 3;
    draw_msg_.position.layout.data_offset = 0;
    draw_msg_.position.data.resize(draw_msg_.position.layout.dim[0].size *
      draw_msg_.position.layout.dim[1].size);

    // Defines a 2D array for draw_msg_.quaternion. The array has dimensions
    // [num_links][4].
    draw_msg_.quaternion.layout.dim.push_back(std_msgs::MultiArrayDimension());
    draw_msg_.quaternion.layout.dim.push_back(std_msgs::MultiArrayDimension());
    draw_msg_.quaternion.layout.dim[0].label = "link";
    draw_msg_.quaternion.layout.dim[1].label = "wxyz";
    draw_msg_.quaternion.layout.dim[0].size = tree_->bodies.size();
    draw_msg_.quaternion.layout.dim[1].size = 4;
    draw_msg_.quaternion.layout.dim[0].stride = tree_->bodies.size() * 4;
    draw_msg_.quaternion.layout.dim[1].stride = 4;
    draw_msg_.quaternion.layout.data_offset = 0;
    draw_msg_.quaternion.data.resize(draw_msg_.quaternion.layout.dim[0].size *
      draw_msg_.quaternion.layout.dim[1].size);

    // Initializes variable draw_msg_.
    for (const auto &body : tree_->bodies) {
      draw_msg_.link_name.push_back(body->linkname);
      draw_msg_.robot_num.push_back(body->robotnum);
      for (int ii = 0; ii < position.size(); ii++) {
        draw_msg_.position.data.push_back(position[ii]);
      }
      for (int ii = 0; ii < quaternion.size(); ii++) {
        draw_msg_.quaternion.data.push_back(quaternion[ii]);
      }
    }
  }

  void publishLoadRobot() const {
    drake::rost_viewer_load_robot vr;
    // vr.num_links = tree_->bodies.size();
    for (const auto &body : tree_->bodies) {
      drake::rost_viewer_link_data link;
      link.name = body->linkname;
      link.robot_num = body->robotnum;
      // link.num_geom = body->visual_elements.size();
      for (const auto &v : body->visual_elements) {
        drake::rost_viewer_geometry_data gdata;

        const DrakeShapes::Geometry &geometry = v.getGeometry();

        switch (v.getShape()) {  // would prefer to do this through virtual
                                 // methods, but don't want to introduce any LCM
                                 // dependency on the Geometry classes
          case DrakeShapes::BOX: {
            gdata.type = gdata.BOX;
            auto b = dynamic_cast<const DrakeShapes::Box &>(geometry);
            for (int i = 0; i < 3; i++) {
              gdata.float_data.push_back(static_cast<float>(b.size(i)));
            }
            break;
          }
          case DrakeShapes::SPHERE: {
            gdata.type = gdata.SPHERE;
            auto b = dynamic_cast<const DrakeShapes::Sphere &>(geometry);
            gdata.float_data.push_back(static_cast<float>(b.radius));
            break;
          }
          case DrakeShapes::CYLINDER: {
            gdata.type = gdata.CYLINDER;
            auto c = dynamic_cast<const DrakeShapes::Cylinder &>(geometry);
            gdata.float_data.push_back(static_cast<float>(c.radius));
            gdata.float_data.push_back(static_cast<float>(c.length));
            break;
          }
          case DrakeShapes::MESH: {
            gdata.type = gdata.MESH;
            auto m = dynamic_cast<const DrakeShapes::Mesh &>(geometry);
            gdata.float_data.push_back(static_cast<float>(m.scale));
            gdata.string_data = m.resolved_filename;
            break;
          }
          case DrakeShapes::CAPSULE: {
            gdata.type = gdata.CAPSULE;
            auto c = dynamic_cast<const DrakeShapes::Capsule &>(geometry);
            gdata.float_data.push_back(static_cast<float>(c.radius));
            gdata.float_data.push_back(static_cast<float>(c.length));
            break;
          }
          default: {
            // Intentionally do nothing.
            break;
          }
        }

        Eigen::Isometry3d T = v.getLocalTransform();
        auto position = T.translation().cast<float>();
        for (int ii = 0; ii < gdata.position.size(); ii++) {
          gdata.position[ii] = position[ii];
        }

        auto quaternion = rotmat2quat(T.rotation()).cast<float>();
        for (int ii = 0; ii < gdata.position.size(); ii++) {
          gdata.quaternion[ii] = quaternion[ii];
        }

        auto color = v.getMaterial().template cast<float>();
        for (int ii = 0; ii < gdata.color.size(); ii++) {
          gdata.color[ii] = color[ii];
        }

        link.geom.push_back(gdata);
      }
      vr.link.push_back(link);
    }

    // Waits for the Drake Visualizer to subscribe to the ROS topic on which
    // the rost_viewer_load_robot message will be published.
    while (drake_viewer_load_robot_publisher_.getNumSubscribers() == 0) {
      std::cout << "BotVisualizerROS::publishLoadRobot: Waiting for a ROS node "
                   "to subscribe to ROS topic \""
                << drake_viewer_load_robot_publisher_.getTopic() << "\"..."
                << std::endl;
      ros::Duration(0.5).sleep(); // sleep for half a second
    }

    drake_viewer_load_robot_publisher_.publish(vr);
  }

  StateVector<double> dynamics(const double &t, const StateVector<double> &x,
                               const InputVector<double> &u) const {
    return StateVector<double>();
  }

  OutputVector<double> output(const double &t, const StateVector<double> &x,
                              const InputVector<double> &u) {
    draw_msg_.time_stamp = static_cast<int64_t>(t * 1000.0);

    auto uvec = toEigen(u);
    auto q = uvec.head(tree_->num_positions);
    KinematicsCache<double> cache = tree_->doKinematics(q);

    int i, j;
    for (i = 0; i < tree_->bodies.size(); i++) {
      auto transform = tree_->relativeTransform(cache, 0, i);
      auto quat = rotmat2quat(transform.linear());

      auto translation = transform.translation();
      for (j = 0; j < 3; j++) {
        draw_msg_.position.data[i * 3 + j] = static_cast<float>(translation(j));
      }

      for (j = 0; j < 4; j++) {
        draw_msg_.quaternion.data[i * 4 + j] = static_cast<float>(quat(j));
      }
    }

    drake_viewer_draw_publisher_.publish(draw_msg_);

    return u;  // pass the output through
  }

  bool isTimeVarying() const { return true; }
  bool isDirectFeedthrough() const { return true; }

 private:
  /*!
   * The ROS node handle to use when interacting with the ROS network.
   */
  ros::NodeHandle nh_;

  /*!
   * The ROS topic publisher for publishing drake::rost_viewer_load_robot
   * messages.
   */
  ros::Publisher drake_viewer_load_robot_publisher_;

  /*!
   * The ROS topic publisher for publishing drake::rost_viewer_draw messages.
   */
  ros::Publisher drake_viewer_draw_publisher_;

  /*!
   * A pointer to the rigid body tree of the system being simulated.
   */
  std::shared_ptr<RigidBodyTree> tree_;

  /*!
   * The draw message to publish.
   */
  drake::rost_viewer_draw draw_msg_;
};

}  // end namespace plants
}  // end namespace systems
}  // end namespace drake

#endif  // DRAKE_SYSTEMS_PLANTS_BOT_VISUALIZER_ROS_H_
