// TODO(liang.fok) Delete this class once all System 1.0-based demos that use
// it are gone. It is being replaced by rigid_body_tree_visualizer_lcm.h.
#pragma once

#include <string>
#include <vector>

#include <lcm/lcm-cpp.hpp>
#include <Eigen/Dense>

#include "drake/math/rotation_matrix.h"
#include "drake/system1/System.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/joints/floating_base_types.h"

#include "drake/lcmt_viewer_load_robot.hpp"
#include "drake/lcmt_viewer_draw.hpp"

namespace drake {

/**
 * A system that takes the robot state as input and publishes an LCM
 * draw command to the Drake Visualizer.
 *
 * @concept{system_concept}
 *
 * The resulting system has no internal state; the publish command is executed
 * on every call to the output method.
 * For convenience, the input is passed directly through as an output.
 *
 */
template <template <typename> class RobotStateVector>
class BotVisualizer {
 public:
  template <typename ScalarType>
  using StateVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = RobotStateVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = RobotStateVector<ScalarType>;

  BotVisualizer(std::shared_ptr<lcm::LCM> lcm,
                std::shared_ptr<RigidBodyTree> tree)
      : tree_(tree), lcm_(lcm) {
    init();
  }

  BotVisualizer(std::shared_ptr<lcm::LCM> lcm,
                const std::string& urdf_filename,
                const drake::systems::plants::joints::FloatingBaseType
                    floating_base_type)
      : tree_(new RigidBodyTree(urdf_filename, floating_base_type)), lcm_(lcm) {
    init();
  }

  void init() {
    publishLoadRobot();

    draw_msg_.num_links = tree_->bodies.size();
    std::vector<float> position = {0, 0, 0}, quaternion = {0, 0, 0, 1};
    for (const auto& body : tree_->bodies) {
      draw_msg_.link_name.push_back(body->get_name());
      draw_msg_.robot_num.push_back(body->get_model_instance_id());
      draw_msg_.position.push_back(position);
      draw_msg_.quaternion.push_back(quaternion);
    }
  }

  void publishLoadRobot() const {
    drake::lcmt_viewer_load_robot vr;
    vr.num_links = tree_->bodies.size();
    for (const auto& body : tree_->bodies) {
      drake::lcmt_viewer_link_data link;
      link.name = body->get_name();
      link.robot_num = body->get_model_instance_id();
      link.num_geom = body->get_visual_elements().size();
      for (const auto& v : body->get_visual_elements()) {
        drake::lcmt_viewer_geometry_data gdata;

        const DrakeShapes::Geometry& geometry = v.getGeometry();

        switch (v.getShape()) {  // would prefer to do this through virtual
                                 // methods, but don't want to introduce any LCM
                                 // dependency on the Geometry classes
          case DrakeShapes::BOX: {
            gdata.type = gdata.BOX;
            gdata.num_float_data = 3;
            auto b = dynamic_cast<const DrakeShapes::Box&>(geometry);
            for (int i = 0; i < 3; i++)
              gdata.float_data.push_back(static_cast<float>(b.size(i)));
            break;
          }
          case DrakeShapes::SPHERE: {
            gdata.type = gdata.SPHERE;
            gdata.num_float_data = 1;
            auto b = dynamic_cast<const DrakeShapes::Sphere&>(geometry);
            gdata.float_data.push_back(static_cast<float>(b.radius));
            break;
          }
          case DrakeShapes::CYLINDER: {
            gdata.type = gdata.CYLINDER;
            gdata.num_float_data = 2;
            auto c = dynamic_cast<const DrakeShapes::Cylinder&>(geometry);
            gdata.float_data.push_back(static_cast<float>(c.radius));
            gdata.float_data.push_back(static_cast<float>(c.length));
            break;
          }
          case DrakeShapes::MESH: {
            gdata.type = gdata.MESH;
            gdata.num_float_data = 3;
            auto m = dynamic_cast<const DrakeShapes::Mesh&>(geometry);
            gdata.float_data.push_back(static_cast<float>(m.scale_[0]));
            gdata.float_data.push_back(static_cast<float>(m.scale_[1]));
            gdata.float_data.push_back(static_cast<float>(m.scale_[2]));

            if (m.uri_.find("package://") == 0) {
              gdata.string_data = m.uri_;
            } else {
              gdata.string_data = m.resolved_filename_;
            }

            break;
          }
          case DrakeShapes::CAPSULE: {
            gdata.type = gdata.CAPSULE;
            gdata.num_float_data = 2;
            auto c = dynamic_cast<const DrakeShapes::Capsule&>(geometry);
            gdata.float_data.push_back(static_cast<float>(c.radius));
            gdata.float_data.push_back(static_cast<float>(c.length));
            break;
          }
          default: {
            // intentionally do nothing
            break;
          }
        }

        Eigen::Isometry3d T = v.getLocalTransform();
        Eigen::Map<Eigen::Vector3f> position(gdata.position);
        position = T.translation().cast<float>();
        Eigen::Map<Eigen::Vector4f> quaternion(gdata.quaternion);
        quaternion = drake::math::rotmat2quat(T.rotation()).cast<float>();

        Eigen::Map<Eigen::Vector4f> color(gdata.color);
        color = v.getMaterial().template cast<float>();

        link.geom.push_back(gdata);
      }
      vr.link.push_back(link);
    }

    lcm_->publish("DRAKE_VIEWER_LOAD_ROBOT", &vr);
  }

  StateVector<double> dynamics(const double& t, const StateVector<double>& x,
                               const InputVector<double>& u) const {
    return StateVector<double>();
  }

  OutputVector<double> output(const double& t, const StateVector<double>& x,
                              const InputVector<double>& u) const {
    draw_msg_.timestamp = static_cast<int64_t>(t * 1000.0);

    const Eigen::VectorXd q = toEigen(u).head(tree_->get_num_positions());
    KinematicsCache<double> cache = tree_->doKinematics(q);

    for (size_t i = 0; i < tree_->bodies.size(); i++) {
      auto transform = tree_->relativeTransform(cache, 0, i);
      auto quat = drake::math::rotmat2quat(transform.linear());
      std::vector<float>& position = draw_msg_.position[i];
      auto translation = transform.translation();
      for (int j = 0; j < 3; j++) {
        position[j] = static_cast<float>(translation(j));
      }
      std::vector<float>& quaternion = draw_msg_.quaternion[i];
      for (int j = 0; j < 4; j++) {
        quaternion[j] = static_cast<float>(quat(j));
      }
    }

    lcm_->publish("DRAKE_VIEWER_DRAW", &draw_msg_);

    return u;  // pass the output through
  }

  bool isTimeVarying() const { return true; }
  bool isDirectFeedthrough() const { return true; }

 private:
  mutable std::shared_ptr<RigidBodyTree>
      tree_;  // todo: remove mutable tag after RBM cleanup
  std::shared_ptr<lcm::LCM> lcm_;
  mutable drake::lcmt_viewer_draw draw_msg_;
};

}  // end namespace drake
