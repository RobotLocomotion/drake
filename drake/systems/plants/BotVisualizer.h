#ifndef DRAKE_BOTVISUALIZER_H
#define DRAKE_BOTVISUALIZER_H

#include <lcm/lcm-cpp.hpp>
#include <Eigen/Dense>
#include "drake/systems/System.h"
#include "drake/systems/plants/RigidBodyTree.h"

// these could all go in the cpp file:
#include "lcmtypes/drake/lcmt_viewer_load_robot.hpp"
#include "lcmtypes/drake/lcmt_viewer_draw.hpp"
#include "drake/util/drakeGeometryUtil.h"

namespace Drake {

/** BotVisualizer<RobotStateVector>
 * @brief A system which takes the robot state as input and publishes an lcm
 *draw command to the drake visualizer
 * @concept{system_concept}
 *
 * The resulting system has no internal state; the publish command is executed
 *on every call to the output method.
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

  BotVisualizer(std::shared_ptr<lcm::LCM> _lcm,
                std::shared_ptr<RigidBodyTree> tree)
      : tree(tree), lcm(_lcm) {
    init();
  }

  BotVisualizer(std::shared_ptr<lcm::LCM> _lcm,
                const std::string &urdf_filename,
                const DrakeJoint::FloatingBaseType floating_base_type)
      : tree(new RigidBodyTree(urdf_filename, floating_base_type)), lcm(_lcm) {
    init();
  }

  void init() {
    publishLoadRobot();

    draw_msg.num_links = tree->bodies.size();
    std::vector<float> position = {0, 0, 0}, quaternion = {0, 0, 0, 1};
    for (const auto &body : tree->bodies) {
      draw_msg.link_name.push_back(body->linkname);
      draw_msg.robot_num.push_back(body->robotnum);
      draw_msg.position.push_back(position);
      draw_msg.quaternion.push_back(quaternion);
    }
  }

  void publishLoadRobot() const {
    drake::lcmt_viewer_load_robot vr;
    vr.num_links = tree->bodies.size();
    for (const auto &body : tree->bodies) {
      drake::lcmt_viewer_link_data link;
      link.name = body->linkname;
      link.robot_num = body->robotnum;
      link.num_geom = body->visual_elements.size();
      for (const auto &v : body->visual_elements) {
        drake::lcmt_viewer_geometry_data gdata;

        const DrakeShapes::Geometry &geometry = v.getGeometry();

        switch (v.getShape()) {  // would prefer to do this through virtual
                                 // methods, but don't want to introduce any LCM
                                 // dependency on the Geometry classes
          case DrakeShapes::BOX: {
            gdata.type = gdata.BOX;
            gdata.num_float_data = 3;
            auto b = dynamic_cast<const DrakeShapes::Box &>(geometry);
            for (int i = 0; i < 3; i++)
              gdata.float_data.push_back(static_cast<float>(b.size(i)));
            break;
          }
          case DrakeShapes::SPHERE: {
            gdata.type = gdata.SPHERE;
            gdata.num_float_data = 1;
            auto b = dynamic_cast<const DrakeShapes::Sphere &>(geometry);
            gdata.float_data.push_back(static_cast<float>(b.radius));
            break;
          }
          case DrakeShapes::CYLINDER: {
            gdata.type = gdata.CYLINDER;
            gdata.num_float_data = 2;
            auto c = dynamic_cast<const DrakeShapes::Cylinder &>(geometry);
            gdata.float_data.push_back(static_cast<float>(c.radius));
            gdata.float_data.push_back(static_cast<float>(c.length));
            break;
          }
          case DrakeShapes::MESH: {
            gdata.type = gdata.MESH;
            gdata.num_float_data = 1;
            auto m = dynamic_cast<const DrakeShapes::Mesh &>(geometry);
            gdata.float_data.push_back(static_cast<float>(m.scale));
            gdata.string_data = m.resolved_filename;  // looks like this could
                                                      // be empty, but it is
                                                      // what's used in the get
                                                      // mesh points...
            break;
          }
          case DrakeShapes::CAPSULE: {
            gdata.type = gdata.CAPSULE;
            gdata.num_float_data = 2;
            auto c = dynamic_cast<const DrakeShapes::Capsule &>(geometry);
            gdata.float_data.push_back(static_cast<float>(c.radius));
            gdata.float_data.push_back(static_cast<float>(c.length));
            break;
          }
          case DrakeShapes::HEIGHT_MAP_TERRAIN: {
            gdata.type = gdata.MESH;
            auto terrain = dynamic_cast<const DrakeShapes::HeightMapTerrain &>(geometry);
            gdata.num_float_data = 1;
            gdata.float_data.push_back(static_cast<float>(1.0)); //scale
            gdata.string_data = terrain.fname;
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
        quaternion = rotmat2quat(T.rotation()).cast<float>();

        Eigen::Map<Eigen::Vector4f> color(gdata.color);
        color = v.getMaterial().template cast<float>();

        link.geom.push_back(gdata);
      }
      vr.link.push_back(link);
    }

    lcm->publish("DRAKE_VIEWER_LOAD_ROBOT", &vr);
  }

  StateVector<double> dynamics(const double &t, const StateVector<double> &x,
                               const InputVector<double> &u) const {
    return StateVector<double>();
  };

  OutputVector<double> output(const double &t, const StateVector<double> &x,
                              const InputVector<double> &u) const {
    draw_msg.timestamp = static_cast<int64_t>(t * 1000.0);

    auto uvec = toEigen(u);
    auto q = uvec.head(tree->num_positions);
    KinematicsCache<double> cache = tree->doKinematics(q);

    int i, j;
    for (i = 0; i < tree->bodies.size(); i++) {
      auto transform = tree->relativeTransform(cache, 0, i);
      auto quat = rotmat2quat(transform.linear());
      std::vector<float> &position = draw_msg.position[i];
      auto translation = transform.translation();
      for (j = 0; j < 3; j++) position[j] = static_cast<float>(translation(j));
      std::vector<float> &quaternion = draw_msg.quaternion[i];
      for (j = 0; j < 4; j++) quaternion[j] = static_cast<float>(quat(j));
    }

    lcm->publish("DRAKE_VIEWER_DRAW", &draw_msg);

    return u;  // pass the output through
  }

  bool isTimeVarying() const { return true; }
  bool isDirectFeedthrough() const { return true; }

 private:
  mutable std::shared_ptr<RigidBodyTree>
      tree;  // todo: remove mutable tag after RBM cleanup
  std::shared_ptr<lcm::LCM> lcm;
  mutable drake::lcmt_viewer_draw draw_msg;
};

}  // end namespace Drake

#endif  // DRAKE_BOTVISUALIZER_H
