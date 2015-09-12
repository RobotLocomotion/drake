#ifndef DRAKE_BOTVISUALIZER_H
#define DRAKE_BOTVISUALIZER_H

#include <lcm/lcm-cpp.hpp>
#include <Eigen/Dense>
#include "../DrakeSystem.h"
#include "RigidBodyManipulator.h"

// these could all go in the cpp file:
#include "lcmtypes/drake/lcmt_viewer_load_robot.hpp"
#include "lcmtypes/drake/lcmt_viewer_draw.hpp"
#include "drakeGeometryUtil.h"

class BotVisualizer : public DrakeSystem {
public:
  BotVisualizer(const std::shared_ptr<lcm::LCM>& _lcm, const std::string& urdf_filename, const DrakeJoint::FloatingBaseType floating_base_type) :
          DrakeSystem("BotVisualizer"),
          manip(urdf_filename,floating_base_type),
          lcm(_lcm)
  {
    publishLoadRobot();

    draw_msg.num_links = manip.bodies.size();
    std::vector<float> position = {0,0,0}, quaternion = {0,0,0,1};
    for (const auto& body : manip.bodies) {
      draw_msg.link_name.push_back(body->linkname);
      draw_msg.robot_num.push_back(body->robotnum);
      draw_msg.position.push_back(position);
      draw_msg.quaternion.push_back(quaternion);
    }
  }

  void publishLoadRobot() const {
    drake::lcmt_viewer_load_robot vr;
    vr.num_links = manip.bodies.size();
    for (const auto& body : manip.bodies) {
      drake::lcmt_viewer_link_data link;
      link.name = body->linkname;
      link.robot_num = body->robotnum;
      link.num_geom = body->visual_elements.size();
      for (const auto& v : body->visual_elements) {
        drake::lcmt_viewer_geometry_data gdata;

        const DrakeShapes::Geometry& geometry = v.getGeometry();

        switch (v.getShape()) { // would prefer to do this through virtual methods, but don't want to introduce any LCM dependency on the Geometry classes
          case DrakeShapes::BOX: {
            gdata.type = gdata.BOX;
            gdata.num_float_data = 3;
            auto b = dynamic_cast<const DrakeShapes::Box&>(geometry);
            for (int i = 0; i < 3; i++) gdata.float_data.push_back(static_cast<float>(b.size(i)));
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
            auto c =  dynamic_cast<const DrakeShapes::Cylinder&>(geometry);
            gdata.float_data.push_back(static_cast<float>(c.radius));
            gdata.float_data.push_back(static_cast<float>(c.length));
            break;
          }
          case DrakeShapes::MESH: {
            gdata.type = gdata.MESH;
            gdata.num_float_data = 1;
            auto m = dynamic_cast<const DrakeShapes::Mesh&>(geometry);
            gdata.float_data.push_back(static_cast<float>(m.scale));
            gdata.string_data = m.resolved_filename;  // looks like this could be empty, but it is what's used in the get mesh points...
            break;
          }
          case DrakeShapes::CAPSULE: {
            gdata.type = gdata.CAPSULE;
            gdata.num_float_data = 2;
            auto c =  dynamic_cast<const DrakeShapes::Capsule&>(geometry);
            gdata.float_data.push_back(static_cast<float>(c.radius));
            gdata.float_data.push_back(static_cast<float>(c.length));
            break;
          }
          default: {
            // intentionally do nothing
            break;
          }
        }

        Eigen::Matrix4d T = v.getLocalTransform();
        Eigen::Map<Eigen::Vector3f> position(gdata.position);
        position = T.topRightCorner<3,1>().cast<float>();
        Eigen::Map<Eigen::Vector4f> quaternion(gdata.quaternion);
        quaternion = rotmat2quat(T.topLeftCorner<3,3>()).cast<float>();

        Eigen::Map<Eigen::Vector4f> color(gdata.color);
        color = v.getMaterial().cast<float>();

        link.geom.push_back(gdata);
      }
      vr.link.push_back(link);
    }

    lcm->publish("DRAKE_VIEWER_LOAD_ROBOT",&vr);
  }

  virtual Eigen::VectorXd output(double t, const Eigen::VectorXd& unused, const Eigen::VectorXd& u) const override {
    draw_msg.timestamp = static_cast<int64_t>(t*1000.0);

    auto q = u.head(manip.num_positions);
    KinematicsCache<double> cache = manip.doKinematics(q, 0);

    Eigen::Vector3d points = Eigen::Vector3d::Zero();
    int i,j;
    for (i=0; i<manip.bodies.size(); i++) {
      auto pose = manip.forwardKin(cache, points,i,0,2,0).value();
      std::vector<float>& position = draw_msg.position[i];
      for (j=0; j<3; j++) position[j] = static_cast<float>(pose(j));
      std::vector<float>& quaternion = draw_msg.quaternion[i];
      for (j=0; j<4; j++) quaternion[j] = static_cast<float>(pose(j+3));
    }

    lcm->publish("DRAKE_VIEWER_DRAW",&draw_msg);

    return Eigen::VectorXd::Zero(0);
  }


private:
  mutable RigidBodyManipulator manip;  // todo: remove mutable tag after RBM cleanup
  std::shared_ptr<lcm::LCM> lcm;
  mutable drake::lcmt_viewer_draw draw_msg;
};

#endif //DRAKE_BOTVISUALIZER_H
