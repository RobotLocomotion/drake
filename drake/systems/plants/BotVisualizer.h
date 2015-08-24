#ifndef DRAKE_BOTVISUALIZER_H
#define DRAKE_BOTVISUALIZER_H

#include <lcm/lcm-cpp.hpp>
#include <Eigen/Dense>
#include "../DrakeSystem.h"
#include "RigidBodyManipulator.h"
#include "lcmtypes/drake/lcmt_viewer_load_robot.hpp"
#include "drakeGeometryUtil.h"

class BotVisualizer : public DrakeSystem {
public:
  BotVisualizer(const std::string& urdf_filename, const std::shared_ptr<lcm::LCM>& _lcm) :
          DrakeSystem("BotVisualizer"),
          manip(urdf_filename),
          lcm(_lcm)
  {
    publishLoadRobot();
  }

  void publishLoadRobot() {
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
        position = T.topRightCorner<1,3>().cast<float>();
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

private:
  RigidBodyManipulator manip;
  std::shared_ptr<lcm::LCM> lcm;
};

#endif //DRAKE_BOTVISUALIZER_H
