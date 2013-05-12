
#ifndef URDF_H
#define URDF_H

#define BOT_VIS_SUPPORT  // adds a bunch of dependencies, which are not necessary for all functionality

#include "RigidBodyManipulator.h"
#include "urdf_interface/model.h"
#include <string>

void ROS_ERROR(const char* format, ...);

class URDFRigidBodyManipulator : public RigidBodyManipulator
{
public:
  URDFRigidBodyManipulator(boost::shared_ptr<urdf::ModelInterface> _urdf_model, std::map<std::string, int> jointname_to_jointnum, const std::string &root_dir = ".", const std::string &floating_base_type = "rpy");
  
  void drawBody(void) {};
  
#ifdef BOT_VIS_SUPPORT  // adds a bunch of dependencies, which are not necessary for all functionality
// todo: add wavefront mesh map
#endif
  std::map<std::string, int> joint_map;
  boost::shared_ptr<urdf::ModelInterface> urdf_model;
};

URDFRigidBodyManipulator* loadURDFfromXML(const std::string &xml_string, const std::string &root_dir = ".");
URDFRigidBodyManipulator* loadURDFfromFile(const std::string &urdf_filename);

#endif
