
#ifndef URDF_H
#define URDF_H

#include "RigidBodyManipulator.h"
#include "urdf_interface/model.h"

void ROS_ERROR(const char* format, ...);

class URDFRigidBodyManipulator : public RigidBodyManipulator
{
public:
  URDFRigidBodyManipulator(boost::shared_ptr<urdf::ModelInterface> _urdf_model, std::map<std::string, int> jointname_to_jointnum, const std::string &root_dir = ".");
  
  void drawLink(void) {};
  void drawBody(void) {};
  
  std::map<std::string, int> joint_map;
  boost::shared_ptr<urdf::ModelInterface> urdf_model;
};

URDFRigidBodyManipulator* loadURDFfromXML(const std::string &xml_string, const std::string &root_dir = ".");
URDFRigidBodyManipulator* loadURDFfromFile(const std::string &urdf_filename);

#endif
