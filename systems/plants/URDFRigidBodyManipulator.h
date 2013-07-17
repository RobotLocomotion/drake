
#ifndef URDF_H
#define URDF_H

#include "RigidBodyManipulator.h"
#include "urdf_interface/model.h"
#include <string>

void ROS_ERROR(const char* format, ...);

class URDFRigidBodyManipulator : public RigidBodyManipulator
{
public:
  URDFRigidBodyManipulator(void);
  virtual ~URDFRigidBodyManipulator(void);

  virtual bool addURDF(boost::shared_ptr<urdf::ModelInterface> _urdf_model, std::map<std::string, int> jointname_to_jointnum, std::map<std::string,int> dofname_to_dofnum, const std::string & root_dir = ".");
  bool addURDFfromXML(const std::string &xml_string, const std::string &root_dir = ".");

  virtual void draw(void);

  std::map<std::string, int> robot_map;
  std::vector< std::map<std::string, int> > joint_map, dof_map;
  std::vector<boost::shared_ptr<urdf::ModelInterface> > urdf_model;
};

URDFRigidBodyManipulator* loadURDFfromXML(const std::string &xml_string, const std::string &root_dir = ".");
URDFRigidBodyManipulator* loadURDFfromFile(const std::string &urdf_filename);

std::string rospack(std::string package);

#endif
