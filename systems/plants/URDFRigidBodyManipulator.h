
#ifndef URDF_H
#define URDF_H

#include "RigidBodyManipulator.h"
#include <boost/shared_ptr.hpp>
#include <string>
#include <map>

namespace urdf
{
  class ModelInterface;
}

void ROS_ERROR(const char* format, ...);

class URDFRigidBodyManipulator : public RigidBodyManipulator
{
public:
  URDFRigidBodyManipulator(void);
  virtual ~URDFRigidBodyManipulator(void);

  virtual bool addURDF(boost::shared_ptr<urdf::ModelInterface> _urdf_model, std::map<std::string, int> jointname_to_jointnum, std::map<std::string,int> dofname_to_dofnum, const std::string & root_dir = ".");
  bool addURDFfromXML(const std::string &xml_string, const std::string &root_dir = ".");

  std::map<std::string, int> robot_map;
  std::vector< std::map<std::string, int> > joint_map, dof_map;
  std::vector<boost::shared_ptr<urdf::ModelInterface> > urdf_model;
  std::set<std::string> joint_name_set; // Keeps track of all of the joint
                                        // names in the manipulator, so that 
                                        // they can be made unique
};

URDFRigidBodyManipulator* loadURDFfromXML(const std::string &xml_string, const std::string &root_dir = ".");
URDFRigidBodyManipulator* loadURDFfromFile(const std::string &urdf_filename);

std::string rospack(std::string package);

std::map<std::string,int>::const_iterator findWithSuffix(const std::map<std::string,int>& m, const std::string& str);

#endif
