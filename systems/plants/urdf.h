
#ifndef URDF_H
#define URDF_H

#define BOT_VIS_SUPPORT  // adds a bunch of dependencies, which are not necessary for all functionality

#include "RigidBodyManipulator.h"
#include "urdf_interface/model.h"
#include <string>

#ifdef BOT_VIS_SUPPORT
#include <bot_vis/bot_vis.h>
#endif

void ROS_ERROR(const char* format, ...);

class URDFRigidBodyManipulator : public RigidBodyManipulator
{
public:
  URDFRigidBodyManipulator(void);
  virtual ~URDFRigidBodyManipulator(void);

  void drawBody(void) {};

  void addURDF(boost::shared_ptr<urdf::ModelInterface> _urdf_model, std::map<std::string, int> jointname_to_jointnum, std::map<std::string,int> dofname_to_dofnum, const std::string & root_dir = ".");
  void addURDFfromXML(const std::string &xml_string, const std::string &root_dir = ".");

#ifdef BOT_VIS_SUPPORT  // adds a bunch of dependencies, which are not necessary for all functionality
  std::map<std::string, BotWavefrontModel*> mesh_map;
#endif
  std::map<std::string, int> joint_map, dof_map;
  std::vector<boost::shared_ptr<urdf::ModelInterface> > urdf_model;
};

URDFRigidBodyManipulator* loadURDFfromXML(const std::string &xml_string, const std::string &root_dir = ".");
URDFRigidBodyManipulator* loadURDFfromFile(const std::string &urdf_filename);

std::string rospack(std::string package);

#endif
