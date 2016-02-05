#include <string>
#include <fstream>
#include <sstream>

#include "spruce.hh"

#include "drake/thirdParty/tinyxml2/tinyxml2.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "joints/DrakeJoints.h"

#include "drake/Path.h"
#include "xmlUtil.h"

// from http://stackoverflow.com/questions/478898/how-to-execute-a-command-and-get-output-of-command-within-c
#if defined(WIN32) || defined(WIN64)
  #define POPEN _popen
  #define PCLOSE _pclose
#else
  #define POPEN popen
  #define PCLOSE pclose
#endif

using namespace std;
using namespace Eigen;
using namespace tinyxml2;



void parseSDF(RigidBodyTree * model, XMLDocument * xml_doc, map<string,string>& package_map, const string &root_dir, const DrakeJoint::FloatingBaseType floating_base_type)
{
  populatePackageMap(package_map);

//  parseRobot(model, node, package_map, root_dir, floating_base_type);

  model->compile();
}


void RigidBodyTree::addRobotFromSDF(const string &urdf_filename, const DrakeJoint::FloatingBaseType floating_base_type)
{
  map<string,string> package_map;

  XMLDocument xml_doc;
  xml_doc.LoadFile(urdf_filename.data());
  if (xml_doc.ErrorID()) {
    throw std::runtime_error("failed to parse xml in file " + urdf_filename + "\n" + xml_doc.ErrorName());
  }

  string root_dir=".";
  size_t found = urdf_filename.find_last_of("/\\");
  if (found != string::npos) {
    root_dir = urdf_filename.substr(0, found);
  }

  parseSDF(this,&xml_doc,package_map,root_dir,floating_base_type);
}

