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

void parseSDFInertial(shared_ptr<RigidBody> body, XMLElement* node, RigidBodyTree * model, Isometry3d& T_element_to_link)
{
  Isometry3d T = Isometry3d::Identity();
  XMLElement* pose = node->FirstChildElement("pose");
  if (pose) originAttributesToTransform(pose, T);
  T = T_element_to_link*T;

  XMLElement* mass = node->FirstChildElement("mass");
  if (mass) parseScalarAttribute(mass, "value", body->mass);

  body->com << T(0, 3), T(1, 3), T(2, 3);

  Matrix<double, TWIST_SIZE, TWIST_SIZE> I = Matrix<double, TWIST_SIZE, TWIST_SIZE>::Zero();
  I.block(3, 3, 3, 3) << body->mass * Matrix3d::Identity();

  XMLElement* inertia = node->FirstChildElement("inertia");
  if (inertia) {
    parseScalarValue(inertia, "ixx", I(0, 0));
    parseScalarValue(inertia, "ixy", I(0, 1));
    I(1, 0) = I(0, 1);
    parseScalarValue(inertia, "ixz", I(0, 2));
    I(2, 0) = I(0, 2);
    parseScalarValue(inertia, "iyy", I(1, 1));
    parseScalarValue(inertia, "iyz", I(1, 2));
    I(2, 1) = I(1, 2);
    parseScalarValue(inertia, "izz", I(2, 2));
  }

  auto bodyI = transformSpatialInertia(T, static_cast<Gradient<Isometry3d::MatrixType, Eigen::Dynamic>::type*>(NULL), I);
  body->I = bodyI.value();
}

void parseSDFLink(RigidBodyTree * model, XMLElement* node, const map<string, Vector4d, less<string>, aligned_allocator<pair<string, Vector4d> > >& materials, const map<string,string>& package_map, const string& root_dir)
{
  const char* attr = node->Attribute("drake_ignore");
  if (attr && strcmp(attr, "true") == 0) return;

  shared_ptr<RigidBody> body(new RigidBody());

  attr = node->Attribute("name");
  if (!attr) throw runtime_error("ERROR: link tag is missing name attribute");
  body->linkname = attr;

  if (body->linkname == "world") throw runtime_error("ERROR: do not name a link 'world', it is a reserved name");

  Isometry3d T_element_to_link = Isometry3d::Identity();
  XMLElement* pose = node->FirstChildElement("pose");
  if (pose) poseAttributesToTransform(pose, T_element_to_link);

  XMLElement* inertial_node = node->FirstChildElement("inertial");
  if (inertial_node) parseSDFInertial(body, inertial_node, model, T_element_to_link);

/*
  for (XMLElement* visual_node = node->FirstChildElement("visual"); visual_node; visual_node = visual_node->NextSiblingElement("visual")) {
    parseVisual(body, visual_node, model, materials, package_map, root_dir);
  }

  for (XMLElement* collision_node = node->FirstChildElement("collision"); collision_node; collision_node = collision_node->NextSiblingElement("collision")) {
    parseCollision(body, collision_node, model, package_map, root_dir);
  }
*/

  model->bodies.push_back(body);
  body->body_index = static_cast<int>(model->bodies.size()) - 1;
}

void parseModel(RigidBodyTree * model, XMLElement* node, const map<string,string> package_map, const string &root_dir, const DrakeJoint::FloatingBaseType floating_base_type) {
/*
  if (!node->Attribute("name"))
    throw runtime_error("Error: your model must have a name attribute");
  string model_name = node->Attribute("name");
*/

  // parse material elements
  map<string, Vector4d, less<string>, aligned_allocator<pair<string, Vector4d> > > materials;

  // parse link elements
  for (XMLElement *link_node = node->FirstChildElement("link"); link_node; link_node = link_node->NextSiblingElement("link"))
    parseSDFLink(model, link_node, materials, package_map, root_dir);
}

void parseSDF(RigidBodyTree * model, XMLDocument * xml_doc, map<string,string>& package_map, const string &root_dir, const DrakeJoint::FloatingBaseType floating_base_type)
{
  populatePackageMap(package_map);

  XMLElement * node = xml_doc->FirstChildElement("sdf");
  if (!node) throw std::runtime_error("ERROR: This xml file does not contain an sdf tag");

  for (XMLElement *model_node = node->FirstChildElement("model"); model_node; model_node = model_node->NextSiblingElement("model"))
    parseModel(model, model_node, package_map, root_dir, floating_base_type);

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

