#include <string>
#include <fstream>
#include <sstream>

#include "tinyxml.h"
#include "RigidBodyManipulator.h"
#include "joints/drakeJointUtil.h"
#include "joints/HelicalJoint.h"
#include "joints/PrismaticJoint.h"
#include "joints/RevoluteJoint.h"
#include "joints/QuaternionFloatingJoint.h"
#include "joints/RollPitchYawFloatingJoint.h"

using namespace std;

// todo: rectify this with findLinkId in the class (which makes more assumptions)
int findLinkIndex(RigidBodyManipulator* model, string linkname)
{
  int index = -1;
  for (unsigned int i = 0; i < model->bodies.size(); i++) {
    if (linkname.compare(model->bodies[i]->linkname) == 0) {
      index = i;
      break;
    }
  }
  return index;
}

int findLinkIndexByJointName(RigidBodyManipulator* model, string jointname)
{
  int index = -1;
  for (unsigned int i = 0; i < model->bodies.size(); i++) {
    if (model->bodies[i]->hasParent() && jointname.compare(model->bodies[i]->getJoint().getName())==0) {
      index = i;
      break;
    }
  }
  return index;
}

// only writes values if they exist
bool parseVectorAttribute(TiXmlElement* node, const char* attribute_name, Vector3d &val)
{
  const char* attr = node->Attribute(attribute_name);
  if (attr) {
    stringstream s(attr);
    s >> val(0) >> val(1) >> val(2);
    return true;
  }
  return false;
}

void poseAttributesToTransform(TiXmlElement* node, Matrix4d& T)
{
  double x = 0.0, y = 0.0, z = 0.0, roll = 0.0, pitch = 0.0, yaw = 0.0;

  const char* attr = node->Attribute("xyz");
  if (attr) {
    stringstream s(attr);
    s >> x >> y >> z;
  }

  attr = node->Attribute("rpy");
  if (attr) {
    stringstream s(attr);
    s >> roll >> pitch >> yaw;
  }

  T << cos(yaw) * cos(pitch), cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll), cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll), x, sin(yaw) * cos(pitch), sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll), sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll), y, -sin(
      pitch), cos(pitch) * sin(roll), cos(pitch) * cos(roll), z, 0, 0, 0, 1;
}

bool parseInertial(shared_ptr<RigidBody> body, TiXmlElement* node, RigidBodyManipulator* model)
{
  Isometry3d T = Isometry3d::Identity();

  TiXmlElement* origin = node->FirstChildElement("origin");
  if (origin)
    poseAttributesToTransform(origin, T.matrix());

  TiXmlElement* mass = node->FirstChildElement("mass");
  if (mass)
    mass->Attribute("value", &(body->mass));

  body->com << T(0, 3), T(1, 3), T(2, 3), 1.0;

  Matrix<double, TWIST_SIZE, TWIST_SIZE> I = Matrix<double, TWIST_SIZE, TWIST_SIZE>::Zero();
  I.block(3, 3, 3, 3) << body->mass * Matrix3d::Identity();

  TiXmlElement* inertia = node->FirstChildElement("inertia");
  if (inertia) {
    inertia->Attribute("ixx", &I(0, 0));
    inertia->Attribute("ixy", &I(0, 1));
    I(1, 0) = I(0, 1);
    inertia->Attribute("ixz", &I(0, 2));
    I(2, 0) = I(0, 2);
    inertia->Attribute("iyy", &I(1, 1));
    inertia->Attribute("iyz", &I(1, 2));
    I(2, 1) = I(1, 2);
    inertia->Attribute("izz", &I(2, 2));
  }

  auto bodyI = transformSpatialInertia(T, static_cast<Gradient<Isometry3d::MatrixType, Eigen::Dynamic>::type*>(NULL), I);
  body->I = bodyI.value();

  return true;
}

bool parseVisual(shared_ptr<RigidBody> body, TiXmlElement* node, RigidBodyManipulator* model)
{
  // todo:  consider implementing this, but I don't need it yet
  return true;
}

bool parseCollision(int body_index, TiXmlElement* node, RigidBodyManipulator* model)
{
  Isometry3d T = Isometry3d::Identity();
  TiXmlElement* origin = node->FirstChildElement("origin");
  if (origin)
    poseAttributesToTransform(origin, T.matrix());

  bool create_collision_element(true);
  const char* attr;
  DrakeCollision::Shape shape = DrakeCollision::UNKNOWN;
  vector<double> params;

  TiXmlElement* geometry_node = node->FirstChildElement("geometry");
  if (!geometry_node)
    return false;

  TiXmlElement* shape_node;
  if ((shape_node = geometry_node->FirstChildElement("box"))) {
    shape = DrakeCollision::BOX;
    double x = 0, y = 0, z = 0;
    attr = shape_node->Attribute("size");
    if (attr) {
      stringstream s(attr);
      s >> x >> y >> z;
    }
    params.push_back(x);
    params.push_back(y);
    params.push_back(z);
  } else if ((shape_node = geometry_node->FirstChildElement("sphere"))) {
    shape = DrakeCollision::SPHERE;
    double r = 0;
    attr = shape_node->Attribute("radius");
    if (attr) {
      stringstream s(attr);
      s >> r;
    }
    params.push_back(r);
  } else if ((shape_node = geometry_node->FirstChildElement("cylinder"))) {
    shape = DrakeCollision::CYLINDER;
    double r = 0, l = 0;
    attr = shape_node->Attribute("radius");
    if (attr) {
      stringstream s(attr);
      s >> r;
    }
    attr = shape_node->Attribute("length");
    if (attr) {
      stringstream s(attr);
      s >> l;
    }
    params.push_back(r);
    params.push_back(l);
  } else if ((shape_node = geometry_node->FirstChildElement("capsule"))) {
    shape = DrakeCollision::CAPSULE;
    double r = 0, l = 0;
    attr = shape_node->Attribute("radius");
    if (attr) {
      stringstream s(attr);
      s >> r;
    }
    attr = shape_node->Attribute("length");
    if (attr) {
      stringstream s(attr);
      s >> l;
    }
    params.push_back(r);
    params.push_back(l);
  } else if ((shape_node = geometry_node->FirstChildElement("mesh"))) {
    shape = DrakeCollision::MESH;
    cerr << "Warning: mesh collision elements will be ignored (until I re-implement the logic below sans boost)" << endl;
    create_collision_element = false;
    /*
     boost::shared_ptr<urdf::Mesh> mesh(boost::dynamic_pointer_cast<urdf::Mesh>(cptr->geometry));
     boost::filesystem::path mesh_filename(root_dir);
     boost::regex package(".*package://.*");
     if (!boost::regex_match(mesh->filename, package)) {
     mesh_filename /= mesh->filename;
     readObjFile(mesh_filename,params);
     } else {
     create_collision_element = false;
     if (print_mesh_package_warning) {
     cerr << "Warning: The robot '" << _urdf_model->getName()
     << "' contains collision geometries that specify mesh "
     << "files with the 'package://' syntax, which "
     << "URDFRigidBodyManipulator does not support. These "
     << "collision geometries will be ignored." << endl;
     print_mesh_package_warning = false;
     }
     }
     */
  } else {
    cerr << "ERROR: Link " << model->bodies[body_index]->linkname << " has a collision element with an unknown type" << endl;
    return false;
  }

  if (create_collision_element) {
    model->addCollisionElement(body_index,T.matrix(),shape,params);
  }

  return true;
}

bool parseLink(RigidBodyManipulator* model, TiXmlElement* node)
{
  const char* attr = node->Attribute("drake_ignore");
  if (attr && strcmp(attr, "true") == 0)
    return true;
  shared_ptr<RigidBody> body(new RigidBody());

  attr = node->Attribute("name");
  if (!attr) {
    cerr << "ERROR: link tag is missing name attribute" << endl;
    return false;
  }
  body->linkname = attr;

  TiXmlElement* inertial_node = node->FirstChildElement("inertial");
  if (inertial_node)
    if (!parseInertial(body, inertial_node, model))
      return false;

  for (TiXmlElement* visual_node = node->FirstChildElement("visual"); visual_node; visual_node = visual_node->NextSiblingElement("visual")) {
    if (!parseVisual(body, visual_node, model)) {
      return false;
    }
  }

  model->bodies.push_back(body);
  body->body_index = model->bodies.size() - 1;

  for (TiXmlElement* collision_node = node->FirstChildElement("collision"); collision_node; collision_node = collision_node->NextSiblingElement("collision")) {
    if (!parseCollision(body->body_index, collision_node, model))
      return false;
  }

  return true;
}

bool parseJoint(RigidBodyManipulator* model, TiXmlElement* node)
{
  const char* attr = node->Attribute("drake_ignore");
  if (attr && strcmp(attr, "true") == 0)
    return true;

  attr = node->Attribute("name");
  if (!attr) {
    cerr << "ERROR: joint tag is missing name attribute" << endl;
    return false;
  }
  string name(attr);

  attr = node->Attribute("type");
  if (!attr) {
    cerr << "ERROR: joint " << name << " is missing the type attribute" << endl;
    return false;
  }
  string type(attr);

  // parse parent
  TiXmlElement* parent_node = node->FirstChildElement("parent");
  if (!parent_node) {
    cerr << "ERROR: joint " << name << " doesn't have a parent node" << endl;
    return false;
  }
  attr = parent_node->Attribute("link");
  if (!attr) {
    cerr << "ERROR: joint " << name << " parent does not have a link attribute" << endl;
    return false;
  }
  string parent_name(attr);

  int parent_index = findLinkIndex(model, parent_name);
  if (parent_index < 0) {
    cerr << "ERROR: could not find parent link named " << parent_name << endl;
    return false;
  }

  // parse child
  TiXmlElement* child_node = node->FirstChildElement("child");
  if (!child_node) {
    cerr << "ERROR: joint " << name << " doesn't have a child node" << endl;
    return false;
  }
  attr = child_node->Attribute("link");
  if (!attr) {
    cerr << "ERROR: joint " << name << " child does not have a link attribute" << endl;
    return false;
  }
  string child_name(attr);

  int child_index = findLinkIndex(model, child_name);
  if (child_index < 0) {
    cerr << "ERROR: could not find child link named " << child_name << endl;
    return false;
  }

  Isometry3d Ttree = Isometry3d::Identity();
  TiXmlElement* origin = node->FirstChildElement("origin");
  if (origin) {
    poseAttributesToTransform(origin, Ttree.matrix());
    model->bodies[child_index]->Ttree = Ttree.matrix(); // scheduled for deletion
  }

  Vector3d axis;
  axis << 1, 0, 0;
  TiXmlElement* axis_node = node->FirstChildElement("axis");
  if (axis_node) {
    parseVectorAttribute(axis_node, "xyz", axis);
    if (axis.norm()<1e-8) {
      cerr << "ERROR: axis is zero.  don't do that" << endl;
      return false;
    }
    axis.normalize();
  }

  // todo: add damping, etc?

  TiXmlElement* limit_node = node->FirstChildElement("limit");
  if (limit_node) {
    cerr << "Warning: joint limits not (re-)implemented yet; they will be ignored." << endl;
  }

  // now construct the actual joint (based on it's type)
  DrakeJoint* joint = NULL;
  if (type.compare("revolute") == 0 || type.compare("continuous") == 0) {
    joint = new RevoluteJoint(name, Ttree, axis);
  } else if (type.compare("fixed") == 0) {
    // FIXME: implement a fixed joint class
    RevoluteJoint* rj = new RevoluteJoint(name, Ttree, axis);
    rj->setJointLimits(0, 0);
    joint = rj;
  } else if (type.compare("prismatic") == 0) {
    joint = new PrismaticJoint(name, Ttree, axis);
  } else if (type.compare("floating") == 0) {
    joint = new RollPitchYawFloatingJoint(name, Ttree);
  } else {
    cerr << "ERROR: Unrecognized joint type: " << type << endl;
    return false;
  }

  model->bodies[child_index]->setJoint(std::unique_ptr<DrakeJoint>(joint));
  model->bodies[child_index]->parent = model->bodies[parent_index];

  return true;
}

bool parseTransmission(RigidBodyManipulator* model, TiXmlElement* node)
{
  const char* attr = node->Attribute("type");
  if (!attr) {
    cerr << "ERROR: transmission element is missing the type attribute" << endl;
    return false;
  }
  string type(attr);
  if (type.find("SimpleTransmission")==string::npos) {
    cerr << "WARNING: only SimpleTransmissions are supported right now.  this element will be skipped." << endl;
    return true;
  }

  TiXmlElement* joint_node = node->FirstChildElement("joint");
  if (!joint_node || !joint_node->Attribute("name"))  {
    cerr << "ERROR: transmission is missing a joint element" << endl;
    return false;
  }
  string joint_name(joint_node->Attribute("name"));
  int body_index = findLinkIndexByJointName(model,joint_name);

  TiXmlElement* reduction_node = node->FirstChildElement("mechanicalReduction");
  double gain = 1.0;
  if (reduction_node) sscanf(reduction_node->Value(),"%lf",&gain);

  RigidBodyActuator a(joint_name,model->bodies[body_index],gain);
  model->actuators.push_back(a);
  return true;
}

bool parseLoop(RigidBodyManipulator* model, TiXmlElement* node)
{
  Vector3d ptA,ptB;

  TiXmlElement* link_node = node->FirstChildElement("link1");
  string linkname = link_node->Attribute("link");
  int bodyA = findLinkIndex(model,linkname);
  if (bodyA<0) {
    cerr << "couldn't find link %s referenced in loop joint " << linkname << endl;
    return false;
  }
  if (!parseVectorAttribute(link_node, "xyz", ptA)) {
    cerr << "ERROR parsing loop joint xyz" << endl;
    return false;
  }

  link_node = node->FirstChildElement("link2");
  linkname = link_node->Attribute("link");
  int bodyB = findLinkIndex(model,linkname);
  if (bodyB<0) {
    cerr << "couldn't find link %s referenced in loop joint " << linkname << endl;
    return false;
  }
  if (!parseVectorAttribute(link_node, "xyz", ptB)) {
    cerr << "ERROR parsing loop joint xyz" << endl;
    return false;
  }

  RigidBodyLoop l(model->bodies[bodyA],ptA,model->bodies[bodyB],ptB);
  model->loops.push_back(l);
  return true;
}

bool parseRobot(RigidBodyManipulator* model, TiXmlElement* node, const string &root_dir)
{
  string robotname = node->Attribute("name");

  // parse link elements
  for (TiXmlElement* link_node = node->FirstChildElement("link"); link_node; link_node = link_node->NextSiblingElement("link"))
    if (!parseLink(model, link_node))
      return false;

  // todo: parse collision filter groups

  // parse joints
  for (TiXmlElement* joint_node = node->FirstChildElement("joint"); joint_node; joint_node = joint_node->NextSiblingElement("joint"))
    if (!parseJoint(model, joint_node))
      return false;

  // parse transmission elements
  for (TiXmlElement* transmission_node = node->FirstChildElement("transmission"); transmission_node; transmission_node = transmission_node->NextSiblingElement("transmission"))
    if (!parseTransmission(model, transmission_node))
      return false;

  // parse loop joints
  for (TiXmlElement* loop_node = node->FirstChildElement("loop_joint"); loop_node; loop_node = loop_node->NextSiblingElement("loop_joint"))
    if (!parseLoop(model, loop_node))
      return false;

  for (unsigned int i = 1; i < model->bodies.size(); i++) {
    if (model->bodies[i]->parent == nullptr) {  // attach the root nodes to the world with a floating base joint
      model->bodies[i]->parent = model->bodies[0];
      model->bodies[i]->setJoint(std::unique_ptr<DrakeJoint>(new RollPitchYawFloatingJoint("floating_rpy", Isometry3d::Identity())));
    }
  }
  return true;
}

bool parseURDF(RigidBodyManipulator* model, TiXmlDocument * xml_doc, const string &root_dir)
{
  TiXmlElement *node = xml_doc->FirstChildElement("robot");
  if (!node) {
    cerr << "ERROR: This urdf does not contain a robot tag" << endl;
    return false;
  }

  if (!parseRobot(model, node, root_dir))
    return false;

  model->compile();
  return true;
}

bool RigidBodyManipulator::addRobotFromURDFString(const string &xml_string, const string &root_dir)
{
  TiXmlDocument xml_doc;
  xml_doc.Parse(xml_string.c_str());
  return parseURDF(this,&xml_doc,root_dir);
}

bool RigidBodyManipulator::addRobotFromURDF(const string &urdf_filename)
{
  TiXmlDocument xml_doc(urdf_filename);
  if (!xml_doc.LoadFile()) {
    cerr << "ERROR: failed to load file " << urdf_filename << endl;
    return false;
  }

  string root_dir="";
  size_t found = urdf_filename.find_last_of("/\\");
  if (found != string::npos) {
    root_dir = urdf_filename.substr(0, found);
  }

  return parseURDF(this,&xml_doc,root_dir);
}
