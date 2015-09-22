#include <string>
#include <fstream>
#include <sstream>

#include "spruce.hh"

#include "tinyxml.h"
#include "RigidBodyManipulator.h"
#include "joints/FixedJoint.h"
#include "joints/HelicalJoint.h"
#include "joints/PrismaticJoint.h"
#include "joints/RevoluteJoint.h"
#include "joints/QuaternionFloatingJoint.h"
#include "joints/RollPitchYawFloatingJoint.h"

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

string exec(string cmd)
{
	FILE* pipe = POPEN(cmd.c_str(), "r");
	if (!pipe) return "ERROR";
	char buffer[128];
	string result = "";
	while(!feof(pipe)) {
		if(fgets(buffer, 128, pipe) != NULL)
			result += buffer;
    }
	PCLOSE(pipe);
	return result;
}

void searchenvvar(map<string,string> &package_map, string envvar)
{
	char* cstrpath = getenv(envvar.c_str());
	if (!cstrpath) return;

	string path(cstrpath), token, t;
	istringstream iss(path);

	while (getline(iss,token,':')) {
		istringstream p(exec("find -L "+token+" -iname package.xml"));
	  while (getline(p,t)) {
      spruce::path mypath_s(t);
      auto path_split = mypath_s.split();
      if (path_split.size() > 2) {
        string package = path_split.at(path_split.size()-2);
        auto package_iter = package_map.find(package);
        // Don't overwrite entries in the map
        if (package_iter == package_map.end()) {
          package_map.insert(make_pair(package, mypath_s.root().append("/")));
        }
        //cout << mypath.getFileName() << endl;
      }
	  }
	}
}

void populatePackageMap(map<string,string>& package_map)
{
  searchenvvar(package_map,"ROS_ROOT");
  searchenvvar(package_map,"ROS_PACKAGE_PATH");
}

bool rospack(const string& package, const map<string,string>& package_map, string& package_path)
{
	// my own quick and dirty implementation of the rospack algorithm (based on my matlab version in rospack.m)
	auto iter = package_map.find(package);
	if (iter != package_map.end()) {
    package_path = iter->second;
		return true;
  } else {
    cerr << "Warning: Couldn't find package '" << package << "' in ROS_ROOT, ROS_PACKAGE_PATH, or user supplied package map" << endl;
    return false;
  }
}

string resolveFilename(const string& filename, const map<string,string>& package_map, const string& root_dir)
{
  spruce::path mesh_filename_s;
  spruce::path raw_filename_s(filename);

  auto split_filename = raw_filename_s.split();

  if (split_filename.front() == "package:") {
    string package_path_string;
    if (rospack(split_filename.at(2), package_map, package_path_string)) {
      spruce::path package_path_s = spruce::path(package_path_string);
      mesh_filename_s = package_path_s;

      auto split_raw = raw_filename_s.split();
      for (int i = 1; i < split_raw.size()-2; ++i) {
        mesh_filename_s.append(split_raw.at(i+2));
      }
    } else {
      cerr << "Warning: Mesh '" << filename << "' could not be resolved and will be ignored by Drake." << endl;
      return string();
    }
  } else {
    mesh_filename_s = spruce::path(root_dir);
    mesh_filename_s.append(filename);
  }
  if (!mesh_filename_s.exists()) {
    cerr << "Warning: File '" << mesh_filename_s.getStr() << "' could not be found." << endl;
    cerr << "Warning: Mesh '" << filename << "' could not be resolved and will be ignored by Drake." << endl;
    return string();
  }
  return mesh_filename_s.getStr();
}

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

bool parseScalarValue(TiXmlElement* node, double &val)
{
  const char* strval = node->FirstChild()->Value();
  if (strval) {
    stringstream s(strval);
    s >> val;
    return true;
  }
  return false;
}

bool parseScalarAttribute(TiXmlElement* node, const char* attribute_name, double& val)
{
  const char* attr = node->Attribute(attribute_name);
  if (attr) {
    stringstream s(attr);
    s >> val;
    return true;
  }
  return false;
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

bool parseVectorAttribute(TiXmlElement* node, const char* attribute_name, Vector4d &val)
{
  const char* attr = node->Attribute(attribute_name);
  if (attr) {
    stringstream s(attr);
    s >> val(0) >> val(1) >> val(2) >> val(3);
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

  body->com << T(0, 3), T(1, 3), T(2, 3);

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

bool parseMaterial(TiXmlElement* node, map<string, Vector4d, less<string>, aligned_allocator<pair<string, Vector4d> > > & materials)
{
  const char* attr;
  attr = node->Attribute("name");
  if (!attr || strlen(attr) == 0) {
    cerr << "WARNING: material tag is missing a name" << endl;
    return false;
  }
  string name(attr);
  auto material_iter = materials.find(name);
  bool already_in_map = false;
  if (material_iter != materials.end()) {
    already_in_map = true;
  }

  Vector4d rgba;
  TiXmlElement* color_node = node->FirstChildElement("color");
  if (color_node) {
    if (!parseVectorAttribute(color_node, "rgba", rgba)) {
      cerr << "WARNING: color tag is missing rgba attribute" << endl;
      return false;
    }
    materials[name] = rgba;
  } else if (!already_in_map) {
    cerr << "WARNING: material \"" << name << "\" is not a simple color material (so is currently unsupported)" << endl;
    return false;
  }
  return true;
}

bool parseGeometry(TiXmlElement* node, const map<string,string>& package_map, const string& root_dir, DrakeShapes::Element& element)
{
  // DEBUG
  //cout << "parseGeometry: START" << endl;
  // END_DEBUG
  const char* attr;
  TiXmlElement* shape_node;
  if ((shape_node = node->FirstChildElement("box"))) {
    double x = 0, y = 0, z = 0;
    attr = shape_node->Attribute("size");
    if (attr) {
      stringstream s(attr);
      s >> x >> y >> z;
    } else {
      cerr << "ERROR parsing box element size" << endl;
      return false;
    }
    element.setGeometry(DrakeShapes::Box(Vector3d(x, y, z)));
  } else if ((shape_node = node->FirstChildElement("sphere"))) {
    double r = 0;
    attr = shape_node->Attribute("radius");
    if (attr) {
      stringstream s(attr);
      s >> r;
    } else {
      cerr << "ERROR parsing sphere element radius" << endl;
      return false;
    }
    element.setGeometry(DrakeShapes::Sphere(max(MIN_RADIUS, r)));
  } else if ((shape_node = node->FirstChildElement("cylinder"))) {
    double r = 0, l = 0;
    attr = shape_node->Attribute("radius");
    if (attr) {
      stringstream s(attr);
      s >> r;
    } else {
      cerr << "ERROR parsing cylinder element radius" << endl;
      return false;
    }

    attr = shape_node->Attribute("length");
    if (attr) {
      stringstream s(attr);
      s >> l;
    } else {
      cerr << "ERROR parsing cylinder element length" << endl;
      return false;
    }
    element.setGeometry(DrakeShapes::Cylinder(r, l));
  } else if ((shape_node = node->FirstChildElement("capsule"))) {
    double r = 0, l = 0;
    attr = shape_node->Attribute("radius");
    if (attr) {
      stringstream s(attr);
      s >> r;
    } else {
      cerr << "ERROR parsing capsule element radius" << endl;
      return false;
    }

    attr = shape_node->Attribute("length");
    if (attr) {
      stringstream s(attr);
      s >> l;
    } else {
      cerr << "ERROR: Failed to parse capsule element length" << endl;
      return false;
    }
    element.setGeometry(DrakeShapes::Capsule(r, l));
  } else if ((shape_node = node->FirstChildElement("mesh"))) {
    attr = shape_node->Attribute("filename");
    if (!attr) {
      cerr << "ERROR mesh element has no filename tag" << endl;
      return false;
    }
    string filename(attr);
    string resolved_filename = resolveFilename(filename, package_map, root_dir);
    DrakeShapes::Mesh mesh(filename, resolved_filename);

    attr = shape_node->Attribute("scale");
    if (attr) {
      stringstream s(attr);
      s >> mesh.scale;
    }

    element.setGeometry(mesh);
  } else {
    cerr << "Warning: geometry element has an unknown type and will be ignored." << endl;
  }
  // DEBUG
  //cout << "parseGeometry: END" << endl;
  // END_DEBUG
  return true;
}

bool parseVisual(shared_ptr<RigidBody> body, TiXmlElement* node, RigidBodyManipulator* model, const map<string, Vector4d, less<string>, aligned_allocator<pair<string, Vector4d> > >& materials, const map<string,string>& package_map, const string& root_dir)
{
  // DEBUG
  //cout << "parseVisual: START" << endl;
  // END_DEBUG
  Matrix4d T_element_to_link = Matrix4d::Identity();
  TiXmlElement* origin = node->FirstChildElement("origin");
  if (origin)
    poseAttributesToTransform(origin, T_element_to_link);

  string group_name;

  TiXmlElement* geometry_node = node->FirstChildElement("geometry");
  if (!geometry_node) {
    cerr << "ERROR: Link " << body->linkname << " has a visual element without geometry." << endl;
    return false;
  }

  DrakeShapes::VisualElement element(T_element_to_link);
  if (!parseGeometry(geometry_node, package_map, root_dir, element)) {
    cerr << "ERROR: Failed to parse visual element in link " << body->linkname << "." << endl;
    return false;
  }

  TiXmlElement* material_node = node->FirstChildElement("material");
  if (material_node) {
    const char* attr;
    attr = material_node->Attribute("name");
    if (attr && strlen(attr) > 0 && materials.find(attr) != materials.end()) {
      element.setMaterial(materials.at(attr));
    } else {
      TiXmlElement* color_node = material_node->FirstChildElement("color");
      if (color_node) {
        Vector4d rgba;
        if (!parseVectorAttribute(color_node, "rgba", rgba)) {
          cerr << "WARNING: Failed to parse color element rgba in visual" << endl;
        } else {
          element.setMaterial(rgba);
        }
      } else {
        cerr << "WARNING: visual element had a material with neither a name nor a nested color element" << endl;
      }
    }
  }

  if (element.hasGeometry()) {
    // DEBUG
    //cout << "parseVisual: Adding element to body" << endl;
    // END_DEBUG
    body->addVisualElement(element);
  }

  // DEBUG
  //cout << "parseVisual: END" << endl;
  // END_DEBUG
  return true;
}

bool parseCollision(shared_ptr<RigidBody> body, TiXmlElement* node, RigidBodyManipulator* model, const map<string,string>& package_map, const string& root_dir)
{
  Matrix4d T_element_to_link = Matrix4d::Identity();
  TiXmlElement* origin = node->FirstChildElement("origin");
  if (origin)
    poseAttributesToTransform(origin, T_element_to_link);

  const char* attr;
  string group_name;

  attr = node->Attribute("group");
  if (attr) {
    group_name = attr;
  } else {
    group_name = "default";;
  }

  TiXmlElement* geometry_node = node->FirstChildElement("geometry");
  if (!geometry_node) {
    cerr << "ERROR: Link " << body->linkname << " has a collision element without geometry" << endl;
    return false;
  }

  RigidBody::CollisionElement element(T_element_to_link, body);
  if (!parseGeometry(geometry_node, package_map, root_dir, element)) {
    cerr << "ERROR: Failed to parse collision element in link " << body->linkname << "." << endl;
    return false;
  }

  if (element.hasGeometry()) {
    model->addCollisionElement(element, body, group_name);
  }

  return true;
}

bool parseLink(RigidBodyManipulator* model, TiXmlElement* node, const map<string, Vector4d, less<string>, aligned_allocator<pair<string, Vector4d> > >& materials, const map<string,string>& package_map, const string& root_dir)
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
  if (body->linkname == "world")
    throw runtime_error("ERROR: do not name a link 'world', it is a reserved name");

  TiXmlElement* inertial_node = node->FirstChildElement("inertial");
  if (inertial_node)
    if (!parseInertial(body, inertial_node, model))
      return false;

  for (TiXmlElement* visual_node = node->FirstChildElement("visual"); visual_node; visual_node = visual_node->NextSiblingElement("visual")) {
    if (!parseVisual(body, visual_node, model, materials, package_map, root_dir)) {
      printf("error parsing visual\n");
      return false;
    }
  }

  for (TiXmlElement* collision_node = node->FirstChildElement("collision"); collision_node; collision_node = collision_node->NextSiblingElement("collision")) {
    if (!parseCollision(body, collision_node, model, package_map, root_dir)) {
      printf("error parsing collision\n");
      return false;
    }
  }

  model->bodies.push_back(body);
  body->body_index = static_cast<int>(model->bodies.size()) - 1;

  return true;
}

template <typename JointType>
void setLimits(TiXmlElement *node, FixedAxisOneDoFJoint<JointType> *fjoint) {
  TiXmlElement* limit_node = node->FirstChildElement("limit");
  if (fjoint != nullptr && limit_node) {
    double lower = -numeric_limits<double>::infinity(), upper = numeric_limits<double>::infinity();
    parseScalarAttribute(limit_node,"lower",lower);
    parseScalarAttribute(limit_node,"upper",upper);
    fjoint->setJointLimits(lower,upper);
  }
}

template <typename JointType>
void setDynamics(RigidBodyManipulator *model, TiXmlElement *node, FixedAxisOneDoFJoint<JointType> *fjoint) {
  TiXmlElement* dynamics_node = node->FirstChildElement("dynamics");
  if (fjoint != nullptr && dynamics_node) {
    model->warnOnce("joint_dynamics", "Warning: joint dynamics xml tag is parsed, but not included in the dynamics methods yet.");
    double damping=0.0, coulomb_friction=0.0, coulomb_window=0.0;
    parseScalarAttribute(dynamics_node,"damping",damping);
    parseScalarAttribute(dynamics_node,"friction",coulomb_friction);
    parseScalarAttribute(dynamics_node,"coulomb_window",coulomb_window);
    fjoint->setDynamics(damping,coulomb_friction,coulomb_window);
  }
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

  // now construct the actual joint (based on it's type)
  DrakeJoint* joint = nullptr;

  if (type.compare("revolute") == 0 || type.compare("continuous") == 0) {
    FixedAxisOneDoFJoint<RevoluteJoint>* fjoint = new RevoluteJoint(name, Ttree, axis);
    setDynamics(model, node, fjoint);
    setLimits(node, fjoint);
    joint = fjoint;
  } else if (type.compare("fixed") == 0) {
    joint = new FixedJoint(name, Ttree);
  } else if (type.compare("prismatic") == 0) {
    FixedAxisOneDoFJoint<PrismaticJoint>* fjoint = new PrismaticJoint(name, Ttree, axis);
    setDynamics(model, node, fjoint);
    setLimits(node, fjoint);
    joint = fjoint;
  } else if (type.compare("floating") == 0) {
    joint = new RollPitchYawFloatingJoint(name, Ttree);
  } else {
    cerr << "ERROR: Unrecognized joint type: " << type << endl;
    return false;
  }


  unique_ptr<DrakeJoint> joint_unique_ptr(joint);
  model->bodies[child_index]->setJoint(move(joint_unique_ptr));
  model->bodies[child_index]->parent = model->bodies[parent_index];

  return true;
}

bool parseTransmission(RigidBodyManipulator* model, TiXmlElement* node)
{
  const char* attr = nullptr;
  TiXmlElement* type_node = node->FirstChildElement("type");
  if ( type_node ) {
    attr = type_node->GetText();
  }

  if (!attr) {
    attr = node->Attribute("type"); // old URDF format, kept for convenience

    if (!attr) {
      cerr << "ERROR: transmission element is missing the type child" << endl;
      return false;
    }
  }
  string type(attr);
  if (type.find("SimpleTransmission")==string::npos) {
    cerr << "WARNING: only SimpleTransmissions are supported right now.  this element will be skipped." << endl;
    return true;
  }

  TiXmlElement* actuator_node = node->FirstChildElement("actuator");
  if (!actuator_node || !actuator_node->Attribute("name"))  {
    cerr << "ERROR: transmission is missing an actuator element" << endl;
    return false;
  }
  string actuator_name(actuator_node->Attribute("name"));


  TiXmlElement* joint_node = node->FirstChildElement("joint");
  if (!joint_node || !joint_node->Attribute("name"))  {
    cerr << "ERROR: transmission is missing a joint element" << endl;
    return false;
  }
  string joint_name(joint_node->Attribute("name"));

  int body_index = findLinkIndexByJointName(model,joint_name);

  if (model->bodies[body_index]->getJoint().getNumPositions() == 0) {
    cerr << "WARNING: Skipping transmission since it's attached to a fixed joint: " << joint_name << endl;
    return true;
  }

  TiXmlElement* reduction_node = node->FirstChildElement("mechanicalReduction");
  double gain = 1.0;
  if (reduction_node) parseScalarValue(reduction_node, gain);

  RigidBodyActuator a(actuator_name,model->bodies[body_index],gain);
  model->actuators.push_back(a);
  return true;
}

bool parseLoop(RigidBodyManipulator* model, TiXmlElement* node)
{
  Vector3d xyz=Vector3d::Zero(),rpy=Vector3d::Zero(),axis;
  axis << 1.0, 0.0, 0.0;

  if (!node || !node->Attribute("name"))  {
    cerr << "ERROR: loop is missing a name element" << endl;
    return false;
  }
  string name(node->Attribute("name"));


  TiXmlElement* link_node = node->FirstChildElement("link1");
  string linkname = link_node->Attribute("link");
  std::shared_ptr<RigidBody> body = model->findLink(linkname);
  if (!body) {
    cerr << "couldn't find link %s referenced in loop joint " << linkname << endl;
    return false;
  }
  if (!parseVectorAttribute(link_node, "xyz", xyz)) {
    cerr << "ERROR parsing loop joint xyz" << endl;
    return false;
  }
  if (!parseVectorAttribute(link_node, "rpy", rpy)) {
    cerr << "ERROR parsing loop joint rpy" << endl;
    return false;
  }
  std::shared_ptr<RigidBodyFrame> frameA = make_shared<RigidBodyFrame>(name+"FrameA",body,xyz,rpy);

  link_node = node->FirstChildElement("link2");
  linkname = link_node->Attribute("link");
  xyz=Vector3d::Zero();
  rpy=Vector3d::Zero();
  body = model->findLink(linkname);
  if (!body) {
    cerr << "couldn't find link %s referenced in loop joint " << linkname << endl;
    return false;
  }
  if (!parseVectorAttribute(link_node, "xyz", xyz)) {
    cerr << "ERROR parsing loop joint xyz" << endl;
    return false;
  }
  if (!parseVectorAttribute(link_node, "rpy", rpy)) {
    cerr << "ERROR parsing loop joint rpy" << endl;
    return false;
  }
  std::shared_ptr<RigidBodyFrame> frameB = make_shared<RigidBodyFrame>(name+"FrameB",body,xyz,rpy);

  TiXmlElement* axis_node = node->FirstChildElement("axis");
  if (axis_node && !parseVectorAttribute(axis_node, "xyz", axis)) {
    cerr << "ERROR parsing loop joint axis" << endl;
    return false;
  }

  model->addFrame(frameA);
  model->addFrame(frameB);
  RigidBodyLoop l(frameA,frameB,axis);
  model->loops.push_back(l);

  return true;
}

bool parseFrame(RigidBodyManipulator* model, TiXmlElement* node)
{
  Vector3d xyz=Vector3d::Zero(), rpy=Vector3d::Zero();

  parseVectorAttribute(node, "xyz", xyz);
  parseVectorAttribute(node, "rpy", rpy);

  const char* frame_link = node->Attribute("link");

  if (!frame_link) {
    cerr << "ERROR parsing Drake frame linkname" << endl;
    return false;
  }
  const std::shared_ptr<RigidBody> body = model->findLink(frame_link);
  if (!body) {
    cerr << "ERROR parsing Drake frame: couldn't find link " << frame_link << endl;
    return false;
  }

  const char* frame_name = node->Attribute("name");

  if (!frame_name) {
    cerr << "ERROR parsing Drake frame name" << endl;
    return false;
  }


  Matrix4d T;
  T << rpy2rotmat(rpy), xyz, 0,0,0,1;

  std::shared_ptr<RigidBodyFrame> frame = make_shared<RigidBodyFrame>(frame_name,body,T);
  model->addFrame(frame);


  return true;
}

bool parseRobot(RigidBodyManipulator* model, TiXmlElement* node, const map<string,string> package_map, const string &root_dir, const DrakeJoint::FloatingBaseType floating_base_type)
{
  if (!node->Attribute("name")) {
    cerr << "Error: your robot must have a name attribute" << endl;
    return false;
  }
  string robotname = node->Attribute("name");

  // parse material elements
  map<string, Vector4d, less<string>, aligned_allocator<pair<string, Vector4d> > > materials;
  for (TiXmlElement* link_node = node->FirstChildElement("material"); link_node; link_node = link_node->NextSiblingElement("material")) {
    parseMaterial(link_node, materials);  // accept failed material parsing
  }

  // parse link elements
  for (TiXmlElement* link_node = node->FirstChildElement("link"); link_node; link_node = link_node->NextSiblingElement("link"))
    if (!parseLink(model, link_node, materials, package_map, root_dir)) {
      return false;
    }
  //DEBUG
    //else {
      //cout << "Parsed link" << endl;
      //cout << "model->bodies.size() = " << model->bodies.size() << endl;
      //cout << "model->num_bodies = " << model->num_bodies << endl;
    //}
  //END_DEBUG

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

  // parse Drake frames
  for (TiXmlElement* frame_node = node->FirstChildElement("frame"); frame_node; frame_node = frame_node->NextSiblingElement("frame"))
    if (!parseFrame(model, frame_node))
      return false;

  for (unsigned int i = 1; i < model->bodies.size(); i++) {
    if (model->bodies[i]->parent == nullptr) {  // attach the root nodes to the world with a floating base joint
			model->bodies[i]->parent = model->bodies[0];
			switch (floating_base_type) {
      case DrakeJoint::FIXED:
        {
          unique_ptr<DrakeJoint> joint(new FixedJoint("base", Isometry3d::Identity()));
          model->bodies[i]->setJoint(move(joint));
        }
        break;
			case DrakeJoint::ROLLPITCHYAW:
        {
          unique_ptr<DrakeJoint> joint(new RollPitchYawFloatingJoint("base", Isometry3d::Identity()));
          model->bodies[i]->setJoint(move(joint));
        }
				break;
			case DrakeJoint::QUATERNION:
        {
          unique_ptr<DrakeJoint> joint(new FixedJoint("base", Isometry3d::Identity()));
          model->bodies[i]->setJoint(move(joint));
        }
				break;
			default:
			  throw std::runtime_error("unknown floating base type");
    	}
    }
  }
  return true;
}

bool parseURDF(RigidBodyManipulator* model, TiXmlDocument * xml_doc, map<string,string>& package_map, const string &root_dir, const DrakeJoint::FloatingBaseType floating_base_type)
{
  populatePackageMap(package_map);
  TiXmlElement *node = xml_doc->FirstChildElement("robot");
  if (!node) {
    throw std::runtime_error("ERROR: This urdf does not contain a robot tag");
  }

  if (!parseRobot(model, node, package_map, root_dir, floating_base_type)) {
    throw std::runtime_error("ERROR: Failed to parse robot");
  }

  model->compile();
  return true;
}

bool RigidBodyManipulator::addRobotFromURDFString(const string &xml_string, const string &root_dir, const DrakeJoint::FloatingBaseType floating_base_type)
{
  map<string,string> package_map;
  return addRobotFromURDFString(xml_string, package_map, root_dir, floating_base_type);
}

bool RigidBodyManipulator::addRobotFromURDFString(const string &xml_string, map<string, string>& package_map, const string &root_dir, const DrakeJoint::FloatingBaseType floating_base_type)
{
  TiXmlDocument xml_doc;
  xml_doc.Parse(xml_string.c_str());
  return parseURDF(this,&xml_doc,package_map,root_dir,floating_base_type);
}

bool RigidBodyManipulator::addRobotFromURDF(const string &urdf_filename, const DrakeJoint::FloatingBaseType floating_base_type)
{
  map<string,string> package_map;
  return addRobotFromURDF(urdf_filename, package_map, floating_base_type);
}

bool RigidBodyManipulator::addRobotFromURDF(const string &urdf_filename, map<string,string>& package_map, const DrakeJoint::FloatingBaseType floating_base_type)
{
  TiXmlDocument xml_doc(urdf_filename);
  if (!xml_doc.LoadFile()) {
    
    throw std::runtime_error("failed to parse xml in file " + urdf_filename + "\n" + xml_doc.ErrorDesc());
  }

  string root_dir=".";
  size_t found = urdf_filename.find_last_of("/\\");
  if (found != string::npos) {
    root_dir = urdf_filename.substr(0, found);
  }

  return parseURDF(this,&xml_doc,package_map,root_dir,floating_base_type);
}
