
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
  for (int i=0; i<model->bodies.size(); i++) {
    if (linkname.compare(model->bodies[i]->linkname)==0) {
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
  double x=0.0,y=0.0,z=0.0, roll=0.0,pitch=0.0,yaw=0.0;

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

  T <<  cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll), x,
        sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll), y,
        -sin(pitch),         cos(pitch)*sin(roll),                             cos(pitch)*cos(roll),                             z,
        0, 0, 0, 1;
}

bool parseInertial(RigidBody* body, TiXmlElement* node, RigidBodyManipulator* model)
{
  Isometry3d T = Isometry3d::Identity();

  TiXmlElement* origin = node->FirstChildElement("origin");
  if (origin) poseAttributesToTransform(origin,T.matrix());

  TiXmlElement* mass = node->FirstChildElement("mass");
  if (mass) mass->Attribute("value",&(body->mass));

  body->com << T(0,3), T(1,3), T(2,3), 1.0;

  Matrix<double,TWIST_SIZE,TWIST_SIZE> I = Matrix<double,TWIST_SIZE,TWIST_SIZE>::Zero();
  I.block(3,3,3,3) << body->mass*Matrix3d::Identity();

  TiXmlElement* inertia = node->FirstChildElement("inertia");
  if (inertia) {
    inertia->Attribute("ixx",&I(0,0));
    inertia->Attribute("ixy",&I(0,1)); I(1,0)=I(0,1);
    inertia->Attribute("ixz",&I(0,2)); I(2,0)=I(0,2);
    inertia->Attribute("iyy",&I(1,1));
    inertia->Attribute("iyz",&I(1,2)); I(2,1)=I(1,2);
    inertia->Attribute("izz",&I(2,2));
  }

  auto bodyI = transformSpatialInertia(T,static_cast<  Gradient<Isometry3d::MatrixType, Eigen::Dynamic>::type* >(NULL),I);
  body->I = bodyI.value();

  return true;
}

bool parseVisual(RigidBody* body, TiXmlElement* node, RigidBodyManipulator* model)
{
  // todo:  consider implementing this, but I don't need it yet
  return true;
}

bool parseCollision(int body_index, TiXmlElement* node, RigidBodyManipulator* model)
{
  Isometry3d T = Isometry3d::Identity();
  TiXmlElement* origin = node->FirstChildElement("origin");
  if (origin) poseAttributesToTransform(origin,T.matrix());

  bool create_collision_element(true);
  const char* attr;
  DrakeCollision::Shape shape = DrakeCollision::Shape::UNKNOWN;
  vector<double> params;

  TiXmlElement* shape_node;
  if ((shape_node = node->FirstChildElement("box"))) {
    shape = DrakeCollision::Shape::BOX;
    double x=0,y=0,z=0;
    attr = shape_node->Attribute("size");
    if (attr) {
      stringstream s(attr);
      s >> x >> y >> z;
    }
    params.push_back(x);
    params.push_back(y);
    params.push_back(z);
  } else if ((shape_node = node->FirstChildElement("sphere"))) {
    shape = DrakeCollision::Shape::SPHERE;
    double r=0;
    attr = shape_node->Attribute("radius");
    if (attr) {
      stringstream s(attr);
      s >> r;
    }
    params.push_back(r);
  } else if ((shape_node = node->FirstChildElement("cylinder"))) {
    shape = DrakeCollision::CYLINDER;
    double r=0,l=0;
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
  } else if ((shape_node = node->FirstChildElement("mesh"))) {
    shape = DrakeCollision::Shape::MESH;
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

  if (create_collision_element){
    model->addCollisionElement(body_index,T.matrix(),shape,params);
  }

  return true;
}

bool parseLink(RigidBodyManipulator* model, TiXmlElement* node)
{
  const char* attr = node->Attribute("drake_ignore");
  if (attr && strcmp(attr,"true")==0) return true;
  RigidBody* body = new RigidBody();

  attr = node->Attribute("name");
  if (!attr) {
    cerr << "ERROR: link tag is missing name attribute" << endl;
    delete body;
    return false;
  }
  body->linkname = attr;

  TiXmlElement* inertial_node = node->FirstChildElement("inertial");
  if (inertial_node) if (!parseInertial(body,inertial_node,model)) return false;

  for (TiXmlElement* visual_node = node->FirstChildElement("visual"); visual_node; visual_node = visual_node->NextSiblingElement("visual")) {
    if (!parseVisual(body,visual_node,model)) {
      delete body;
      return false;
    }
  }

  model->bodies.push_back(std::unique_ptr<RigidBody>(body));
  int body_index = model->bodies.size();

  for (TiXmlElement* collision_node = node->FirstChildElement("collision"); collision_node; collision_node = collision_node->NextSiblingElement("collision")) {
    if (!parseCollision(body_index,collision_node,model)) return false;
  }

  return true;
}

bool parseJoint(RigidBodyManipulator* model, TiXmlElement* node)
{
  const char* attr = node->Attribute("drake_ignore");
  if (attr && strcmp(attr,"true")==0) return true;

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

  int parent_index = findLinkIndex(model,parent_name);
  if (parent_index<0) {
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

  int child_index = findLinkIndex(model,child_name);
  if (child_index<0) {
    cerr << "ERROR: could not find child link named " << child_name << endl;
    return false;
  }

  Isometry3d T = Isometry3d::Identity();
  TiXmlElement* origin = node->FirstChildElement("origin");
  if (origin) poseAttributesToTransform(origin,T.matrix());

  Vector3d axis;  axis << 1,0,0;
  TiXmlElement* axis_node = node->FirstChildElement("axis");
  if (axis_node) {
    parseVectorAttribute(axis_node,"xyz",axis);
    axis.normalize();
  }

  // todo: add damping, etc?

  TiXmlElement* limit_node = node->FirstChildElement("limit");
  if (limit_node) {
    cerr << "Warning: joint limits not (re-)implemented yet; they will be ignored." << endl;
  }


  // now construct the actual joint (based on it's type)
  DrakeJoint* joint = NULL;
  if (type.compare("revolute")==0 || type.compare("continuous")==0) {
    joint = new RevoluteJoint(name, T, axis);
  } else if (type.compare("fixed")==0) {
    // FIXME: implement a fixed joint class
    RevoluteJoint* rj = new RevoluteJoint(name, T, axis);
    rj->setJointLimits(0,0);
    joint = rj;
  } else if (type.compare("prismatic")==0) {
    joint = new PrismaticJoint(name, T, axis);
  } else if (type.compare("floating")==0) {
    joint = new RollPitchYawFloatingJoint(name, T);
  } else {
    cerr << "ERROR: Unrecognized joint type: " << type << endl;
    return false;
  }

  model->bodies[child_index]->setJoint(std::unique_ptr<DrakeJoint>(joint));
  model->bodies[child_index]->parent = parent_index;

  return true;
}

bool parseTransmission(RigidBodyManipulator* model, TiXmlElement* node)
{
/*
    TiXmlElement* node = transmission_xml->FirstChildElement("joint");
    if (!node) continue;

    int _dofnum = 0;
//    map<string, int>::const_iterator dn=findWithSuffix(dofname_to_dofnum,node->Attribute("name"));
//    if (dn == dofname_to_dofnum.end()) ROS_ERROR("can't find joint %s for transmission element.  this shouldn't happen");
//    _dofnum = dn->second;
//    cout << "adding actuator to joint " << node->Attribute("name") << " (dof: " << _dofnum << ")" << endl;

    node = transmission_xml->FirstChildElement("mechanicalReduction");
    double gain = 1.0;
    if (node) sscanf(node->Value(),"%lf",&gain);

    VectorXd B_col = VectorXd::Zero(num_velocities);
    B_col(_dofnum) = gain;

    B.conservativeResize(num_velocities, B.cols()+1);
    B.rightCols(1) = B_col;
  }
  */
  return true;
}

bool parseLoop(RigidBodyManipulator* model, TiXmlElement* node)
{
/*
    urdf::Vector3 pt;
    TiXmlElement* node = loop_xml->FirstChildElement("link1");
    int bodyA=-1,bodyB=-1;

    string linkname = node->Attribute("link");
    for (int i=0; i<num_bodies; i++)
      if (linkname==bodies[i]->linkname) {
        bodyA=i;
        break;
      }
    if (bodyA<0) ROS_ERROR("couldn't find link %s referenced in loop joint",linkname.c_str());
    pt.init(node->Attribute("xyz"));
    Vector3d ptA;  ptA << pt.x, pt.y, pt.z;

    node = loop_xml->FirstChildElement("link2");
    linkname = node->Attribute("link");
    for (int i=0; i<num_bodies; i++)
      if (linkname==bodies[i]->linkname) {
        bodyB=i;
        break;
      }
    if (bodyB<0) ROS_ERROR("couldn't find link %s referenced in loop joint",linkname.c_str());
    pt.init(node->Attribute("xyz"));
    Vector3d ptB;  ptB << pt.x, pt.y, pt.z;

    RigidBodyLoop l(bodyA,ptA,bodyB,ptB);
    loops.push_back(l);
 */
  return true;
}

bool parseRobot(RigidBodyManipulator* model, TiXmlElement* node, const string &root_dir)
{
  string robotname = node->Attribute("name");

  // parse link elements
  for (TiXmlElement* link_node = node->FirstChildElement("link"); link_node; link_node = link_node->NextSiblingElement("link"))
    if (!parseLink(model,link_node)) return false;

  // todo: parse collision filter groups

  // parse joints
  for (TiXmlElement* joint_node = node->FirstChildElement("joint"); joint_node; joint_node = joint_node->NextSiblingElement("joint"))
    if (!parseJoint(model,joint_node)) return false;

  // parse transmission elements
  for (TiXmlElement* transmission_node = node->FirstChildElement("transmission"); transmission_node; transmission_node = transmission_node->NextSiblingElement("transmission"))
    if (!parseTransmission(model,transmission_node)) return false;

  // parse loop joints
  for (TiXmlElement* loop_node = node->FirstChildElement("loop_joint"); loop_node; loop_node = loop_node->NextSiblingElement("loop_joint"))
    if (!parseLoop(model,loop_node)) return false;

  for (int i=1; i<model->bodies.size(); i++) {
    if (model->bodies[i]->parent<0) {  // attach the root nodes to the world with a floating base joint
      model->bodies[i]->parent = 0;
      model->bodies[i]->setJoint(std::unique_ptr<DrakeJoint>(new RollPitchYawFloatingJoint("floating_rpy", Isometry3d::Identity())));
    }
  }
  return true;
}


bool RigidBodyManipulator::addRobotFromURDFString(const string &xml_string, const string &root_dir)
{
  TiXmlDocument xml_doc;
  xml_doc.Parse(xml_string.c_str());  // a little inefficient to parse a second time, but ok for now
  // eventually, we'll probably just crop out the ros urdf parser completely.

  TiXmlElement *node = xml_doc.FirstChildElement("robot");
  if (!node) {
    cerr << "ERROR: This urdf does not contain a robot tag" << endl;
    return false;
  }

  if (!parseRobot(this, node, root_dir))
    return false;

  compile();
  return true;
}

bool RigidBodyManipulator::addRobotFromURDF(const string &urdf_filename)
{
  string token;
  istringstream iss(urdf_filename);

  while (getline(iss,token,':')) {
    fstream xml_file(token.c_str(), fstream::in);
    string xml_string;
    if (xml_file.is_open()) {
      while ( xml_file.good() ) {
        string line;
        getline( xml_file, line);
        xml_string += (line + "\n");
      }
      xml_file.close();
    } else {
      cerr << "Could not open file ["<<urdf_filename.c_str()<<"] for parsing."<< endl;
      return false;
    }

    string pathname="";
//    boost::filesystem::path mypath(urdf_filename);
//    if (!mypath.empty() && mypath.has_parent_path())    // note: if you see a segfault on has_parent_path(), then you probably tried to load the model without a parent path. (it shouldn't segfault, but a workaround is to load the model with a parent path, e.g. ./FallingBrick.urdf instead of FallingBrick.urdf)
//      pathname = mypath.parent_path().string();
    // I got too many segfaults with boost.  Doing it the old school way...
    size_t found = urdf_filename.find_last_of("/\\");
    if (found != string::npos) {
      pathname = urdf_filename.substr(0,found);
    }

    // parse URDF to get model
    addRobotFromURDFString(xml_string,pathname);
  }

  return true;
}
