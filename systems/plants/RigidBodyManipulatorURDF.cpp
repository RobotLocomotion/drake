
#include <string>
#include <fstream>
#include <sstream>

#include "tinyxml.h"
#include "RigidBodyManipulator.h"

using namespace std;

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

  cout << "I = " << body->I << endl;

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
    return false;
  }
  body->linkname = attr;

  TiXmlElement* inertial_node = node->FirstChildElement("inertial");
  if (inertial_node) if (!parseInertial(body,inertial_node,model)) return false;

  for (TiXmlElement* visual_node = node->FirstChildElement("visual"); visual_node; visual_node = visual_node->NextSiblingElement("visual")) {
    if (!parseVisual(body,visual_node,model)) return false;
  }

  model->bodies.push_back(std::unique_ptr<RigidBody>(body));
  int body_index = model->bodies.size();

  for (TiXmlElement* collision_node = node->FirstChildElement("collision"); collision_node; collision_node = collision_node->NextSiblingElement("collision")) {
    if (!parseCollision(body_index,collision_node,model)) return false;
  }

  return true;
}

bool parseJoint()
{
  return true;
}

bool parseTransmission()
{
  return true;
}

bool parseLoop()
{
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


  // parse transmission elements
/*
  for (TiXmlElement* transmission_xml = node->FirstChildElement("transmission"); transmission_xml; transmission_xml = transmission_xml->NextSiblingElement("transmission"))
  {
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

  // parse loop joints
  for (TiXmlElement* loop_xml = robot_xml->FirstChildElement("loop_joint"); loop_xml; loop_xml = loop_xml->NextSiblingElement("loop_joint"))
  { // note: pushing this in without all of the surrounding drakeFunction logic just to get things moving.  this one needs to be fast.
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
  }
  */

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
