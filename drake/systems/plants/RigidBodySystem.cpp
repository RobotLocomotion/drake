
#include <stdexcept>
#include "RigidBodySystem.h"
#include "RigidBodyIK.h"  // required for resolving initial conditions
#include "urdfParsingUtil.h"

using namespace std;
using namespace Eigen;
using namespace Drake;


RigidBodySystem::StateVector<double> Drake::getInitialState(const RigidBodySystem& sys) {

  VectorXd x0 = Matrix<double,Dynamic,1>::Random(sys.tree->num_positions+sys.tree->num_velocities);

  if (sys.tree->getNumPositionConstraints()) {
    // todo: move this up to the system level?

    std::vector<RigidBodyLoop,Eigen::aligned_allocator<RigidBodyLoop> > const& loops = sys.tree->loops;

    int nq = sys.tree->num_positions;
    int num_constraints = 2*loops.size();
    RigidBodyConstraint** constraint_array = new RigidBodyConstraint*[num_constraints];

    Matrix<double,7,1> bTbp = Matrix<double,7,1>::Zero();  bTbp(3)=1.0;
    Vector2d tspan; tspan<<-std::numeric_limits<double>::infinity(),std::numeric_limits<double>::infinity();
    Vector3d zero = Vector3d::Zero();
    for (int i=0; i<loops.size(); i++) {
      constraint_array[2*i] = new RelativePositionConstraint(sys.tree.get(), zero, zero, zero, loops[i].frameA->frame_index, loops[i].frameB->frame_index,bTbp,tspan);
      constraint_array[2*i+1] = new RelativePositionConstraint(sys.tree.get(), loops[i].axis, loops[i].axis, loops[i].axis, loops[i].frameA->frame_index, loops[i].frameB->frame_index,bTbp,tspan);
    }

    int info;
    vector<string> infeasible_constraint;
    IKoptions ikoptions(sys.tree.get());

    VectorXd q_guess = x0.topRows(nq);
    VectorXd q(nq);

    inverseKin(sys.tree.get(),q_guess,q_guess,num_constraints,constraint_array,q,info,infeasible_constraint,ikoptions);
    if (info>=10) {
      cout << "INFO = " << info << endl;
      cout << infeasible_constraint.size() << " infeasible constraints:";
      size_t limit = infeasible_constraint.size();
      if (limit>5) { cout << " (only printing first 5)" << endl; limit=5; }
      cout << endl;
      for (int i=0; i<limit; i++)
        cout << infeasible_constraint[i] << endl;
    }
    x0 << q, VectorXd::Zero(sys.tree->num_velocities);

    for (int i=0; i<num_constraints; i++) {
      delete constraint_array[i];
    }
    delete[] constraint_array;
  }
  return x0;
}

RigidBodyPropellor::RigidBodyPropellor(RigidBodySystem *sys, TiXmlElement *node, std::string name) :
        sys(sys), name(name),
        scale_factor_thrust(1.0), scale_factor_moment(1.0),
        lower_limit(-numeric_limits<double>::infinity()),
        upper_limit(numeric_limits<double>::infinity())
{
  auto tree = sys->getRigidBodyTree();

  TiXmlElement* parent_node = node->FirstChildElement("parent");
  if (!parent_node) throw runtime_error("propellor " + name + " is missing the parent node");
  frame = allocate_shared<RigidBodyFrame>(aligned_allocator<RigidBodyFrame>(),tree.get(),parent_node,node->FirstChildElement("origin"),name+"Frame");
  tree->addFrame(frame);

  axis << 1.0, 0.0, 0.0;
  TiXmlElement* axis_node = node->FirstChildElement("axis");
  if (axis_node) {
    parseVectorAttribute(axis_node, "xyz", axis);
    if (axis.norm()<1e-8) throw runtime_error("ERROR: axis is zero.  don't do that");
    axis.normalize();
  }

  parseScalarAttribute(node,"scale_factor_thrust",scale_factor_thrust);
  parseScalarAttribute(node,"scale_factor_moment",scale_factor_moment);
  parseScalarAttribute(node,"lower_limit",lower_limit);
  parseScalarAttribute(node,"upper_limit",upper_limit);

  // todo: parse visual info?
}

void parseForceElement(RigidBodySystem *sys, TiXmlElement* node) {
  string name = node->Attribute("name");

  if (TiXmlElement* propellor_node = node->FirstChildElement("propellor")) {
    sys->addForceElement(allocate_shared<RigidBodyPropellor>(Eigen::aligned_allocator<RigidBodyPropellor>(),sys,propellor_node,name));
  }
}

void parseRobot(RigidBodySystem *sys, TiXmlElement* node)
{
  if (!node->Attribute("name"))
    throw runtime_error("Error: your robot must have a name attribute");
  string robotname = node->Attribute("name");

  // parse force elements
  for (TiXmlElement* force_node = node->FirstChildElement("force_element"); force_node; force_node = force_node->NextSiblingElement("force_element"))
    parseForceElement(sys, force_node);
}

void parseURDF(RigidBodySystem *sys, TiXmlDocument *xml_doc)
{
  TiXmlElement *node = xml_doc->FirstChildElement("robot");
  if (!node) throw std::runtime_error("ERROR: This urdf does not contain a robot tag");

  parseRobot(sys, node);
}

void RigidBodySystem::addRobotFromURDFString(const string &xml_string, const string &root_dir, const DrakeJoint::FloatingBaseType floating_base_type)
{
  // first add the urdf to the rigid body tree
  tree->addRobotFromURDFString(xml_string, root_dir, floating_base_type);

  // now parse additional tags understood by rigid body system (actuators, sensors, etc)
  TiXmlDocument xml_doc;
  xml_doc.Parse(xml_string.c_str());
  parseURDF(this,&xml_doc);
}


void RigidBodySystem::addRobotFromURDF(const string &urdf_filename, const DrakeJoint::FloatingBaseType floating_base_type)
{
  // first add the urdf to the rigid body tree
  tree->addRobotFromURDF(urdf_filename, floating_base_type);

  // now parse additional tags understood by rigid body system (actuators, sensors, etc)
  TiXmlDocument xml_doc(urdf_filename);
  if (!xml_doc.LoadFile()) {
    throw std::runtime_error("failed to parse xml in file " + urdf_filename + "\n" + xml_doc.ErrorDesc());
  }
  parseURDF(this,&xml_doc);
}

