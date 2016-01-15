
#include <stdexcept>
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/RigidBodyIK.h"  // required for resolving initial conditions
#include "urdfParsingUtil.h"

using namespace std;
using namespace Eigen;
using namespace Drake;
using namespace tinyxml2;

size_t RigidBodySystem::getNumInputs(void) const {
  size_t num = tree->actuators.size();
  for (auto const & f : force_elements) { num += f->getNumInputs(); }
  return num;
}

RigidBodySystem::StateVector<double> RigidBodySystem::dynamics(const double& t, const RigidBodySystem::StateVector<double>& x, const RigidBodySystem::InputVector<double>& u) const {
  using namespace std;
  using namespace Eigen;
  eigen_aligned_unordered_map<const RigidBody *, Matrix<double, 6, 1> > f_ext;

  // todo: make kinematics cache once and re-use it (but have to make one per type)
  auto nq = tree->num_positions;
  auto nv = tree->num_velocities;
  auto num_actuators = tree->actuators.size();
  auto q = x.topRows(nq);
  auto v = x.bottomRows(nv);
  auto kinsol = tree->doKinematics(q,v);

  // todo: preallocate the optimization problem and constraints, and simply update them then solve on each function eval.
  // happily, this clunkier version seems fast enough for now
  // the optimization framework should support this (though it has not been tested thoroughly yet)
  OptimizationProblem prog;
  auto const & vdot = prog.addContinuousVariables(nv,"vdot");

  auto H = tree->massMatrix(kinsol);
  Eigen::MatrixXd H_and_neg_JT = H;

  VectorXd C = tree->dynamicsBiasTerm(kinsol,f_ext);
  if (num_actuators > 0) C -= tree->B*u.topRows(num_actuators);

  { // loop through rigid body force elements

    // todo: distinguish between AppliedForce and ConstraintForce

    size_t u_index = 0;
    for (auto const & f : force_elements) {
      size_t num_inputs = f->getNumInputs();
      VectorXd force_input(u.middleRows(u_index,num_inputs));
      C -= f->output(t,force_input,kinsol);
      u_index += num_inputs;
    }
  }

  { // apply joint limit forces
    for (auto const& b : tree->bodies) {
      if (!b->hasParent()) continue;
      auto const& joint = b->getJoint();
      if (joint.getNumPositions()==1 && joint.getNumVelocities()==1) {  // taking advantage of only single-axis joints having joint limits makes things easier/faster here
        double qmin = joint.getJointLimitMin()(0), qmax = joint.getJointLimitMax()(0);
        // tau = k*(qlimit-q) - b(qdot)
        if (q(b->position_num_start)<qmin)
          C(b->velocity_num_start) -= penetration_stiffness*(qmin-q(b->position_num_start)) - penetration_damping*v(b->velocity_num_start);
        else if (q(b->position_num_start)>qmax)
          C(b->velocity_num_start) -= penetration_stiffness*(qmax-q(b->position_num_start)) - penetration_damping*v(b->velocity_num_start);
      }
    }
  }

  { // apply contact forces
    VectorXd phi;
    Matrix3Xd normal, xA, xB;
    vector<int> bodyA_idx, bodyB_idx;
    if (use_multi_contact)
      tree->potentialCollisions(kinsol,phi,normal,xA,xB,bodyA_idx,bodyB_idx);
    else
      tree->collisionDetect(kinsol,phi,normal,xA,xB,bodyA_idx,bodyB_idx);

    for (int i=0; i<phi.rows(); i++) {
      if (phi(i)<0.0) { // then i have contact
        // todo: move this entire block to a shared an updated "contactJacobian" method in RigidBodyTree
        auto JA = tree->transformPointsJacobian(kinsol, xA.col(i), bodyA_idx[i], 0, false);
        auto JB = tree->transformPointsJacobian(kinsol, xB.col(i), bodyB_idx[i], 0, false);
        Vector3d this_normal = normal.col(i);

        // compute the surface tangent basis
        Vector3d tangent1;
        if (1.0 - this_normal(2) < EPSILON) { // handle the unit-normal case (since it's unit length, just check z)
          tangent1 << 1.0, 0.0, 0.0;
        } else if (1 + this_normal(2) < EPSILON) {
          tangent1 << -1.0, 0.0, 0.0;  //same for the reflected case
        } else {// now the general case
          tangent1 << this_normal(1), -this_normal(0), 0.0;
          tangent1 /= sqrt(this_normal(1)*this_normal(1) + this_normal(0)*this_normal(0));
        }
        Vector3d tangent2 = tangent1.cross(this_normal);
        Matrix3d R;  // rotation into normal coordinates
        R << tangent1, tangent2, this_normal;
        auto J = R*(JA-JB);  // J = [ D1; D2; n ]
        auto relative_velocity = J*v;  // [ tangent1dot; tangent2dot; phidot ]

        if (true) {
          // spring law for normal force:  fA_normal = -k*phi - b*phidot
          // and damping for tangential force:  fA_tangent = -b*tangentdot (bounded by the friction cone)
          Vector3d fA;
          fA(2) = -penetration_stiffness * phi(i) - penetration_damping * relative_velocity(2);
          fA.head(2) = -std::min(penetration_damping, friction_coefficient*fA(2)/(relative_velocity.head(2).norm()+EPSILON)) * relative_velocity.head(2);  // epsilon to avoid divide by zero

          // equal and opposite: fB = -fA.
          // tau = (R*JA)^T fA + (R*JB)^T fB = J^T fA
          C -= J.transpose()*fA;
        } else { // linear acceleration constraints (more expensive, but less tuning required for robot mass, etc)
          // note: this does not work for the multi-contact case (it overly constrains the motion of the link).  Perhaps if I made them inequality constraints...
          assert(!use_multi_contact && "The acceleration contact constraints do not play well with multi-contact");

          // phiddot = -2*alpha*phidot - alpha^2*phi   // critically damped response
          // tangential_velocity_dot = -2*alpha*tangential_velocity
          double alpha = 20;  // todo: put this somewhere better... or make them parameters?
          Vector3d desired_relative_acceleration = -2*alpha*relative_velocity;
          desired_relative_acceleration(2) += -alpha*alpha*phi(i);
          // relative_acceleration = J*vdot + R*(JAdotv - JBdotv) // uses the standard dnormal/dq = 0 assumption

          prog.addContinuousVariables(3,"contact normal force");
          auto JAdotv = tree->transformPointsJacobianDotTimesV(kinsol, xA.col(i).eval(), bodyA_idx[i], 0);
          auto JBdotv = tree->transformPointsJacobianDotTimesV(kinsol, xB.col(i).eval(), bodyB_idx[i], 0);

          prog.addLinearEqualityConstraint(J,desired_relative_acceleration - R*(JAdotv - JBdotv),{vdot});
          H_and_neg_JT.conservativeResize(NoChange,H_and_neg_JT.cols()+3);
          H_and_neg_JT.rightCols(3) = -J.transpose();
        }
      }
    }
  }

  if (tree->getNumPositionConstraints()) {
    int nc = tree->getNumPositionConstraints();
    const double alpha = 5.0;  // 1/time constant of position constraint satisfaction (see my latex rigid body notes)

    prog.addContinuousVariables(nc,"position constraint force");  // don't actually need to use the decision variable reference that would be returned...

    // then compute the constraint force
    auto phi = tree->positionConstraints(kinsol);
    auto J = tree->positionConstraintsJacobian(kinsol,false);
    auto Jdotv = tree->positionConstraintsJacDotTimesV(kinsol);

    // phiddot = -2 alpha phidot - alpha^2 phi  (0 + critically damped stabilization term)
    prog.addLinearEqualityConstraint(J,-(Jdotv + 2*alpha*J*v + alpha*alpha*phi),{vdot});
    H_and_neg_JT.conservativeResize(NoChange,H_and_neg_JT.cols()+J.rows());
    H_and_neg_JT.rightCols(J.rows()) = -J.transpose();
  }

  // add [H,-J^T]*[vdot;f] = -C
  prog.addLinearEqualityConstraint(H_and_neg_JT,-C);

  prog.solve();
  //      prog.printSolution();

  StateVector<double> dot(nq+nv);
  dot << kinsol.transformPositionDotMappingToVelocityMapping(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(nq, nq))*v, vdot.value();
  return dot;
}


DRAKERBSYSTEM_EXPORT RigidBodySystem::StateVector<double> Drake::getInitialState(const RigidBodySystem& sys) {

  VectorXd x0(sys.tree->num_positions + sys.tree->num_velocities);
  default_random_engine generator;
  x0 << sys.tree->getRandomConfiguration(generator), VectorXd::Random(sys.tree->num_velocities);

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

RigidBodyPropellor::RigidBodyPropellor(RigidBodySystem *sys, XMLElement *node, std::string name) :
        RigidBodyForceElement(sys,name),
        scale_factor_thrust(1.0), scale_factor_moment(1.0),
        lower_limit(-numeric_limits<double>::infinity()),
        upper_limit(numeric_limits<double>::infinity())
{
  auto tree = sys->getRigidBodyTree();

  XMLElement* parent_node = node->FirstChildElement("parent");
  if (!parent_node) throw runtime_error("propellor " + name + " is missing the parent node");
  frame = allocate_shared<RigidBodyFrame>(aligned_allocator<RigidBodyFrame>(),tree.get(),parent_node,node->FirstChildElement("origin"),name+"Frame");
  tree->addFrame(frame);

  axis << 1.0, 0.0, 0.0;
  XMLElement* axis_node = node->FirstChildElement("axis");
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

RigidBodySpringDamper::RigidBodySpringDamper(RigidBodySystem *sys, XMLElement *node, std::string name) :
        RigidBodyForceElement(sys,name),
        stiffness(0.0), damping(0.0), rest_length(0.0)
{
  auto tree = sys->getRigidBodyTree();

  parseScalarAttribute(node,"rest_length",rest_length);
  parseScalarAttribute(node,"stiffness",stiffness);
  parseScalarAttribute(node,"damping",damping);

  XMLElement* link_ref_node = node->FirstChildElement("link1");
  if (!link_ref_node) throw runtime_error("linear_spring_damper " + name + " is missing the link1 node");
  frameA = allocate_shared<RigidBodyFrame>(aligned_allocator<RigidBodyFrame>(),tree.get(),link_ref_node,link_ref_node,name+"FrameA");
  tree->addFrame(frameA);

  link_ref_node = node->FirstChildElement("link2");
  if (!link_ref_node) throw runtime_error("linear_spring_damper " + name + " is missing the link2 node");
  frameB = allocate_shared<RigidBodyFrame>(aligned_allocator<RigidBodyFrame>(),tree.get(),link_ref_node,link_ref_node,name+"FrameB");
  tree->addFrame(frameB);
}

void parseForceElement(RigidBodySystem *sys, XMLElement* node) {
  string name = node->Attribute("name");

  if (XMLElement* propellor_node = node->FirstChildElement("propellor")) {
    sys->addForceElement(allocate_shared<RigidBodyPropellor>(Eigen::aligned_allocator<RigidBodyPropellor>(),sys,propellor_node,name));
  } else if (XMLElement* spring_damper_node = node->FirstChildElement("linear_spring_damper")) {
    sys->addForceElement(allocate_shared<RigidBodySpringDamper>(Eigen::aligned_allocator<RigidBodySpringDamper>(),sys,spring_damper_node,name));
  }
}

void parseRobot(RigidBodySystem *sys, XMLElement* node)
{
  if (!node->Attribute("name"))
    throw runtime_error("Error: your robot must have a name attribute");
  string robotname = node->Attribute("name");

  // parse force elements
  for (XMLElement* force_node = node->FirstChildElement("force_element"); force_node; force_node = force_node->NextSiblingElement("force_element"))
    parseForceElement(sys, force_node);
}

void parseURDF(RigidBodySystem *sys, XMLDocument *xml_doc)
{
  XMLElement *node = xml_doc->FirstChildElement("robot");
  if (!node) throw std::runtime_error("ERROR: This urdf does not contain a robot tag");

  parseRobot(sys, node);
}

void RigidBodySystem::addRobotFromURDFString(const string &xml_string, const string &root_dir, const DrakeJoint::FloatingBaseType floating_base_type)
{
  // first add the urdf to the rigid body tree
  tree->addRobotFromURDFString(xml_string, root_dir, floating_base_type);

  // now parse additional tags understood by rigid body system (actuators, sensors, etc)
  XMLDocument xml_doc;
  xml_doc.Parse(xml_string.c_str());
  parseURDF(this,&xml_doc);
}


void RigidBodySystem::addRobotFromURDF(const string &urdf_filename, const DrakeJoint::FloatingBaseType floating_base_type)
{
  // first add the urdf to the rigid body tree
  tree->addRobotFromURDF(urdf_filename, floating_base_type);

  // now parse additional tags understood by rigid body system (actuators, sensors, etc)
  XMLDocument xml_doc;
  xml_doc.LoadFile(urdf_filename.data());
  if (xml_doc.ErrorID() != XML_SUCCESS) {
    throw std::runtime_error("failed to parse xml in file " + urdf_filename + "\n" + xml_doc.ErrorName());
  }
  parseURDF(this,&xml_doc);
}
