
#include <stdexcept>
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/solvers/Optimization.h"
#include "drake/systems/plants/constraint/RigidBodyConstraint.h"
#include "xmlUtil.h"
#include "spruce.hh"

using namespace std;
using namespace Eigen;
using namespace Drake;
using namespace tinyxml2;

size_t RigidBodySystem::getNumInputs(void) const {
  size_t num = tree->actuators.size();
  for (auto const& f : force_elements) {
    num += f->getNumInputs();
  }
  return num;
}

size_t RigidBodySystem::getNumOutputs() const {
    int n = getNumStates();
    for (const auto& s : sensors) {
      n += s->getNumOutputs();
    }
    return n;
}


void RigidBodySystem::addSensor(std::shared_ptr<RigidBodySensor> s) {
  if (s->isDirectFeedthrough()) {
    direct_feedthrough = true;
  }
  sensors.push_back(s);
}

RigidBodySystem::StateVector<double> RigidBodySystem::dynamics(
    const double& t, const RigidBodySystem::StateVector<double>& x,
    const RigidBodySystem::InputVector<double>& u) const {
  using namespace std;
  using namespace Eigen;
  eigen_aligned_unordered_map<const RigidBody*, Matrix<double, 6, 1> > f_ext;

  // todo: make kinematics cache once and re-use it (but have to make one per
  // type)
  auto nq = tree->num_positions;
  auto nv = tree->num_velocities;
  auto num_actuators = tree->actuators.size();
  auto q = x.topRows(nq);
  auto v = x.bottomRows(nv);
  auto kinsol = tree->doKinematics(q, v);

  std::cout << "RigidBodySystem::dynamics" << std::endl;
  PRINT_VAR(nq);
  PRINT_VAR(nv);
  PRINT_VAR(num_actuators);

  // todo: preallocate the optimization problem and constraints, and simply
  // update them then solve on each function eval.
  // happily, this clunkier version seems fast enough for now
  // the optimization framework should support this (though it has not been
  // tested thoroughly yet)
  OptimizationProblem prog;
  auto const& vdot = prog.addContinuousVariables(nv, "vdot");

  auto H = tree->massMatrix(kinsol);
  Eigen::MatrixXd H_and_neg_JT = H;

  PRINT_VAR(H.rows());
  PRINT_VAR(H.cols());

  VectorXd C = tree->dynamicsBiasTerm(kinsol, f_ext);
  if (num_actuators > 0) C -= tree->B * u.topRows(num_actuators);

  PRINT_VAR(C.rows());
  PRINT_VAR(C.cols());

  PRINT_VAR(tree->B.rows());
  PRINT_VAR(tree->B.cols());

  PRINT_VAR(force_elements.size());

  {  // loop through rigid body force elements

    // todo: distinguish between AppliedForce and ConstraintForce

    size_t u_index = 0;
    for (auto const& f : force_elements) {
      size_t num_inputs = f->getNumInputs();
      VectorXd force_input(u.middleRows(u_index, num_inputs));
      C -= f->output(t, force_input, kinsol);
      u_index += num_inputs;
    }
  }

  {  // apply joint limit forces
    for (auto const& b : tree->bodies) {
      if (!b->hasParent()) continue;
      auto const& joint = b->getJoint();
      if (joint.getNumPositions() == 1 &&
          joint.getNumVelocities() ==
              1) {  // taking advantage of only single-axis joints having joint
                    // limits makes things easier/faster here
        double qmin = joint.getJointLimitMin()(0),
               qmax = joint.getJointLimitMax()(0);
        // tau = k*(qlimit-q) - b(qdot)
        if (q(b->position_num_start) < qmin)
          C(b->velocity_num_start) -=
              penetration_stiffness * (qmin - q(b->position_num_start)) -
              penetration_damping * v(b->velocity_num_start);
        else if (q(b->position_num_start) > qmax)
          C(b->velocity_num_start) -=
              penetration_stiffness * (qmax - q(b->position_num_start)) -
              penetration_damping * v(b->velocity_num_start);
      }
    }
  }

  {  // apply contact forces
    VectorXd phi;
    Matrix3Xd normal, xA, xB;
    vector<int> bodyA_idx, bodyB_idx;
    if (use_multi_contact)
      tree->potentialCollisions(kinsol, phi, normal, xA, xB, bodyA_idx,
                                bodyB_idx);
    else
      tree->collisionDetect(kinsol, phi, normal, xA, xB, bodyA_idx, bodyB_idx);
    //normal is body B's normal and therefore pointing out from body B

    PRINT_VAR(phi.rows());

    if((phi.array()<0).any()) {
      PRINT_MSG("***************************************");
      PRINT_MSG("*** WE HAVE CONTACT!!");      
      PRINT_MSG("***************************************");
      PRINT_MSG("");
    }

    for (int i = 0; i < phi.rows(); i++) {
      if (phi(i) < 0.0) {  // then i have contact

        PRINT_VAR(tree->bodies[bodyA_idx[i]]->linkname);
        PRINT_VAR(tree->bodies[bodyB_idx[i]]->linkname);

        // todo: move this entire block to a shared an updated "contactJacobian"
        // method in RigidBodyTree
        auto JA = tree->transformPointsJacobian(kinsol, xA.col(i), bodyA_idx[i],
                                                0, false);
        auto JB = tree->transformPointsJacobian(kinsol, xB.col(i), bodyB_idx[i],
                                                0, false);
        Vector3d this_normal = normal.col(i);

        PRINT_VAR(i);
        PRINT_VAR(this_normal.transpose());

        // compute the surface tangent basis
        Vector3d tangent1;
        if (1.0 - this_normal(2) < EPSILON) {  // handle the unit-normal case
                                               // (since it's unit length, just
                                               // check z)
          tangent1 << 1.0, 0.0, 0.0;
        } else if (1 + this_normal(2) < EPSILON) {
          tangent1 << -1.0, 0.0, 0.0;  // same for the reflected case
        } else {                       // now the general case
          tangent1 << this_normal(1), -this_normal(0), 0.0;   //AMC-BUCHE: This is not normal to this_normal!!!
          tangent1 /= sqrt(this_normal(1) * this_normal(1) +
                           this_normal(0) * this_normal(0));
        }
        Vector3d tangent2 = tangent1.cross(this_normal); //AMC: Shouldn't this be the oposite? is not right-hand sided!!!
        Matrix3d R;  // rotation into normal coordinates
        R << tangent1, tangent2, this_normal;

        PRINT_VAR(tangent1.transpose());
        PRINT_VAR(tangent2.transpose());
        PRINT_MSG("R:");
        PRINT_MSG(R);

        auto J = R * (JA - JB);          // J = [ D1; D2; n ]
        auto relative_velocity = J * v;  // [ tangent1dot; tangent2dot; phidot ]

        {
          // spring law for normal force:  fA_normal = -k*phi - b*phidot
          // and damping for tangential force:  fA_tangent = -b*tangentdot
          // (bounded by the friction cone)
          Vector3d fA;
          fA(2) =
              std::max<double>(-penetration_stiffness * phi(i) -
                                   penetration_damping * relative_velocity(2),
                               0.0);
          fA.head(2) =
              -std::min<double>(
                  penetration_damping,
                  friction_coefficient * fA(2) /
                      (relative_velocity.head(2).norm() + EPSILON)) *
              relative_velocity.head(2);  // epsilon to avoid divide by zero

          // equal and opposite: fB = -fA.
          // tau = (R*JA)^T fA + (R*JB)^T fB = J^T fA
          C -= J.transpose() * fA;
        }
      }
    }
  }

  PRINT_VAR(tree->getNumPositionConstraints());

  if (tree->getNumPositionConstraints()) {
    int nc = tree->getNumPositionConstraints();
    const double alpha = 5.0;  // 1/time constant of position constraint
                               // satisfaction (see my latex rigid body notes)

    prog.addContinuousVariables(
        nc, "position constraint force");  // don't actually need to use the
                                           // decision variable reference that
                                           // would be returned...

    // then compute the constraint force
    auto phi = tree->positionConstraints(kinsol);
    auto J = tree->positionConstraintsJacobian(kinsol, false);
    auto Jdotv = tree->positionConstraintsJacDotTimesV(kinsol);

    // phiddot = -2 alpha phidot - alpha^2 phi  (0 + critically damped
    // stabilization term)
    prog.addLinearEqualityConstraint(
        J, -(Jdotv + 2 * alpha * J * v + alpha * alpha * phi), {vdot});
    H_and_neg_JT.conservativeResize(NoChange, H_and_neg_JT.cols() + J.rows());
    H_and_neg_JT.rightCols(J.rows()) = -J.transpose();
  }

  // add [H,-J^T]*[vdot;f] = -C
  prog.addLinearEqualityConstraint(H_and_neg_JT, -C);

  prog.solve();
  //      prog.printSolution();

  StateVector<double> dot(nq + nv);
  dot << kinsol.transformPositionDotMappingToVelocityMapping(
             Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(
                 nq, nq)) *
             v,
      vdot.value();
  return dot;
}

RigidBodySystem::OutputVector<double> RigidBodySystem::output(const double& t,
                                                              const RigidBodySystem::StateVector<double>& x,
                                                              const RigidBodySystem::InputVector<double>& u) const
{
  auto kinsol = tree->doKinematics(x.topRows(tree->num_positions), x.bottomRows(tree->num_velocities));
  Eigen::VectorXd y(getNumOutputs());

  assert(getNumStates() == x.size());
  assert(getNumInputs() == u.size());

  y.segment(0, getNumStates()) << x;
  int index=getNumStates();
  for (const auto& s : sensors) {
    y.segment(index, s->getNumOutputs()) = s->output(t, kinsol, u);
    index+=s->getNumOutputs();
  }
  return y;
}

// todo: move this to a more central location
class SingleTimeKinematicConstraintWrapper : public Constraint {
 public:
  SingleTimeKinematicConstraintWrapper(
      const shared_ptr<SingleTimeKinematicConstraint>& rigid_body_constraint)
      : Constraint(rigid_body_constraint->getNumConstraint(nullptr)),
        rigid_body_constraint(rigid_body_constraint),
        kinsol(rigid_body_constraint->getRobotPointer()->bodies) {
    rigid_body_constraint->bounds(nullptr, lower_bound, upper_bound);
  }
  virtual ~SingleTimeKinematicConstraintWrapper() {}

  virtual void eval(const Eigen::Ref<const Eigen::VectorXd>& q,
                    Eigen::VectorXd& y) const override {
    kinsol.initialize(q);
    rigid_body_constraint->getRobotPointer()->doKinematics(kinsol);
    MatrixXd dy;
    rigid_body_constraint->eval(nullptr, kinsol, y, dy);
  }
  virtual void eval(const Eigen::Ref<const TaylorVecXd>& tq,
                    TaylorVecXd& ty) const override {
    kinsol.initialize(autoDiffToValueMatrix(tq));
    rigid_body_constraint->getRobotPointer()->doKinematics(kinsol);
    VectorXd y;
    MatrixXd dy;
    rigid_body_constraint->eval(nullptr, kinsol, y, dy);
    initializeAutoDiffGivenGradientMatrix(
        y, (dy * autoDiffToGradientMatrix(tq)).eval(), ty);
  }

 private:
  shared_ptr<SingleTimeKinematicConstraint> rigid_body_constraint;
  mutable KinematicsCache<double> kinsol;
};

DRAKERBSYSTEM_EXPORT RigidBodySystem::StateVector<double>
Drake::getInitialState(const RigidBodySystem& sys) {
  VectorXd x0(sys.tree->num_positions + sys.tree->num_velocities);
  default_random_engine generator;
  x0 << sys.tree->getRandomConfiguration(generator),
      VectorXd::Random(sys.tree->num_velocities);

  // todo: implement joint limits, etc.

  if (sys.tree->getNumPositionConstraints()) {
    // todo: move this up to the system level?

    OptimizationProblem prog;
    std::vector<RigidBodyLoop, Eigen::aligned_allocator<RigidBodyLoop> > const&
        loops = sys.tree->loops;

    int nq = sys.tree->num_positions;
    auto qvar = prog.addContinuousVariables(nq);

    Matrix<double, 7, 1> bTbp = Matrix<double, 7, 1>::Zero();
    bTbp(3) = 1.0;
    Vector2d tspan;
    tspan << -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity();
    Vector3d zero = Vector3d::Zero();
    for (int i = 0; i < loops.size(); i++) {
      auto con1 = make_shared<RelativePositionConstraint>(
          sys.tree.get(), zero, zero, zero, loops[i].frameA->frame_index,
          loops[i].frameB->frame_index, bTbp, tspan);
      std::shared_ptr<SingleTimeKinematicConstraintWrapper> con1wrapper(
          new SingleTimeKinematicConstraintWrapper(con1));
      prog.addConstraint(con1wrapper, {qvar});
      auto con2 = make_shared<RelativePositionConstraint>(
          sys.tree.get(), loops[i].axis, loops[i].axis, loops[i].axis,
          loops[i].frameA->frame_index, loops[i].frameB->frame_index, bTbp,
          tspan);
      std::shared_ptr<SingleTimeKinematicConstraintWrapper> con2wrapper(
          new SingleTimeKinematicConstraintWrapper(con2));
      prog.addConstraint(con2wrapper, {qvar});
    }

    VectorXd q_guess = x0.topRows(nq);
    prog.addQuadraticCost(MatrixXd::Identity(nq, nq), q_guess);
    prog.solve();

    x0 << qvar.value(), VectorXd::Zero(sys.tree->num_velocities);
  }
  return x0;
}

RigidBodyPropellor::RigidBodyPropellor(RigidBodySystem &sys, XMLElement* node,
                                       const std::string& name)
    : RigidBodyForceElement(sys, name),
      scale_factor_thrust(1.0),
      scale_factor_moment(1.0),
      lower_limit(-numeric_limits<double>::infinity()),
      upper_limit(numeric_limits<double>::infinity()) {

  auto tree = sys.getRigidBodyTree();

  XMLElement* parent_node = node->FirstChildElement("parent");
  if (!parent_node)
    throw runtime_error("propellor " + name + " is missing the parent node");
  frame = allocate_shared<RigidBodyFrame>(
      aligned_allocator<RigidBodyFrame>(), tree.get(), parent_node,
      node->FirstChildElement("origin"), name + "Frame");
  tree->addFrame(frame);

  axis << 1.0, 0.0, 0.0;
  XMLElement* axis_node = node->FirstChildElement("axis");
  if (axis_node) {
    parseVectorAttribute(axis_node, "xyz", axis);
    if (axis.norm() < 1e-8)
      throw runtime_error("ERROR: axis is zero.  don't do that");
    axis.normalize();
  }

  parseScalarAttribute(node, "scale_factor_thrust", scale_factor_thrust);
  parseScalarAttribute(node, "scale_factor_moment", scale_factor_moment);
  parseScalarAttribute(node, "lower_limit", lower_limit);
  parseScalarAttribute(node, "upper_limit", upper_limit);

  // todo: parse visual info?
}

RigidBodySpringDamper::RigidBodySpringDamper(RigidBodySystem &sys,
                                             XMLElement* node,
                                             const std::string& name)
    : RigidBodyForceElement(sys, name),
      stiffness(0.0),
      damping(0.0),
      rest_length(0.0) {
  auto tree = sys.getRigidBodyTree();

  parseScalarAttribute(node, "rest_length", rest_length);
  parseScalarAttribute(node, "stiffness", stiffness);
  parseScalarAttribute(node, "damping", damping);

  XMLElement* link_ref_node = node->FirstChildElement("link1");
  if (!link_ref_node)
    throw runtime_error("linear_spring_damper " + name +
                        " is missing the link1 node");
  frameA = allocate_shared<RigidBodyFrame>(aligned_allocator<RigidBodyFrame>(),
                                           tree.get(), link_ref_node,
                                           link_ref_node, name + "FrameA");
  tree->addFrame(frameA);

  link_ref_node = node->FirstChildElement("link2");
  if (!link_ref_node)
    throw runtime_error("linear_spring_damper " + name +
                        " is missing the link2 node");
  frameB = allocate_shared<RigidBodyFrame>(aligned_allocator<RigidBodyFrame>(),
                                           tree.get(), link_ref_node,
                                           link_ref_node, name + "FrameB");
  tree->addFrame(frameB);
}

RigidBodyMagnetometer::RigidBodyMagnetometer(RigidBodySystem const& sys,
                                             const std::string& name,
                                             const std::shared_ptr<RigidBodyFrame> frame,
                                             double declination)
    : RigidBodySensor(sys, name), frame(frame) {
  setDeclination(declination);
}

RigidBodyAccelerometer::RigidBodyAccelerometer(RigidBodySystem const& sys,
                                               const std::string& name,
                                               const std::shared_ptr<RigidBodyFrame> frame)
  : RigidBodySensor(sys, name), frame(frame), gravity_compensation(false) {  }


Eigen::VectorXd RigidBodyAccelerometer::output(const double &t,
                                               const KinematicsCache<double> &rigid_body_state,
                                               const RigidBodySystem::InputVector<double>& u) const
{
  VectorXd x = rigid_body_state.getX();
  auto xdd = sys.dynamics(t, x, u);
  auto const& tree = sys.getRigidBodyTree();
  auto v_dot = xdd.bottomRows(rigid_body_state.getNumVelocities());
  Vector3d sensor_origin = Vector3d::Zero(); // assumes sensor coincides with the frame's origin;
  auto J = tree->transformPointsJacobian(rigid_body_state, sensor_origin, frame->frame_index, 0, false);
  auto Jdot_times_v = tree->transformPointsJacobianDotTimesV(rigid_body_state, sensor_origin, frame->frame_index, 0);
  
  Vector4d quat_world_to_body = tree->relativeQuaternion(rigid_body_state, 0, frame->frame_index);
  Vector3d accel_base = Jdot_times_v + J * v_dot;
  Vector3d accel_body = quatRotateVec(quat_world_to_body, accel_base);

  if(gravity_compensation) {
    Vector3d gravity(0, 0, 9.81);
    accel_body += quatRotateVec(quat_world_to_body, gravity);
  }
  
  return noise_model ? noise_model->generateNoise(accel_body) : accel_body;
}


RigidBodyGyroscope::RigidBodyGyroscope(RigidBodySystem const& sys, const std::string& name, const std::shared_ptr<RigidBodyFrame> frame)
    : RigidBodySensor(sys, name), frame(frame)
{ }

Eigen::VectorXd RigidBodyMagnetometer::output(const double &t,
                                               const KinematicsCache<double> &rigid_body_state,
                                               const RigidBodySystem::InputVector<double>& u) const {
  auto const& tree = sys.getRigidBodyTree();

  Vector4d quat_world_to_body = tree->relativeQuaternion(rigid_body_state, 0, frame->frame_index);

  Vector3d mag_body = quatRotateVec(quat_world_to_body, magnetic_north);
  
  return noise_model ? noise_model->generateNoise(mag_body) : mag_body;
}

Eigen::VectorXd RigidBodyGyroscope::output(const double &t,
                                           const KinematicsCache<double> &rigid_body_state,
                                           const RigidBodySystem::InputVector<double>& u) const {
  // relative twist of body with respect to world expressed in body
  auto const& tree = sys.getRigidBodyTree();
  auto relative_twist = tree->relativeTwist(rigid_body_state, 0, frame->frame_index, frame->frame_index);
  Eigen::Vector3d angular_rates = relative_twist.head<3>();

  return noise_model ? noise_model->generateNoise(angular_rates) : angular_rates;
}

RigidBodyDepthSensor::RigidBodyDepthSensor(RigidBodySystem const& sys,
                                           const std::string& name,
                                           const std::shared_ptr<RigidBodyFrame> frame,
                                           std::size_t samples, double min_angle, double max_angle, double range)
    : RigidBodySensor(sys, name), frame(frame), min_pitch(0.0), max_pitch(0.0),
      min_yaw(min_angle), max_yaw(max_angle), num_pixel_rows(1), num_pixel_cols(samples), min_range(0.0), max_range(range) {
  cacheRaycastEndpoints();
}

RigidBodyDepthSensor::RigidBodyDepthSensor(
    RigidBodySystem const& sys, const std::string& name,
    std::shared_ptr<RigidBodyFrame> frame, tinyxml2::XMLElement* node)
    : RigidBodySensor(sys, name),
      frame(frame),
      min_pitch(0.0),
      max_pitch(0.0),
      min_yaw(0.0),
      max_yaw(0.0),
      num_pixel_rows(1),
      num_pixel_cols(1),
      min_range(0.0),
      max_range(10.0) {

  string type(node->Attribute("type"));

  if (type.compare("ray") == 0) {
    XMLElement* ray_node = node->FirstChildElement("ray");
    if (!ray_node)
      throw std::runtime_error("ray sensor does not have a ray element");
    XMLElement* scan_node = ray_node->FirstChildElement("scan");
    if (!scan_node)
      throw std::runtime_error("ray element does not have a scan element");

    XMLElement* horizontal_node = scan_node->FirstChildElement("horizontal");
    if (horizontal_node) {  // it's required by the xml spec, but don't actually
                            // require it here
      parseScalarValue(horizontal_node, "samples", num_pixel_cols);
      parseScalarValue(horizontal_node, "min_angle", min_yaw);
      parseScalarValue(horizontal_node, "max_angle", max_yaw);
      double resolution;
      parseScalarValue(horizontal_node, "resolution", resolution);
      num_pixel_cols = static_cast<int>(resolution * num_pixel_cols);
    }

    XMLElement* vertical_node = scan_node->FirstChildElement("vertical");
    if (vertical_node) {
      parseScalarValue(vertical_node, "samples", num_pixel_rows);
      parseScalarValue(vertical_node, "min_angle", min_pitch);
      parseScalarValue(vertical_node, "max_angle", max_pitch);
      double resolution;
      parseScalarValue(vertical_node, "resolution", resolution);
      num_pixel_rows = static_cast<int>(resolution * num_pixel_rows);
    }

    XMLElement* range_node = ray_node->FirstChildElement("range");
    if (!range_node)
      throw std::runtime_error("ray sensor does not have a range element");
    parseScalarValue(range_node, "min", min_range);
    parseScalarValue(range_node, "max", max_range);
  }

  cacheRaycastEndpoints();
}

void RigidBodyDepthSensor::cacheRaycastEndpoints() {
  raycast_endpoints.resize(3, num_pixel_rows * num_pixel_cols);
  for (size_t i = 0; i < num_pixel_rows; i++) {
    double pitch =
        min_pitch +
        (num_pixel_rows > 1 ? static_cast<double>(i) / (num_pixel_rows - 1)
                            : 0.0) *
        (max_pitch - min_pitch);
    for (size_t j = 0; j < num_pixel_cols; j++) {
      double yaw =
          min_yaw +
          (num_pixel_cols > 1 ? static_cast<double>(j) / (num_pixel_cols - 1)
                              : 0.0) *
          (max_yaw - min_yaw);
      raycast_endpoints.col(num_pixel_cols * i + j) =
          max_range *
          Vector3d(
              cos(yaw) * cos(pitch), sin(yaw),
              -cos(yaw) *
              sin(pitch));  // rolled out from roty(pitch)*rotz(yaw)*[1;0;0]
    }
  }
}



Eigen::VectorXd RigidBodyDepthSensor::output(const double &t,
                                             const KinematicsCache<double> &rigid_body_state,
                                             const RigidBodySystem::InputVector<double>& u) const {
  const size_t num_distances = num_pixel_cols * num_pixel_rows;
  VectorXd distances(num_distances);

  Vector3d origin = sys.getRigidBodyTree()->transformPoints(rigid_body_state,
                                                             Vector3d::Zero(),
                                                             frame->frame_index,0);

  auto raycast_endpoints_world = sys.getRigidBodyTree()->transformPoints(rigid_body_state,
                                                                          raycast_endpoints,
                                                                          frame->frame_index,0);

  sys.getRigidBodyTree()->collisionRaycast(rigid_body_state, origin, raycast_endpoints_world, distances);

  for(size_t i = 0; i < num_distances; i++) {
    if(distances[i] < 0.0 || distances[i] > max_range) {
      distances[i] = max_range;
    } else if(distances[i] < min_range) {
      distances[i] = min_range;
    }
  }

  return distances;
}

void parseForceElement(RigidBodySystem &sys, XMLElement* node) {
  string name = node->Attribute("name");

  if (XMLElement* propellor_node = node->FirstChildElement("propellor")) {
    sys.addForceElement(allocate_shared<RigidBodyPropellor>(
        Eigen::aligned_allocator<RigidBodyPropellor>(), sys, propellor_node,
        name));
  } else if (XMLElement* spring_damper_node =
                 node->FirstChildElement("linear_spring_damper")) {
    sys.addForceElement(allocate_shared<RigidBodySpringDamper>(
        Eigen::aligned_allocator<RigidBodySpringDamper>(), sys,
        spring_damper_node, name));
  }
}

void parseRobot(RigidBodySystem &sys, XMLElement* node)
{
  if (!node->Attribute("name"))
    throw runtime_error("Error: your robot must have a name attribute");
  string robotname = node->Attribute("name");

  // parse force elements
  for (XMLElement* force_node = node->FirstChildElement("force_element");
       force_node; force_node = force_node->NextSiblingElement("force_element"))
    parseForceElement(sys, force_node);
}


void parseURDF(RigidBodySystem &sys, XMLDocument *xml_doc)
{
  XMLElement *node = xml_doc->FirstChildElement("robot");
  if (!node) throw std::runtime_error("ERROR: This urdf does not contain a robot tag");
  parseRobot(sys, node);
}

void parseSDFJoint(RigidBodySystem &sys, string model_name, XMLElement* node, PoseMap& pose_map)
{
  // todo: parse joint sensors
}

void parseSDFLink(RigidBodySystem &sys, string model_name, XMLElement* node, PoseMap& pose_map)
{
  const char* attr = node->Attribute("name");
  if (!attr) throw runtime_error("ERROR: link tag is missing name attribute");
  string link_name(attr);

  auto body = sys.getRigidBodyTree()->findLink(link_name, model_name);

  Isometry3d transform_link_to_model = Isometry3d::Identity();
  XMLElement* pose = node->FirstChildElement("pose");
  if (pose) {
    poseValueToTransform(pose, pose_map, transform_link_to_model);
    pose_map.insert(
        std::pair<string, Isometry3d>(body->linkname, transform_link_to_model));
  }

  for (XMLElement* elnode = node->FirstChildElement("sensor"); elnode;
       elnode = elnode->NextSiblingElement("sensor")) {
    attr = elnode->Attribute("name");
    if (!attr)
      throw runtime_error("ERROR: sensor tag is missing name attribute");
    string sensor_name(attr);

    attr = elnode->Attribute("type");
    if (!attr)
      throw runtime_error("ERROR: sensor tag is missing type attribute");
    string type(attr);

    Isometry3d transform_sensor_to_model = transform_link_to_model;
    XMLElement* pose = elnode->FirstChildElement("pose");
    if (pose) {
      poseValueToTransform(pose, pose_map, transform_sensor_to_model,
                           transform_link_to_model);
    }

    if (type.compare("ray") == 0) {
      auto frame = allocate_shared<RigidBodyFrame>(
          Eigen::aligned_allocator<RigidBodyFrame>(), sensor_name, body,
          transform_link_to_model.inverse() * transform_sensor_to_model);
      sys.getRigidBodyTree()->addFrame(frame);
      sys.addSensor(allocate_shared<RigidBodyDepthSensor>(
          Eigen::aligned_allocator<RigidBodyDepthSensor>(), sys, sensor_name,
          frame, elnode));
    }
  }
}


void parseSDFModel(RigidBodySystem &sys, XMLElement* node)
{
  PoseMap pose_map;  // because sdf specifies almost everything in the global (actually model) coordinates instead of relative coordinates.  sigh...

  if (!node->Attribute("name"))
    throw runtime_error("Error: your model must have a name attribute");
  string model_name = node->Attribute("name");

  for (XMLElement* elnode = node->FirstChildElement("link"); elnode;
       elnode = elnode->NextSiblingElement("link"))
    parseSDFLink(sys, model_name, elnode, pose_map);

  for (XMLElement* elnode = node->FirstChildElement("joint"); elnode;
       elnode = elnode->NextSiblingElement("joint"))
    parseSDFJoint(sys, model_name, elnode, pose_map);
}


void parseSDF(RigidBodySystem &sys, XMLDocument *xml_doc)
{
  XMLElement *node = xml_doc->FirstChildElement("sdf");
  if (!node) throw std::runtime_error("ERROR: This xml file does not contain an sdf tag");

  for (XMLElement* elnode = node->FirstChildElement("model"); elnode;
       elnode = node->NextSiblingElement("model"))
    parseSDFModel(sys, elnode);
}

void RigidBodySystem::addRobotFromURDFString(
    const string& xml_string, const string& root_dir,
    const DrakeJoint::FloatingBaseType floating_base_type) {
  // first add the urdf to the rigid body tree
  tree->addRobotFromURDFString(xml_string, root_dir, floating_base_type);

  // now parse additional tags understood by rigid body system (actuators,
  // sensors, etc)
  XMLDocument xml_doc;
  xml_doc.Parse(xml_string.c_str());

  parseURDF(*this, &xml_doc);
}

void RigidBodySystem::addRobotFromURDF(
    const string& urdf_filename,
    const DrakeJoint::FloatingBaseType floating_base_type) {
  // first add the urdf to the rigid body tree
  tree->addRobotFromURDF(urdf_filename, floating_base_type);

  // now parse additional tags understood by rigid body system (actuators,
  // sensors, etc)
  XMLDocument xml_doc;
  xml_doc.LoadFile(urdf_filename.data());
  if (xml_doc.ErrorID() != XML_SUCCESS) {
    throw std::runtime_error("failed to parse xml in file " + urdf_filename +
                             "\n" + xml_doc.ErrorName());
  }
  parseURDF(*this, &xml_doc);
}

void RigidBodySystem::addRobotFromSDF(
    const string& sdf_filename,
    const DrakeJoint::FloatingBaseType floating_base_type) {
  tree->addRobotFromSDF(sdf_filename, floating_base_type);

  // now parse additional tags understood by rigid body system (actuators,
  // sensors, etc)

  XMLDocument xml_doc;
  xml_doc.LoadFile(sdf_filename.data());
  if (xml_doc.ErrorID() != XML_SUCCESS) {
    throw std::runtime_error("failed to parse xml in file " + sdf_filename +
                             "\n" + xml_doc.ErrorName());
  }
  parseSDF(*this, &xml_doc);
}

void RigidBodySystem::addRobotFromFile(
    const std::string& filename,
    const DrakeJoint::FloatingBaseType floating_base_type) {
  spruce::path p(filename);
  auto ext = p.extension();

  std::transform(ext.begin(), ext.end(), ext.begin(),
                 ::tolower);  // convert to lower case

  if (ext.compare(".urdf") == 0) {
    addRobotFromURDF(filename, floating_base_type);
  } else if (ext.compare(".sdf") == 0) {
    addRobotFromSDF(filename, floating_base_type);
  } else {
    throw runtime_error("unknown file extension: " + ext);
  }
}

