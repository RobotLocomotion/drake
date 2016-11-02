#include "drake/systems/plants/RigidBodySystem.h"

#include <algorithm>
#include <limits>
#include <list>
#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/math/quaternion.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/systems/plants/ConstraintWrappers.h"
#include "drake/systems/plants/constraint/RigidBodyConstraint.h"
#include "drake/systems/plants/joints/floating_base_types.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/parser_sdf.h"
#include "drake/systems/plants/parser_urdf.h"
#include "drake/systems/plants/pose_map.h"
#include "spruce.hh"
#include "xmlUtil.h"

using Eigen::Isometry3d;
using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::Matrix3Xd;

using std::allocate_shared;
using std::default_random_engine;
using std::make_shared;
using std::numeric_limits;
using std::runtime_error;
using std::shared_ptr;
using std::string;

using drake::math::quatRotateVec;
using drake::parsers::ModelInstanceIdTable;
using drake::systems::plants::joints::FloatingBaseType;

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;
using tinyxml2::XML_SUCCESS;

namespace drake {

size_t RigidBodySystem::getNumStates() const {
  return tree->get_num_positions() + tree->get_num_velocities();
}

// TODO(sam.creasey) This whole file should be in this namespace.
using drake::systems::plants::SingleTimeKinematicConstraintWrapper;
using drake::systems::plants::KinematicsCacheHelper;

size_t RigidBodySystem::getNumInputs(void) const {
  size_t num = tree->actuators.size();
  for (auto const& f : force_elements) {
    num += f->getNumInputs();
  }
  return num;
}

size_t RigidBodySystem::getNumOutputs() const {
  size_t n = getNumStates();
  for (const auto& s : sensors) {
    n += s->getNumOutputs();
  }
  return n;
}

int RigidBodySystem::get_num_positions() const {
  return tree->get_num_positions();
}

// TODO(liang.fok) Remove this deprecated method prior to release 1.0.
#ifndef SWIG
  DRAKE_DEPRECATED("Please use get_num_positions().")
#endif
int RigidBodySystem::number_of_positions() const {
  return get_num_positions();
}

int RigidBodySystem::get_num_velocities() const {
  return tree->get_num_velocities();
}

// TODO(liang.fok) Remove this deprecated method prior to release 1.0.
#ifndef SWIG
  DRAKE_DEPRECATED("Please use get_num_velocities().")
#endif
int RigidBodySystem::number_of_velocities() const {
  return get_num_velocities();
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
  using namespace std;  // NOLINT(build/namespaces)
  using namespace Eigen;  // NOLINT(build/namespaces)

  // todo: make kinematics cache once and re-use it (but have to make one per
  // type)
  auto nq = tree->get_num_positions();
  auto nv = tree->get_num_velocities();
  auto num_actuators = tree->actuators.size();
  auto q = x.topRows(nq);
  auto v = x.bottomRows(nv);
  auto kinsol = tree->doKinematics(q, v);

  // todo: preallocate the optimization problem and constraints, and simply
  // update them then solve on each function eval.
  // happily, this clunkier version seems fast enough for now
  // the optimization framework should support this (though it has not been
  // tested thoroughly yet)
  drake::solvers::MathematicalProgram prog;
  auto const& vdot = prog.AddContinuousVariables(nv, "vdot");

  auto H = tree->massMatrix(kinsol);
  Eigen::MatrixXd H_and_neg_JT = H;

  const RigidBodyTree::BodyToWrenchMap<double> no_external_wrenches;
  VectorXd C = tree->dynamicsBiasTerm(kinsol, no_external_wrenches);
  if (num_actuators > 0) C -= tree->B * u.topRows(num_actuators);

  // loop through rigid body force elements
  {
    // todo: distinguish between AppliedForce and ConstraintForce

    size_t u_index = 0;
    for (auto const& f : force_elements) {
      size_t num_inputs = f->getNumInputs();
      VectorXd force_input(u.middleRows(u_index, num_inputs));
      C -= f->output(t, force_input, kinsol);
      u_index += num_inputs;
    }
  }

  // apply joint limit forces
  {
    for (auto const& b : tree->bodies) {
      if (!b->has_parent_body()) continue;
      auto const& joint = b->getJoint();
      if (joint.get_num_positions() == 1 &&
          joint.get_num_velocities() ==
              1) {  // taking advantage of only single-axis joints having joint
                    // limits makes things easier/faster here
        double qmin = joint.getJointLimitMin()(0),
               qmax = joint.getJointLimitMax()(0);
        // tau = k*(qlimit-q) - b(qdot)
        if (q(b->get_position_start_index()) < qmin)
          C(b->get_velocity_start_index()) -=
              penetration_stiffness * (qmin - q(b->get_position_start_index()))
              - penetration_damping * v(b->get_velocity_start_index());
        else if (q(b->get_position_start_index()) > qmax)
          C(b->get_velocity_start_index()) -=
              penetration_stiffness * (qmax - q(b->get_position_start_index()))
              - penetration_damping * v(b->get_velocity_start_index());
      }
    }
  }

  // apply contact forces
  {
    VectorXd phi;
    Matrix3Xd normal, xA, xB;
    vector<int> bodyA_idx, bodyB_idx;
    if (use_multi_contact)
      tree->potentialCollisions(kinsol, phi, normal, xA, xB, bodyA_idx,
                                bodyB_idx);
    else
      tree->collisionDetect(kinsol, phi, normal, xA, xB, bodyA_idx, bodyB_idx);

    for (int i = 0; i < phi.rows(); i++) {
      if (phi(i) < 0.0) {  // then i have contact
        // todo: move this entire block to a shared an updated "contactJacobian"
        // method in RigidBodyTree
        auto JA = tree->transformPointsJacobian(kinsol, xA.col(i), bodyA_idx[i],
                                                0, false);
        auto JB = tree->transformPointsJacobian(kinsol, xB.col(i), bodyB_idx[i],
                                                0, false);
        Vector3d this_normal = normal.col(i);

        // compute the surface tangent basis
        Vector3d tangent1;
        if (1.0 - this_normal(2) < EPSILON) {  // handle the unit-normal case
                                               // (since it's unit length, just
                                               // check z)
          tangent1 << 1.0, 0.0, 0.0;
        } else if (1 + this_normal(2) < EPSILON) {
          tangent1 << -1.0, 0.0, 0.0;  // same for the reflected case
        } else {                       // now the general case
          tangent1 << this_normal(1), -this_normal(0), 0.0;
          tangent1 /= sqrt(this_normal(1) * this_normal(1) +
                           this_normal(0) * this_normal(0));
        }
        Vector3d tangent2 = this_normal.cross(tangent1);
        Matrix3d R;  // rotation into normal coordinates
        R.row(0) = tangent1;
        R.row(1) = tangent2;
        R.row(2) = this_normal;
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

  if (tree->getNumPositionConstraints()) {
    size_t nc = tree->getNumPositionConstraints();
    const double alpha = 5.0;  // 1/time constant of position constraint
                               // satisfaction (see my latex rigid body notes)

    prog.AddContinuousVariables(
        nc, "position constraint force");  // don't actually need to use the
                                           // decision variable reference that
                                           // would be returned...

    // then compute the constraint force
    auto phi = tree->positionConstraints(kinsol);
    auto J = tree->positionConstraintsJacobian(kinsol, false);
    auto Jdotv = tree->positionConstraintsJacDotTimesV(kinsol);

    // phiddot = -2 alpha phidot - alpha^2 phi  (0 + critically damped
    // stabilization term)
    prog.AddLinearEqualityConstraint(
        J, -(Jdotv + 2 * alpha * J * v + alpha * alpha * phi), {vdot});
    H_and_neg_JT.conservativeResize(NoChange, H_and_neg_JT.cols() + J.rows());
    H_and_neg_JT.rightCols(J.rows()) = -J.transpose();
  }

  // add [H,-J^T]*[vdot;f] = -C
  prog.AddLinearEqualityConstraint(H_and_neg_JT, -C);

  prog.Solve();
  //      prog.PrintSolution();

  StateVector<double> dot(nq + nv);
  dot << kinsol.transformPositionDotMappingToVelocityMapping(
             Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(
                 nq, nq)) *
             v,
      vdot.value();
  return dot;
}

RigidBodySystem::OutputVector<double> RigidBodySystem::output(
    const double& t, const RigidBodySystem::StateVector<double>& x,
    const RigidBodySystem::InputVector<double>& u) const {
  auto kinsol = tree->doKinematics(x.topRows(tree->get_num_positions()),
                                   x.bottomRows(tree->get_num_velocities()));
  Eigen::VectorXd y(getNumOutputs());

  DRAKE_ASSERT(static_cast<int>(getNumStates()) == x.size());
  DRAKE_ASSERT(static_cast<int>(getNumInputs()) == u.size());

  y.segment(0, getNumStates()) << x;
  size_t index = getNumStates();
  for (const auto& s : sensors) {
    y.segment(index, s->getNumOutputs()) = s->output(t, kinsol, u);
    index += s->getNumOutputs();
  }
  return y;
}

std::vector<const RigidBodySensor*> RigidBodySystem::GetSensors() const {
  std::vector<const RigidBodySensor*> result;
  for (size_t ii = 0; ii < sensors.size(); ii++) {
    result.push_back(sensors[ii].get());
  }
  return result;
}

DRAKE_EXPORT RigidBodySystem::StateVector<double> getInitialState(
    const RigidBodySystem& sys) {
  VectorXd x0(sys.tree->get_num_positions() +
              sys.tree->get_num_velocities());

  default_random_engine generator;
  x0 << sys.tree->getRandomConfiguration(generator),
      VectorXd::Random(sys.tree->get_num_velocities());

  // todo: implement joint limits, etc.

  if (sys.tree->getNumPositionConstraints()) {
    // todo: move this up to the system level?

    drake::solvers::MathematicalProgram prog;
    std::vector<RigidBodyLoop, Eigen::aligned_allocator<RigidBodyLoop>> const&
        loops = sys.tree->loops;

    int nq = sys.tree->get_num_positions();
    auto qvar = prog.AddContinuousVariables(nq);

    Matrix<double, 7, 1> bTbp = Matrix<double, 7, 1>::Zero();
    bTbp(3) = 1.0;
    Vector2d tspan;
    tspan << -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity();
    Vector3d zero = Vector3d::Zero();
    KinematicsCacheHelper<double> kin_helper(sys.tree->bodies);
    std::list<RelativePositionConstraint> constraints;
    for (const auto& loop : loops) {
      constraints.push_back(RelativePositionConstraint(
          sys.tree.get(), zero, zero, zero, loop.frameA_->get_frame_index(),
          loop.frameB_->get_frame_index(), bTbp, tspan));
      std::shared_ptr<SingleTimeKinematicConstraintWrapper> con1wrapper(
          new SingleTimeKinematicConstraintWrapper(&constraints.back(),
                                                   &kin_helper));
      prog.AddConstraint(con1wrapper, {qvar});
      constraints.push_back(RelativePositionConstraint(
          sys.tree.get(), loop.axis_, loop.axis_, loop.axis_,
          loop.frameA_->get_frame_index(), loop.frameB_->get_frame_index(),
          bTbp, tspan));
      std::shared_ptr<SingleTimeKinematicConstraintWrapper> con2wrapper(
          new SingleTimeKinematicConstraintWrapper(&constraints.back(),
                                                   &kin_helper));
      prog.AddConstraint(con2wrapper, {qvar});
    }

    VectorXd q_guess = x0.topRows(nq);
    prog.AddQuadraticCost(MatrixXd::Identity(nq, nq), q_guess);
    prog.Solve();

    x0 << qvar.value(), VectorXd::Zero(sys.tree->get_num_velocities());
  }
  return x0;
}

RigidBodyPropellor::RigidBodyPropellor(RigidBodySystem& sys, XMLElement* node,
                                       const std::string& name,
                                       int model_instance_id)
    : RigidBodyForceElement(sys, name, model_instance_id),
      scale_factor_thrust(1.0),
      scale_factor_moment(1.0),
      lower_limit(-numeric_limits<double>::infinity()),
      upper_limit(numeric_limits<double>::infinity()) {
  auto tree = sys.getRigidBodyTree();

  XMLElement* parent_node = node->FirstChildElement("parent");
  if (!parent_node)
    throw runtime_error("propellor " + name + " is missing the parent node");
  frame = drake::parsers::urdf::MakeRigidBodyFrameFromUrdfNode(
      *tree, *parent_node, node->FirstChildElement("origin"), name + "Frame",
      model_instance_id);
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

RigidBodySpringDamper::RigidBodySpringDamper(RigidBodySystem& sys,
                                             XMLElement* node,
                                             const std::string& name,
                                             int model_instance_id)
    : RigidBodyForceElement(sys, name, model_instance_id),
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
  frameA = drake::parsers::urdf::MakeRigidBodyFrameFromUrdfNode(
      *tree, *link_ref_node, link_ref_node, name + "FrameA",
      model_instance_id);
  tree->addFrame(frameA);

  link_ref_node = node->FirstChildElement("link2");
  if (!link_ref_node)
    throw runtime_error("linear_spring_damper " + name +
                        " is missing the link2 node");
  frameB = drake::parsers::urdf::MakeRigidBodyFrameFromUrdfNode(
      *tree, *link_ref_node, link_ref_node, name + "FrameB",
      model_instance_id);
  tree->addFrame(frameB);
}

const std::string& RigidBodySensor::get_model_name() const {
  return frame_->get_rigid_body().get_model_name();
}

const RigidBodyFrame& RigidBodySensor::get_frame() const {
  return *frame_.get();
}

/// Returns the rigid body system to which this sensor attaches.
const RigidBodySystem& RigidBodySensor::get_rigid_body_system() const {
  return sys_;
}

RigidBodyMagnetometer::RigidBodyMagnetometer(
    RigidBodySystem const& sys, const std::string& name,
    const std::shared_ptr<RigidBodyFrame> frame, double declination)
    : RigidBodySensor(sys, name, frame) {
  setDeclination(declination);
}

RigidBodyAccelerometer::RigidBodyAccelerometer(
    RigidBodySystem const& sys, const std::string& name,
    const std::shared_ptr<RigidBodyFrame> frame)
    : RigidBodySensor(sys, name, frame),
      gravity_compensation(false) {}

Eigen::VectorXd RigidBodyAccelerometer::output(
    const double& t, const KinematicsCache<double>& rigid_body_state,
    const RigidBodySystem::InputVector<double>& u) const {
  VectorXd x = rigid_body_state.getX();
  auto xdd = get_rigid_body_system().dynamics(t, x, u);
  auto const& tree = get_rigid_body_system().getRigidBodyTree();
  auto v_dot = xdd.bottomRows(rigid_body_state.get_num_velocities());

  Vector3d sensor_origin =
      Vector3d::Zero();  // assumes sensor coincides with the frame's origin;
  auto J = tree->transformPointsJacobian(rigid_body_state, sensor_origin,
                                         get_frame().get_frame_index(), 0,
                                         false);
  auto Jdot_times_v = tree->transformPointsJacobianDotTimesV(
      rigid_body_state, sensor_origin, get_frame().get_frame_index(), 0);

  Vector4d quat_world_to_body =
      tree->relativeQuaternion(rigid_body_state, 0,
          get_frame().get_frame_index());

  Vector3d accel_base = Jdot_times_v + J * v_dot;
  Vector3d accel_body = quatRotateVec(quat_world_to_body, accel_base);

  if (gravity_compensation) {
    Vector3d gravity(0, 0, 9.81);
    accel_body += quatRotateVec(quat_world_to_body, gravity);
  }

  return noise_model ? noise_model->generateNoise(accel_body) : accel_body;
}

RigidBodyGyroscope::RigidBodyGyroscope(
    RigidBodySystem const& sys, const std::string& name,
    const std::shared_ptr<RigidBodyFrame> frame)
    : RigidBodySensor(sys, name, frame) {}

Eigen::VectorXd RigidBodyMagnetometer::output(
    const double& t, const KinematicsCache<double>& rigid_body_state,
    const RigidBodySystem::InputVector<double>& u) const {
  auto const& tree = get_rigid_body_system().getRigidBodyTree();

  Vector4d quat_world_to_body =
      tree->relativeQuaternion(rigid_body_state, 0,
          get_frame().get_frame_index());

  Vector3d mag_body = quatRotateVec(quat_world_to_body, magnetic_north);

  return noise_model ? noise_model->generateNoise(mag_body) : mag_body;
}

Eigen::VectorXd RigidBodyGyroscope::output(
    const double& t, const KinematicsCache<double>& rigid_body_state,
    const RigidBodySystem::InputVector<double>& u) const {
  // relative twist of body with respect to world expressed in body
  auto const& tree = get_rigid_body_system().getRigidBodyTree();
  auto relative_twist = tree->relativeTwist(
      rigid_body_state, 0, get_frame().get_frame_index(),
          get_frame().get_frame_index());
  Eigen::Vector3d angular_rates = relative_twist.head<3>();

  return noise_model ? noise_model->generateNoise(angular_rates)
                     : angular_rates;
}

RigidBodyDepthSensor::RigidBodyDepthSensor(
    RigidBodySystem const& sys, const std::string& name,
    const std::shared_ptr<RigidBodyFrame> frame, std::size_t samples,
    double min_angle, double max_angle, double range)
    : RigidBodySensor(sys, name, frame),
      min_yaw_(min_angle),
      max_yaw_(max_angle),
      num_pixel_cols_(samples),
      max_range_(range) {
  CheckValidConfiguration();
  cacheRaycastEndpoints();
}

RigidBodyDepthSensor::RigidBodyDepthSensor(
    RigidBodySystem const& sys, const std::string& name,
    std::shared_ptr<RigidBodyFrame> frame, tinyxml2::XMLElement* node)
    : RigidBodySensor(sys, name, frame) {
  string type(node->Attribute("type"));

  if (type.compare("ray") == 0) {
    XMLElement* ray_node = node->FirstChildElement("ray");
    if (!ray_node)
      throw std::runtime_error("Ray sensor does not have a ray element");
    XMLElement* scan_node = ray_node->FirstChildElement("scan");
    if (!scan_node)
      throw std::runtime_error("Ray element does not have a scan element");

    XMLElement* horizontal_node = scan_node->FirstChildElement("horizontal");
    if (horizontal_node) {  // it's required by the xml spec, but don't actually
                            // require it here
      parseScalarValue(horizontal_node, "samples", num_pixel_cols_);
      parseScalarValue(horizontal_node, "min_angle", min_yaw_);
      parseScalarValue(horizontal_node, "max_angle", max_yaw_);
      double resolution;
      parseScalarValue(horizontal_node, "resolution", resolution);
      num_pixel_cols_ = static_cast<int>(resolution * num_pixel_cols_);
    }

    XMLElement* vertical_node = scan_node->FirstChildElement("vertical");
    if (vertical_node) {
      parseScalarValue(vertical_node, "samples", num_pixel_rows_);
      parseScalarValue(vertical_node, "min_angle", min_pitch_);
      parseScalarValue(vertical_node, "max_angle", max_pitch_);
      double resolution;
      parseScalarValue(vertical_node, "resolution", resolution);
      num_pixel_rows_ = static_cast<int>(resolution * num_pixel_rows_);
    }

    XMLElement* range_node = ray_node->FirstChildElement("range");
    if (!range_node)
      throw std::runtime_error("ray sensor does not have a range element");
    parseScalarValue(range_node, "min", min_range_);
    parseScalarValue(range_node, "max", max_range_);
  }

  CheckValidConfiguration();
  cacheRaycastEndpoints();
}

void RigidBodyDepthSensor::CheckValidConfiguration() {
  // Verifies that the minimum pitch is less than or equal to the maximum pitch.
  if (min_pitch_ > max_pitch_) {
    std::stringstream error_msg;
    error_msg
        << "ERROR: RigidBodyDepthSensor: min pitch is greater than max pitch!"
        << std::endl
        << "  - sensor name: " << get_name() << std::endl
        << "  - model name: " << get_model_name() << std::endl
        << "  - min pitch: " << min_pitch_ << std::endl
        << "  - max pitch: " << max_pitch_ << std::endl
        << std::endl;
    throw std::runtime_error(error_msg.str());
  }

  // Verifies that the minimum yaw is less than or equal to the maximum yaw.
  if (min_yaw_ > max_yaw_) {
    std::stringstream error_msg;
    error_msg << "ERROR: RigidBodyDepthSensor: min yaw is greater than max yaw!"
              << std::endl
              << "  - sensor name: " << get_name() << std::endl
              << "  - model name: " << get_model_name() << std::endl
              << "  - min yaw: " << min_yaw_ << std::endl
              << "  - max yaw: " << max_yaw_ << std::endl
              << std::endl;
    throw std::runtime_error(error_msg.str());
  }

  // Verifies that if the min or max pitch is non-zero that there is a non-zero
  // number of pixels per row.
  if (((min_pitch_ != 0) || (max_pitch_ != 0)) && (num_pixel_rows_ == 0)) {
    std::stringstream error_msg;
    error_msg << "ERROR: RigidBodyDepthSensor: Configuration problem. "
                 "Contradiction between min/max pitch and number of pixels per "
                 "row."
              << std::endl
              << "  - sensor name: " << get_name() << std::endl
              << "  - model name: " << get_model_name() << std::endl
              << "  - min pitch: " << min_pitch_ << std::endl
              << "  - max pitch: " << max_pitch_ << std::endl
              << "  - number of pixels per row: " << num_pixel_rows_
              << std::endl;
    throw std::runtime_error(error_msg.str());
  }

  // Verifies that if the min or max yaw is non-zero that there is a non-zero
  // number of pixels per column.
  if (((min_yaw_ != 0) || (max_yaw_ != 0)) && (num_pixel_cols_ == 0)) {
    std::stringstream error_msg;
    error_msg << "ERROR: RigidBodyDepthSensor: Configuration problem. "
                 "Contradiction between min/max yaw and number of pixels per "
                 "column."
              << std::endl
              << "  - sensor name: " << get_name() << std::endl
              << "  - model name: " << get_model_name() << std::endl
              << "  - min yaw: " << min_yaw_ << std::endl
              << "  - max yaw: " << max_yaw_ << std::endl
              << "  - number of pixels per row: " << num_pixel_cols_
              << std::endl;
    throw std::runtime_error(error_msg.str());
  }
}

void RigidBodyDepthSensor::cacheRaycastEndpoints() {
  raycast_endpoints.resize(3, num_pixel_rows_ * num_pixel_cols_);
  for (size_t i = 0; i < num_pixel_rows_; i++) {
    double pitch =
        min_pitch_ +
        (num_pixel_rows_ > 1 ? static_cast<double>(i) / (num_pixel_rows_ - 1)
                             : 0.0) *
            (max_pitch_ - min_pitch_);
    for (size_t j = 0; j < num_pixel_cols_; j++) {
      double yaw =
          min_yaw_ +
          (num_pixel_cols_ > 1 ? static_cast<double>(j) / (num_pixel_cols_ - 1)
                               : 0.0) *
              (max_yaw_ - min_yaw_);
      raycast_endpoints.col(num_pixel_cols_ * i + j) =
          max_range_ *
          Vector3d(
              cos(yaw) * cos(pitch), sin(yaw),
              -cos(yaw) *
                  sin(pitch));  // rolled out from roty(pitch)*rotz(yaw)*[1;0;0]
    }
  }
}

Eigen::VectorXd RigidBodyDepthSensor::output(
    const double& t, const KinematicsCache<double>& rigid_body_state,
    const RigidBodySystem::InputVector<double>& u) const {
  const size_t num_distances = num_pixel_cols_ * num_pixel_rows_;
  VectorXd distances(num_distances);

  // Computes the origin of the rays in the world frame. The origin of
  // of the rays in the frame of the sensor is [0,0,0] (the Vector3d::Zero()).
  Vector3d origin = get_rigid_body_system().getRigidBodyTree()->transformPoints(
      rigid_body_state, Vector3d::Zero(), get_frame().get_frame_index(), 0);

  // Computes the end of the casted rays in the world frame.
  Matrix3Xd raycast_endpoints_world =
      get_rigid_body_system().getRigidBodyTree()->transformPoints(
          rigid_body_state, raycast_endpoints, get_frame().get_frame_index(),
              0);

  get_rigid_body_system().getRigidBodyTree()->collisionRaycast(
      rigid_body_state, origin, raycast_endpoints_world, distances);

  for (size_t i = 0; i < num_distances; i++) {
    if (distances[i] < 0.0 || distances[i] > max_range_) {
      distances[i] = max_range_;
    } else if (distances[i] < min_range_) {
      distances[i] = min_range_;
    }
  }

  return distances;
}

size_t RigidBodyDepthSensor::getNumOutputs() const {
  return num_pixel_rows_ * num_pixel_cols_;
}

bool RigidBodyDepthSensor::is_horizontal_scanner() const {
  return num_pixel_cols_ > 1;
}

bool RigidBodyDepthSensor::is_vertical_scanner() const {
  return num_pixel_rows_ > 1;
}

size_t RigidBodyDepthSensor::num_pixel_rows() const { return num_pixel_rows_; }

size_t RigidBodyDepthSensor::num_pixel_cols() const { return num_pixel_cols_; }

double RigidBodyDepthSensor::min_pitch() const { return min_pitch_; }

double RigidBodyDepthSensor::max_pitch() const { return max_pitch_; }

double RigidBodyDepthSensor::min_yaw() const { return min_yaw_; }

double RigidBodyDepthSensor::max_yaw() const { return max_yaw_; }

double RigidBodyDepthSensor::min_range() const { return min_range_; }

double RigidBodyDepthSensor::max_range() const { return max_range_; }

// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
void ParseUrdfForceElement(RigidBodySystem& sys, XMLElement* node,
    int model_instance_id) {
  string name = node->Attribute("name");

  if (XMLElement* propellor_node = node->FirstChildElement("propellor")) {
    sys.addForceElement(allocate_shared<RigidBodyPropellor>(
        Eigen::aligned_allocator<RigidBodyPropellor>(), sys, propellor_node,
        name, model_instance_id));
  } else if (XMLElement* spring_damper_node =
                 node->FirstChildElement("linear_spring_damper")) {
    sys.addForceElement(allocate_shared<RigidBodySpringDamper>(
        Eigen::aligned_allocator<RigidBodySpringDamper>(), sys,
        spring_damper_node, name, model_instance_id));
  }
}

// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
void ParseUrdfModel(RigidBodySystem& sys, XMLElement* node,
    const ModelInstanceIdTable& model_instance_id_table) {

  if (!node->Attribute("name"))
    throw runtime_error("Error: the model must have a name attribute");
  string model_name = node->Attribute("name");

  // Obtains the model instance ID. Throws an exception if the model instance ID
  // cannot be determined.
  if (model_instance_id_table.find(model_name) ==
      model_instance_id_table.end()) {
    throw std::runtime_error("Model named \"" + model_name + "\" does not "
        "exist in the model_instance_id_table.");
  }
  int model_instance_id = model_instance_id_table.at(model_name);

  // Parses the force elements in the model.
  for (XMLElement* force_node = node->FirstChildElement("force_element");
       force_node; force_node = force_node->NextSiblingElement("force_element"))
    ParseUrdfForceElement(sys, force_node, model_instance_id);
}

// Parses a single model from a URDF specification. Adds it to @p sys.
//
// @param[out] sys The `RigidBodySystem` to which the modeling elements should
// be added.
//
// @param[in] xml_doc The XML document containing the URDF specification.
//
// @param[out] model_instance_id_table A reference to a map storing model
// names and their instance IDs.
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
void ParseUrdf(RigidBodySystem& sys, XMLDocument* xml_doc,
    const ModelInstanceIdTable& model_instance_id_table) {
  XMLElement* node = xml_doc->FirstChildElement("robot");
  if (!node)
    throw std::runtime_error("ERROR: This urdf does not contain a robot tag");
  ParseUrdfModel(sys, node, model_instance_id_table);
}

// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
void parseSdfJoint(RigidBodySystem& sys, int model_instance_id,
                   // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                   XMLElement* node, PoseMap& pose_map) {
  // Obtains the name of the joint.
  const char* attr = node->Attribute("name");
  if (!attr) throw runtime_error("ERROR: joint tag is missing name attribute");
  string joint_name(attr);

  if (node->FirstChildElement("sensor") != nullptr) {
    // TODO(liangfok): Add support for sensors on joints.
    throw runtime_error(
        "Error: Joint sensors are not supported yet."
        "Unable to parse sensor on joint " +
        joint_name + ".");
  }
}

// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
void parseSdfLink(RigidBodySystem& sys, int model_instance_id, XMLElement* node,
                  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                  PoseMap& pose_map) {
  // Obtains the name of the body.
  const char* attr = node->Attribute("name");
  if (!attr) throw runtime_error("ERROR: link tag is missing name attribute");
  string body_name(attr);

  // Obtains the corresponding body from the rigid body tree.
  auto body = sys.getRigidBodyTree()->FindBody(body_name, "",
      model_instance_id);

  // Obtains the transform from the link to the model.
  Isometry3d transform_link_to_model = Isometry3d::Identity();
  XMLElement* link_pose = node->FirstChildElement("pose");
  if (link_pose) {
    poseValueToTransform(link_pose, pose_map, transform_link_to_model);
    pose_map.insert(std::pair<string, Isometry3d>(body->get_name(),
                                                  transform_link_to_model));
  }

  // Processes each sensor element within the link.
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
    XMLElement* sensor_pose = elnode->FirstChildElement("pose");
    if (sensor_pose) {
      poseValueToTransform(sensor_pose, pose_map, transform_sensor_to_model,
                           transform_link_to_model);
    }

    if (type == "ray") {
      auto frame = allocate_shared<RigidBodyFrame>(
          Eigen::aligned_allocator<RigidBodyFrame>(), sensor_name, body,
          transform_link_to_model.inverse() * transform_sensor_to_model);
      sys.getRigidBodyTree()->addFrame(frame);
      sys.addSensor(allocate_shared<RigidBodyDepthSensor>(
          Eigen::aligned_allocator<RigidBodyDepthSensor>(), sys, sensor_name,
          frame, elnode));
    } else {
      throw std::runtime_error(
          "ERROR: Drake C++ currently does not support sensors of type " +
          type + ".");
    }
  }
}

// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
void parseSdfModel(RigidBodySystem& sys, XMLElement* node,
    const ModelInstanceIdTable& model_instance_id_table) {
  // A pose map is necessary since SDF specifies almost everything in the
  // global coordinate frame. The pose map contains transforms from a link's
  // coordinate frame to the model's coordinate frame.
  PoseMap pose_map;

  // Obtains the name of the model.
  if (!node->Attribute("name"))
    throw runtime_error("Error: your model must have a name attribute");
  std::string model_name = node->Attribute("name");

  // Obtains the model instance ID. Throws an exception if the model instance ID
  // cannot be determined.
  if (model_instance_id_table.find(model_name) ==
      model_instance_id_table.end()) {
    throw std::runtime_error("Model named \"" + model_name + "\" does not "
        "exist in the model_instance_id_table.");
  }
  int model_instance_id = model_instance_id_table.at(model_name);

  // Parses each link element within the model.
  for (XMLElement* elnode = node->FirstChildElement("link"); elnode;
       elnode = elnode->NextSiblingElement("link"))
    parseSdfLink(sys, model_instance_id, elnode, pose_map);

  // Parses each joint element within the model.
  for (XMLElement* elnode = node->FirstChildElement("joint"); elnode;
       elnode = elnode->NextSiblingElement("joint"))
    parseSdfJoint(sys, model_instance_id, elnode, pose_map);
}

// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
void parseSdf(RigidBodySystem& sys, XMLDocument* xml_doc,
    const ModelInstanceIdTable& model_instance_id_table) {
  XMLElement* node = xml_doc->FirstChildElement("sdf");

  if (!node) {
    throw std::runtime_error(
        "ERROR: This xml file does not contain an sdf tag");
  }

  // Parses each model in the SDF. This includes parsing and instantiating
  // simulated sensors as specified by the SDF description.
  for (XMLElement* model_node = node->FirstChildElement("model"); model_node;
       model_node = model_node->NextSiblingElement("model")) {
    parseSdfModel(sys, model_node, model_instance_id_table);
  }
}

ModelInstanceIdTable RigidBodySystem::AddModelInstanceFromUrdfString(
    const string& urdf_string, const string& root_dir,
    const FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame) {

  // Adds the URDF to the RigidBodyTree.
  ModelInstanceIdTable model_instance_id_table =
      drake::parsers::urdf::AddModelInstanceFromUrdfString(urdf_string,
          root_dir, floating_base_type, weld_to_frame, tree.get());

  // Parses the additional tags understood by the RigidBodySystem. These include
  // actuators, sensors, etc.
  XMLDocument xml_doc;
  xml_doc.Parse(urdf_string.c_str());

  ParseUrdf(*this, &xml_doc, model_instance_id_table);

  return model_instance_id_table;
}

ModelInstanceIdTable RigidBodySystem::AddModelInstanceFromUrdfFile(
    const string& filename,
    const FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame) {

  // Adds the URDF to the rigid body tree.
  ModelInstanceIdTable model_instance_id_table =
      drake::parsers::urdf::AddModelInstanceFromUrdfFile(
          filename, floating_base_type, weld_to_frame, tree.get());

  // Parses additional tags understood by rigid body system (e.g., actuators,
  // sensors, etc).
  XMLDocument xml_doc;
  xml_doc.LoadFile(filename.data());
  if (xml_doc.ErrorID() != XML_SUCCESS) {
    throw std::runtime_error(
        "RigidBodySystem::AddModelInstanceFromUrdfFile: ERROR: Failed to parse "
        "xml in file " + filename + "\n" + xml_doc.ErrorName());
  }

  ParseUrdf(*this, &xml_doc, model_instance_id_table);

  return model_instance_id_table;
}

ModelInstanceIdTable RigidBodySystem::AddModelInstancesFromSdfFile(
    const string& filename,
    const FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame) {

  // Adds the robot to the rigid body tree.
  ModelInstanceIdTable model_instance_id_table =
      drake::parsers::sdf::AddModelInstancesFromSdfFile(
          filename, floating_base_type, weld_to_frame, tree.get());

  // Parses the additional SDF elements that are understood by RigidBodySystem,
  // namely (actuators, sensors, etc.).
  XMLDocument xml_doc;
  xml_doc.LoadFile(filename.data());
  if (xml_doc.ErrorID() != XML_SUCCESS) {
    throw std::runtime_error(
        "RigidBodySystem::AddModelInstancesFromSdfFile: ERROR: Failed to parse"
        "xml in file " + filename + "\n" + xml_doc.ErrorName());
  }

  parseSdf(*this, &xml_doc, model_instance_id_table);

  return model_instance_id_table;
}

ModelInstanceIdTable RigidBodySystem::AddModelInstancesFromSdfString(
    const string& sdf_string,
    const FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame) {

  // Adds the robot to the rigid body tree.
  ModelInstanceIdTable model_instance_id_table =
      drake::parsers::sdf::AddModelInstancesFromSdfString(
          sdf_string, floating_base_type, weld_to_frame, tree.get());

  // Parses the additional SDF elements that are understood by RigidBodySystem,
  // namely (actuators, sensors, etc.).
  XMLDocument xml_doc;
  xml_doc.Parse(sdf_string.c_str());
  if (xml_doc.ErrorID() != XML_SUCCESS) {
    throw std::runtime_error(
        "RigidBodySystem::AddModelInstancesFromSdfString: ERROR: Failed "
        " to parse XML in SDF string: " +
            std::string(xml_doc.ErrorName()));
  }
  parseSdf(*this, &xml_doc, model_instance_id_table);

  return model_instance_id_table;
}

ModelInstanceIdTable RigidBodySystem::AddModelInstanceFromFile(
    const std::string& filename,
    const FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame) {
  spruce::path p(filename);
  auto ext = p.extension();

  // Converts the file extension to be lower case.
  std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

  ModelInstanceIdTable model_instance_id_table;

  if (ext == ".urdf") {
    model_instance_id_table = AddModelInstanceFromUrdfFile(filename,
        floating_base_type, weld_to_frame);
  } else if (ext == ".sdf") {
    model_instance_id_table = AddModelInstancesFromSdfFile(filename,
        floating_base_type, weld_to_frame);
  } else {
    throw runtime_error(
        "RigidBodySystem::AddModelInstanceFromFile: ERROR: Unknown file "
        "extension: " + ext);
  }

  return model_instance_id_table;
}

ModelInstanceIdTable RigidBodySystem::AddModelInstancesFromString(
    const std::string& string_description,
    const FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame) {

  // Parse the string using an XML parser.
  XMLDocument xml_doc;
  xml_doc.Parse(string_description.c_str());
  if (xml_doc.ErrorID()) {
    throw std::runtime_error("Failed to parse XML string: " +
        std::string(xml_doc.ErrorName()));
  }

  if (xml_doc.FirstChildElement("sdf") != nullptr) {
    return AddModelInstancesFromSdfString(
        string_description, floating_base_type, weld_to_frame);
  } else {
    // Assume that it is a URDF file.
    const std::string root_dir = ".";
    return AddModelInstanceFromUrdfString(
        string_description, root_dir, floating_base_type, weld_to_frame);
  }
}

Eigen::VectorXd spatialForceInFrameToJointTorque(
    const RigidBodyTree* tree, const KinematicsCache<double>& rigid_body_state,
    const RigidBodyFrame* frame, const Eigen::Matrix<double, 6, 1>& wrench) {
  auto T_frame_to_world =
      tree->relativeTransform(rigid_body_state, 0, frame->get_frame_index());
  auto force_in_world = transformSpatialForce(T_frame_to_world, wrench);
  std::vector<int> v_indices;
  auto J = tree->geometricJacobian(rigid_body_state, 0,
                                   frame->get_frame_index(), 0,
                                   false, &v_indices);
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(tree->get_num_velocities());
  for (size_t i = 0; i < v_indices.size(); i++) {
    tau(v_indices[i]) = J.col(i).dot(force_in_world);
  }
  return tau;
}

}  // namespace drake
