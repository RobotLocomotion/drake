#include "drake/systems/controllers/InstantaneousQPController.h"

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include <lcm/lcm-cpp.hpp>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/math/quaternion.h"
#include "drake/solvers/fast_qp.h"
#include "drake/systems/controllers/controlUtil.h"
#include "drake/systems/plants/parser_urdf.h"
#include "drake/util/lcmUtil.h"
#include "drake/util/yaml/yamlUtil.h"
#include "drake/lcmt_zmp_com_observer_state.hpp"

const double REG = 1e-8;

const bool CHECK_CENTROIDAL_MOMENTUM_RATE_MATCHES_TOTAL_WRENCH = false;
const bool PUBLISH_ZMP_COM_OBSERVER_STATE = true;

// TODO(jwnimmer-tri) Someone with gurobi has to fix this.
// NOLINTNEXTLINE(build/namespaces)
using namespace Eigen;

#define LEG_INTEGRATOR_DEACTIVATION_MARGIN 0.07

#define MU_VERY_SMALL 0.001

void InstantaneousQPController::initialize() {
  body_or_frame_name_to_id = computeBodyOrFrameNameToIdMap(*(this->robot));

  int nq = robot->get_num_positions();
  int nu = static_cast<int>(robot->actuators.size());

  umin.resize(nu);
  umax.resize(nu);
  for (size_t i = 0; i < robot->actuators.size(); i++) {
    umin(i) = robot->actuators.at(i).effort_limit_min_;
    umax(i) = robot->actuators.at(i).effort_limit_max_;
  }

  qdd_lb = Eigen::VectorXd::Zero(nq).array() -
           std::numeric_limits<double>::infinity();
  qdd_ub = Eigen::VectorXd::Zero(nq).array() +
           std::numeric_limits<double>::infinity();

  int error = GRBloadenv(&(env), NULL);
  if (error) {
    std::cerr << "Gurobi error code: " << error << std::endl;
    throw std::runtime_error("Cannot load gurobi environment");
  }

  CGE(GRBsetintparam(env, "outputflag", INSTQP_GUROBI_OUTPUTFLAG), env);
  CGE(GRBsetintparam(env, "method", INSTQP_GUROBI_METHOD), env);
  CGE(GRBsetintparam(env, "presolve", INSTQP_GUROBI_PRESOLVE), env);
  if (INSTQP_GUROBI_METHOD == 2) {
    CGE(GRBsetintparam(env, "bariterlimit", INSTQP_GUROBI_BARITERLIMIT), env);
    CGE(GRBsetintparam(env, "barhomogeneous", INSTQP_GUROBI_BARHOMOGENEOUS),
        env);
    CGE(GRBsetdblparam(env, "barconvtol", INSTQP_GUROBI_BARCONVTOL), env);
  }

  // preallocate some memory
  H.resize(nq, nq);
  H_float.resize(6, nq);
  H_act.resize(nu, nq);

  C.resize(nq);
  C_float.resize(6);
  C_act.resize(nu);

  J.resize(3, nq);
  J_xy.resize(2, nq);
  Hqp.resize(nq, nq);
  fqp.resize(nq);
  Ag.resize(6, nq);
  Ak.resize(3, nq);

  controller_state.vbasis_len = 0;
  controller_state.cbasis_len = 0;
  controller_state.vbasis = NULL;
  controller_state.cbasis = NULL;

  controller_state.t_prev = 0;
  controller_state.vref_integrator_state =
      Eigen::VectorXd::Zero(robot->get_num_velocities());
  controller_state.q_integrator_state =
      Eigen::VectorXd::Zero(robot->get_num_positions());
  controller_state.foot_contact_prev[0] = false;
  controller_state.foot_contact_prev[1] = false;
  controller_state.num_active_contact_pts = 0;

  controller_state.center_of_mass_observer_state = Eigen::Vector4d::Zero();
  controller_state.last_com_ddot = Eigen::Vector3d::Zero();
}

void InstantaneousQPController::loadConfigurationFromYAML(
    const std::string& control_config_filename) {
  YAML::Node control_config = YAML::LoadFile(control_config_filename);
  std::ofstream debug_file(control_config_filename + ".debug.yaml");
  // Important note: assigning the result of loadAllParamSets to a local
  // variable before assigning to the param_sets field is to make it so that the
  // copy assignment operator is used instead of the move assignment operator.
  // See #2165 for details.
  auto param_sets_local = loadAllParamSets(
      control_config["qp_controller_params"], *robot, debug_file);
  param_sets = param_sets_local;
  rpc = parseKinematicTreeMetadata(control_config["kinematic_tree_metadata"],
                                   *robot);
}

// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
void applyURDFModifications(std::unique_ptr<RigidBodyTree>& robot,
                            const KinematicModifications& modifications) {
  for (auto it = modifications.attachments.begin();
       it != modifications.attachments.end(); ++it) {
    std::shared_ptr<RigidBodyFrame> attach_to_frame =
        robot->findFrame(it->attach_to_frame);
    if (!attach_to_frame) {
      std::cerr << "frame name: " << it->attach_to_frame << std::endl;
      throw std::runtime_error(
          "Could not find attachment frame when handling urdf modifications");
    }
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        drake::GetDrakePath() + "/" + it->urdf_filename, it->joint_type,
        attach_to_frame, robot.get());
  }

  auto filter = [&](const std::string& group_name) {
    return modifications.collision_groups_to_keep.find(group_name) ==
           modifications.collision_groups_to_keep.end();
  };
  robot->removeCollisionGroupsIf(filter);
  robot->compile();
}

// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
void applyURDFModifications(std::unique_ptr<RigidBodyTree>& robot,
                            const std::string& urdf_modifications_filename) {
  KinematicModifications modifications =
      parseKinematicModifications(YAML::LoadFile(urdf_modifications_filename));
  applyURDFModifications(robot, modifications);
}

double logisticSigmoid(double L, double k, double x, double x0) {
  // Compute the value of the logistic sigmoid f(x) = L / (1 + exp(-k(x - x0)))
  return L / (1.0 + exp(-k * (x - x0)));
}

PIDOutput InstantaneousQPController::wholeBodyPID(
    double t, const Ref<const VectorXd>& q, const Ref<const VectorXd>& qd,
    const Ref<const VectorXd>& q_des, const WholeBodyParams& params) {
  // Run a PID controller on the whole-body state to produce desired
  // accelerations and reference posture
  PIDOutput out;
  double dt = 0;
  int nq = robot->get_num_positions();
  DRAKE_ASSERT(q.size() == nq);
  DRAKE_ASSERT(qd.size() == robot->get_num_velocities());
  DRAKE_ASSERT(q_des.size() == params.integrator.gains.size());
  if (nq != robot->get_num_velocities()) {
    throw std::runtime_error(
        "this function will need to be rewritten when num_pos != num_vel");
  }
  if (controller_state.t_prev != 0) {
    dt = t - controller_state.t_prev;
  }
  controller_state.q_integrator_state =
      (1 - params.integrator.eta) * controller_state.q_integrator_state +
      params.integrator.gains.cwiseProduct(q_des - q) * dt;
  controller_state.q_integrator_state =
      controller_state.q_integrator_state.array().max(
          -params.integrator.clamps.array());
  controller_state.q_integrator_state =
      controller_state.q_integrator_state.array().min(
          params.integrator.clamps.array());
  out.q_ref = q_des + controller_state.q_integrator_state;
  out.q_ref = out.q_ref.array().max(
      (robot->joint_limit_min - params.integrator.clamps).array());
  out.q_ref = out.q_ref.array().min(
      (robot->joint_limit_max + params.integrator.clamps).array());

  controller_state.q_integrator_state =
      controller_state.q_integrator_state.array().max(
          -params.integrator.clamps.array());
  controller_state.q_integrator_state =
      controller_state.q_integrator_state.array().min(
          params.integrator.clamps.array());

  VectorXd err_q;
  err_q.resize(nq);
  err_q.head<3>() = q_des.head<3>() - q.head<3>();
  for (int j = 3; j < nq; j++) {
    err_q(j) = angleDiff(q(j), q_des(j));
  }
  out.qddot_des = params.Kp.cwiseProduct(err_q) - params.Kd.cwiseProduct(qd);
  out.qddot_des = out.qddot_des.array().max(params.qdd_bounds.min.array());
  out.qddot_des = out.qddot_des.array().min(params.qdd_bounds.max.array());
  return out;
}

VectorXd InstantaneousQPController::velocityReference(
    double t, const Ref<const VectorXd>& q, const Ref<const VectorXd>& qd,
    const Ref<const VectorXd>& qdd, bool foot_contact[2],
    const VRefIntegratorParams& params) {
  // Integrate expected accelerations to determine a target feed-forward
  // velocity, which we can pass in to Atlas
  DRAKE_ASSERT(qdd.size() == robot->get_num_velocities());

  double dt = 0;
  if (controller_state.t_prev != 0) {
    dt = t - controller_state.t_prev;
  }

  VectorXd qdd_limited = qdd;
  // Do not wind the vref integrator up against the joint limits for the legs
  for (const int& pos_ind : rpc.position_indices.legs.at(Side::RIGHT)) {
    if (q(pos_ind) <=
        robot->joint_limit_min(pos_ind) + LEG_INTEGRATOR_DEACTIVATION_MARGIN) {
      qdd_limited(pos_ind) = std::max(qdd(pos_ind), 0.0);
    } else if (q(pos_ind) >= robot->joint_limit_max(pos_ind) -
                                 LEG_INTEGRATOR_DEACTIVATION_MARGIN) {
      qdd_limited(pos_ind) = std::min(qdd(pos_ind), 0.0);
    }
  }
  for (const int& pos_ind : rpc.position_indices.legs.at(Side::LEFT)) {
    if (q(pos_ind) <=
        robot->joint_limit_min(pos_ind) + LEG_INTEGRATOR_DEACTIVATION_MARGIN) {
      qdd_limited(pos_ind) = std::max(qdd(pos_ind), 0.0);
    } else if (q(pos_ind) >= robot->joint_limit_max(pos_ind) -
                                 LEG_INTEGRATOR_DEACTIVATION_MARGIN) {
      qdd_limited(pos_ind) = std::min(qdd(pos_ind), 0.0);
    }
  }

  controller_state.vref_integrator_state =
      (1 - params.eta) * controller_state.vref_integrator_state +
      params.eta * qd + qdd_limited * dt;

  if (params.zero_ankles_on_contact && foot_contact[0] == 1) {
    for (const int& pos_ind : rpc.position_indices.ankles.at(Side::LEFT)) {
      controller_state.vref_integrator_state(pos_ind) = 0;
    }
  }
  if (params.zero_ankles_on_contact && foot_contact[1] == 1) {
    for (const int& pos_ind : rpc.position_indices.ankles.at(Side::RIGHT)) {
      controller_state.vref_integrator_state(pos_ind) = 0;
    }
  }
  if (controller_state.foot_contact_prev[0] != foot_contact[0]) {
    // contact state changed, reset integrated velocities
    for (const int& pos_ind : rpc.position_indices.legs.at(Side::LEFT)) {
      controller_state.vref_integrator_state(pos_ind) = qd(pos_ind);
    }
  }
  if (controller_state.foot_contact_prev[1] != foot_contact[1]) {
    // contact state changed, reset integrated velocities
    for (const int& pos_ind : rpc.position_indices.legs.at(Side::RIGHT)) {
      controller_state.vref_integrator_state(pos_ind) = qd(pos_ind);
    }
  }

  controller_state.foot_contact_prev[0] = foot_contact[0];
  controller_state.foot_contact_prev[1] = foot_contact[1];

  VectorXd qd_err = controller_state.vref_integrator_state - qd;

  // do not velocity control ankles when in contact
  if (params.zero_ankles_on_contact && foot_contact[0] == 1) {
    for (const int& pos_ind : rpc.position_indices.ankles.at(Side::LEFT)) {
      qd_err(pos_ind) = 0;
    }
  }
  if (params.zero_ankles_on_contact && foot_contact[1] == 1) {
    for (const int& pos_ind : rpc.position_indices.ankles.at(Side::RIGHT)) {
      qd_err(pos_ind) = 0;
    }
  }

  VectorXd qd_ref = qd_err.array().max(-params.delta_max);
  qd_ref = qd_ref.array().min(params.delta_max);
  return qd_ref;
}

drake::eigen_aligned_std_vector<SupportStateElement>
InstantaneousQPController::loadAvailableSupports(
    const drake::lcmt_qp_controller_input& qp_input) {
  // Parse a qp_input LCM message to extract its available supports as a vector
  // of SupportStateElements
  drake::eigen_aligned_std_vector<SupportStateElement> available_supports;
  available_supports.resize(qp_input.num_support_data);
  for (int i = 0; i < qp_input.num_support_data; i++) {
    available_supports[i].body_idx =
        body_or_frame_name_to_id.at(qp_input.support_data[i].body_name);
    for (int j = 0; j < 4; j++) {
      available_supports[i].support_logic_map[j] =
          qp_input.support_data[i].support_logic_map[j];
      available_supports[i].support_surface[j] =
          qp_input.support_data[i].support_surface[j];
    }
    available_supports[i].contact_pts.resize(
        qp_input.support_data[i].num_contact_pts);
    for (int j = 0; j < qp_input.support_data[i].num_contact_pts; j++) {
      for (int k = 0; k < 3; k++) {
        available_supports[i].contact_pts[j][k] =
            qp_input.support_data[i].contact_pts[k][j];
      }
    }
  }
  return available_supports;
}

void addJointSoftLimits(
    const JointSoftLimitParams& params, const DrakeRobotState& robot_state,
    const VectorXd& q_des,
    const std::vector<SupportStateElement,
                      Eigen::aligned_allocator<SupportStateElement>>& supports,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    std::vector<drake::lcmt_joint_pd_override>& joint_pd_override) {
  Matrix<bool, Dynamic, 1> has_joint_override =
      Matrix<bool, Dynamic, 1>::Zero(q_des.size());
  for (auto it = joint_pd_override.begin(); it != joint_pd_override.end();
       ++it) {
    has_joint_override(it->position_ind - 1) = true;
  }
  for (int i = 0; i < params.lb.size(); i++) {
    if (!has_joint_override(i) && params.enabled(i)) {
      int disable_body_1idx = params.disable_when_body_in_support(i);
      if (disable_body_1idx == 0 ||
          !inSupport(supports, disable_body_1idx - 1)) {
        double w_lb = 0;
        double w_ub = 0;
        if (!std::isinf(params.lb(i))) {
          w_lb = logisticSigmoid(params.weight(i), params.k_logistic(i),
                                 params.lb(i), robot_state.q(i));
        }
        if (!std::isinf(params.ub(i))) {
          w_ub = logisticSigmoid(params.weight(i), params.k_logistic(i),
                                 robot_state.q(i), params.ub(i));
        }
        double weight = std::max(w_ub, w_lb);
        drake::lcmt_joint_pd_override override;
        override.position_ind = i + 1;
        override.qi_des = q_des(i);
        override.qdi_des = 0;
        override.kp = params.kp(i);
        override.kd = params.kd(i);
        override.weight = weight;
        joint_pd_override.push_back(override);
      }
    }
  }
}

void applyJointPDOverride(
    const std::vector<drake::lcmt_joint_pd_override>& joint_pd_override,
    const DrakeRobotState& robot_state,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    PIDOutput& pid_out, VectorXd& w_qdd) {
  for (std::vector<drake::lcmt_joint_pd_override>::const_iterator it =
           joint_pd_override.begin();
       it != joint_pd_override.end(); ++it) {
    int ind = it->position_ind - 1;
    double err_q = it->qi_des - robot_state.q(ind);
    double err_qd = it->qdi_des - robot_state.qd(ind);
    pid_out.qddot_des(ind) = it->kp * err_q + it->kd * err_qd;
    w_qdd(ind) = it->weight;
  }
}

double averageContactPointHeight(
    const RigidBodyTree& robot, const KinematicsCache<double>& cache,
    std::vector<SupportStateElement,
                // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                Eigen::aligned_allocator<SupportStateElement>>& active_supports,
    int nc) {
  Eigen::Matrix3Xd contact_positions_world(3, nc);
  int col = 0;
  for (auto support_it = active_supports.begin();
       support_it != active_supports.end(); ++support_it) {
    const SupportStateElement& support = *support_it;
    for (auto contact_position_it = support.contact_pts.begin();
         contact_position_it != support.contact_pts.end();
         ++contact_position_it) {
      Vector3d contact_point = contact_position_it->head<3>();  // copy, ah well
      contact_positions_world.col(col++) =
          robot.transformPoints(cache, contact_point, support.body_idx, 0);
    }
  }
  double average_contact_point_height = contact_positions_world.row(2).mean();
  return average_contact_point_height;
}

Vector2d computeCoP(
    const RigidBodyTree& robot, const KinematicsCache<double>& cache,
    const drake::eigen_aligned_std_map<Side, ForceTorqueMeasurement>&
        foot_force_torque_measurements,
    Vector3d point_on_contact_plane, Eigen::Vector3d normal) {
  std::vector<ForceTorqueMeasurement> force_torque_measurements;
  for (auto it = foot_force_torque_measurements.begin();
       it != foot_force_torque_measurements.end(); ++it) {
    force_torque_measurements.push_back(it->second);
  }
  std::pair<Eigen::Vector3d, double> cop_and_normal_torque =
      robot.resolveCenterOfPressure(cache, force_torque_measurements, normal,
                                    point_on_contact_plane);
  Vector2d zmp_from_force_sensors = cop_and_normal_torque.first.head<2>();
  return zmp_from_force_sensors;
}

void InstantaneousQPController::estimateCoMBasedOnMeasuredZMP(
    const QPControllerParams& params,
    drake::eigen_aligned_std_vector<SupportStateElement>& active_supports,
    int num_contact_points,
    const drake::eigen_aligned_std_map<Side, ForceTorqueMeasurement>&
        foot_force_torque_measurements,
    double dt, Vector3d& xcom, Vector3d& xcomdot) {
  /*
   * Derivation:
   * We have two sources of information for estimating the COM, one via the
   *robot state (IMU+leg odomentry$\rightarrow$ full dynamic model) and the
   *other via the instantaneous ground reaction forces (ZMP).  The ZMP informs
   *us about the instantaneous position of the COM, but not its velocity.  The
   *IMU is better for high-frequency components, so we will only use the
   *observation of COM velocity from the robot state, and end up with the
   *following model:
   * $$x = \begin{bmatrix} x_{com} \\ y_{com} \\ \dot{x}_{com} \\ \dot{y}_{com}
   *\end{bmatrix}, \quad u = \begin{bmatrix} \ddot{x}_{com} \\ \ddot{y}_{com}
   *\end{bmatrix},\quad y = \begin{bmatrix} x_{zmp} \\ y_{zmp} \\
   *\dot{x}_{comFRS} \\ \dot{y}_{comFRS} \end{bmatrix},$$
   * where $FRS$ stands for ``from robot state".
   * \begin{gather*}
   * \dot{x} = Ax + Bu, \quad y = Cx + Du \\
   * A = \begin{bmatrix} 0_{2 \times 2} & I_{2 \times 2} \\ 0_{2 \times 2} &
   *0_{2 \times 2} \end{bmatrix}, \quad B = \begin{bmatrix} 0_{2 \times 2} \\
   *I_{2 \times 2} \end{bmatrix} \\
   * C = I_{4 \times 4} \quad D = \begin{bmatrix} -\frac{z_{comFRS}}{g} I_{2
   *\times 2} \\ 0_{2 \times 2} \end{bmatrix}
   * \\
   * \dot{\hat{x}} = A\hat{x} + Bu + L(y - C\hat{x} - Du)
   * \end{gather*}
   * where $L$ is the observer gain matrix.  This will give a BIBO stable
   *observer so long as all eigenvalues of $(A-LC)$ are in the left-half plane.
   *
   * If we parameterize $L$ as a diagonal matrix then the eigenvalues of
   *$(A-LC)$ are simply the negative of those diagonal entries.  I recommend
   *trying diagonal entries $\begin{bmatrix} l_{zmp}, l_{zmp}, l_{com dot},
   *l_{com dot} \end{bmatrix}$, with $l_{comdot} \approx 2\sqrt{l_{zmp}}$ to get
   *a critically-damped response.
   */

  // assume flat ground at average of contact points for ZMP computation. TODO:
  // figure out what works best
  double average_contact_point_height = averageContactPointHeight(
      *robot, cache, active_supports, num_contact_points);
  Vector3d point_on_contact_plane;
  point_on_contact_plane << 0.0, 0.0, average_contact_point_height;
  Vector2d zmp_from_force_sensors =
      computeCoP(*robot, cache, foot_force_torque_measurements,
                 point_on_contact_plane, Vector3d::UnitZ().eval());
  if (controller_state.t_prev == 0) {
    controller_state.center_of_mass_observer_state.topRows<2>() =
        xcom.topRows<2>();
    controller_state.center_of_mass_observer_state.bottomRows<2>() =
        xcomdot.topRows<2>();
  }

  const auto& comdot_from_robot_state = xcomdot.topRows<2>();
  double com_height = xcom(2) - average_contact_point_height;
  double grav = -robot->a_grav(5);
  const auto& last_commanded_comddot = controller_state.last_com_ddot;
  const Matrix4d& L = params.center_of_mass_observer_gain;
  Vector4d& xhat = controller_state.center_of_mass_observer_state;

  // y_err = (y - C*xhat - D*u)
  Vector4d y_err;
  y_err << zmp_from_force_sensors - xhat.topRows<2>() +
               com_height * last_commanded_comddot.topRows(2) /
                   (last_commanded_comddot(2) + grav),
      comdot_from_robot_state - xhat.bottomRows<2>();

  // xhatdot = Axhat + Bu + L*y_err)
  Vector4d xhatdot = L * y_err;
  xhatdot.topRows<2>() += comdot_from_robot_state;  // xhat.bottomRows<2>();
  xhatdot.bottomRows<2>() += last_commanded_comddot.topRows(2);

  xhat.noalias() += dt * xhatdot;

  // overwrite the com position and velocity with the observer's estimates:
  xcom.topRows<2>() =
      controller_state.center_of_mass_observer_state.topRows<2>();
  // xcomdot.topRows<2>() =
  // controller_state.center_of_mass_observer_state.bottomRows<2>();

  if (PUBLISH_ZMP_COM_OBSERVER_STATE) {
    std::unique_ptr<lcm::LCM> lcm(new lcm::LCM);
    if (lcm->good()) {
      drake::lcmt_zmp_com_observer_state zmp_com_observer_state_msg;
      eigenVectorToCArray(
          controller_state.center_of_mass_observer_state.head<2>(),
          zmp_com_observer_state_msg.com);
      eigenVectorToCArray(
          controller_state.center_of_mass_observer_state.tail<2>(),
          zmp_com_observer_state_msg.comd);
      zmp_com_observer_state_msg.ground_plane_height =
          point_on_contact_plane(2);
      lcm->publish("ZMP_COM_OBSERVER_STATE", &zmp_com_observer_state_msg);
    }
  }
}

void checkCentroidalMomentumMatchesTotalWrench(
    const RigidBodyTree& robot,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    KinematicsCache<double>& cache,
    const VectorXd& qdd,
    const std::vector<SupportStateElement,
                      Eigen::aligned_allocator<SupportStateElement>>&
        active_supports,
    const MatrixXd& B, const VectorXd& beta) {
  std::map<int, Side> foot_body_index_to_side;
  foot_body_index_to_side[robot.FindBodyIndex("l_foot")] = Side::LEFT;
  foot_body_index_to_side[robot.FindBodyIndex("r_foot")] = Side::RIGHT;
  // compute sum of wrenches, compare to rate of change of momentum from vd
  Vector6d total_wrench_in_world = Vector6d::Zero();
  const int n_basis_vectors_per_contact = 2 * m_surface_tangents;

  int beta_start = 0;
  for (size_t j = 0; j < active_supports.size(); j++) {
    const auto& active_support = active_supports[j];
    const auto& contact_pts = active_support.contact_pts;
    int ncj = static_cast<int>(contact_pts.size());
    int active_support_length = n_basis_vectors_per_contact * ncj;
    const auto& Bj = B.middleCols(beta_start, active_support_length);
    const auto& betaj = beta.segment(beta_start, active_support_length);
    Vector6d wrench_for_body_in_body_frame = Vector6d::Zero();
    Matrix3d R_world_to_body =
        robot.relativeTransform(cache, active_support.body_idx, 0).linear();

    for (size_t k = 0; k < contact_pts.size(); k++) {
      // for (auto k = contact_pts.begin(); k!= contact_pts.end(); k++) {
      const auto& Bblock = Bj.middleCols(k * n_basis_vectors_per_contact,
                                         n_basis_vectors_per_contact);
      const auto& betablock = betaj.segment(k * n_basis_vectors_per_contact,
                                            n_basis_vectors_per_contact);
      Vector3d point_force = R_world_to_body * Bblock * betablock;
      Vector3d contact_pt = contact_pts[k].head(3);
      auto torquejk = contact_pt.cross(point_force);
      wrench_for_body_in_body_frame.head<3>() += torquejk;
      wrench_for_body_in_body_frame.tail<3>() += point_force;
    }

    Isometry3d transform_to_world =
        robot.relativeTransform(cache, 0, active_support.body_idx);
    total_wrench_in_world += transformSpatialForce(
        transform_to_world, wrench_for_body_in_body_frame);
    beta_start += active_support_length;
  }

  double mass = robot.getMass();
  Vector6d gravitational_wrench_in_com = robot.a_grav * mass;
  Isometry3d com_to_world = Isometry3d::Identity();
  com_to_world.translation() = robot.centerOfMass(cache);
  Vector6d gravitational_wrench_in_world =
      transformSpatialForce(com_to_world, gravitational_wrench_in_com);
  total_wrench_in_world += gravitational_wrench_in_world;

  auto world_momentum_matrix = robot.worldMomentumMatrix(cache);
  auto world_momentum_matrix_dot_times_v =
      robot.worldMomentumMatrixDotTimesV(cache);
  Vector6d momentum_rate_of_change =
      world_momentum_matrix * qdd + world_momentum_matrix_dot_times_v;

  std::string explanation;
  if (!drake::CompareMatrices(total_wrench_in_world, momentum_rate_of_change,
                              1e-6, drake::MatrixCompareType::absolute,
                              &explanation)) {
    throw std::runtime_error("Drake:ValueCheck ERROR:" + explanation);
  }
}

std::unordered_map<std::string, int> computeBodyOrFrameNameToIdMap(
    const RigidBodyTree& robot) {
  auto id_map = std::unordered_map<std::string, int>();
  for (auto it = robot.bodies.begin(); it != robot.bodies.end(); ++it) {
    id_map[(*it)->get_name()] = it - robot.bodies.begin();
  }

  for (auto it = robot.frames.begin(); it != robot.frames.end(); ++it) {
    id_map[(*it)->get_name()] = -(it - robot.frames.begin()) - 2;
  }
  return id_map;
}

const QPControllerParams& InstantaneousQPController::FindParams(
    const std::string& param_set_name) {
  // look up the param set by name
  auto it = param_sets.find(param_set_name);
  if (it == param_sets.end()) {
    std::cout
        << "Got a param set I don't recognize! Using standing params instead";
    it = param_sets.find("standing");
    if (it == param_sets.end()) {
      throw std::runtime_error(
          "Could not fall back to standing parameters either. I have to give "
          "up here.");
    }
  }
  // cout << "using params set: " + it->first + ", ";
  const QPControllerParams& params = it->second;
  // mexPrintf("Kp_accel: %f, ", params.Kp_accel);
  return params;
}

int InstantaneousQPController::setupAndSolveQP(
    const drake::lcmt_qp_controller_input& qp_input,
    const DrakeRobotState& robot_state,
    const Ref<const Matrix<bool, Dynamic, 1>>& contact_detected,
    const drake::eigen_aligned_std_map<Side, ForceTorqueMeasurement>&
        foot_force_torque_measurements,
    QPControllerOutput& qp_output, QPControllerDebugData* debug) {
  // The primary solve loop for our controller. This constructs and solves a
  // Quadratic Program and produces the instantaneous desired torques, along
  // with reference positions, velocities, and accelerations. It mirrors the
  // Matlab implementation in
  // atlasControllers.InstantaneousQPController.setupAndSolveQP(), and more
  // documentation can be found there.
  // Note: argument `debug` MAY be set to NULL, which signals that no debug
  // information is requested.

  double dt = 0.0;
  if (controller_state.t_prev != 0.0) {
    dt = robot_state.t - controller_state.t_prev;
  }

  const QPControllerParams& params = FindParams(qp_input.param_set_name);

  int nu = robot->B.cols();
  int nq = robot->get_num_positions();

  // zmp_data
  Map<const Matrix<double, 4, 4, RowMajor>> A_ls(&qp_input.zmp_data.A[0][0]);
  Map<const Matrix<double, 4, 2, RowMajor>> B_ls(&qp_input.zmp_data.B[0][0]);
  Map<const Matrix<double, 2, 4, RowMajor>> C_ls(&qp_input.zmp_data.C[0][0]);
  Map<const Matrix<double, 2, 2, RowMajor>> D_ls(&qp_input.zmp_data.D[0][0]);
  Map<const Matrix<double, 4, 1>> x0(&qp_input.zmp_data.x0[0][0]);
  Map<const Matrix<double, 2, 1>> y0(&qp_input.zmp_data.y0[0][0]);
  Map<const Matrix<double, 2, 1>> u0(&qp_input.zmp_data.u0[0][0]);
  Map<const Matrix<double, 2, 2, RowMajor>> R_ls(&qp_input.zmp_data.R[0][0]);
  Map<const Matrix<double, 2, 2, RowMajor>> Qy(&qp_input.zmp_data.Qy[0][0]);
  Map<const Matrix<double, 4, 4, RowMajor>> S(&qp_input.zmp_data.S[0][0]);
  Map<const Matrix<double, 4, 1>> s1(&qp_input.zmp_data.s1[0][0]);
  Map<const Matrix<double, 4, 1>> s1dot(&qp_input.zmp_data.s1dot[0][0]);

  // Active supports
  std::vector<SupportStateElement,
              Eigen::aligned_allocator<SupportStateElement>>
      available_supports = loadAvailableSupports(qp_input);
  drake::eigen_aligned_std_vector<SupportStateElement> active_supports =
      getActiveSupports(*robot, robot_state.q, robot_state.qd,
                        available_supports, contact_detected,
                        params.contact_threshold);

  // // whole_body_data
  if (qp_input.whole_body_data.num_positions != nq)
    throw std::runtime_error(
        "number of positions doesn't match num_dof for this robot");
  Map<const VectorXd> q_des(qp_input.whole_body_data.q_des.data(), nq);
  if (static_cast<int32_t>(qp_input.whole_body_data.constrained_dofs.size()) !=
      qp_input.whole_body_data.num_constrained_dofs) {
    throw std::runtime_error(
        "size of constrained dofs does not match num_constrained_dofs");
  }
  Map<const VectorXi> condof(qp_input.whole_body_data.constrained_dofs.data(),
                             qp_input.whole_body_data.num_constrained_dofs);

  PIDOutput pid_out = wholeBodyPID(robot_state.t, robot_state.q, robot_state.qd,
                                   q_des, params.whole_body);
  VectorXd w_qdd = params.whole_body.w_qdd;

  auto joint_pd_override = qp_input.joint_pd_override;
  addJointSoftLimits(params.joint_soft_limits, robot_state, q_des,
                     active_supports, joint_pd_override);
  applyJointPDOverride(joint_pd_override, robot_state, pid_out, w_qdd);

  qp_output.q_ref = pid_out.q_ref;

  // mu
  // NOTE: we're using the same mu for all supports
  double mu;
  if (qp_input.num_support_data == 0) {
    mu = 1.0;
  } else {
    mu = qp_input.support_data[0].mu;
    for (int i = 1; i < qp_input.num_support_data; i++) {
      if (qp_input.support_data[i].mu != mu) {
        std::cout << "Currently, we assume that all supports have the same "
                     "value of mu";
      }
    }
  }

  const int dim = 3,  // 3D
      nd = 2 *
           m_surface_tangents;  // for friction cone approx, hard coded for now

  DRAKE_ASSERT(nu + 6 == nq);

  std::vector<DesiredBodyAcceleration> desired_body_accelerations;
  desired_body_accelerations.resize(qp_input.num_tracked_bodies);
  Vector6d body_v_des, body_vdot_des;
  Vector6d body_vdot;
  Isometry3d body_pose_des;

  for (int i = 0; i < qp_input.num_tracked_bodies; i++) {
    int body_or_frame_id0 = body_or_frame_name_to_id.at(
        qp_input.body_motion_data[i].body_or_frame_name);
    if (body_or_frame_id0 == -1) {
      std::cerr << "Body motion data with CoM as desired body is not allowed"
                << std::endl;
      continue;
    }

    int true_body_id0 = robot->parseBodyOrFrameID(body_or_frame_id0);
    double weight = params.body_motion[true_body_id0].weight;
    desired_body_accelerations[i].body_or_frame_id0 = body_or_frame_id0;
    Map<const Vector4d> quat_task_to_world(
        qp_input.body_motion_data[i].quat_task_to_world);
    Map<const Vector3d> translation_task_to_world(
        qp_input.body_motion_data[i].translation_task_to_world);
    desired_body_accelerations[i].T_task_to_world.linear() =
        drake::math::quat2rotmat(quat_task_to_world);
    desired_body_accelerations[i].T_task_to_world.translation() =
        translation_task_to_world;
    Map<const Vector3d> xyz_kp_multiplier(
        qp_input.body_motion_data[i].xyz_kp_multiplier);
    Map<const Vector3d> xyz_damping_ratio_multiplier(
        qp_input.body_motion_data[i].xyz_damping_ratio_multiplier);
    double expmap_kp_multiplier =
        qp_input.body_motion_data[i].expmap_kp_multiplier;
    double expmap_damping_ratio_multiplier =
        qp_input.body_motion_data[i].expmap_damping_ratio_multiplier;
    memcpy(desired_body_accelerations[i].weight_multiplier.data(),
           qp_input.body_motion_data[i].weight_multiplier, sizeof(double) * 6);
    desired_body_accelerations[i].body_path = robot->findKinematicPath(
        0, desired_body_accelerations[i].body_or_frame_id0);

    auto spline =
        decodePiecewisePolynomial(qp_input.body_motion_data[i].spline);
    evaluateXYZExpmapCubicSpline(robot_state.t, spline, body_pose_des,
                                 body_v_des, body_vdot_des);

    Vector6d body_Kp;
    body_Kp.head<3>() =
        (params.body_motion[true_body_id0].Kp.head<3>().array() *
         xyz_kp_multiplier.array())
            .matrix();
    body_Kp.tail<3>() =
        params.body_motion[true_body_id0].Kp.tail<3>() * expmap_kp_multiplier;
    Vector6d body_Kd;
    body_Kd.head<3>() =
        (params.body_motion[true_body_id0].Kd.head<3>().array() *
         xyz_damping_ratio_multiplier.array() *
         xyz_kp_multiplier.array().sqrt())
            .matrix();
    body_Kd.tail<3>() = params.body_motion[true_body_id0].Kd.tail<3>() *
                        sqrt(expmap_kp_multiplier) *
                        expmap_damping_ratio_multiplier;

    desired_body_accelerations[i].body_vdot = bodySpatialMotionPD(
        *robot, robot_state, body_or_frame_id0, body_pose_des, body_v_des,
        body_vdot_des, body_Kp, body_Kd,
        desired_body_accelerations[i].T_task_to_world);

    desired_body_accelerations[i].weight = weight;
    desired_body_accelerations[i].accel_bounds =
        params.body_motion[true_body_id0].accel_bounds;
    desired_body_accelerations[i].control_pose_when_in_contact =
        qp_input.body_motion_data[i].control_pose_when_in_contact;
  }

  int n_body_accel_eq_constraints = 0;
  for (size_t i = 0; i < desired_body_accelerations.size(); i++) {
    if (desired_body_accelerations[i].weight < 0) n_body_accel_eq_constraints++;
  }

  MatrixXd R_DQyD_ls = R_ls + D_ls.transpose() * Qy * D_ls;

  cache.initialize(robot_state.q, robot_state.qd);
  robot->doKinematics(cache, true);

  //---------------------------------------------------------------------

  int num_active_contact_pts = 0;
  for (std::vector<SupportStateElement,
                   Eigen::aligned_allocator<SupportStateElement>>::iterator
           iter = active_supports.begin();
       iter != active_supports.end(); iter++) {
    num_active_contact_pts += iter->contact_pts.size();
  }

  // handle external wrenches to compensate for
  RigidBodyTree::BodyToWrenchMap<double> external_wrenches;
  for (auto it = qp_input.body_wrench_data.begin();
       it != qp_input.body_wrench_data.end(); ++it) {
    const drake::lcmt_body_wrench_data& body_wrench_data = *it;
    int body_id = body_or_frame_name_to_id.at(body_wrench_data.body_name);
    auto f_ext_i =
        Map<const drake::WrenchVector<double>>(body_wrench_data.wrench);
    external_wrenches.insert({robot->bodies[body_id].get(), f_ext_i});
  }

  H = robot->massMatrix(cache);
  C = robot->dynamicsBiasTerm(cache, external_wrenches);

  H_float = H.topRows(6);
  H_act = H.bottomRows(nu);
  C_float = C.head<6>();
  C_act = C.tail(nu);

  bool include_angular_momentum = (params.W_kdot.array().maxCoeff() > 1e-10);

  if (include_angular_momentum) {
    Ag = robot->centroidalMomentumMatrix(cache);
    Agdot_times_v = robot->centroidalMomentumMatrixDotTimesV(cache);
    Ak = Ag.topRows<3>();
    Akdot_times_v = Agdot_times_v.topRows<3>();
  }
  Vector3d xcom;
  // consider making all J's into row-major

  xcom = robot->centerOfMass(cache);
  J = robot->centerOfMassJacobian(cache);
  Jdotv = robot->centerOfMassJacobianDotTimesV(cache);
  J_xy = J.topRows(2);
  Jdotv_xy = Jdotv.head<2>();

  MatrixXd Jcom;
  VectorXd Jcomdotv;

  if (x0.size() == 6) {
    Jcom = J;
    Jcomdotv = Jdotv;
  } else {
    Jcom = J_xy;
    Jcomdotv = Jdotv_xy;
  }

  Vector3d xcomdot = J * robot_state.qd;

  MatrixXd B, JB, Jp, normals;
  VectorXd Jpdotv;
  std::vector<double> adjusted_mus(active_supports.size());
  for (size_t i = 0; i < active_supports.size(); ++i) {
    int body_id = active_supports[i].body_idx;
    if ((body_id == rpc.foot_ids.at(Side::RIGHT) ||
         body_id == rpc.foot_ids.at(Side::LEFT)) &&
        (contact_detected(active_supports[i].body_idx) == 0)) {
      adjusted_mus[i] = MU_VERY_SMALL;
    } else {
      adjusted_mus[i] = mu;
    }
    // std::cout << adjusted_mus[i] << " ";
  }
  // std::cout << std::endl;
  int nc =
      contactConstraintsBV(*robot, cache, num_active_contact_pts, adjusted_mus,
                           active_supports, B, JB, Jp, Jpdotv, normals);
  int neps = nc * dim;

  if (params.use_center_of_mass_observer &&
      foot_force_torque_measurements.size() > 0) {
    estimateCoMBasedOnMeasuredZMP(params, active_supports, nc,
                                  foot_force_torque_measurements, dt, xcom,
                                  xcomdot);
  }

  VectorXd x_bar, xlimp;
  MatrixXd D_float(6, JB.cols()), D_act(nu, JB.cols());
  if (nc > 0) {
    if (x0.size() == 6) {
      // x, y, z com
      xlimp.resize(6);
      xlimp.topRows(3) = xcom;
      xlimp.bottomRows(3) = xcomdot;
    } else {
      xlimp.resize(4);
      xlimp.topRows(2) = xcom.topRows(2);
      xlimp.bottomRows(2) = xcomdot.topRows(2);
    }
    x_bar = xlimp - x0;

    D_float = JB.topRows(6);
    D_act = JB.bottomRows(nu);
  }

  int nf = nc * nd;  // number of contact force variables
  int nparams = nq + nf + neps;

  Vector3d kdot_des;
  if (include_angular_momentum) {
    VectorXd k = Ak * robot_state.qd;
    kdot_des = -params.Kp_ang * k;  // TODO(rdeits): parameterize
  }

  //----------------------------------------------------------------------
  // QP cost function ----------------------------------------------------
  //
  //  min: ybar*Qy*ybar + ubar*R*ubar + (2*S*xbar + s1)*(A*x + B*u) +
  //    w_qdd*quad(qddot_ref - qdd) + w_eps*quad(epsilon) +
  //    w_grf*quad(beta) + quad(kdot_des - (A*qdd + Adot*qd))
  VectorXd f(nparams);
  {
    if (nc > 0) {
      // NOTE: moved Hqp calcs below, because I compute the inverse directly for
      // FastQP (and sparse Hqp for gurobi)
      VectorXd tmp = C_ls * xlimp;
      VectorXd tmp1 = Jcomdotv;
      MatrixXd tmp2 = R_DQyD_ls * Jcom;

      fqp = tmp.transpose() * Qy * D_ls * Jcom;
      // mexPrintf("fqp head: %f %f %f\n", fqp(0), fqp(1), fqp(2));
      fqp += tmp1.transpose() * tmp2;
      fqp += (S * x_bar + 0.5 * s1).transpose() * B_ls * Jcom;
      fqp -= u0.transpose() * tmp2;
      fqp -= y0.transpose() * Qy * D_ls * Jcom;
      fqp -= (w_qdd.array() * pid_out.qddot_des.array()).matrix().transpose();
      if (include_angular_momentum) {
        fqp += Akdot_times_v.transpose() * params.W_kdot * Ak;
        fqp -= kdot_des.transpose() * params.W_kdot * Ak;
      }
      f.head(nq) = fqp.transpose();
    } else {
      f.head(nq) = -pid_out.qddot_des;
    }
  }
  f.tail(nf + neps) = VectorXd::Zero(nf + neps);

  int neq = 6 + neps + 6 * n_body_accel_eq_constraints +
            qp_input.whole_body_data.num_constrained_dofs;
  MatrixXd Aeq = MatrixXd::Zero(neq, nparams);
  VectorXd beq = VectorXd::Zero(neq);

  // constrained floating base dynamics
  //  H_float*qdd - J_float'*lambda - Dbar_float*beta = -C_float
  Aeq.topLeftCorner(6, nq) = H_float;
  beq.topRows(6) = -C_float;

  if (nc > 0) {
    Aeq.block(0, nq, 6, nc * nd) = -D_float;
  }

  if (nc > 0) {
    // relative acceleration constraint
    Aeq.block(6, 0, neps, nq) = Jp;
    Aeq.block(6, nq, neps, nf) =
        MatrixXd::Zero(neps, nf);  // note: obvious sparsity here
    Aeq.block(6, nq + nf, neps, neps) =
        MatrixXd::Identity(neps, neps);  // note: obvious sparsity here
    beq.segment(6, neps) = -Jpdotv - params.Kp_accel * Jp * robot_state.qd;
  }

  // add in body spatial equality constraints
  // VectorXd body_vdot;
  int equality_ind = 6 + neps;
  MatrixXd Jb(6, nq);
  Vector6d Jbdotv;
  for (size_t i = 0; i < desired_body_accelerations.size(); i++) {
    if (desired_body_accelerations[i].weight <
        0) {  // negative implies constraint
      int body_id0 = robot->parseBodyOrFrameID(
          desired_body_accelerations[i].body_or_frame_id0);
      if (desired_body_accelerations[i].control_pose_when_in_contact ||
          !inSupport(active_supports, body_id0)) {
        Matrix<double, 6, Dynamic> Jb_compact = robot->geometricJacobian(
            cache, 0, desired_body_accelerations[i].body_or_frame_id0,
            desired_body_accelerations[i].body_or_frame_id0, true);
        Jb = robot->compactToFull<Matrix<double, 6, Dynamic>>(
            Jb_compact, desired_body_accelerations[i].body_path.joint_path,
            true);
        Jbdotv = robot->geometricJacobianDotTimesV(
            cache, 0, desired_body_accelerations[i].body_or_frame_id0,
            desired_body_accelerations[i].body_or_frame_id0);

        if (qp_input.body_motion_data[i].in_floating_base_nullspace) {
          Jb.block(0, 0, 6, 6) = MatrixXd::Zero(6, 6);
          // Jbdot.block(0, 0, 6, 6) = MatrixXd::Zero(6, 6);
        }
        for (int j = 0; j < 6; j++) {
          if (!std::isnan(desired_body_accelerations[i].body_vdot(j))) {
            Aeq.block(equality_ind, 0, 1, nq) = Jb.row(j);
            beq[equality_ind++] =
                -Jbdotv(j) + desired_body_accelerations[i].body_vdot(j);
          }
        }
      }
    }
  }

  if (qp_input.whole_body_data.num_constrained_dofs > 0) {
    // add joint acceleration constraints
    for (int i = 0; i < qp_input.whole_body_data.num_constrained_dofs; i++) {
      Aeq(equality_ind, static_cast<int>(condof[i]) - 1) = 1;
      beq[equality_ind++] = pid_out.qddot_des[static_cast<int>(condof[i]) - 1];
    }
  }

  int n_ineq = 2 * nu + 2 * 6 * desired_body_accelerations.size();
  MatrixXd Ain =
      MatrixXd::Zero(n_ineq, nparams);  // note: obvious sparsity here
  VectorXd bin = VectorXd::Zero(n_ineq);

  auto B_act = robot->B.bottomRows(robot->B.cols());

  // linear input saturation constraints
  // u=B_act'*(H_act*qdd + C_act - Jz_act'*z - Dbar_act*beta)
  // using transpose instead of inverse because B is orthogonal
  Ain.topLeftCorner(nu, nq) = B_act.transpose() * H_act;
  Ain.block(0, nq, nu, nc * nd) = -B_act.transpose() * D_act;
  bin.head(nu) = -B_act.transpose() * C_act + umax;

  Ain.block(nu, 0, nu, nparams) = -1 * Ain.block(0, 0, nu, nparams);
  bin.segment(nu, nu) = B_act.transpose() * C_act - umin;

  int constraint_start_index = 2 * nu;
  for (size_t i = 0; i < desired_body_accelerations.size(); i++) {
    Matrix<double, 6, Dynamic> Jb_compact = robot->geometricJacobian(
        cache, 0, desired_body_accelerations[i].body_or_frame_id0,
        desired_body_accelerations[i].body_or_frame_id0, true);
    Jb = robot->compactToFull<Matrix<double, 6, Dynamic>>(
        Jb_compact, desired_body_accelerations[i].body_path.joint_path, true);
    Jbdotv = robot->geometricJacobianDotTimesV(
        cache, 0, desired_body_accelerations[i].body_or_frame_id0,
        desired_body_accelerations[i].body_or_frame_id0);

    if (qp_input.body_motion_data[i].in_floating_base_nullspace) {
      Jb.block(0, 0, 6, 6) = MatrixXd::Zero(6, 6);
      // Jbdot.block(0, 0, 6, 6) = MatrixXd::Zero(6, 6);
    }
    Ain.block(constraint_start_index, 0, 6, robot->get_num_positions()) = Jb;
    bin.segment(constraint_start_index, 6) =
        -Jbdotv + desired_body_accelerations[i].accel_bounds.max;
    constraint_start_index += 6;
    Ain.block(constraint_start_index, 0, 6, robot->get_num_positions()) = -Jb;
    bin.segment(constraint_start_index, 6) =
        Jbdotv - desired_body_accelerations[i].accel_bounds.min;
    constraint_start_index += 6;
  }

  for (int i = 0; i < n_ineq; ++i) {
    // remove inf constraints---needed by gurobi
    if (std::isinf(bin(i))) {
      Ain.row(i) = 0 * Ain.row(i);
      bin(i) = 0;
    }
  }

  GRBmodel* model = nullptr;
  int info = -1;

  // set obj, lb, up
  VectorXd lb(nparams), ub(nparams);
  lb.head(nq) = qdd_lb;
  ub.head(nq) = qdd_ub;
  lb.segment(nq, nf) = VectorXd::Zero(nf);
  ub.segment(nq, nf) = 1e3 * VectorXd::Ones(nf);
  lb.tail(neps) = -params.slack_limit * VectorXd::Ones(neps);
  ub.tail(neps) = params.slack_limit * VectorXd::Ones(neps);

  VectorXd alpha(nparams);

  MatrixXd Qnfdiag(nf, 1), Qneps(neps, 1);
  std::vector<MatrixXd*> QBlkDiag(
      nc > 0 ? 3 : 1);  // nq, nf, neps   // this one is for gurobi

  VectorXd w = (w_qdd.array() + REG).matrix();

  if (nc != controller_state.num_active_contact_pts) {
    // Number of contact points has changed, so our active set is invalid
    controller_state.active.clear();
  }
  controller_state.num_active_contact_pts = nc;

#ifdef USE_MATRIX_INVERSION_LEMMA
  double max_body_accel_weight = -numeric_limits<double>::infinity();
  for (int i = 0; i < desired_body_accelerations.size(); i++) {
    max_body_accel_weight =
        max(max_body_accel_weight, desired_body_accelerations[i].weight);
  }
  bool include_body_accel_cost_terms =
      desired_body_accelerations.size() > 0 && max_body_accel_weight > 1e-10;
  if (use_fast_qp > 0 && !include_angular_momentum &&
      !include_body_accel_cost_terms) {
    // TODO(rdeits): update to include angular momentum, body accel objectives.

    //    We want Hqp inverse, which I can compute efficiently using the
    //    matrix inversion lemma (see wikipedia):
    //    inv(A + U'CV) = inv(A) - inv(A)*U* inv([ inv(C)+ V*inv(A)*U ]) V
    //    inv(A)
    if (nc > 0) {
      MatrixXd Wi = ((1 / (w_qdd.array() + REG)).matrix()).asDiagonal();
      if (R_DQyD_ls.trace() > 1e-15) {  // R_DQyD_ls is not zero
        Hqp =
            Wi -
            Wi * Jcom.transpose() *
                (R_DQyD_ls.inverse() + Jcom * Wi * Jcom.transpose()).inverse() *
                Jcom * Wi;
      }
    } else {
      Hqp = MatrixXd::Constant(nq, 1, 1 / (1 + REG));
    }

#ifdef TEST_FAST_QP
    if (nc > 0) {
      MatrixXd Hqp_test(nq, nq);
      MatrixXd W = w.asDiagonal();
      Hqp_test = (Jcom.transpose() * R_DQyD_ls * Jcom + W).inverse();
      if (((Hqp_test - Hqp).array().abs()).maxCoeff() > 1e-6) {
        throw std::runtime_error(
            "Q submatrix inverse from matrix inversion lemma does not match "
            "direct Q inverse.");
      }
    }
#endif

    Qnfdiag = MatrixXd::Constant(nf, 1, 1 / REG);
    Qneps = MatrixXd::Constant(neps, 1, 1 / (.001 + REG));

    QBlkDiag[0] = &Hqp;
    if (nc > 0) {
      QBlkDiag[1] = &Qnfdiag;
      QBlkDiag[2] = &Qneps;  // quadratic slack var cost,
                             // Q(nparams-neps:end, nparams-neps:end)=eye(neps)
    }

    MatrixXd Ain_lb_ub(n_ineq + 2 * nparams, nparams);
    VectorXd bin_lb_ub(n_ineq + 2 * nparams);
    Ain_lb_ub << Ain,  // note: obvious sparsity here
        -MatrixXd::Identity(nparams, nparams),
        MatrixXd::Identity(nparams, nparams);
    bin_lb_ub << bin, -lb, ub;

    for (std::set<int>::iterator it = controller_state.active.begin();
         it != controller_state.active.end(); it++) {
      if (std::isnan(bin_lb_ub(*it)) || std::isinf(bin_lb_ub(*it))) {
        controller_state.active.clear();
        break;
      }
    }

    info = fastQPThatTakesQinv(QBlkDiag, f, Aeq, beq, Ain_lb_ub, bin_lb_ub,
                               controller_state.active, alpha);

    // if (info<0)   mexPrintf("fastQP info = %d.  Calling gurobi.\n", info);
  } else {
#endif

    if (nc > 0) {
      Hqp = Jcom.transpose() * R_DQyD_ls * Jcom;
      if (include_angular_momentum) {
        Hqp += Ak.transpose() * params.W_kdot * Ak;
      }
      Hqp += w_qdd.asDiagonal();
      Hqp += REG * MatrixXd::Identity(nq, nq);
    } else {
      Hqp = (1 + REG) * MatrixXd::Identity(nq, nq);
    }

    // add in body spatial acceleration cost terms
    for (size_t i = 0; i < desired_body_accelerations.size(); i++) {
      if (desired_body_accelerations[i].weight > 0) {
        int body_id0 = robot->parseBodyOrFrameID(
            desired_body_accelerations[i].body_or_frame_id0);
        if (desired_body_accelerations[i].control_pose_when_in_contact ||
            !inSupport(active_supports, body_id0)) {
          Matrix<double, 6, Dynamic> Jb_compact = robot->geometricJacobian(
              cache, 0, desired_body_accelerations[i].body_or_frame_id0,
              desired_body_accelerations[i].body_or_frame_id0, true);
          Jb = robot->compactToFull<Matrix<double, 6, Dynamic>>(
              Jb_compact, desired_body_accelerations[i].body_path.joint_path,
              true);
          Jbdotv = robot->geometricJacobianDotTimesV(
              cache, 0, desired_body_accelerations[i].body_or_frame_id0,
              desired_body_accelerations[i].body_or_frame_id0);

          if (qp_input.body_motion_data[i].in_floating_base_nullspace) {
            Jb.block(0, 0, 6, 6) = MatrixXd::Zero(6, 6);
            // Jbdot.block(0, 0, 6, 6) = MatrixXd::Zero(6, 6);
          }
          for (int j = 0; j < 6; j++) {
            if (!std::isnan(desired_body_accelerations[i].body_vdot[j])) {
              Hqp += desired_body_accelerations[i].weight *
                     desired_body_accelerations[i].weight_multiplier(j) *
                     (Jb.row(j)).transpose() * Jb.row(j);
              f.head(nq).noalias() +=
                  desired_body_accelerations[i].weight *
                  desired_body_accelerations[i].weight_multiplier(j) *
                  (Jbdotv(j) - desired_body_accelerations[i].body_vdot[j]) *
                  Jb.row(j).transpose();
            }
          }
        }
      }
    }

    Qnfdiag = MatrixXd::Constant(nf, 1, params.w_grf + REG);
    Qneps = MatrixXd::Constant(neps, 1, params.w_slack + REG);

    QBlkDiag[0] = &Hqp;
    if (nc > 0) {
      QBlkDiag[1] = &Qnfdiag;
      QBlkDiag[2] = &Qneps;  // quadratic slack var cost,
                             // Q(nparams-neps:end, nparams-neps:end)=eye(neps)
    }

    MatrixXd Ain_lb_ub(n_ineq + 2 * nparams, nparams);
    VectorXd bin_lb_ub(n_ineq + 2 * nparams);
    Ain_lb_ub << Ain,  // note: obvious sparsity here
        -MatrixXd::Identity(nparams, nparams),
        MatrixXd::Identity(nparams, nparams);
    bin_lb_ub << bin, -lb, ub;

    for (std::set<int>::iterator it = controller_state.active.begin();
         it != controller_state.active.end(); it++) {
      if (std::isnan(bin_lb_ub(*it)) || std::isinf(bin_lb_ub(*it))) {
        controller_state.active.clear();
        break;
      }
    }

    if (use_fast_qp > 0) {  // set up and call fastqp
      info = fastQP(QBlkDiag, f, Aeq, beq, Ain_lb_ub, bin_lb_ub,
                    controller_state.active, alpha);
      // if (info<0)    mexPrintf("fastQP info=%d... calling Gurobi.\n", info);
    } else {
      // use gurobi active set
      model = gurobiActiveSetQP(
          env, QBlkDiag, f, Aeq, beq, Ain, bin, lb, ub, controller_state.vbasis,
          controller_state.vbasis_len, controller_state.cbasis,
          controller_state.cbasis_len, alpha);
      CGE(GRBgetintattr(model, "NumVars", &(controller_state.vbasis_len)), env);
      CGE(GRBgetintattr(model, "NumConstrs", &(controller_state.cbasis_len)),
          env);
      info = 66;
      // info = -1;
    }

    if (info < 0) {
      model = gurobiQP(env, QBlkDiag, f, Aeq, beq, Ain, bin, lb, ub,
                       controller_state.active, alpha);
      int status;
      CGE(GRBgetintattr(model, "Status", &status), env);
      // if (status!=2) mexPrintf("Gurobi reports non-optimal status = %d\n",
      // status);
    }
#ifdef USE_MATRIX_INVERSION_LEMMA
  }
#endif

  //----------------------------------------------------------------------
  // Solve for inputs ----------------------------------------------------
  qp_output.qdd = alpha.head(nq);
  VectorXd beta = alpha.segment(nq, nc * nd);

  if (params.use_center_of_mass_observer) {
    controller_state.last_com_ddot = Jdotv + J * qp_output.qdd;
  }

  if (CHECK_CENTROIDAL_MOMENTUM_RATE_MATCHES_TOTAL_WRENCH) {
    checkCentroidalMomentumMatchesTotalWrench(*robot, cache, qp_output.qdd,
                                              active_supports, B, beta);
  }

  // use transpose because B_act is orthogonal
  qp_output.u =
      B_act.transpose() * (H_act * qp_output.qdd + C_act - D_act * beta);
  for (int i = 0; i < qp_output.u.size(); i++) {
    if (std::isnan(qp_output.u(i))) qp_output.u(i) = 0;
  }
  // y = B_act.jacobiSvd(ComputeThinU|ComputeThinV).solve(H_act*qdd + C_act -
  // Jz_act.transpose()*lambda - D_act*beta);

  bool foot_contact[2];
  foot_contact[0] = contact_detected(rpc.foot_ids.at(Side::LEFT)) == 1;
  foot_contact[1] = contact_detected(rpc.foot_ids.at(Side::RIGHT)) == 1;
  qp_output.qd_ref =
      velocityReference(robot_state.t, robot_state.q, robot_state.qd,
                        qp_output.qdd, foot_contact, params.vref_integrator);

  // Remember t for next time around
  controller_state.t_prev = robot_state.t;

  // If a debug pointer was passed in, fill it with useful data
  if (debug) {
    debug->active_supports.resize(active_supports.size());
    for (size_t i = 0; i < active_supports.size(); i++) {
      debug->active_supports[i] = active_supports[i];
    }
    debug->nc = nc;
    debug->normals = normals;
    debug->B = B;
    debug->alpha = alpha;
    debug->f = f;
    debug->Aeq = Aeq;
    debug->beq = beq;
    debug->Ain_lb_ub = Ain_lb_ub;
    debug->bin_lb_ub = bin_lb_ub;
    debug->Qnfdiag = Qnfdiag;
    debug->Qneps = Qneps;
    debug->x_bar = x_bar;
    debug->S = S;
    debug->s1 = s1;
    debug->s1dot = s1dot;
    debug->s2dot = qp_input.zmp_data.s2dot;
    debug->A_ls = A_ls;
    debug->B_ls = B_ls;
    debug->Jcom = Jcom;
    debug->Jcomdotv = Jcomdotv;
    debug->beta = beta;
  }

  // if we used gurobi, clean up
  if (model) {
    GRBfreemodel(model);
  }
  //  GRBfreeenv(env);

  return info;
}  // NOLINT(readability/fn_size)
