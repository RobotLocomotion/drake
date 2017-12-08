#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_stl_types.h"
#include "drake/lcmt_qp_controller_input.hpp"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/solvers/gurobi_qp.h"
#include "drake/systems/controllers/QPCommon.h"

#define INSTQP_USE_FASTQP 1
#define INSTQP_GUROBI_OUTPUTFLAG 0
#define INSTQP_GUROBI_METHOD 2
#define INSTQP_GUROBI_PRESOLVE 0
#define INSTQP_GUROBI_BARITERLIMIT 20
#define INSTQP_GUROBI_BARHOMOGENEOUS 0
#define INSTQP_GUROBI_BARCONVTOL (5e-4)

class InstantaneousQPController {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InstantaneousQPController)

  InstantaneousQPController(
      std::unique_ptr<RigidBodyTree<double>> robot_in,
      const drake::eigen_aligned_std_map<std::string, QPControllerParams>&
          param_sets_in,
      const RobotPropertyCache& rpc_in)
      : robot(std::move(robot_in)),
        param_sets(param_sets_in),
        rpc(rpc_in),
        use_fast_qp(INSTQP_USE_FASTQP),
        cache(this->robot->CreateKinematicsCache()) {
    initialize();
  }

  InstantaneousQPController(std::unique_ptr<RigidBodyTree<double>> robot_in,
                            const std::string& control_config_filename)
      : robot(std::move(robot_in)),
        use_fast_qp(INSTQP_USE_FASTQP),
        cache(this->robot->CreateKinematicsCache()) {
    loadConfigurationFromYAML(control_config_filename);
    initialize();
  }

  InstantaneousQPController(const std::string& urdf_filename,
                            const std::string& control_config_filename)
      : robot(std::make_unique<RigidBodyTree<double>>()),
        use_fast_qp(INSTQP_USE_FASTQP),
        cache(this->robot->CreateKinematicsCache()) {
    drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        urdf_filename, drake::multibody::joints::kRollPitchYaw, robot.get());
    loadConfigurationFromYAML(control_config_filename);
    initialize();
  }

  int setupAndSolveQP(
      const drake::lcmt_qp_controller_input& qp_input,
      const DrakeRobotState& robot_state,
      const Eigen::Ref<const Eigen::Matrix<bool, Eigen::Dynamic, 1>>&
          contact_detected,
      const drake::eigen_aligned_std_map<Side, ForceTorqueMeasurement>&
          foot_force_torque_measurements,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      QPControllerOutput& qp_output,
      QPControllerDebugData* debug = NULL);

  const RigidBodyTree<double>& getRobot() const { return *robot; }

  std::unordered_map<std::string, int> body_or_frame_name_to_id;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  GRBenv* env;
  std::unique_ptr<RigidBodyTree<double>> robot;
  drake::eigen_aligned_std_map<std::string, QPControllerParams> param_sets;
  RobotPropertyCache rpc;
  Eigen::VectorXd umin, umax;
  int use_fast_qp;
  JointNames input_joint_names;

  KinematicsCache<double> cache;
  Eigen::MatrixXd H, H_float, H_act;
  Eigen::VectorXd C, C_float, C_act;
  Eigen::MatrixXd J;
  Eigen::Vector3d Jdotv;
  Eigen::MatrixXd J_xy;
  Eigen::Vector2d Jdotv_xy;
  Eigen::MatrixXd Hqp;
  Eigen::RowVectorXd fqp;
  Eigen::VectorXd qdd_lb;
  Eigen::VectorXd qdd_ub;

  // momentum controller-specific
  Eigen::MatrixXd Ag;      // centroidal momentum matrix
  Vector6d Agdot_times_v;  // centroidal momentum velocity-dependent bias
  Eigen::MatrixXd Ak;      // centroidal angular momentum matrix
  Eigen::Vector3d
      Akdot_times_v;  // centroidal angular momentum velocity-dependent bias

  // logical separation for the controller state, that is, things we expect to
  // change at every iteration
  // and which must persist to the next iteration
  QPControllerState controller_state;

  PIDOutput wholeBodyPID(double t, const Eigen::Ref<const Eigen::VectorXd>& q,
                         const Eigen::Ref<const Eigen::VectorXd>& qd,
                         const Eigen::Ref<const Eigen::VectorXd>& q_des,
                         const WholeBodyParams& params);

  Eigen::VectorXd velocityReference(
      double t, const Eigen::Ref<const Eigen::VectorXd>& q,
      const Eigen::Ref<const Eigen::VectorXd>& qd,
      const Eigen::Ref<const Eigen::VectorXd>& qdd, bool foot_contact[2],
      const VRefIntegratorParams& params);

  drake::eigen_aligned_std_vector<SupportStateElement> loadAvailableSupports(
      const drake::lcmt_qp_controller_input& qp_input);

  void estimateCoMBasedOnMeasuredZMP(
      const QPControllerParams& params,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      drake::eigen_aligned_std_vector<SupportStateElement>& active_supports,
      int num_contact_points,
      const drake::eigen_aligned_std_map<Side, ForceTorqueMeasurement>&
          foot_force_torque_measurements,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      double dt, Eigen::Vector3d& xcom, Eigen::Vector3d& xcomdot);

  void initialize();
  void loadConfigurationFromYAML(const std::string& control_config_filename);

  // Select a parameter set by name from the collection of parameter sets.
  // If not found, fall back to "standing". If that fails, throw an exception.
  const QPControllerParams& FindParams(const std::string& param_set_name);
};

void applyURDFModifications(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    std::unique_ptr<RigidBodyTree<double>>& robot,
    const KinematicModifications& modifications);
void applyURDFModifications(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    std::unique_ptr<RigidBodyTree<double>>& robot,
    const std::string& urdf_modifications_filename);
