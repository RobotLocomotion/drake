#ifndef INSTANTANEOUSQPCONTROLLER_H
#define INSTANTANEOUSQPCONTROLLER_H

#include <memory>
#include "QPCommon.h"

#define USE_FASTQP 1
#define GUROBI_OUTPUTFLAG 0
#define GUROBI_METHOD 2
#define GUROBI_PRESOLVE 0
#define GUROBI_BARITERLIMIT 20
#define GUROBI_BARHOMOGENEOUS 0
#define GUROBI_BARCONVTOL (5e-4)


class InstantaneousQPController {

public:

  InstantaneousQPController(std::unique_ptr<RigidBodyTree> robot_in, const std::map<std::string,QPControllerParams>& param_sets_in, const RobotPropertyCache& rpc_in):
    robot(std::move(robot_in)), 
    cache(this->robot->bodies),
    param_sets(param_sets_in),
    rpc(rpc_in),
    use_fast_qp(USE_FASTQP) {
    initialize();
  }

  InstantaneousQPController(std::unique_ptr<RigidBodyTree> robot_in, const std::string& control_config_filename):
    robot(std::move(robot_in)), 
    cache(this->robot->bodies),
    use_fast_qp(USE_FASTQP) {
    loadConfigurationFromYAML(control_config_filename);
    initialize();
  }

  InstantaneousQPController(const std::string& urdf_filename, const std::string& control_config_filename):
    robot(std::unique_ptr<RigidBodyTree>(new RigidBodyTree(urdf_filename))), 
    cache(this->robot->bodies),
    use_fast_qp(USE_FASTQP) {
    loadConfigurationFromYAML(control_config_filename);
    initialize();
  }

  GRBenv *env;
  std::unique_ptr<RigidBodyTree> robot;
  std::map<std::string,QPControllerParams> param_sets;
  RobotPropertyCache rpc;
  Eigen::VectorXd umin,umax;
  int use_fast_qp;
  JointNames input_joint_names;

  // preallocate memory
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
  Eigen::MatrixXd Ag; // centroidal momentum matrix
  Vector6d Agdot_times_v; // centroidal momentum velocity-dependent bias
  Eigen::MatrixXd Ak; // centroidal angular momentum matrix
  Eigen::Vector3d Akdot_times_v; // centroidal angular momentum velocity-dependent bias

  std::unordered_map<std::string, int> body_or_frame_name_to_id;

  // logical separation for the controller state, that is, things we expect to change at every iteration
  // and which must persist to the next iteration
  QPControllerState controller_state;

  int setupAndSolveQP(const drake::lcmt_qp_controller_input& qp_input, const DrakeRobotState &robot_state, const Eigen::Ref<const Eigen::Matrix<bool, Eigen::Dynamic, 1> > &contact_detected, const std::map<Side, ForceTorqueMeasurement>& foot_force_torque_measurements, QPControllerOutput& qp_output, QPControllerDebugData* debug=NULL);

private:
  PIDOutput wholeBodyPID(double t, const Eigen::Ref<const Eigen::VectorXd> &q, const Eigen::Ref<const Eigen::VectorXd> &qd, const Eigen::Ref<const Eigen::VectorXd> &q_des, const WholeBodyParams& params);

  Eigen::VectorXd velocityReference(double t, const Eigen::Ref<const Eigen::VectorXd> &q, const Eigen::Ref<const Eigen::VectorXd> &qd, const Eigen::Ref<const Eigen::VectorXd> &qdd, bool foot_contact[2], const VRefIntegratorParams& params);

  std::vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>> loadAvailableSupports(const drake::lcmt_qp_controller_input& qp_input);

  void estimateCoMBasedOnMeasuredZMP(const QPControllerParams& params, std::vector<SupportStateElement, Eigen::aligned_allocator<SupportStateElement> >& active_supports, int num_contact_points, const std::map<Side, ForceTorqueMeasurement>& foot_force_torque_measurements, double dt, Eigen::Vector3d& xcom, Eigen::Vector3d& xcomdot);

  void initialize();
  void loadConfigurationFromYAML(const std::string& control_config_filename);
};

void applyURDFModifications(std::unique_ptr<RigidBodyTree>& robot, const KinematicModifications& modifications);
void applyURDFModifications(std::unique_ptr<RigidBodyTree>& robot, const std::string& urdf_modifications_filename);


#endif