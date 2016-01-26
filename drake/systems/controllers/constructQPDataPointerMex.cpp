#include "QPCommon.h"
#include <Eigen/StdVector>
#include "drake/util/drakeMexUtil.h"
#include "drake/systems/controllers/controlMexUtil.h"
#include "drake/util/yaml/yamlUtil.h"
#include "Path.h"
#include <regex>

// #include <limits>
// #include <cmath>
// #include "drake/util/drakeUtil.h"

using namespace std;
using namespace Eigen;
using namespace YAML;


vector<int> findPositionIndices(const RigidBodyTree &robot, const vector<string> &joint_names) {
  vector<int> position_indices;
  position_indices.reserve(joint_names.size());
  for (const auto& joint_name : joint_names) {
    const RigidBody &body = *robot.findJoint(joint_name);
    for (int i = 0; i < body.getJoint().getNumPositions(); i++) {
      position_indices.push_back(body.position_num_start + i);
    }
  }
  return position_indices;
}

RobotPropertyCache parseKinematicTreeMetadata(const YAML::Node& metadata, const RigidBodyTree& robot) {
  RobotPropertyCache ret;
  map<Side, string> side_identifiers = { {Side::RIGHT, "r"}, {Side::LEFT, "l"} };

  Node body_names = metadata["body_names"];
  Node feet = body_names["feet"];
  for (const auto& side : Side::values) {
    ret.foot_ids[side] = robot.findLinkId(feet[side_identifiers[side]].as<string>());
  }

  Node joint_group_names = metadata["joint_group_names"];
  for (const auto& side : Side::values) {
    auto side_id = side_identifiers.at(side);
    ret.position_indices.legs[side] = findPositionIndices(robot, joint_group_names["legs"][side_id].as<vector<string>>());
    ret.position_indices.knees[side] = robot.findJoint(joint_group_names["knees"][side_id].as<string>())->position_num_start;
    ret.position_indices.ankles[side] = findPositionIndices(robot, joint_group_names["ankles"][side_id].as<vector<string>>());
    ret.position_indices.arms[side] = findPositionIndices(robot, joint_group_names["arms"][side_id].as<vector<string>>());
  }
  ret.position_indices.neck = findPositionIndices(robot, joint_group_names["neck"].as<vector<string>>());
  ret.position_indices.back_bkz = robot.findJoint(joint_group_names["back_bkz"].as<string>())->position_num_start;
  ret.position_indices.back_bky = robot.findJoint(joint_group_names["back_bky"].as<string>())->position_num_start;

  return ret;
}

JointNames parseRobotJointNames(const string& hardware_data_file_name, const RigidBodyTree& tree) {
  JointNames ret;
  ret.drake.resize(tree.actuators.size());
  transform(tree.actuators.begin(), tree.actuators.end(), ret.drake.begin(),
            [](const RigidBodyActuator &actuator) { return actuator.body->getJoint().getName(); });
  Node hardware_data = LoadFile(hardware_data_file_name);
  ret.robot = hardware_data["joint_names"].as<vector<string>>();

  return ret;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs<1) mexErrMsgTxt("usage: ptr = constructQPDataPointerMex(robot_obj, control_config_filename, B, umin, umax, gurobi_opts);");

  if (nrhs == 1) {
    // By convention, calling the constructor with just one argument (the pointer) should delete the pointer
    if (isa(prhs[0],"DrakeMexPointer")) { 
      destroyDrakeMexPointer<NewQPControllerData*>(prhs[0]);
      return;
    } else {
      mexErrMsgIdAndTxt("Drake:constructQPDataPointerMex:BadInputs", "Expected a DrakeMexPointer (or a subclass)");
    }
  }


  if (nlhs<1) mexErrMsgTxt("take at least one output... please.");

  int narg = 0;

  auto urdf_filename = mxGetStdString(prhs[narg++]);
  unique_ptr<RigidBodyTree> robot_ptr(new RigidBodyTree(urdf_filename));
  set<string> collision_groups_to_keep = {"heel", "toe"};
  auto filter = [&](const string &group_name) { return collision_groups_to_keep.find(group_name) == collision_groups_to_keep.end(); };
  robot_ptr->removeCollisionGroupsIf(filter);
  robot_ptr->compile();

  NewQPControllerData* pdata = new NewQPControllerData(move(robot_ptr));


  // kinematic tree metadata & param sets
  std::string control_config_filename = mxGetStdString(prhs[narg]);
  YAML::Node control_config = LoadFile(control_config_filename);
  std::string replacement = ".out.yaml";
  std::regex pattern("\\.yaml");
  std::string debug_filename = regex_replace(control_config_filename, pattern, replacement);
  std::ofstream debug_file(debug_filename);
  pdata->param_sets = loadAllParamSets(control_config["qp_controller_params"], *(pdata->r), debug_file); 
  pdata->rpc = parseKinematicTreeMetadata(control_config["kinematic_tree_metadata"], *(pdata->r));
  narg++;

  // umin
  int nq = pdata->r->num_positions, nu = static_cast<int>(pdata->r->actuators.size());

  pdata->umin.resize(nu);
  pdata->umax.resize(nu);
  for (int i = 0; i < pdata->r->actuators.size(); i++) {
    pdata->umin(i) = pdata->r->actuators.at(i).effort_limit_min;
    pdata->umax(i) = pdata->r->actuators.at(i).effort_limit_max;
  }

  // use_fast_qp
  pdata->use_fast_qp = (int) mxGetScalar(prhs[narg]);
  narg++;

  // gurobi_opts
  const mxArray* psolveropts = prhs[narg];
  narg++;

  // input_coordinate_names
  pdata->input_joint_names = parseRobotJointNames(mxGetStdString(prhs[narg]), *(pdata->r));
  narg++;

  // Done parsing inputs


  pdata->qdd_lb = VectorXd::Zero(nq).array() - numeric_limits<double>::infinity();
  pdata->qdd_ub = VectorXd::Zero(nq).array() + numeric_limits<double>::infinity();

  //create gurobi environment
  int error = GRBloadenv(&(pdata->env),NULL);
  if (error) {
    mexPrintf("Gurobi error code: %d\n", error);
    mexErrMsgTxt("Cannot load gurobi environment");
  }

  // set solver params (http://www.gurobi.com/documentation/5.5/reference-manual/node798#sec:Parameters)
  int method = (int) mxGetScalar(myGetField(psolveropts,"method"));
  CGE ( GRBsetintparam(pdata->env,"outputflag",0), pdata->env );
  CGE ( GRBsetintparam(pdata->env,"method",method), pdata->env );
  // CGE ( GRBsetintparam(pdata->env,"method",method), pdata->env );
  CGE ( GRBsetintparam(pdata->env,"presolve",0), pdata->env );
  if (method==2) {
    CGE ( GRBsetintparam(pdata->env,"bariterlimit",20), pdata->env );
    CGE ( GRBsetintparam(pdata->env,"barhomogeneous",0), pdata->env );
    CGE ( GRBsetdblparam(pdata->env,"barconvtol",0.0005), pdata->env );
  }

  // preallocate some memory
  pdata->H.resize(nq,nq);
  pdata->H_float.resize(6,nq);
  pdata->H_act.resize(nu,nq);

  pdata->C.resize(nq);
  pdata->C_float.resize(6);
  pdata->C_act.resize(nu);

  pdata->J.resize(3,nq);
  pdata->J_xy.resize(2,nq);
  pdata->Hqp.resize(nq,nq);
  pdata->fqp.resize(nq);
  pdata->Ag.resize(6,nq);
  pdata->Ak.resize(3,nq);

  pdata->state.vbasis_len = 0;
  pdata->state.cbasis_len = 0;
  pdata->state.vbasis = NULL;
  pdata->state.cbasis = NULL;

  pdata->state.t_prev = 0;
  pdata->state.vref_integrator_state = VectorXd::Zero(pdata->r->num_velocities);
  pdata->state.q_integrator_state = VectorXd::Zero(pdata->r->num_positions);
  pdata->state.foot_contact_prev[0] = false;
  pdata->state.foot_contact_prev[1] = false;
  pdata->state.num_active_contact_pts = 0;

  pdata->state.center_of_mass_observer_state = Vector4d::Zero();
  pdata->state.last_com_ddot = Vector3d::Zero();

  plhs[0] = createDrakeMexPointer((void*) pdata, "NewQPControllerData");

  return;
}












