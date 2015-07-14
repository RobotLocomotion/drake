#include "controlMexUtil.h"
#include "drakeMexUtil.h"
#include "lcmUtil.h"

using namespace std;
using namespace Eigen;

std::vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>> parseSupportData(const mxArray* supp_data) {
  double *logic_map_double;
  int nsupp = mxGetN(supp_data);
  if (mxGetM(supp_data) != 1) {
    mexErrMsgIdAndTxt("Drake:parseSupportData:BadInputs", "the support data should be a 1xN struct array");
  }
  int i, j;
  MatrixXd contact_pts;
  Vector3d contact_pt = Vector3d::Zero();
  int num_pts;
  std::vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>> supports;
  const mxArray* pm;

  for (i = 0; i < nsupp; i++) {
    SupportStateElement se;

    se.body_idx = ((int) mxGetScalar(mxGetField(supp_data, i, "body_id"))) - 1;

    num_pts = mxGetN(mxGetField(supp_data, i, "contact_pts"));
    pm = mxGetField(supp_data, i, "support_logic_map");
    if (mxIsDouble(pm)) {
      logic_map_double = mxGetPrSafe(pm);
      assert(mxGetM(pm)==4);
      for (j = 0; j < 4; j++) {
        se.support_logic_map[j] = logic_map_double[j] != 0;
      }
    } else {
      mexErrMsgTxt("Please convert support_logic_map to double");
    }
    pm = mxGetField(supp_data, i, "contact_pts");
    contact_pts.resize(mxGetM(pm), mxGetN(pm));
    memcpy(contact_pts.data(), mxGetPrSafe(pm), sizeof(double)*mxGetNumberOfElements(pm));

    for (j = 0; j < num_pts; j++) {
      contact_pt.head(3) = contact_pts.col(j);
      se.contact_pts.push_back(contact_pt);
    }
    supports.push_back(se);
  }
  return supports;
}

std::shared_ptr<drake::lcmt_qp_controller_input> encodeQPInputLCM(const mxArray *qp_input) {
  // Take a matlab data structure corresponding to a QPInputConstantHeight object and parse it down to its representation as an equivalent LCM message. 
  std::shared_ptr<drake::lcmt_qp_controller_input> msg (new drake::lcmt_qp_controller_input());

  msg->be_silent = mxGetScalar(myGetProperty(qp_input, "be_silent")) > 0.5;

  msg->timestamp = (int64_t) (mxGetScalar(myGetProperty(qp_input, "timestamp")) * 1000000);

  const mxArray* zmp_data = myGetProperty(qp_input, "zmp_data");

  matlabToCArrayOfArrays(mxGetFieldOrPropertySafe(zmp_data, 0, "A"), msg->zmp_data.A);
  matlabToCArrayOfArrays(mxGetFieldOrPropertySafe(zmp_data, 0, "B"), msg->zmp_data.B);
  matlabToCArrayOfArrays(mxGetFieldOrPropertySafe(zmp_data, 0, "C"), msg->zmp_data.C);
  matlabToCArrayOfArrays(mxGetFieldOrPropertySafe(zmp_data, 0, "D"), msg->zmp_data.D);
  matlabToCArrayOfArrays(mxGetFieldOrPropertySafe(zmp_data, 0, "x0"), msg->zmp_data.x0);
  matlabToCArrayOfArrays(mxGetFieldOrPropertySafe(zmp_data, 0, "y0"), msg->zmp_data.y0);
  matlabToCArrayOfArrays(mxGetFieldOrPropertySafe(zmp_data, 0, "u0"), msg->zmp_data.u0);
  matlabToCArrayOfArrays(mxGetFieldOrPropertySafe(zmp_data, 0, "R"), msg->zmp_data.R);
  matlabToCArrayOfArrays(mxGetFieldOrPropertySafe(zmp_data, 0, "Qy"), msg->zmp_data.Qy);
  matlabToCArrayOfArrays(mxGetFieldOrPropertySafe(zmp_data, 0, "S"), msg->zmp_data.S);
  matlabToCArrayOfArrays(mxGetFieldOrPropertySafe(zmp_data, 0, "s1"), msg->zmp_data.s1);
  matlabToCArrayOfArrays(mxGetFieldOrPropertySafe(zmp_data, 0, "s1dot"), msg->zmp_data.s1dot);
  msg->zmp_data.s2 = mxGetScalar(mxGetFieldSafe(zmp_data, "s2"));
  msg->zmp_data.s2dot = mxGetScalar(mxGetFieldSafe(zmp_data, "s2dot"));
  msg->zmp_data.timestamp = msg->timestamp;


  const mxArray* support_data = myGetProperty(qp_input, "support_data");
  int nsupp = mxGetN(support_data);
  msg->num_support_data = (int32_t) nsupp;
  double double_logic_map[4][1];
  msg->support_data.resize(nsupp);
  if (nsupp > 0) {
    if (mxGetM(support_data) != 1) {
      mexErrMsgTxt("support data should be a struct array with M=1");
    }
    for (int i=0; i < nsupp; i++) {
      msg->support_data[i].timestamp = msg->timestamp;
      msg->support_data[i].body_id = (int32_t) mxGetScalar(myGetField(support_data, i, "body_id"));

      const mxArray *contact_pts = myGetField(support_data, i, "contact_pts");
      if (!contact_pts) mexErrMsgTxt("couldn't get points");
      Map<MatrixXd>contact_pts_mat(mxGetPrSafe(contact_pts), mxGetM(contact_pts), mxGetN(contact_pts));
      msg->support_data[i].num_contact_pts = (int32_t) mxGetN(contact_pts);
      msg->support_data[i].contact_pts.resize(3);
      for (int j=0; j < 3; j++) {
        msg->support_data[i].contact_pts[j].resize(msg->support_data[i].num_contact_pts);
        for (int k=0; k < msg->support_data[i].num_contact_pts; k++) {
          msg->support_data[i].contact_pts[j][k] = contact_pts_mat(j, k);
        }
      }

      matlabToCArrayOfArrays(mxGetFieldOrPropertySafe(support_data, i, "support_logic_map"), double_logic_map);
      for (int j=0; j < 4; j++) {
        msg->support_data[i].support_logic_map[j] = (double_logic_map[j][0] != 0);
      }
      msg->support_data[i].mu = mxGetScalar(myGetField(support_data, i, "mu"));
      
      double use_support_surface_dbl = mxGetScalar(myGetField(support_data, i, "use_support_surface"));
      msg->support_data[i].use_support_surface = (use_support_surface_dbl != 0);

      const mxArray *support_surface = myGetField(support_data, i, "support_surface");
      if (!support_surface) mexErrMsgTxt("couldn't get support surface");
      Map<Vector4d>support_surface_vec(mxGetPrSafe(support_surface));
      for (int j=0; j < 4; j++) {
        msg->support_data[i].support_surface[j] = support_surface_vec(j);
      }
    }
  }

  const mxArray* body_motion_data = myGetProperty(qp_input, "body_motion_data");
  const int nbod = mxGetN(body_motion_data);
  msg->num_tracked_bodies = nbod;
  msg->body_motion_data.resize(nbod);
  if (nbod > 0) {
    if (mxGetM(body_motion_data) != 1) {
      mexErrMsgTxt("body motion data should be a 1xN struct array");
    }
    for (int i=0; i < nbod; i++) {
      msg->body_motion_data[i].timestamp = msg->timestamp;
      msg->body_motion_data[i].body_id = (int32_t) mxGetScalar(mxGetFieldSafe(body_motion_data, i, "body_id"));

      PiecewisePolynomial<double> spline = matlabToPiecewisePolynomial(body_motion_data, i);
      encodePiecewisePolynomial(spline, msg->body_motion_data[i].spline);

      msg->body_motion_data[i].in_floating_base_nullspace = static_cast<bool>(mxGetScalar(mxGetFieldSafe(body_motion_data, i, "in_floating_base_nullspace")));
      msg->body_motion_data[i].control_pose_when_in_contact = static_cast<bool>(mxGetScalar(mxGetFieldSafe(body_motion_data, i, "control_pose_when_in_contact")));
      const mxArray* quat_task_to_world = mxGetFieldSafe(body_motion_data, i, "quat_task_to_world"); 
      sizecheck(quat_task_to_world,4,1);
      memcpy(msg->body_motion_data[i].quat_task_to_world,mxGetPrSafe(quat_task_to_world),sizeof(double)*4);
      const mxArray* translation_task_to_world = mxGetFieldSafe(body_motion_data, i, "translation_task_to_world");
      sizecheck(translation_task_to_world,3,1);
      memcpy(msg->body_motion_data[i].translation_task_to_world, mxGetPrSafe(translation_task_to_world),sizeof(double)*3);
      const mxArray* xyz_kp_multiplier = mxGetFieldSafe(body_motion_data, i, "xyz_kp_multiplier");
      sizecheck(xyz_kp_multiplier,3,1);
      memcpy(msg->body_motion_data[i].xyz_kp_multiplier, mxGetPrSafe(xyz_kp_multiplier), sizeof(double)*3);
      const mxArray* xyz_damping_ratio_multiplier = mxGetFieldSafe(body_motion_data, i, "xyz_damping_ratio_multiplier");
      sizecheck(xyz_damping_ratio_multiplier,3,1);
      memcpy(msg->body_motion_data[i].xyz_damping_ratio_multiplier, mxGetPrSafe(xyz_damping_ratio_multiplier), sizeof(double)*3);
      const mxArray* expmap_kp_multiplier = mxGetFieldSafe(body_motion_data, i, "expmap_kp_multiplier");
      sizecheck(expmap_kp_multiplier,1,1);
      msg->body_motion_data[i].expmap_kp_multiplier = mxGetScalar(expmap_kp_multiplier);
      const mxArray* expmap_damping_ratio_multiplier = mxGetFieldSafe(body_motion_data, i, "expmap_damping_ratio_multiplier");
      sizecheck(expmap_damping_ratio_multiplier,1,1);
      msg->body_motion_data[i].expmap_damping_ratio_multiplier = mxGetScalar(expmap_damping_ratio_multiplier);
      const mxArray* weight_multiplier = mxGetFieldSafe(body_motion_data, i, "weight_multiplier");
      sizecheck(weight_multiplier,6,1);
      memcpy(msg->body_motion_data[i].weight_multiplier, mxGetPrSafe(weight_multiplier), sizeof(double)*6);
    }
  }

  const mxArray* body_wrench_data = myGetProperty(qp_input, "body_wrench_data");
  const int num_external_wrenches = mxGetN(body_wrench_data);
  msg->num_external_wrenches = num_external_wrenches;
  msg->body_wrench_data.resize(num_external_wrenches);
  const int wrench_size = 6;
  if (num_external_wrenches > 0) {
    if (mxGetM(body_wrench_data) != 1) {
      mexErrMsgTxt("body wrench data should be a 1xN struct array");
    }
    for (int i = 0; i < num_external_wrenches; i++) {
      msg->body_wrench_data[i].timestamp = msg->timestamp;
      msg->body_wrench_data[i].body_id = (int32_t) mxGetScalar(myGetField(body_wrench_data, i, "body_id"));
      const mxArray* wrench = myGetField(body_wrench_data, i, "wrench");
      sizecheck(wrench, wrench_size, 1);
      memcpy(msg->body_wrench_data[i].wrench, mxGetPrSafe(wrench), wrench_size * sizeof(double));
    }
  }

  const mxArray* whole_body_data = myGetProperty(qp_input, "whole_body_data");
  if (mxGetN(whole_body_data) != 1 || mxGetM(whole_body_data) != 1) mexErrMsgTxt("whole_body_data should be a 1x1 struct");
  const mxArray* q_des = myGetField(whole_body_data, "q_des");
  if (mxGetN(q_des) != 1) mexErrMsgTxt("q_des should be a column vector");
  const int npos = mxGetM(q_des);
  msg->whole_body_data.timestamp = msg->timestamp;
  msg->whole_body_data.num_positions = npos;
  Map<VectorXd>q_des_vec(mxGetPrSafe(q_des), npos);
  msg->whole_body_data.q_des.resize(npos);

  for (int i=0; i < npos; i++) {
    msg->whole_body_data.q_des[i] = q_des_vec(i);
  }

  const mxArray* condof = myGetField(whole_body_data, "constrained_dofs");
  const int ncons = mxGetNumberOfElements(condof);
  msg->whole_body_data.num_constrained_dofs = ncons;
  msg->whole_body_data.constrained_dofs.resize(ncons);
  if (ncons > 0) {
    if (mxGetN(condof) != 1) mexErrMsgTxt("constrained dofs should be a column vector");
    Map<VectorXd>condof_vec(mxGetPrSafe(condof), ncons);

    for (int i=0; i < ncons; i++) {
      msg->whole_body_data.constrained_dofs[i] = static_cast<int32_t>(condof_vec(i));
    }
  }

  const mxArray* joint_override = myGetProperty(qp_input, "joint_pd_override");
  int num_joint_pd_overrides = mxGetNumberOfElements(joint_override);
  msg->num_joint_pd_overrides = num_joint_pd_overrides;
  msg->joint_pd_override.resize(num_joint_pd_overrides);
  for (int i=0; i < num_joint_pd_overrides; i++) {
    msg->joint_pd_override[i].position_ind = (int32_t) mxGetScalar(myGetField(joint_override, i, "position_ind"));
    msg->joint_pd_override[i].qi_des = mxGetScalar(myGetField(joint_override, i, "qi_des"));
    msg->joint_pd_override[i].qdi_des = mxGetScalar(myGetField(joint_override, i, "qdi_des"));
    msg->joint_pd_override[i].kp = mxGetScalar(myGetField(joint_override, i, "kp"));
    msg->joint_pd_override[i].kd = mxGetScalar(myGetField(joint_override, i, "kd"));
    msg->joint_pd_override[i].weight = mxGetScalar(myGetField(joint_override, i, "weight"));
  }

  msg->param_set_name = mxArrayToString(myGetProperty(qp_input, "param_set_name"));
  return msg;
}

PiecewisePolynomial<double> matlabToPiecewisePolynomial(const mxArray* pobj, int index) {
  std::vector<double> ts = matlabToStdVector<double>(mxGetField(pobj, index, "ts"));
  const int num_segments = ts.size() - 1;
  const mxArray *coefs = mxGetFieldSafe(pobj, index, "coefs");
  if (mxGetNumberOfDimensions(coefs) != 3) mexErrMsgTxt("coefs should be a dimension-3 array");
  const mwSize* dim = mxGetDimensions(coefs);
  if (dim[0] != 6 || dim[1] != num_segments || dim[2] != 4) mexErrMsgTxt("coefs should be size 6xNx4, where N is the number of spline segments");

  Map<VectorXd> coefs_flat(mxGetPrSafe(coefs), dim[0] * dim[1] * dim[2]);
  // coefs_flat[i + j * dim[0] + k * dim[0] * dim[1]] = coefs(i, j, k);
  std::vector<Matrix<Polynomial<double>, Dynamic, Dynamic>> poly_matrices;
  poly_matrices.reserve(num_segments);
  for (int j=0; j < num_segments; ++j) {
    Matrix<Polynomial<double>, Dynamic, Dynamic> poly_matrix(6, 1);
    for (int i=0; i < 6; ++i) {
      poly_matrix(i) = Polynomial<double>(Vector4d(coefs_flat(i + j * 6 + 3 * 6 * num_segments),
                                                   coefs_flat(i + j * 6 + 2 * 6 * num_segments),
                                                   coefs_flat(i + j * 6 + 1 * 6 * num_segments),
                                                   coefs_flat(i + j * 6 + 0 * 6 * num_segments)));
    }
    poly_matrices.push_back(poly_matrix);
  }
  return PiecewisePolynomial<double>(poly_matrices, ts);
}

void parsePositionIndices(const mxArray *pobj, std::map<std::string, VectorXi> &position_indices) {
  int num_fields = mxGetNumberOfFields(pobj);
  for (int i=0; i < num_fields; ++i) {
    const mxArray* pfield = mxGetFieldByNumber(pobj, 0, i);
    Map<VectorXd> indices_double(mxGetPrSafe(pfield), mxGetNumberOfElements(pfield));
    VectorXi indices = indices_double.cast<int> ();
    position_indices[std::string(mxGetFieldNameByNumber(pobj, i))] = indices.array() - 1;
  }
  return;
}

void parseContactGroups(const mxArray* pobj, std::vector<RobotPropertyCache::ContactGroupNameToContactPointsMap> &contact_groups) {
  const int num_groups = mxGetNumberOfElements(pobj);
  contact_groups.reserve(num_groups);

  for (int i=0; i < num_groups; ++i) {
    RobotPropertyCache::ContactGroupNameToContactPointsMap groups;
    const mxArray* cell_obj = mxGetCell(pobj, i);
    int num_fields = mxGetNumberOfFields(cell_obj);
    for (int j=0; j < num_fields; ++j) {
      const mxArray* pfield = mxGetFieldByNumber(cell_obj, 0, j);
      if (mxGetM(pfield) != 3) {
        mexErrMsgTxt("expected contact points of size 3xN");
      }
      Map<Matrix<double, 3, Dynamic>> pts(mxGetPrSafe(pfield), 3, mxGetN(pfield));
      groups[std::string(mxGetFieldNameByNumber(cell_obj, j))] = pts;
    }
    contact_groups.push_back(groups);
  }
  return;
}

void parseRobotPropertyCache(const mxArray *rpc_obj, RobotPropertyCache *rpc) {
  const mxArray *pobj;

  parsePositionIndices(mxGetFieldSafe(rpc_obj, "position_indices"), rpc->position_indices);
  parseContactGroups(mxGetFieldSafe(rpc_obj, "contact_groups"), rpc->contact_groups);

  rpc->body_ids.r_foot = (int) mxGetScalar(myGetField(myGetField(rpc_obj, "body_ids"), "r_foot")) - 1;
  rpc->body_ids.l_foot = (int) mxGetScalar(myGetField(myGetField(rpc_obj, "body_ids"), "l_foot")) - 1;
  rpc->body_ids.pelvis = (int) mxGetScalar(myGetField(myGetField(rpc_obj, "body_ids"), "pelvis")) - 1;

  pobj = myGetField(rpc_obj, "actuated_indices");
  Map<VectorXd>actuated_indices(mxGetPrSafe(pobj), mxGetNumberOfElements(pobj));
  rpc->actuated_indices = actuated_indices.cast<int>().array() - 1;

  pobj = myGetField(rpc_obj, "num_bodies");
  rpc->num_bodies = (int) mxGetScalar(pobj);

  return;
}

mxArray* myGetProperty(const mxArray* pobj, const char* propname)
{
  mxArray* pm = mxGetProperty(pobj,0,propname);
  if (!pm) mexErrMsgIdAndTxt("Drake:controlMexUtil:BadInput","ControlUtil is trying to load object property '%s', but failed.", propname);
  return pm;
}

mxArray* myGetField(const mxArray* pobj, const int idx, const char* propname)
{
  mxArray* pm = mxGetField(pobj,idx,propname);
  if (!pm) mexErrMsgIdAndTxt("Drake:controlMexUtil:BadInput","ControlUtil is trying to load object field '%s', but failed.", propname);
  return pm;
}

mxArray* myGetField(const mxArray* pobj, const char* propname)
{
  mxArray* pm = myGetField(pobj, 0, propname);
  return pm;
}

// convert Matlab cell array of strings into a C++ vector of strings
std::vector<std::string> get_strings(const mxArray *rhs) {
  int num = mxGetNumberOfElements(rhs);
  std::vector<std::string> strings(num);
  for (int i=0; i<num; i++) {
    // const mxArray *ptr = mxGetCell(rhs,i);
    strings[i] = std::string(mxArrayToString(mxGetCell(rhs, i)));
    // int buflen = mxGetN(ptr)*sizeof(mxChar)+1;
    // char* str = (char*)mxMalloc(buflen);
    // mxGetString(ptr, str, buflen);
    // strings[i] = string(str);
    // mxFree(str);
  }
  return strings;
}
