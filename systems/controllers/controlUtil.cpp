#include "controlUtil.h"
#include "drakeUtil.h"
#include "mex.h"

template <typename DerivedA, typename DerivedB>
void getRows(std::set<int> &rows, MatrixBase<DerivedA> const &M, MatrixBase<DerivedB> &Msub)
{
  if (rows.size()==M.rows()) {
    Msub = M; 
    return;
  }
  
  int i=0;
  for (std::set<int>::iterator iter=rows.begin(); iter!=rows.end(); iter++)
    Msub.row(i++) = M.row(*iter);
}

template <typename DerivedA, typename DerivedB>
void getCols(std::set<int> &cols, MatrixBase<DerivedA> const &M, MatrixBase<DerivedB> &Msub)
{
  if (cols.size()==M.cols()) {
    Msub = M;
    return;
  }
  int i=0;
  for (std::set<int>::iterator iter=cols.begin(); iter!=cols.end(); iter++)
    Msub.col(i++) = M.col(*iter);
}

template <typename DerivedPhi1, typename DerivedPhi2, typename DerivedD>
void angleDiff(MatrixBase<DerivedPhi1> const &phi1, MatrixBase<DerivedPhi2> const &phi2, MatrixBase<DerivedD> &d) {
  d = phi2 - phi1;
  
  for (int i = 0; i < phi1.rows(); i++) {
    for (int j = 0; j < phi1.cols(); j++) {
      if (d(i,j) < -M_PI) {
        d(i,j) = fmod(d(i,j) + M_PI, 2*M_PI) + M_PI;
      } else {
        d(i,j) = fmod(d(i,j) + M_PI, 2*M_PI) - M_PI;
      }
    }
  }
}

mxArray* myGetProperty(const mxArray* pobj, const char* propname)
{
  mxArray* pm = mxGetProperty(pobj,0,propname);
  if (!pm) mexErrMsgIdAndTxt("DRC:ControlUtil:BadInput","ControlUtil is trying to load object property '%s', but failed.", propname);
  return pm;
}

mxArray* myGetField(const mxArray* pobj, const int idx, const char* propname)
{
  mxArray* pm = mxGetField(pobj,idx,propname);
  if (!pm) mexErrMsgIdAndTxt("DRC:ControlUtil:BadInput","ControlUtil is trying to load object field '%s', but failed.", propname);
  return pm;
}

mxArray* myGetField(const mxArray* pobj, const char* propname)
{
  mxArray* pm = myGetField(pobj, 0, propname);
  return pm;
}

bool inSupport(std::vector<SupportStateElement> &supports, int body_idx) {
  // HANDLE IF BODY_IDX IS A FRAME ID?
  for (int i=0; i<supports.size(); i++) {
    if (supports[i].body_idx == body_idx)
      return true;
  }
  return false;
}

void collisionDetect(void* map_ptr, Vector3d const &contact_pos, Vector3d &pos, Vector3d *normal, double terrain_height)
{
  if (map_ptr) {
#ifdef USE_MAPS    
    Vector3d oNormal;
    double height;
    auto state = static_cast<terrainmap::TerrainMap*>(map_ptr);
    if (state != NULL) {
      state->getHeightAndNormal(contact_pos(0), contact_pos(1), height, oNormal);
      pos << contact_pos.topRows(2), height;
      if (normal) {
        *normal = oNormal.cast<double>();
        return;
      }
    }
#endif      
  } else {
//    mexPrintf("Warning: using 0,0,1 as normal\n");
    pos << contact_pos.topRows(2), terrain_height;
    if (normal) *normal << 0,0,1;
  }
}

void surfaceTangents(const Vector3d & normal, Matrix<double,3,m_surface_tangents> & d)
{
  Vector3d t1,t2;
  double theta;
  
  if (1 - normal(2) < EPSILON) { // handle the unit-normal case (since it's unit length, just check z)
    t1 << 1,0,0;
  } else if(1 + normal(2) < EPSILON) {
    t1 << -1,0,0;  //same for the reflected case
  } else {// now the general case
  t1 << normal(1), -normal(0) , 0;
    t1 /= sqrt(normal(1)*normal(1) + normal(0)*normal(0));
  }
      
  t2 = t1.cross(normal);
      
  for (int k=0; k<m_surface_tangents; k++) {
    theta = k*M_PI/m_surface_tangents;
    d.col(k)=cos(theta)*t1 + sin(theta)*t2;
  }
}

int contactPhi(RigidBodyManipulator* r, SupportStateElement& supp, void *map_ptr, VectorXd &phi, double terrain_height)
{
  int nc = static_cast<int>(supp.contact_pts.size());
  phi.resize(nc);

  if (nc<1) return nc;

  Vector3d contact_pos,pos,posB,normal;

  int i=0;
  for (std::vector<Vector4d,aligned_allocator<Vector4d>>::iterator pt_iter=supp.contact_pts.begin(); pt_iter!=supp.contact_pts.end(); pt_iter++) {
    
    r->forwardKin(supp.body_idx,*pt_iter,0,contact_pos);
    collisionDetect(map_ptr,contact_pos,pos,NULL,terrain_height);
    pos -= contact_pos;  // now -rel_pos in matlab version
    
    phi(i) = pos.norm();
    if (pos.dot(normal)>0)
      phi(i)=-phi(i);
    i++;
  }
  return nc;
}

int contactConstraints(RigidBodyManipulator *r, int nc, std::vector<SupportStateElement>& supp, void *map_ptr, MatrixXd &n, MatrixXd &D, MatrixXd &Jp, MatrixXd &Jpdot,double terrain_height)
{
  int j, k=0, nq = r->num_positions;

  n.resize(nc,nq);
  D.resize(nq,nc*2*m_surface_tangents);
  Jp.resize(3*nc,nq);
  Jpdot.resize(3*nc,nq);
  
  Vector3d contact_pos,pos,posB,normal;
  MatrixXd J(3,nq);
  Matrix<double,3,m_surface_tangents> d;
  
  for (std::vector<SupportStateElement>::iterator iter = supp.begin(); iter!=supp.end(); iter++) {
    if (nc>0) {
      for (std::vector<Vector4d,aligned_allocator<Vector4d>>::iterator pt_iter=iter->contact_pts.begin(); pt_iter!=iter->contact_pts.end(); pt_iter++) {
        r->forwardKin(iter->body_idx,*pt_iter,0,contact_pos);
        r->forwardJac(iter->body_idx,*pt_iter,0,J);

        collisionDetect(map_ptr,contact_pos,pos,&normal,terrain_height);
        surfaceTangents(normal,d);

        n.row(k) = normal.transpose()*J;
        for (j=0; j<m_surface_tangents; j++) {
          D.col(2*k*m_surface_tangents+j) = J.transpose()*d.col(j);
          D.col((2*k+1)*m_surface_tangents+j) = -D.col(2*k*m_surface_tangents+j);
        }

        // store away kin sols into Jp and Jpdot
        // NOTE: I'm cheating and using a slightly different ordering of J and Jdot here
        Jp.block(3*k,0,3,nq) = J;
        r->forwardJacDot(iter->body_idx,*pt_iter,0,J);
        Jpdot.block(3*k,0,3,nq) = J;
        
        k++;
      }
    }
  }
  
  return k;
}


int contactConstraintsBV(RigidBodyManipulator *r, int nc, double mu, std::vector<SupportStateElement>& supp, void *map_ptr, MatrixXd &B, MatrixXd &JB, MatrixXd &Jp, VectorXd &Jpdotv, MatrixXd &normals, double terrain_height)
{
  int j, k=0, nq = r->num_positions;

  B.resize(3,nc*2*m_surface_tangents);
  JB.resize(nq,nc*2*m_surface_tangents);
  Jp.resize(3*nc,nq);
  Jpdotv.resize(3*nc);
  normals.resize(3, nc);
  
  Vector3d contact_pos,pos,posB,normal; 
  MatrixXd J(3,nq);
  Matrix<double,3,m_surface_tangents> d;
  double norm = sqrt(1+mu*mu); // because normals and ds are orthogonal, the norm has a simple form
  
  for (std::vector<SupportStateElement>::iterator iter = supp.begin(); iter!=supp.end(); iter++) {
    if (nc>0) {
      for (std::vector<Vector4d,aligned_allocator<Vector4d>>::iterator pt_iter=iter->contact_pts.begin(); pt_iter!=iter->contact_pts.end(); pt_iter++) {
        r->forwardKin(iter->body_idx,*pt_iter,0,contact_pos);
        r->forwardJac(iter->body_idx,*pt_iter,0,J);

        collisionDetect(map_ptr,contact_pos,pos,&normal,terrain_height);
        surfaceTangents(normal,d);
        for (j=0; j<m_surface_tangents; j++) {
          B.col(2*k*m_surface_tangents+j) = (normal + mu*d.col(j)) / norm; 
          B.col((2*k+1)*m_surface_tangents+j) = (normal - mu*d.col(j)) / norm; 
    
          JB.col(2*k*m_surface_tangents+j) = J.transpose()*B.col(2*k*m_surface_tangents+j);
          JB.col((2*k+1)*m_surface_tangents+j) = J.transpose()*B.col((2*k+1)*m_surface_tangents+j);
        }

        // store away kin sols into Jp and Jpdotv
        // NOTE: I'm cheating and using a slightly different ordering of J and Jdot here
        Jp.block(3*k,0,3,nq) = J;
        Vector3d pt = (*pt_iter).head(3);
        auto Jpdotv_grad = r->forwardJacDotTimesV(pt,iter->body_idx,0,0,0);
        Jpdotv.block(3*k,0,3,1) = Jpdotv_grad.value();
        normals.col(k) = normal;
        
        k++;
      }
    }
  }
  
  return k;
}

MatrixXd individualSupportCOPs(RigidBodyManipulator* r, const std::vector<SupportStateElement>& active_supports,
    const MatrixXd& normals, const MatrixXd& B, const VectorXd& beta)
{
  const int n_basis_vectors_per_contact = static_cast<int>(B.cols() / normals.cols());
  const int n = static_cast<int>(active_supports.size());

  int normals_start = 0;
  int beta_start = 0;

  MatrixXd individual_cops(3, n);
  individual_cops.fill(std::numeric_limits<double>::quiet_NaN());

  for (int j = 0; j < active_supports.size(); j++) {
    auto active_support = active_supports[j];
    auto contact_pts = active_support.contact_pts;

    int ncj = static_cast<int>(contact_pts.size());
    int active_support_length = n_basis_vectors_per_contact * ncj;
    auto normalsj = normals.middleCols(normals_start, ncj);
    Vector3d normal = normalsj.col(0);
    bool normals_identical = (normalsj.colwise().operator-(normal)).squaredNorm() < 1e-15;

    if (normals_identical) { // otherwise computing a COP doesn't make sense
      const auto& Bj = B.middleCols(beta_start, active_support_length);
      const auto& betaj = beta.segment(beta_start, active_support_length);

      const auto& contact_positions = r->bodies[active_support.body_idx]->contact_pts;
      Vector3d force = Vector3d::Zero();
      Vector3d torque = Vector3d::Zero();

      for (int k = 0; k < contact_pts.size(); k++) {
      // for (auto k = contact_pts.begin(); k!= contact_pts.end(); k++) { 
        const auto& Bblock = Bj.middleCols(k * n_basis_vectors_per_contact, n_basis_vectors_per_contact);
        const auto& betablock = betaj.segment(k * n_basis_vectors_per_contact, n_basis_vectors_per_contact);
        Vector3d point_force = Bblock * betablock;
        force += point_force;
        Vector3d contact_pt = contact_pts[k].head(3);
        auto torquejk = contact_pt.cross(point_force);
        torque += torquejk;
      }

      Vector3d point_on_contact_plane = contact_positions.col(0);
      std::pair<Vector3d, double> cop_and_normal_torque = resolveCenterOfPressure(torque, force, normal, point_on_contact_plane);
      Vector4d cop_body;
      cop_body << cop_and_normal_torque.first, 1.0;
      Vector3d cop_world;
      r->forwardKin(active_support.body_idx, cop_body, 0, cop_world);
      individual_cops.col(j) = cop_world;
    }

    normals_start += ncj;
    beta_start += active_support_length;
  }
  return individual_cops;
}

std::vector<SupportStateElement> parseSupportData(const mxArray* supp_data) {
  double *logic_map_double;
  int nsupp = mxGetN(supp_data);
  if (mxGetM(supp_data) != 1) {
    mexErrMsgIdAndTxt("Drake:parseSupportData:BadInputs", "the support data should be a 1xN struct array");
  }
  int i, j;
  MatrixXd contact_pts;
  Vector4d contact_pt = Vector4d::Zero();
  contact_pt(3) = 1.0;
  int num_pts;
  std::vector<SupportStateElement> supports;
  const mxArray* pm;

  for (i = 0; i < nsupp; i++) {
    SupportStateElement se;

    se.body_idx = ((int) mxGetScalar(mxGetField(supp_data, i, "body_id"))) - 1;

    num_pts = mxGetN(mxGetField(supp_data, i, "contact_pts"));
    pm = mxGetField(supp_data, i, "support_logic_map");
    if (mxIsDouble(pm)) {
      logic_map_double = mxGetPrSafe(pm);
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
    se.contact_surface = ((int) mxGetScalar(mxGetField(supp_data, i, "contact_surfaces"))) - 1;
    supports.push_back(se);
  }
  return supports;
}

bool isSupportElementActive(SupportStateElement* se, bool contact_force_detected, bool kinematic_contact_detected) {
  bool is_active;

  // Implement the logic described in QPInputConstantHeight.m
  if (!contact_force_detected && !kinematic_contact_detected) {
    is_active = se->support_logic_map[0]; 
  } else if (!contact_force_detected && kinematic_contact_detected) {
    is_active = se->support_logic_map[1];
  } else if (contact_force_detected && !kinematic_contact_detected) {
    is_active = se->support_logic_map[2];
  } else  { // (contact_force_detected && kinematic_contact_detected)
    is_active = se->support_logic_map[3];
  }
  return is_active;
}

Matrix<bool, Dynamic, 1> getActiveSupportMask(RigidBodyManipulator* r, void* map_ptr, VectorXd q, VectorXd qd, std::vector<SupportStateElement> &available_supports, const Ref<const Matrix<bool, Dynamic, 1>> &contact_force_detected, double contact_threshold, double terrain_height) {
  r->doKinematics(q, false, qd);

  int nsupp = available_supports.size();
  Matrix<bool, Dynamic, 1> active_supp_mask = Matrix<bool, Dynamic, 1>::Zero(nsupp);
  VectorXd phi;
  SupportStateElement se;
  bool needs_kin_check;
  bool kin_contact;
  bool force_contact;

  for (int i = 0; i < nsupp; i++) {
    // mexPrintf("evaluating support: %d\n", i);
    se = available_supports[i];

    force_contact = (contact_force_detected(se.body_idx) != 0);
    // Determine if the body needs to be checked for kinematic contact. We only
    // need to check for kin contact if the logic map indicates that the
    // presence or absence of such contact would  affect the decision about
    // whether to use that body as a support.
    needs_kin_check = (((se.support_logic_map[1] != se.support_logic_map[0]) && (contact_force_detected(se.body_idx) == 0)) ||
                       ((se.support_logic_map[3] != se.support_logic_map[2]) && (contact_force_detected(se.body_idx) == 1)));


    if (needs_kin_check) {
      if (contact_threshold == -1) {
        kin_contact = true;
      } else {
        contactPhi(r,se,map_ptr,phi,terrain_height);
        kin_contact = (phi.minCoeff()<=contact_threshold);
      }
    } else {
      kin_contact = false; // we've determined already that kin contact doesn't matter for this support element
    }

    active_supp_mask(i) = isSupportElementActive(&se, force_contact, kin_contact);
    // mexPrintf("needs check: %d force contact: %d kin_contact: %d is_active: %d\n", needs_kin_check, force_contact, kin_contact, active_supp_mask(i));
  }
  return active_supp_mask;
}

std::vector<SupportStateElement> getActiveSupports(RigidBodyManipulator* r, void* map_ptr, VectorXd q, VectorXd qd, std::vector<SupportStateElement> &available_supports, const Ref<const Matrix<bool, Dynamic, 1>> &contact_force_detected, double contact_threshold, double terrain_height) {

  Matrix<bool, Dynamic, 1> active_supp_mask = getActiveSupportMask(r, map_ptr, q, qd, available_supports, contact_force_detected, contact_threshold, terrain_height);

  std::vector<SupportStateElement> active_supports;

  for (int i=0; i < available_supports.size(); i++) {
    if (active_supp_mask(i)) {
      active_supports.push_back(available_supports[i]);
    }
  }
  return active_supports;
}

Vector6d bodySpatialMotionPD(RigidBodyManipulator *r, DrakeRobotState &robot_state, const int body_index, const Isometry3d &body_pose_des, const Ref<const Vector6d> &body_v_des, const Ref<const Vector6d> &body_vdot_des, const Ref<const Vector6d> &Kp, const Ref<const Vector6d> &Kd, const Isometry3d &T_task_to_world)
{
  // @param body_pose_des  desired pose in the task frame, this is the homogeneous transformation from desired body frame to task frame 
  // @param body_v_des    desired [xyzdot;angular_velocity] in task frame
  // @param body_vdot_des    desired [xyzddot;angular_acceleration] in task frame
  // @param Kp     The gain in task frame
  // @param Kd     The gain in task frame 
  // @param T_task_to_world  The homogeneous transform from task to world
  // @retval twist_dot, [angular_acceleration, xyz_acceleration] in body frame

  if(!r->getUseNewKinsol())
  {
    throw std::runtime_error("bodySpatialMotionPD requires new kinsol format");
  }
  Isometry3d T_world_to_task = T_task_to_world.inverse();
  r->doKinematicsNew(robot_state.q,robot_state.qd,false);

  Vector3d origin = Vector3d::Zero();
  auto body_pose = r->forwardKinNew(origin,body_index,0,2,0);
  Vector3d body_xyz = body_pose.value().head<3>();
  Vector3d body_xyz_task = T_world_to_task * body_xyz.colwise().homogeneous();
  Vector4d body_quat = body_pose.value().tail<4>();
  std::vector<int> v_indices;
  auto J_geometric = r->geometricJacobian<double>(0,body_index,body_index,0,true,&v_indices);
  VectorXd v_compact(v_indices.size());
  for(int i = 0;i<v_indices.size();i++)
  {
    v_compact(i) = robot_state.qd(v_indices[i]);
  }
  Vector6d body_twist = J_geometric.value() * v_compact;
  Matrix3d R_body_to_world = quat2rotmat(body_quat);
  Matrix3d R_world_to_body = R_body_to_world.transpose();
  Matrix3d R_body_to_task = T_world_to_task.linear() * R_body_to_world;
  Vector3d body_angular_vel = R_body_to_world * body_twist.head<3>();// body_angular velocity in world frame
  Vector3d body_xyzdot = R_body_to_world * body_twist.tail<3>();// body_xyzdot in world frame
  Vector3d body_angular_vel_task = T_world_to_task.linear() * body_angular_vel;
  Vector3d body_xyzdot_task = T_world_to_task.linear() * body_xyzdot;

  Vector3d body_xyz_des = body_pose_des.translation();
  Vector3d body_angular_vel_des = body_v_des.tail<3>();
  Vector3d body_angular_vel_dot_des = body_vdot_des.tail<3>();

  Vector3d xyz_err_task = body_xyz_des-body_xyz_task;

  Matrix3d R_des = body_pose_des.linear();
  Matrix3d R_err_task = R_des * R_body_to_task.transpose();
  Vector4d angleAxis_err_task = rotmat2axis(R_err_task); 
  Vector3d angular_err_task = angleAxis_err_task.head<3>() * angleAxis_err_task(3);

  Vector3d xyzdot_err_task = body_v_des.head<3>() - body_xyzdot_task;
  Vector3d angular_vel_err_task = body_angular_vel_des - body_angular_vel_task;

  Vector3d Kp_xyz = Kp.head<3>();
  Vector3d Kd_xyz = Kd.head<3>();
  Vector3d Kp_angular = Kp.tail<3>();
  Vector3d Kd_angular = Kd.tail<3>();
  Vector3d body_xyzddot_task = (Kp_xyz.array() * xyz_err_task.array()).matrix() + (Kd_xyz.array() * xyzdot_err_task.array()).matrix() + body_vdot_des.head<3>();
  Vector3d body_angular_vel_dot_task = (Kp_angular.array() * angular_err_task.array()).matrix() + (Kd_angular.array() * angular_vel_err_task.array()).matrix() + body_angular_vel_dot_des;
  
  Vector6d twist_dot = Vector6d::Zero();
  Vector3d body_xyzddot = T_task_to_world.linear() * body_xyzddot_task;
  Vector3d body_angular_vel_dot = T_task_to_world.linear() * body_angular_vel_dot_task;
  twist_dot.head<3>() = R_world_to_body * body_angular_vel_dot;
  twist_dot.tail<3>() = R_world_to_body * body_xyzddot - body_twist.head<3>().cross(body_twist.tail<3>());
  return twist_dot;
}

void evaluateCubicSplineSegment(double t, const Ref<const Matrix<double, 6, 4>> &coefs, Vector6d &y, Vector6d &ydot, Vector6d &yddot) {
  // evaluate a cubic spline with coefficients coef and starting time 0 at time t
  y = coefs.col(0) * pow(t, 3) + coefs.col(1) * pow(t, 2) + coefs.col(2) * t + coefs.col(3);
  ydot = 3*coefs.col(0) * pow(t,2) + 2*coefs.col(1)*t + coefs.col(2);
  yddot = 6*coefs.col(0)*t + 2*coefs.col(1);
}

void evaluateXYZExpmapCubicSplineSegment(double t, const Matrix<double,6,4> &coefs, Isometry3d &body_pose_des, Vector6d &xyzdot_angular_vel, Vector6d &xyzddot_angular_accel)
{
  double t2 = t*t;
  double t3 = pow(t,3);
  Vector6d xyzexp = coefs.col(0)*t3 + coefs.col(1)*t2 + coefs.col(2)*t + coefs.col(3);
  Vector6d xyzexpdot = coefs.col(0)*3.0*t2 + coefs.col(1)*2.0*t + coefs.col(2);
  Vector6d xyzexpddot = coefs.col(0)*6.0*t + coefs.col(1)*2.0;
  xyzdot_angular_vel.head<3>() = xyzexpdot.head<3>();
  xyzddot_angular_accel.head<3>() = xyzexpddot.head<3>();
  Vector3d expmap = xyzexp.tail<3>();
  auto quat_grad = expmap2quat(expmap,2);
  Vector4d quat = quat_grad.value();
  body_pose_des.linear() = quat2rotmat(quat);
  body_pose_des.translation() = xyzexp.head<3>();
  Vector4d quat_dot = quat_grad.gradient().value() * xyzexpdot.tail<3>();
  quat_grad.gradient().gradient().value().resize(12,3);
  Matrix<double,12,3> dE = quat_grad.gradient().gradient().value();
  Vector3d expdot = xyzexpdot.tail<3>();
  Matrix<double,4,3> Edot = matGradMult(dE,expdot);
  Vector4d quat_ddot = quat_grad.gradient().value()*xyzexpddot.tail<3>() + Edot*expdot;
  Matrix<double,3,4> M;
  Matrix<double,12,4> dM;
  quatdot2angularvelMatrix(quat,M,&dM);
  xyzdot_angular_vel.tail<3>() = M*quat_dot;
  xyzddot_angular_accel.tail<3>() = M*quat_ddot + matGradMult(dM,quat_dot)*quat_dot;
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

void getRobotJointIndexMap(JointNames *joint_names, RobotJointIndexMap *joint_map) {
  if (joint_names->drake.size() != joint_names->robot.size()) {
    mexErrMsgTxt("Cannot create joint name map: joint_names->drake and joint_names->robot must have the same length");
  }
  int njoints = joint_names->drake.size();
  joint_map->drake_to_robot.resize(njoints);
  joint_map->robot_to_drake.resize(njoints);

  bool has_match;
  for (int i=0; i < njoints; i++) {
    has_match = false;
    for (int j=0; j < njoints; j++) {
      if (joint_names->drake[i].compare(joint_names->robot[j]) == 0) {
        has_match = true;
        joint_map->drake_to_robot[i] = j;
        joint_map->robot_to_drake[j] = i;
      }
    }
    if (!has_match) {
      std::cout << "Could not match joint: " << joint_names->drake[i] << std::endl;
      mexErrMsgTxt("Could not find a match for drake joint name");
    }
  }
  return;
}

template drakeControlUtilEXPORT void getRows(std::set<int> &, const MatrixBase< MatrixXd > &, MatrixBase< MatrixXd > &);
template drakeControlUtilEXPORT void getCols(std::set<int> &, const MatrixBase< MatrixXd > &, MatrixBase< MatrixXd > &);
template drakeControlUtilEXPORT void angleDiff(const MatrixBase<MatrixXd> &, const MatrixBase<MatrixXd> &, MatrixBase<MatrixXd> &);
template drakeControlUtilEXPORT void angleDiff(const MatrixBase<Vector3d> &, const MatrixBase<Vector3d> &, MatrixBase<Vector3d> &);
template drakeControlUtilEXPORT mxArray* eigenToMatlab(const MatrixXd &);
template drakeControlUtilEXPORT mxArray* eigenToMatlab(const VectorXd &);
template drakeControlUtilEXPORT mxArray* eigenToMatlab(const Vector6d &);
template drakeControlUtilEXPORT mxArray* eigenToMatlab(const Vector3d &);
