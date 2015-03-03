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

bool inSupport(std::vector<SupportStateElement> supports, int body_idx) {
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
  for (std::vector<Vector4d>::iterator pt_iter=supp.contact_pts.begin(); pt_iter!=supp.contact_pts.end(); pt_iter++) {
    
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
  int j, k=0, nq = r->num_dof;

  n.resize(nc,nq);
  D.resize(nq,nc*2*m_surface_tangents);
  Jp.resize(3*nc,nq);
  Jpdot.resize(3*nc,nq);
  
  Vector3d contact_pos,pos,posB,normal;
  MatrixXd J(3,nq);
  Matrix<double,3,m_surface_tangents> d;
  
  for (std::vector<SupportStateElement>::iterator iter = supp.begin(); iter!=supp.end(); iter++) {
    if (nc>0) {
      for (std::vector<Vector4d>::iterator pt_iter=iter->contact_pts.begin(); pt_iter!=iter->contact_pts.end(); pt_iter++) {
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


int contactConstraintsBV(RigidBodyManipulator *r, int nc, double mu, std::vector<SupportStateElement>& supp, void *map_ptr, MatrixXd &B, MatrixXd &JB, MatrixXd &Jp, MatrixXd &Jpdot, MatrixXd &normals, double terrain_height)
{
  int j, k=0, nq = r->num_dof;

  B.resize(3,nc*2*m_surface_tangents);
  JB.resize(nq,nc*2*m_surface_tangents);
  Jp.resize(3*nc,nq);
  Jpdot.resize(3*nc,nq);
  normals.resize(3, nc);
  
  Vector3d contact_pos,pos,posB,normal; 
  MatrixXd J(3,nq);
  Matrix<double,3,m_surface_tangents> d;
  double norm = sqrt(1+mu*mu); // because normals and ds are orthogonal, the norm has a simple form
  
  for (std::vector<SupportStateElement>::iterator iter = supp.begin(); iter!=supp.end(); iter++) {
    if (nc>0) {
      for (std::vector<Vector4d>::iterator pt_iter=iter->contact_pts.begin(); pt_iter!=iter->contact_pts.end(); pt_iter++) {
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

        // store away kin sols into Jp and Jpdot
        // NOTE: I'm cheating and using a slightly different ordering of J and Jdot here
        Jp.block(3*k,0,3,nq) = J;
        r->forwardJacDot(iter->body_idx,*pt_iter,0,J);
        Jpdot.block(3*k,0,3,nq) = J;
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
      logic_map_double = mxGetPr(pm);
      for (j = 0; j < 4; j++) {
        se.support_logic_map[j] = logic_map_double[j] != 0;
      }
    } else {
      mexErrMsgTxt("Please convert support_logic_map to double");
    }
    pm = mxGetField(supp_data, i, "contact_pts");
    contact_pts.resize(mxGetM(pm), mxGetN(pm));
    memcpy(contact_pts.data(), mxGetPr(pm), sizeof(double)*mxGetNumberOfElements(pm));

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

  // Implement the logic described in QPInput2D.m
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

Matrix<bool, Dynamic, 1> getActiveSupportMask(RigidBodyManipulator* r, void* map_ptr, double* q, double* qd, std::vector<SupportStateElement> available_supports, Matrix<bool, Dynamic, 1> contact_force_detected, double contact_threshold, double terrain_height) {
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

std::vector<SupportStateElement> getActiveSupports(RigidBodyManipulator* r, void* map_ptr, double* q, double* qd, std::vector<SupportStateElement> available_supports, Matrix<bool, Dynamic, 1> contact_force_detected, double contact_threshold, double terrain_height) {

  Matrix<bool, Dynamic, 1> active_supp_mask = getActiveSupportMask(r, map_ptr, q, qd, available_supports, contact_force_detected, contact_threshold, terrain_height);

  std::vector<SupportStateElement> active_supports;

  for (int i=0; i < available_supports.size(); i++) {
    if (active_supp_mask(i)) {
      active_supports.push_back(available_supports[i]);
    }
  }
  return active_supports;
}

void sizecheck(const mxArray* mat, int M, int N) {
  if (mxGetM(mat) != M) {
    mexErrMsgIdAndTxt("Drake:WrongSize", "wrong number of rows. Expected: %d but got: %d", M, mxGetM(mat));
  }
  if (mxGetN(mat) != N) {
    mexErrMsgIdAndTxt("Drake:WrongSize", "wrong number of columns. Expected: %d but got: %d", N, mxGetN(mat));
  }
  return;
}

Vector6d bodyMotionPD(RigidBodyManipulator *r, double *q, double *qd, const int body_index, Vector6d body_pose_des, Vector6d body_v_des, Vector6d body_vdot_des, Vector6d Kp, Vector6d Kd) {
  int nq = r->num_dof;
  Map< VectorXd > qdvec(qd,nq);

  r->doKinematics(q,false,qd);

  // TODO: this must be updated to use quaternions/spatial velocity
  Vector6d body_pose;
  MatrixXd J = MatrixXd::Zero(6,r->num_dof);
  Vector4d zero = Vector4d::Zero();
  zero(3) = 1.0;
  r->forwardKin(body_index,zero,1,body_pose);
  r->forwardJac(body_index,zero,1,J);

  Vector6d body_error;
  body_error.head<3>()= body_pose_des.head<3>()-body_pose.head<3>();

  Vector3d error_rpy,pose_rpy,des_rpy;
  pose_rpy = body_pose.tail<3>();
  des_rpy = body_pose_des.tail<3>();
  angleDiff(pose_rpy,des_rpy,error_rpy);
  body_error.tail(3) = error_rpy;

  Vector6d body_vdot = (Kp.array()*body_error.array()).matrix() + (Kd.array()*(body_v_des-J*qdvec).array()).matrix() + body_vdot_des;
  return body_vdot;
}

void evaluateCubicSplineSegment(double t, Matrix<double, 6, 4> coefs, Vector6d &y, Vector6d &ydot, Vector6d &yddot) {
  // evaluate a cubic spline with coefficients coef and starting time 0 at time t
  y = coefs.col(0)*pow(t, 3) + coefs.col(1)*pow(t, 2) + coefs.col(2)*t + coefs.col(3);
  ydot = 3*coefs.col(0)*pow(t,2) + 2*coefs.col(1)*t + coefs.col(2);
  yddot = 6*coefs.col(0)*t + 2*coefs.col(1);
}

template<int M, int N>
void matlabToCArrayOfArrays(const mxArray *source, const int idx, const char *fieldname, double *destination)  {
  const mxArray *field = myGetField(source, idx, fieldname);
  if (!mxIsDouble(field)) {
    std::cout << fieldname << std::endl;
    mexErrMsgTxt("Numeric data must be passed in as doubles\n");
  }
  sizecheck(field, M, N);
  Map<Matrix<double, M, N>>A(mxGetPr(field));
  // C is row-major, matlab is column-major
  Matrix<double, N, M> A_t = A.transpose();
  memcpy(destination, A_t.data(), sizeof(double)*M*N);
  return;
}

std::shared_ptr<drake::lcmt_qp_controller_input> encodeQPInputLCM(const mxArray *qp_input) {
  std::shared_ptr<drake::lcmt_qp_controller_input> msg (new drake::lcmt_qp_controller_input());

  msg->timestamp = (int64_t) (mxGetScalar(myGetProperty(qp_input, "timestamp")) * 1000000);

  const mxArray* zmp_data = myGetProperty(qp_input, "zmp_data");

  matlabToCArrayOfArrays<4, 4>(zmp_data, 0, "A", &msg->zmp_data.A[0][0]);
  matlabToCArrayOfArrays<4, 2>(zmp_data, 0, "B", &msg->zmp_data.B[0][0]);
  matlabToCArrayOfArrays<2, 4>(zmp_data, 0, "C", &msg->zmp_data.C[0][0]);
  matlabToCArrayOfArrays<2, 2>(zmp_data, 0, "D", &msg->zmp_data.D[0][0]);
  matlabToCArrayOfArrays<4, 1>(zmp_data, 0, "x0", &msg->zmp_data.x0[0][0]);
  matlabToCArrayOfArrays<2, 1>(zmp_data, 0, "y0", &msg->zmp_data.y0[0][0]);
  matlabToCArrayOfArrays<2, 1>(zmp_data, 0, "u0", &msg->zmp_data.u0[0][0]);
  matlabToCArrayOfArrays<2, 2>(zmp_data, 0, "R", &msg->zmp_data.R[0][0]);
  matlabToCArrayOfArrays<2, 2>(zmp_data, 0, "Qy", &msg->zmp_data.Qy[0][0]);
  matlabToCArrayOfArrays<4, 4>(zmp_data, 0, "S", &msg->zmp_data.S[0][0]);
  matlabToCArrayOfArrays<4, 1>(zmp_data, 0, "s1", &msg->zmp_data.s1[0][0]);
  matlabToCArrayOfArrays<4, 1>(zmp_data, 0, "s1dot", &msg->zmp_data.s1dot[0][0]);
  msg->zmp_data.s2 = mxGetScalar(myGetField(zmp_data, "s2"));
  msg->zmp_data.s2dot = mxGetScalar(myGetField(zmp_data, "s2dot"));
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
      Map<MatrixXd>contact_pts_mat(mxGetPr(contact_pts), mxGetM(contact_pts), mxGetN(contact_pts));
      msg->support_data[i].num_contact_pts = (int32_t) mxGetN(contact_pts);
      msg->support_data[i].contact_pts.resize(3);
      for (int j=0; j < 3; j++) {
        msg->support_data[i].contact_pts[j].resize(msg->support_data[i].num_contact_pts);
        for (int k=0; k < msg->support_data[i].num_contact_pts; k++) {
          msg->support_data[i].contact_pts[j][k] = contact_pts_mat(j, k);
        }
      }

      matlabToCArrayOfArrays<4, 1>(support_data, i, "support_logic_map", &double_logic_map[0][0]);
      for (int j=0; j < 4; j++) {
        msg->support_data[i].support_logic_map[j] = (double_logic_map[j][0] != 0);
      }
      msg->support_data[i].mu = mxGetScalar(myGetField(support_data, i, "mu"));
      msg->support_data[i].contact_surfaces = (int32_t) mxGetScalar(myGetField(support_data, i, "contact_surfaces"));
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
      msg->body_motion_data[i].body_id = (int32_t) mxGetScalar(myGetField(body_motion_data, i, "body_id"));
      memcpy(msg->body_motion_data[i].ts, mxGetPr(myGetField(body_motion_data, i, "ts")), 2*sizeof(double));
      const mxArray* coefs = myGetField(body_motion_data, i, "coefs");
      if (mxGetNumberOfDimensions(coefs) != 3) mexErrMsgTxt("coefs should be a dimension-3 array");
      const size_t *dim = mxGetDimensions(coefs);
      if (dim[0] != 6 || dim[1] != 1 || dim[2] != 4) mexErrMsgTxt("coefs should be size 6x4");
      matlabToCArrayOfArrays<6, 4>(body_motion_data, i, "coefs", &msg->body_motion_data[i].coefs[0][0]);
    }
  }

  const mxArray* whole_body_data = myGetProperty(qp_input, "whole_body_data");
  if (mxGetN(whole_body_data) != 1 || mxGetM(whole_body_data) != 1) mexErrMsgTxt("whole_body_data should be a 1x1 struct");
  const mxArray* q_des = myGetField(whole_body_data, "q_des");
  if (mxGetN(q_des) != 1) mexErrMsgTxt("q_des should be a column vector");
  const int npos = mxGetM(q_des);
  msg->whole_body_data.timestamp = msg->timestamp;
  msg->whole_body_data.num_positions = npos;
  Map<VectorXd>q_des_vec(mxGetPr(q_des), npos);
  msg->whole_body_data.q_des.resize(npos);

  for (int i=0; i < npos; i++) {
    msg->whole_body_data.q_des[i] = q_des_vec(i);
  }

  const mxArray* condof = myGetField(whole_body_data, "constrained_dofs");
  const int ncons = mxGetM(condof);
  msg->whole_body_data.num_constrained_dofs = ncons;
  msg->whole_body_data.constrained_dofs.resize(ncons);
  if (ncons > 0) {
    if (mxGetN(condof) != 1) mexErrMsgTxt("constrained dofs should be a column vector");
    Map<VectorXd>condof_vec(mxGetPr(condof), ncons);

    for (int i=0; i < ncons; i++) {
      msg->whole_body_data.constrained_dofs[i] = condof_vec(i);
    }
  }

  msg->param_set_name = mxArrayToString(myGetProperty(qp_input, "param_set_name"));
  return msg;
}


template drakeControlUtilEXPORT void getRows(std::set<int> &, const MatrixBase< MatrixXd > &, MatrixBase< MatrixXd > &);
template drakeControlUtilEXPORT void getCols(std::set<int> &, const MatrixBase< MatrixXd > &, MatrixBase< MatrixXd > &);
template drakeControlUtilEXPORT void angleDiff(const MatrixBase<MatrixXd> &, const MatrixBase<MatrixXd> &, MatrixBase<MatrixXd> &);
template drakeControlUtilEXPORT void angleDiff(const MatrixBase<Vector3d> &, const MatrixBase<Vector3d> &, MatrixBase<Vector3d> &);
template drakeControlUtilEXPORT mxArray* eigenToMatlab(const MatrixXd &);
template drakeControlUtilEXPORT mxArray* eigenToMatlab(const VectorXd &);
template drakeControlUtilEXPORT mxArray* eigenToMatlab(const Vector6d &);
template drakeControlUtilEXPORT mxArray* eigenToMatlab(const Vector3d &);
