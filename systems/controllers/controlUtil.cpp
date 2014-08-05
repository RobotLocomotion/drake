#include "controlUtil.h"

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

template <int Rows, int Cols>
mxArray* eigenToMatlab(Matrix<double,Rows,Cols> &m)
{
 mxArray* pm = mxCreateDoubleMatrix(m.rows(),m.cols(),mxREAL);
 if (m.rows()*m.cols()>0)
   memcpy(mxGetPr(pm),m.data(),sizeof(double)*m.rows()*m.cols());
 return pm;
}

mxArray* myGetProperty(const mxArray* pobj, const char* propname)
{
  mxArray* pm = mxGetProperty(pobj,0,propname);
  if (!pm) mexErrMsgIdAndTxt("DRC:ControlUtil:BadInput","ControlUtil is trying to load object property '%s', but failed.", propname);
  return pm;
}

mxArray* myGetField(const mxArray* pobj, const char* propname)
{
  mxArray* pm = mxGetField(pobj,0,propname);
  if (!pm) mexErrMsgIdAndTxt("DRC:ControlUtil:BadInput","ControlUtil is trying to load object field '%s', but failed.", propname);
  return pm;
}

bool inSupport(std::vector<SupportStateElement> supports, int body_idx) {
  for (int i=0; i<supports.size(); i++) {
    if (supports[i].body_idx == body_idx)
      return true;
  }
  return false;
}

void collisionDetect(void* map_ptr, Vector3d const & contact_pos, Vector3d &pos, Vector3d *normal, double terrain_height)
{
  if (map_ptr) {
#ifdef USE_MAPS    
    Vector3f floatPos, floatNormal;
    auto state = static_cast<mexmaps::MapHandle*>(map_ptr);
    if (state != NULL) {
      auto view = state->getView();
      if (view != NULL) {
        if (view->getClosest(contact_pos.cast<float>(),floatPos,floatNormal)) {
          pos = floatPos.cast<double>();
          if (normal) *normal = floatNormal.cast<double>();
          return;
        }
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
  
  if (1 - normal(2) < 10e-8) { // handle the unit-normal case (since it's unit length, just check z)
    t1 << 1,0,0;
  } else { // now the general case
    t1 << normal(2), -normal(1), 0; // normal.cross([0;0;1])
    t1 /= sqrt(normal(1)*normal(1) + normal(2)*normal(2));
  }
      
  t2 = t1.cross(normal);
      
  for (int k=0; k<m_surface_tangents; k++) {
    theta = k*M_PI/m_surface_tangents;
    d.col(k)=cos(theta)*t1 + sin(theta)*t2;
  }
}

int contactPhi(RigidBodyManipulator* r, SupportStateElement& supp, void *map_ptr, VectorXd &phi, double terrain_height)
{
  RigidBody* b = &(r->bodies[supp.body_idx]);
  int nc = supp.contact_pt_inds.size();
  phi.resize(nc);

  if (nc<1) return nc;

  Vector3d contact_pos,pos,posB,normal; Vector4d tmp;

  int i=0;
  for (std::set<int>::iterator pt_iter=supp.contact_pt_inds.begin(); pt_iter!=supp.contact_pt_inds.end(); pt_iter++) {
    if (*pt_iter<0 || *pt_iter>=b->contact_pts.cols()) 
      mexErrMsgIdAndTxt("DRC:ControlUtil:BadInput","requesting contact pt %d but body only has %d pts",*pt_iter,b->contact_pts.cols());
    
    tmp = b->contact_pts.col(*pt_iter);
    r->forwardKin(supp.body_idx,tmp,0,contact_pos);
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
  
  Vector3d contact_pos,pos,posB,normal; Vector4d tmp;
  MatrixXd J(3,nq);
  Matrix<double,3,m_surface_tangents> d;
  
  for (std::vector<SupportStateElement>::iterator iter = supp.begin(); iter!=supp.end(); iter++) {
    RigidBody* b = &(r->bodies[iter->body_idx]);
    if (nc>0) {
      for (std::set<int>::iterator pt_iter=iter->contact_pt_inds.begin(); pt_iter!=iter->contact_pt_inds.end(); pt_iter++) {
        if (*pt_iter<0 || *pt_iter>=b->contact_pts.cols()) mexErrMsgIdAndTxt("DRC:ControlUtil:BadInput","requesting contact pt %d but body only has %d pts",*pt_iter,b->contact_pts.cols());
        tmp = b->contact_pts.col(*pt_iter);
        r->forwardKin(iter->body_idx,tmp,0,contact_pos);
        r->forwardJac(iter->body_idx,tmp,0,J);

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
        r->forwardJacDot(iter->body_idx,tmp,0,J);
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
  
  Vector3d contact_pos,pos,posB,normal; Vector4d tmp;
  MatrixXd J(3,nq);
  Matrix<double,3,m_surface_tangents> d;
  double norm = sqrt(1+mu*mu); // because normals and ds are orthogonal, the norm has a simple form
  
  for (std::vector<SupportStateElement>::iterator iter = supp.begin(); iter!=supp.end(); iter++) {
    RigidBody* b = &(r->bodies[iter->body_idx]);
    if (nc>0) {
      for (std::set<int>::iterator pt_iter=iter->contact_pt_inds.begin(); pt_iter!=iter->contact_pt_inds.end(); pt_iter++) {
        if (*pt_iter<0 || *pt_iter>=b->contact_pts.cols()) mexErrMsgIdAndTxt("DRC:ControlUtil:BadInput","requesting contact pt %d but body only has %d pts",*pt_iter,b->contact_pts.cols());
        tmp = b->contact_pts.col(*pt_iter);
        r->forwardKin(iter->body_idx,tmp,0,contact_pos);
        r->forwardJac(iter->body_idx,tmp,0,J);

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
        r->forwardJacDot(iter->body_idx,tmp,0,J);
        Jpdot.block(3*k,0,3,nq) = J;
        normals.col(k) = normal;
        
        k++;
      }
    }
  }
  
  return k;
}

VectorXd individualSupportCOPs(RigidBodyManipulator* r, const std::vector<SupportStateElement>& active_supports, const int nd, const MatrixXd& normals, const MatrixXd& B, const VectorXd& beta)
{
  VectorXd individual_cops(3 * active_supports.size());
  individual_cops.fill(std::numeric_limits<double>::quiet_NaN());
  int normals_start_col = 0;
  int active_support_start_col = 0;
  for (int j = 0; j < active_supports.size(); j++) {
    auto active_support = active_supports[j];
    auto contact_pt_inds = active_support.contact_pt_inds;

    int ncj = contact_pt_inds.size();
    int active_support_length = nd * ncj;
    auto normalsj = normals.middleCols(normals_start_col, ncj);
    Vector3d normalj = normalsj.col(0);
    bool norms_identical = (normalsj.colwise().operator-(normalj)).squaredNorm() < 1e-15;

    if (norms_identical) { // otherwise computing a COP doesn't make sense
      const auto& Bj = B.middleCols(active_support_start_col, active_support_length);
      const auto& betaj = beta.segment(active_support_start_col, active_support_length);

      const auto& contact_positionsj = r->bodies[active_support.body_idx].contact_pts;
      Vector3d forcej = Vector3d::Zero();
      Vector3d torquej = Vector3d::Zero();
      double min_contact_position_z = std::numeric_limits<double>::infinity();

      for (const auto& k : contact_pt_inds) {
        const auto& Bjk = Bj.middleCols(k * nd, nd);
        const auto& betajk = betaj.segment(k * nd, nd);
        Vector3d contact_positionjk = contact_positionsj.col(k);
        Vector3d forcejk = Bjk * betajk;
        forcej += forcejk;
        auto torquejk = contact_positionjk.cross(forcejk);
        torquej += torquejk;
        double contact_position_z = normalj.dot(contact_positionjk);
        if (contact_position_z < min_contact_position_z) {
          min_contact_position_z = contact_position_z;
        }
      }
      double fzj = normalj.dot(forcej);
      if (std::abs(fzj) > 1e-7) {
        auto normal_torquej = normalj.dot(torquej);
        auto tangential_torquej = torquej - normalj * normal_torquej;
        Vector4d cop_bodyj;
        cop_bodyj << normalj.cross(tangential_torquej) / fzj + min_contact_position_z * normalj, 1.0;
        Vector3d cop_worldj;
        r->forwardKin(active_support.body_idx, cop_bodyj, 0, cop_worldj);
        individual_cops.segment<3>(3 * j) = cop_worldj;
      }
    }
    normals_start_col += ncj;
    active_support_start_col += active_support_length;
  }
  return individual_cops;
}

template void getRows(std::set<int> &, const MatrixBase< MatrixXd > &, MatrixBase< MatrixXd > &);
template void getCols(std::set<int> &, const MatrixBase< MatrixXd > &, MatrixBase< MatrixXd > &);
template void angleDiff(const MatrixBase<MatrixXd> &, const MatrixBase<MatrixXd> &, MatrixBase<MatrixXd> &);
template void angleDiff(const MatrixBase<Vector3d> &, const MatrixBase<Vector3d> &, MatrixBase<Vector3d> &);
template mxArray* eigenToMatlab(Matrix<double,-1,-1> &);
template mxArray* eigenToMatlab(Matrix<double,-1,1> &);
template mxArray* eigenToMatlab(Matrix<double,6,1> &);
