/* 
 * A c++ version of (significant pieces of) the QPController.m mimoOutput method. 
 *
 * Todo:
 *   switch to spatial accelerations in motion constraints
 *   use fixed-size matrices (or at least pre-allocated)
 *       for instance: #define nq 
 *       set MaxRowsAtCompileTime (http://eigen.tuxfamily.org/dox/TutorialMatrixClass.html)
 *   some matrices might be better off using RowMajor
 */

#include "QPCommon.h"
#include <Eigen/StdVector>
#include <limits>
#include <cmath>
#include "drakeMexUtil.h"
#include "controlMexUtil.h"

//#define TEST_FAST_QP
//#define USE_MATRIX_INVERSION_LEMMA

using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  int error;
  if (nrhs<1) mexErrMsgTxt("usage: ptr = QPControllermex(0,control_obj,robot_obj,...); alpha=QPControllermex(ptr,...,...)");
  if (nlhs<1) mexErrMsgTxt("take at least one output... please.");

  struct QPControllerData* pdata;
  mxArray* pm;
  double* pr;
  int i,j;

  if (mxGetScalar(prhs[0])==0) { // then construct the data object and return
    pdata = new struct QPControllerData;
    
    // get control object properties
    const mxArray* pobj = prhs[1];
    
    pm = myGetProperty(pobj,"slack_limit");
    pdata->slack_limit = mxGetScalar(pm);

    pm = myGetProperty(pobj,"W_kdot");
    assert(mxGetM(pm)==3); assert(mxGetN(pm)==3);
    pdata->W_kdot.resize(mxGetM(pm),mxGetN(pm));
    memcpy(pdata->W_kdot.data(),mxGetPrSafe(pm),sizeof(double)*mxGetM(pm)*mxGetN(pm));

    pm= myGetProperty(pobj,"w_grf");
    pdata->w_grf = mxGetScalar(pm);    

    pm= myGetProperty(pobj,"w_slack");
    pdata->w_slack = mxGetScalar(pm);    

    pm = myGetProperty(pobj,"Kp_ang");
    pdata->Kp_ang = mxGetScalar(pm);

    pm = myGetProperty(pobj,"Kp_accel");
    pdata->Kp_accel = mxGetScalar(pm);

    pm= myGetProperty(pobj,"n_body_accel_inputs");
    pdata->n_body_accel_inputs = (int) mxGetScalar(pm); 

    pm= myGetProperty(pobj,"n_body_accel_bounds");
    pdata->n_body_accel_bounds = (int) mxGetScalar(pm); 

    mxArray* body_accel_bounds = myGetProperty(pobj,"body_accel_bounds");
    Vector6d vecbound;
    for (int i=0; i<pdata->n_body_accel_bounds; i++) {
      pdata->accel_bound_body_idx.push_back((int) mxGetScalar(mxGetField(body_accel_bounds,i,"body_idx"))-1);
      pm = mxGetField(body_accel_bounds,i,"min_acceleration");

      assert(mxGetM(pm)==6); assert(mxGetN(pm)==1);
      memcpy(vecbound.data(),mxGetPrSafe(pm),sizeof(double)*6);
      pdata->min_body_acceleration.push_back(vecbound);
      pm = mxGetField(body_accel_bounds,i,"max_acceleration");
      assert(mxGetM(pm)==6); assert(mxGetN(pm)==1);
      memcpy(vecbound.data(),mxGetPrSafe(pm),sizeof(double)*6);
      pdata->max_body_acceleration.push_back(vecbound);
    }

    pm = myGetProperty(pobj,"body_accel_input_weights");
    pdata->body_accel_input_weights.resize(pdata->n_body_accel_inputs);
    memcpy(pdata->body_accel_input_weights.data(),mxGetPrSafe(pm),sizeof(double)*pdata->n_body_accel_inputs);

    pdata->n_body_accel_eq_constraints = 0;
    for (int i=0; i<pdata->n_body_accel_inputs; i++) {
      if (pdata->body_accel_input_weights(i) < 0)
        pdata->n_body_accel_eq_constraints++;
    }

    // get robot mex model ptr
    if (!mxIsNumeric(prhs[2]) || mxGetNumberOfElements(prhs[2])!=1)
      mexErrMsgIdAndTxt("Drake:QPControllermex:BadInputs","the third argument should be the robot mex ptr");
    memcpy(&(pdata->r),mxGetData(prhs[2]),sizeof(pdata->r));
    
    pdata->B.resize(mxGetM(prhs[3]),mxGetN(prhs[3]));
    memcpy(pdata->B.data(),mxGetPrSafe(prhs[3]),sizeof(double)*mxGetM(prhs[3])*mxGetN(prhs[3]));

    int nq = pdata->r->num_positions, nu = pdata->B.cols();
    
    pm = myGetProperty(pobj,"w_qdd");
    pdata->w_qdd.resize(nq);
    memcpy(pdata->w_qdd.data(),mxGetPrSafe(pm),sizeof(double)*nq);

    pdata->umin.resize(nu);
    pdata->umax.resize(nu);
    memcpy(pdata->umin.data(),mxGetPrSafe(prhs[4]),sizeof(double)*nu);
    memcpy(pdata->umax.data(),mxGetPrSafe(prhs[5]),sizeof(double)*nu);

    pdata->B_act.resize(nu,nu);
    pdata->B_act = pdata->B.bottomRows(nu);

     // get the map ptr back from matlab
     if (!mxIsNumeric(prhs[6]) || mxGetNumberOfElements(prhs[6])!=1)
     mexErrMsgIdAndTxt("Drake:QPControllermex:BadInputs","the seventh argument should be the map ptr");
     memcpy(&pdata->map_ptr,mxGetPrSafe(prhs[6]),sizeof(pdata->map_ptr));
    
//    pdata->map_ptr = NULL;
    if (!pdata->map_ptr)
      mexWarnMsgTxt("Map ptr is NULL.  Assuming flat terrain at z=0");
    
    // create gurobi environment
    error = GRBloadenv(&(pdata->env),NULL);

    // set solver params (http://www.gurobi.com/documentation/5.5/reference-manual/node798#sec:Parameters)
    mxArray* psolveropts = myGetProperty(pobj,"gurobi_options");
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

    mxClassID cid;
    if (sizeof(pdata)==4) cid = mxUINT32_CLASS;
    else if (sizeof(pdata)==8) cid = mxUINT64_CLASS;
    else mexErrMsgIdAndTxt("Drake:constructModelmex:PointerSize","Are you on a 32-bit machine or 64-bit machine??");
    
    plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
    memcpy(mxGetData(plhs[0]),&pdata,sizeof(pdata));
    
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

    pdata->vbasis_len = 0;
    pdata->cbasis_len = 0;
    pdata->vbasis = NULL;
    pdata->cbasis = NULL;
    return;
  }
  
  // first get the ptr back from matlab
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("Drake:QPControllermex:BadInputs","the first argument should be the ptr");
  memcpy(&pdata,mxGetData(prhs[0]),sizeof(pdata));

//  for (i=0; i<pdata->r->num_bodies; i++)
//    mexPrintf("body %d (%s) has %d contact points\n", i, pdata->r->bodies[i].linkname.c_str(), pdata->r->bodies[i].contact_pts.cols());

  int nu = pdata->B.cols(), nq = pdata->r->num_positions, nv = pdata->r->num_velocities;
  const int dim = 3, // 3D
  nd = 2*m_surface_tangents; // for friction cone approx, hard coded for now
  
  assert(nu+6 == nq);

  int narg=1;

  int use_fast_qp = (int) mxGetScalar(prhs[narg++]);
  
  Map< VectorXd > qddot_des(mxGetPrSafe(prhs[narg++]),nq);
  
  Map<VectorXd> q(mxGetPrSafe(prhs[narg]), nq);
  Map<VectorXd> qd(mxGetPrSafe(prhs[narg++]) + nq, nv);

  vector<VectorXd,aligned_allocator<VectorXd>> body_accel_inputs;
  for (int i=0; i<pdata->n_body_accel_inputs; i++) {
    assert(mxGetM(prhs[narg])==7); assert(mxGetN(prhs[narg])==1);
    VectorXd v = VectorXd::Zero(7,1);
    memcpy(v.data(),mxGetPrSafe(prhs[narg++]),sizeof(double)*7);
    body_accel_inputs.push_back(v);
  }
  
  int num_condof;
  VectorXd condof;
  if (!mxIsEmpty(prhs[narg])) {
    assert(mxGetN(prhs[narg])==1);
    num_condof=mxGetM(prhs[narg]);
    condof = VectorXd::Zero(num_condof);
    memcpy(condof.data(),mxGetPrSafe(prhs[narg++]),sizeof(double)*num_condof);
  }
  else {
    num_condof=0;
    narg++; // skip over empty vector
  }

  int desired_support_argid = narg++;

  Map<MatrixXd> A_ls(mxGetPrSafe(prhs[narg]),mxGetM(prhs[narg]),mxGetN(prhs[narg])); narg++;
  Map<MatrixXd> B_ls(mxGetPrSafe(prhs[narg]),mxGetM(prhs[narg]),mxGetN(prhs[narg])); narg++;
  Map<MatrixXd> Qy  (mxGetPrSafe(prhs[narg]),mxGetM(prhs[narg]),mxGetN(prhs[narg])); narg++;
  Map<MatrixXd> R_ls(mxGetPrSafe(prhs[narg]),mxGetM(prhs[narg]),mxGetN(prhs[narg])); narg++;
  Map<MatrixXd> C_ls(mxGetPrSafe(prhs[narg]),mxGetM(prhs[narg]),mxGetN(prhs[narg])); narg++;
  Map<MatrixXd> D_ls(mxGetPrSafe(prhs[narg]),mxGetM(prhs[narg]),mxGetN(prhs[narg])); narg++;
  Map<MatrixXd> S   (mxGetPrSafe(prhs[narg]),mxGetM(prhs[narg]),mxGetN(prhs[narg])); narg++;

  Map<VectorXd> s1(mxGetPrSafe(prhs[narg]),mxGetNumberOfElements(prhs[narg])); narg++;
  Map<VectorXd> s1dot(mxGetPrSafe(prhs[narg]),mxGetNumberOfElements(prhs[narg])); narg++;
  double s2dot = mxGetScalar(prhs[narg++]);
  Map<VectorXd> x0(mxGetPrSafe(prhs[narg]),mxGetNumberOfElements(prhs[narg])); narg++;
  Map<VectorXd> u0(mxGetPrSafe(prhs[narg]),mxGetNumberOfElements(prhs[narg])); narg++;
  Map<VectorXd> y0(mxGetPrSafe(prhs[narg]),mxGetNumberOfElements(prhs[narg])); narg++;
  Map<VectorXd> qdd_lb(mxGetPrSafe(prhs[narg]),mxGetNumberOfElements(prhs[narg])); narg++;
  Map<VectorXd> qdd_ub(mxGetPrSafe(prhs[narg]),mxGetNumberOfElements(prhs[narg])); narg++;
  memcpy(pdata->w_qdd.data(),mxGetPrSafe(prhs[narg++]),sizeof(double)*nq); 
  
  double mu = mxGetScalar(prhs[narg++]);
  double terrain_height = mxGetScalar(prhs[narg++]); // nonzero if we're using DRCFlatTerrainMap

  MatrixXd R_DQyD_ls = R_ls + D_ls.transpose()*Qy*D_ls;

  pdata->r->doKinematics(q,false,qd);

  //---------------------------------------------------------------------
  // Compute active support from desired supports -----------------------
  MatrixXd all_body_contact_pts;
  Vector3d contact_pt = Vector3d::Zero();

  vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>> active_supports;
  set<int> contact_bodies; // redundant, clean up later
  int num_active_contact_pts=0;
  if (!mxIsEmpty(prhs[desired_support_argid])) {
    VectorXd phi;
    mxArray* mxBodies = myGetField(prhs[desired_support_argid],"bodies");
    if (!mxBodies) mexErrMsgTxt("couldn't get bodies");
    double* pBodies = mxGetPrSafe(mxBodies);
    mxArray* mxContactPts = myGetField(prhs[desired_support_argid],"contact_pts");
    if (!mxContactPts) mexErrMsgTxt("couldn't get contact points");
    
    for (i=0; i<mxGetNumberOfElements(mxBodies);i++) {
      mxArray* mxBodyContactPts = mxGetCell(mxContactPts,i);
      assert(mxGetM(mxBodyContactPts) == 3);
      int nc = static_cast<int>(mxGetN(mxBodyContactPts));
      if (nc<1) continue;
      
      all_body_contact_pts.resize(mxGetM(mxBodyContactPts),mxGetN(mxBodyContactPts));
      pr = mxGetPrSafe(mxBodyContactPts); 
      memcpy(all_body_contact_pts.data(),pr,sizeof(double)*mxGetM(mxBodyContactPts)*mxGetN(mxBodyContactPts));

      SupportStateElement se;
      se.body_idx = (int) pBodies[i]-1;
      for (j=0; j<nc; j++) {
        contact_pt.head<3>() = all_body_contact_pts.col(j);
        se.contact_pts.push_back(contact_pt);
      }
      
      active_supports.push_back(se);
      num_active_contact_pts += nc;
      contact_bodies.insert((int)se.body_idx); 
    }
  }

  pdata->r->doKinematicsNew(q, qd);
  std::map<int, std::unique_ptr<GradientVar<double, TWIST_SIZE, 1>> > f_ext;
  pdata->H = pdata->r->massMatrix<double>().value();
  pdata->C = pdata->r->inverseDynamics(f_ext).value();

  pdata->H_float = pdata->H.topRows(6);
  pdata->H_act = pdata->H.bottomRows(nu);
  pdata->C_float = pdata->C.head<6>();
  pdata->C_act = pdata->C.tail(nu);

  bool include_angular_momentum = (pdata->W_kdot.array().maxCoeff() > 1e-10);

  if (include_angular_momentum) {
    pdata->Ag = pdata->r->centroidalMomentumMatrix<double>(0).value();
    pdata->Agdot_times_v = pdata->r->centroidalMomentumMatrixDotTimesV<double>(0).value();
    pdata->Ak = pdata->Ag.topRows<3>();
    pdata->Akdot_times_v = pdata->Agdot_times_v.topRows<3>();
  }
  Vector3d xcom;
  // consider making all J's into row-major
  
  pdata->r->getCOM(xcom);
  pdata->r->getCOMJac(pdata->J);
  MatrixXd Jdot;
  pdata->r->getCOMJacDot(Jdot);
  pdata->Jdotv = Jdot*qd;
  pdata->J_xy = pdata->J.topRows(2);
  pdata->Jdotv_xy = pdata->Jdotv.head<2>();

  MatrixXd Jcom;
  VectorXd Jcomdotv;

  if (x0.size()==6) {
    Jcom = pdata->J;
    Jcomdotv = pdata->Jdotv;
  }
  else {
    Jcom = pdata->J_xy;
    Jcomdotv = pdata->Jdotv_xy;
  }

  MatrixXd B,JB,Jp,normals;
  VectorXd Jpdotv;
  std::vector<double> support_mus(active_supports.size(), mu);
  int nc = contactConstraintsBV(pdata->r,num_active_contact_pts,support_mus,active_supports,pdata->map_ptr,B,JB,Jp,Jpdotv,normals,terrain_height);
  int neps = nc*dim;

  VectorXd x_bar,xlimp;
  MatrixXd D_float(6,JB.cols()), D_act(nu,JB.cols());
  if (nc>0) {
    if (x0.size()==6) {
      // x,y,z com 
      xlimp.resize(6); 
      xlimp.topRows(3) = xcom;
      xlimp.bottomRows(3) = Jcom*qd;
    }
    else {
      xlimp.resize(4); 
      xlimp.topRows(2) = xcom.topRows(2);
      xlimp.bottomRows(2) = Jcom*qd;
    }
    x_bar = xlimp-x0;

    D_float = JB.topRows(6);
    D_act = JB.bottomRows(nu);
  }

  int nf = nc*nd; // number of contact force variables
  int nparams = nq+nf+neps;

  Vector3d kdot_des; 
  if (include_angular_momentum) {
    VectorXd k = pdata->Ak*qd;
    kdot_des = -pdata->Kp_ang*k; // TODO: parameterize
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
      // NOTE: moved Hqp calcs below, because I compute the inverse directly for FastQP (and sparse Hqp for gurobi)
      VectorXd tmp = C_ls*xlimp;
      VectorXd tmp1 = Jcomdotv;
      MatrixXd tmp2 = R_DQyD_ls*Jcom;

      pdata->fqp = tmp.transpose()*Qy*D_ls*Jcom;
      pdata->fqp += tmp1.transpose()*tmp2;
      pdata->fqp += (S*x_bar + 0.5*s1).transpose()*B_ls*Jcom;
      pdata->fqp -= u0.transpose()*tmp2;
      pdata->fqp -= y0.transpose()*Qy*D_ls*Jcom;
      pdata->fqp -= (pdata->w_qdd.array()*qddot_des.array()).matrix().transpose();
      if (include_angular_momentum) {
        pdata->fqp += pdata->Akdot_times_v.transpose() * pdata->W_kdot * pdata->Ak;
        pdata->fqp -= kdot_des.transpose()*pdata->W_kdot*pdata->Ak;
      }
      f.head(nq) = pdata->fqp.transpose();
     } else {
      f.head(nq) = -qddot_des;
    } 
  }
  f.tail(nf+neps) = VectorXd::Zero(nf+neps);
  
  int neq = 6+neps+6*pdata->n_body_accel_eq_constraints+num_condof;
  MatrixXd Aeq = MatrixXd::Zero(neq,nparams);
  VectorXd beq = VectorXd::Zero(neq);
  
  // constrained floating base dynamics
  //  H_float*qdd - J_float'*lambda - Dbar_float*beta = -C_float
  Aeq.topLeftCorner(6,nq) = pdata->H_float;
  beq.topRows(6) = -pdata->C_float;
    
  if (nc>0) {
    Aeq.block(0,nq,6,nc*nd) = -D_float;
  }
  
  if (nc > 0) {
    // relative acceleration constraint
    Aeq.block(6,0,neps,nq) = Jp;
    Aeq.block(6,nq,neps,nf) = MatrixXd::Zero(neps,nf);  // note: obvious sparsity here
    Aeq.block(6,nq+nf,neps,neps) = MatrixXd::Identity(neps,neps);             // note: obvious sparsity here
    beq.segment(6,neps) = -Jpdotv -pdata->Kp_accel*Jp*qd; 
  }    
  
  // add in body spatial equality constraints
  VectorXd body_vdot;
  Vector3d orig(0.0, 0.0, 0.0);
  
  int body_idx;
  int equality_ind = 6+neps;
  MatrixXd Jb(6,nq);
  MatrixXd Jbdot(6,nq);
  for (int i=0; i<pdata->n_body_accel_inputs; i++) {
    if (pdata->body_accel_input_weights(i) < 0) {
      // negative implies constraint
      body_vdot = body_accel_inputs[i].bottomRows(6);
      body_idx = (int)(body_accel_inputs[i][0])-1;

      if (!inSupport(active_supports,body_idx)) {
        pdata->r->forwardJac(body_idx,orig,1,Jb);
        pdata->r->forwardJacDot(body_idx,orig,1,Jbdot);

        for (int j=0; j<6; j++) {
          if (!std::isnan(body_vdot[j])) {
            Aeq.block(equality_ind,0,1,nq) = Jb.row(j);
            beq[equality_ind++] = -Jbdot.row(j)*qd + body_vdot[j];
          }
        }
      }
    }
  }

  if (num_condof>0) {
    // add joint acceleration constraints
    for (int i=0; i<num_condof; i++) {
      Aeq(equality_ind,(int)condof[i]-1) = 1;
      beq[equality_ind++] = qddot_des[(int)condof[i]-1];
    }
  }  
  
  int n_ineq = 2*nu+2*6*pdata->n_body_accel_bounds;
  MatrixXd Ain = MatrixXd::Zero(n_ineq,nparams);  // note: obvious sparsity here
  VectorXd bin = VectorXd::Zero(n_ineq);

  // linear input saturation constraints
  // u=B_act'*(H_act*qdd + C_act - Jz_act'*z - Dbar_act*beta)
  // using transpose instead of inverse because B is orthogonal
  Ain.topLeftCorner(nu,nq) = pdata->B_act.transpose()*pdata->H_act;
  Ain.block(0,nq,nu,nc*nd) = -pdata->B_act.transpose()*D_act;
  bin.head(nu) = -pdata->B_act.transpose()*pdata->C_act + pdata->umax;

  Ain.block(nu,0,nu,nparams) = -1*Ain.block(0,0,nu,nparams);
  bin.segment(nu,nu) = pdata->B_act.transpose()*pdata->C_act - pdata->umin;

  int body_index;
  int constraint_start_index = 2*nu;
  for (int i=0; i<pdata->n_body_accel_bounds; i++) {
    body_index = pdata->accel_bound_body_idx[i];
    pdata->r->forwardJac(body_index,orig,1,Jb);
    pdata->r->forwardJacDot(body_index,orig,1,Jbdot);
    Ain.block(constraint_start_index,0,6,pdata->r->num_positions) = Jb;
    bin.segment(constraint_start_index,6) = -Jbdot*qd + pdata->max_body_acceleration[i];
    constraint_start_index += 6;
    Ain.block(constraint_start_index,0,6,pdata->r->num_positions) = -Jb;
    bin.segment(constraint_start_index,6) = Jbdot*qd - pdata->min_body_acceleration[i];
    constraint_start_index += 6;
  }
       
  for (int i=0; i<n_ineq; i++) {
    // remove inf constraints---needed by gurobi
    if (std::isinf(double(bin(i)))) {
      Ain.row(i) = 0*Ain.row(i);
      bin(i)=0;
    }  
  }

  GRBmodel * model = NULL;
  int info=-1;
  
  // set obj,lb,up
  VectorXd lb(nparams), ub(nparams);
  lb.head(nq) = qdd_lb;
  ub.head(nq) = qdd_ub;
  lb.segment(nq,nf) = VectorXd::Zero(nf);
  ub.segment(nq,nf) = 1e3*VectorXd::Ones(nf);
  lb.tail(neps) = -pdata->slack_limit*VectorXd::Ones(neps);
  ub.tail(neps) = pdata->slack_limit*VectorXd::Ones(neps);

  VectorXd alpha(nparams);

  MatrixXd Qnfdiag(nf,1), Qneps(neps,1);
  vector<MatrixXd*> QBlkDiag( nc>0 ? 3 : 1 );  // nq, nf, neps   // this one is for gurobi
  
  VectorXd w = (pdata->w_qdd.array() + REG).matrix();
  #ifdef USE_MATRIX_INVERSION_LEMMA
  bool include_body_accel_cost_terms = pdata->n_body_accel_inputs > 0 && pdata->body_accel_input_weights.array().maxCoeff() > 1e-10;
  if (use_fast_qp > 0 && !include_angular_momentum && !include_body_accel_cost_terms)
  { 
    // TODO: update to include angular momentum, body accel objectives.

  	//    We want Hqp inverse, which I can compute efficiently using the
  	//    matrix inversion lemma (see wikipedia):
  	//    inv(A + U'CV) = inv(A) - inv(A)*U* inv([ inv(C)+ V*inv(A)*U ]) V inv(A)
  	if (nc>0) {
      MatrixXd Wi = ((1/(pdata->w_qdd.array() + REG)).matrix()).asDiagonal();
  		if (R_DQyD_ls.trace()>1e-15) { // R_DQyD_ls is not zero
  			pdata->Hqp = Wi - Wi*Jcom.transpose()*(R_DQyD_ls.inverse() + Jcom*Wi*Jcom.transpose()).inverse()*Jcom*Wi;
      }
  	} 
    else {
    	pdata->Hqp = MatrixXd::Constant(nq,1,1/(1+REG));
  	}

	  #ifdef TEST_FAST_QP
  	  if (nc>0) {
        MatrixXd Hqp_test(nq,nq);
        MatrixXd W = w.asDiagonal();
        Hqp_test = (Jcom.transpose()*R_DQyD_ls*Jcom + W).inverse();
    	  if (((Hqp_test-pdata->Hqp).array().abs()).maxCoeff() > 1e-6) {
    		  mexErrMsgTxt("Q submatrix inverse from matrix inversion lemma does not match direct Q inverse.");
        }
      }
	  #endif

    Qnfdiag = MatrixXd::Constant(nf,1,1/REG);
    Qneps = MatrixXd::Constant(neps,1,1/(.001+REG));

    QBlkDiag[0] = &pdata->Hqp;
    if (nc>0) {
    	QBlkDiag[1] = &Qnfdiag;
    	QBlkDiag[2] = &Qneps;     // quadratic slack var cost, Q(nparams-neps:end,nparams-neps:end)=eye(neps)
    }

    MatrixXd Ain_lb_ub(n_ineq+2*nparams,nparams);
    VectorXd bin_lb_ub(n_ineq+2*nparams);
    Ain_lb_ub << Ain, 			     // note: obvious sparsity here
    		-MatrixXd::Identity(nparams,nparams),
    		MatrixXd::Identity(nparams,nparams);
    bin_lb_ub << bin, -lb, ub;

    info = fastQPThatTakesQinv(QBlkDiag, f, Aeq, beq, Ain_lb_ub, bin_lb_ub, pdata->active, alpha);

    //if (info<0)  	mexPrintf("fastQP info = %d.  Calling gurobi.\n", info);
  }
  else {
  #endif

    if (nc>0) {
      pdata->Hqp = Jcom.transpose()*R_DQyD_ls*Jcom;
      if (include_angular_momentum) {
        pdata->Hqp += pdata->Ak.transpose()*pdata->W_kdot*pdata->Ak;
      }
      pdata->Hqp += pdata->w_qdd.asDiagonal();
      pdata->Hqp += REG*MatrixXd::Identity(nq,nq);
    } else {
      pdata->Hqp = (1+REG)*MatrixXd::Identity(nq,nq);
    }

    // add in body spatial acceleration cost terms
    double w_i;
    for (int i=0; i<pdata->n_body_accel_inputs; i++) {
      w_i=pdata->body_accel_input_weights(i);
      if (w_i > 0) {
        body_vdot = body_accel_inputs[i].bottomRows(6);
        body_idx = (int)(body_accel_inputs[i][0])-1;
        
        if (!inSupport(active_supports,body_idx)) {
          pdata->r->forwardJac(body_idx,orig,1,Jb);
          pdata->r->forwardJacDot(body_idx,orig,1,Jbdot);

          for (int j=0; j<6; j++) {
            if (!std::isnan(body_vdot[j])) {
              pdata->Hqp += w_i*(Jb.row(j)).transpose()*Jb.row(j);
              f.head(nq) += w_i*(qd.transpose()*Jbdot.row(j).transpose() - body_vdot[j])*Jb.row(j).transpose();
            }
          }
        }
      }
    }

    Qnfdiag = MatrixXd::Constant(nf,1,pdata->w_grf+REG);
    Qneps = MatrixXd::Constant(neps,1,pdata->w_slack+REG);

    QBlkDiag[0] = &pdata->Hqp;
    if (nc>0) {
      QBlkDiag[1] = &Qnfdiag;
      QBlkDiag[2] = &Qneps;     // quadratic slack var cost, Q(nparams-neps:end,nparams-neps:end)=eye(neps)
    }


    MatrixXd Ain_lb_ub(n_ineq+2*nparams,nparams);
    VectorXd bin_lb_ub(n_ineq+2*nparams);
    Ain_lb_ub << Ain,            // note: obvious sparsity here
        -MatrixXd::Identity(nparams,nparams),
        MatrixXd::Identity(nparams,nparams);
    bin_lb_ub << bin, -lb, ub;


    if (use_fast_qp > 0)
    { // set up and call fastqp
      info = fastQP(QBlkDiag, f, Aeq, beq, Ain_lb_ub, bin_lb_ub, pdata->active, alpha);
      //if (info<0)    mexPrintf("fastQP info=%d... calling Gurobi.\n", info);
    }
    else {
      // use gurobi active set 
      model = gurobiActiveSetQP(pdata->env,QBlkDiag,f,Aeq,beq,Ain,bin,lb,ub,pdata->vbasis,pdata->vbasis_len,pdata->cbasis,pdata->cbasis_len,alpha);
      CGE(GRBgetintattr(model,"NumVars",&pdata->vbasis_len), pdata->env);
      CGE(GRBgetintattr(model,"NumConstrs",&pdata->cbasis_len), pdata->env);
      info=66;
      //info = -1;
    }

    if (info<0) {
      model = gurobiQP(pdata->env,QBlkDiag,f,Aeq,beq,Ain,bin,lb,ub,pdata->active,alpha);
      int status; CGE(GRBgetintattr(model, "Status", &status), pdata->env);
      //if (status!=2) mexPrintf("Gurobi reports non-optimal status = %d\n", status);
    }
  #ifdef USE_MATRIX_INVERSION_LEMMA
  }
  #endif

  //----------------------------------------------------------------------
  // Solve for inputs ----------------------------------------------------
  VectorXd y(nu);
  VectorXd qdd = alpha.head(nq);
  VectorXd beta = alpha.segment(nq,nc*nd);

  // use transpose because B_act is orthogonal
  y = pdata->B_act.transpose()*(pdata->H_act*qdd + pdata->C_act - D_act*beta);
  //y = pdata->B_act.jacobiSvd(ComputeThinU|ComputeThinV).solve(pdata->H_act*qdd + pdata->C_act - Jz_act.transpose()*lambda - D_act*beta);
  
  if (nlhs>0) {
    plhs[0] = eigenToMatlab(y);
  }
  
  if (nlhs>1) {
    plhs[1] = eigenToMatlab(qdd);
  }

  if (nlhs>2) {
    plhs[2] = mxCreateNumericMatrix(1,1,mxINT32_CLASS,mxREAL);
    memcpy(mxGetData(plhs[2]),&info,sizeof(int));
  }

  if (nlhs>3) {
      plhs[3] = mxCreateDoubleMatrix(1,active_supports.size(),mxREAL);
      pr = mxGetPrSafe(plhs[3]);
      int i=0;
      for (vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>>::iterator iter = active_supports.begin(); iter!=active_supports.end(); iter++) {
          pr[i++] = (double) (iter->body_idx + 1);
      }
  }

  if (nlhs>4) {
    plhs[4] = eigenToMatlab(alpha);
  }

  if (nlhs>5) {
    plhs[5] = eigenToMatlab(pdata->Hqp);
  }

  if (nlhs>6) {
    plhs[6] = eigenToMatlab(f);
  }

  if (nlhs>7) {
    plhs[7] = eigenToMatlab(Aeq);
  }

  if (nlhs>8) {
    plhs[8] = eigenToMatlab(beq);
  }

  if (nlhs>9) {
    plhs[9] = eigenToMatlab(Ain_lb_ub);
  }

  if (nlhs>10) {
    plhs[10] = eigenToMatlab(bin_lb_ub);
  }

  if (nlhs>11) {
    plhs[11] = eigenToMatlab(Qnfdiag);
  }

  if (nlhs>12) {
    plhs[12] = eigenToMatlab(Qneps);
  }

  if (nlhs>13) {
    double Vdot;
    if (nc>0) 
      // note: Sdot is 0 for ZMP/double integrator dynamics, so we omit that term here
      Vdot = ((2*x_bar.transpose()*S + s1.transpose())*(A_ls*x_bar + B_ls*(Jcomdotv + Jcom*qdd)) + s1dot.transpose()*x_bar)(0) + s2dot;
    else
      Vdot = 0;
    plhs[13] = mxCreateDoubleScalar(Vdot);
  }

  if (nlhs>14) {
    RigidBodyManipulator* r = pdata->r;

    VectorXd individual_cops = individualSupportCOPs(r, active_supports, normals, B, beta);
    plhs[14] = eigenToMatlab(individual_cops);
  }

  if (model) { 
    GRBfreemodel(model); 
  } 
  //  GRBfreeenv(env);
} 
