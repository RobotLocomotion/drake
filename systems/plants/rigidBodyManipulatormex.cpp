#include "mex.h"
#include <iostream>
#include <cmath>
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"
#include <stdexcept>

#include "joints/drakeJointUtil.h"

using namespace Eigen;
using namespace std;


/************************************************************************************************
 * This file contains the mex wrappers for the primary RigidBodyManipulator class methods, and
 * rigidBodyManipulatormex(command,...) provides a single entry point into all of the routines.
 *
 * TODO: Consider making this a derived class from RigidBodyManipulator, with the methods below
 * simply overloading the primary methods but accepting the mex i/o arguments.
 ************************************************************************************************/

enum Command { // note: you should be able to copy and paste the lines below into RigidBodyManipulator.m to make sure everything stays in sync
  DELETE_MODEL_COMMAND = 0,
  CONSTRUCT_MODEL_COMMAND = 1,
  H_AND_C_COMMAND = 2,
  DO_KINEMATICS_COMMAND = 3,
  FORWARD_KIN_COMMAND = 4,
  BODY_KIN_COMMAND = 5,
  COLLISION_DETECT_COMMAND = 6,
  SMOOTH_DISTANCE_PENALTY_COMMAND = 7,
  COLLISION_RAYCAST_COMMAND = 8,
  ALL_COLLISIONS_COMMAND = 9,
  GET_CMM_COMMAND = 10,
  FIND_KINEMATIC_PATH_COMMAND = 11,
  GEOMETRIC_JACOBIAN_COMMAND = 12,
  DO_KINEMATICS_NEW_COMMAND = 13,
  FORWARD_KIN_V_COMMAND = 14,
  FORWARD_KIN_POSITION_GRADIENT_COMMAND = 15,
  CENTER_OF_MASS_COMMAND = 16,
  CENTROIDAL_MOMENTUM_MATRIX_COMMAND = 17,
  MASS_MATRIX_COMMAND = 18,
  INVERSE_DYNAMICS_COMMAND = 19
};

// convert Matlab cell array of strings into a C++ vector of strings
vector<string> get_strings(const mxArray *rhs) {
  int num = static_cast<int>(mxGetNumberOfElements(rhs));
  vector<string> strings(num);
  for (int i=0; i<num; i++) {
    const mxArray *ptr = mxGetCell(rhs,i);
    int buflen = static_cast<int>(mxGetN(ptr)*sizeof(mxChar))+1;
    char* str = (char*)mxMalloc(buflen);
    mxGetString(ptr, str, buflen);
    strings[i] = string(str);
    mxFree(str);
  }
  return strings;
}



void constructModel( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )
{
  //DEBUG
  //cout << "rigidBodyManipulatormex:constructModel: START" << endl;
  //END_DEBUG
  char buf[100];
  mxArray *pm;

  if (nrhs!=1) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:constructModel:BadInputs","Usage model_ptr = rigidBodyManipulatormex(CONSTRUCT_MODEL_COMMAND,obj)");
  }

  const mxArray* pRBM = prhs[0];
  RigidBodyManipulator *model=NULL;

//  model->robot_name = get_strings(mxGetProperty(pRBM,0,"name"));

  const mxArray* featherstone = mxGetProperty(pRBM,0,"featherstone");
  if (!featherstone) mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:constructModel:BadInputs", "the featherstone array is invalid");

  const mxArray* pBodies = mxGetProperty(pRBM,0,"body");
  if (!pBodies) mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:constructModel:BadInputs","the body array is invalid");
  int num_bodies = static_cast<int>(mxGetNumberOfElements(pBodies));

  const mxArray* pFrames = mxGetProperty(pRBM,0,"frame");
  if (!pFrames) mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:constructModel:BadInputs","the frame array is invalid");
  int num_frames = static_cast<int>(mxGetNumberOfElements(pFrames));

  // set up the model
  pm = mxGetField(featherstone,0,"NB");
  if (!pm) mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:constructModel:BadInputs","can't find field model.featherstone.NB.  Are you passing in the correct structure?");
  model = new RigidBodyManipulator((int) mxGetScalar(pm), (int) mxGetScalar(pm), num_bodies, num_frames);

  pm = mxGetField(featherstone,0,"parent");
  if (!pm) mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:constructModel:BadInputs","can't find field model.featherstone.parent.");
  double* parent = mxGetPr(pm);

  pm = mxGetField(featherstone,0,"pitch");
  if (!pm) mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:constructModel:BadInputs","can't find field model.featherstone.pitch.");
  double* pitch = mxGetPr(pm);

  pm = mxGetField(featherstone,0,"position_num");
  if (!pm) mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:constructModel:BadInputs","can't find field model.featherstone.position_num.");
  double* dofnum = mxGetPr(pm);
//  throw runtime_error("here");

  pm = mxGetField(featherstone,0,"damping");
  if (!pm) mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:constructModel:BadInputs","can't find field model.featherstone.damping.");
  memcpy(model->damping.data(),mxGetPr(pm),sizeof(double)*model->NB);

  pm = mxGetField(featherstone,0,"coulomb_friction");
  if (!pm) mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:constructModel:BadInputs","can't find field model.featherstone.coulomb_friction.");
  memcpy(model->coulomb_friction.data(),mxGetPr(pm),sizeof(double)*model->NB);

  pm = mxGetField(featherstone,0,"static_friction");
  if (!pm) mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:constructModel:BadInputs","can't find field model.featherstone.static_friction.");
  memcpy(model->static_friction.data(),mxGetPr(pm),sizeof(double)*model->NB);

  pm = mxGetField(featherstone,0,"coulomb_window");
  if (!pm) mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:constructModel:BadInputs","can't find field model.featherstone.coulomb_window.");
  memcpy(model->coulomb_window.data(),mxGetPr(pm),sizeof(double)*model->NB);

  mxArray* pXtree = mxGetField(featherstone,0,"Xtree");
  if (!pXtree) mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:constructModel:BadInputs","can't find field model.featherstone.Xtree.");

  mxArray* pI = mxGetField(featherstone,0,"I");
  if (!pI) mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:constructModel:BadInputs","can't find field model.featherstone.I.");

  for (int i=0; i<model->NB; i++) {
    model->parent[i] = ((int) parent[i]) - 1;  // since it will be used as a C index
    model->pitch[i] = (int) pitch[i];
    model->dofnum[i] = (int) dofnum[i] - 1; // zero-indexed

    mxArray* pXtreei = mxGetCell(pXtree,i);
    if (!pXtreei) mexErrMsgIdAndTxt("Drake:HandCpmex:BadInputs","can't access model.featherstone.Xtree{%d}",i);

    // todo: check that the size is 6x6
    memcpy(model->Xtree[i].data(),mxGetPr(pXtreei),sizeof(double)*6*6);

    mxArray* pIi = mxGetCell(pI,i);
    if (!pIi) mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:constructModel:BadInputs","can't access model.featherstone.I{%d}",i);

    // todo: check that the size is 6x6
    memcpy(model->I[i].data(),mxGetPr(pIi),sizeof(double)*6*6);
  }

  for (int i=0; i<model->num_bodies; i++) {
    //DEBUG
    //cout << "rigidBodyManipulatormex:constructModel: body " << i << endl;
    //END_DEBUG
    pm = mxGetProperty(pBodies,i,"linkname");
    mxGetString(pm,buf,100);
    model->bodies[i]->linkname.assign(buf,strlen(buf));

    pm = mxGetProperty(pBodies,i,"jointname");
    mxGetString(pm,buf,100);
    model->bodies[i]->jointname.assign(buf,strlen(buf));

    pm = mxGetProperty(pBodies,i,"robotnum");
    model->bodies[i]->robotnum = (int) mxGetScalar(pm)-1;

    pm = mxGetProperty(pBodies,i,"mass");
    model->bodies[i]->mass = (double) mxGetScalar(pm);

    pm = mxGetProperty(pBodies,i,"com");
    if (!mxIsEmpty(pm)) memcpy(model->bodies[i]->com.data(),mxGetPr(pm),sizeof(double)*3);

    pm = mxGetProperty(pBodies,i,"I");
    if (!mxIsEmpty(pm)) memcpy(model->bodies[i]->I.data(),mxGetPr(pm),sizeof(double)*6*6);

    pm = mxGetProperty(pBodies,i,"position_num");
    model->bodies[i]->dofnum = (int) mxGetScalar(pm) - 1;  //zero-indexed

    pm = mxGetProperty(pBodies,i,"velocity_num");
    model->bodies[i]->velocity_num_start = (int) mxGetScalar(pm) - 1;  //zero-indexed

    pm = mxGetProperty(pBodies,i,"parent");
    if (!pm || mxIsEmpty(pm))
      model->bodies[i]->parent = -1;
    else
      model->bodies[i]->parent = static_cast<int>(mxGetScalar(pm)) - 1;

    {
      mxGetString(mxGetProperty(pBodies, i, "jointname"), buf, 100);
      string jointname;
      jointname.assign(buf, strlen(buf));

      pm = mxGetProperty(pBodies, i, "Ttree");
      // todo: check that the size is 4x4
      Isometry3d Ttree;
      memcpy(Ttree.data(), mxGetPr(pm), sizeof(double) * 4 * 4);

      int floating = (int) mxGetScalar(mxGetProperty(pBodies, i, "floating"));

      Eigen::Vector3d joint_axis;
      pm = mxGetProperty(pBodies, i, "joint_axis");
      memcpy(joint_axis.data(), mxGetPr(pm), sizeof(double) * 3);

      double pitch = mxGetScalar(mxGetProperty(pBodies, i, "pitch"));

      if (model->bodies[i]->hasParent()) {
        model->bodies[i]->setJoint(createJoint(jointname, Ttree, floating, joint_axis, pitch));
//        mexPrintf((model->bodies[i]->getJoint().getName() + ": " + std::to_string(model->bodies[i]->getJoint().getNumVelocities()) + "\n").c_str());
      }
    }
    {
      pm = mxGetProperty(pBodies,i,"jointname");
      mxGetString(pm,buf,100);
      model->bodies[i]->jointname.assign(buf,strlen(buf));

      pm = mxGetProperty(pBodies,i,"Ttree");
      // todo: check that the size is 4x4
      memcpy(model->bodies[i]->Ttree.data(),mxGetPr(pm),sizeof(double)*4*4);

      pm = mxGetProperty(pBodies,i,"floating");
      model->bodies[i]->floating = (int) mxGetScalar(pm);

      pm = mxGetProperty(pBodies,i,"pitch");
      model->bodies[i]->pitch = (int) mxGetScalar(pm);
    }

    if (model->bodies[i]->dofnum>=0) {
       pm = mxGetProperty(pBodies,i,"joint_limit_min");
       model->joint_limit_min[model->bodies[i]->dofnum] = mxGetScalar(pm);
       pm = mxGetProperty(pBodies,i,"joint_limit_max");
       model->joint_limit_max[model->bodies[i]->dofnum] = mxGetScalar(pm);
    }

    pm = mxGetProperty(pBodies,i,"T_body_to_joint");
    memcpy(model->bodies[i]->T_body_to_joint.data(),mxGetPr(pm),sizeof(double)*4*4);

    //DEBUG
    //cout << "rigidBodyManipulatormex:constructModel: About to parse collision geometry"  << endl;
    //END_DEBUG
    pm = mxGetProperty(pBodies,i,"collision_geometry");
    Matrix4d T;
    if (!mxIsEmpty(pm)){
      for (int j=0; j<mxGetNumberOfElements(pm); j++) {
        //DEBUG
        //cout << "rigidBodyManipulatormex:constructModel: Body " << i << ", Element " << j << endl;
        //END_DEBUG
        mxArray* pShape = mxGetCell(pm,j);
        char* group_name_cstr = mxArrayToString(mxGetProperty(pShape,0,"name"));
        string group_name;
        if (group_name_cstr) {
          group_name = group_name_cstr;
        } else {
          group_name = "default";
        }

        // Get element-to-link transform from MATLAB object
        memcpy(T.data(), mxGetPr(mxGetProperty(pShape,0,"T")), sizeof(double)*4*4);
        auto shape = (DrakeCollision::Shape)static_cast<int>(mxGetScalar(mxGetProperty(pShape,0,"bullet_shape_id")));
        vector<double> params_vec;
        switch (shape) {
          case DrakeCollision::BOX:
          {
            double* params = mxGetPr(mxGetProperty(pShape,0,"size"));
            params_vec.push_back(params[0]);
            params_vec.push_back(params[1]);
            params_vec.push_back(params[2]);
          }
            break;
          case DrakeCollision::SPHERE:
          {
            params_vec.push_back(*mxGetPr(mxGetProperty(pShape,0,"radius")));
          }
            break;
          case DrakeCollision::CYLINDER:
          {
            params_vec.push_back(*mxGetPr(mxGetProperty(pShape,0,"radius")));
            params_vec.push_back(*mxGetPr(mxGetProperty(pShape,0,"len")));
          }
            break;
          case DrakeCollision::MESH:
          {
            mxArray* pPoints;
            mexCallMATLAB(1,&pPoints,1,&pShape,"getPoints");
            double* params = mxGetPr(pPoints);
            int n_params = (int) mxGetNumberOfElements(pPoints);
            for (int k=0; k<n_params; k++)
              params_vec.push_back(params[k]);
            mxDestroyArray(pPoints);
            // The element-to-link transform is applied in
            // RigidBodyMesh/getPoints - don't apply it again!
            T = Matrix4d::Identity();
          }
            break;
          case DrakeCollision::CAPSULE:
          {
            params_vec.push_back(*mxGetPr(mxGetProperty(pShape,0,"radius")));
            params_vec.push_back(*mxGetPr(mxGetProperty(pShape,0,"len")));
          }
            break;
          default:
            // intentionally do nothing..
            break;
        }
        model->addCollisionElement(i,T,shape,params_vec,group_name);
        if (model->bodies[i]->parent<0) {
          model->updateCollisionElements(i);  // update static objects only once - right here on load
        }
      }


      // Set collision filtering bitmasks
      pm = mxGetProperty(pBodies,i,"collision_filter");
      const uint16_t* group = (uint16_t*)mxGetPr(mxGetField(pm,0,"belongs_to"));
      const uint16_t* mask = (uint16_t*)mxGetPr(mxGetField(pm,0,"ignores"));
      //DEBUG
      //cout << "rigidBodyManipulatormex:constructModel: Group: " << *group << endl;
      //cout << "rigidBodyManipulatormex:constructModel: Mask " << *mask << endl;
      //END_DEBUG
      model->setCollisionFilter(i,*group,*mask);
    }
  }

  // THIS IS UGLY: I'm sending the terrain contact points into the
  // contact_pts field of the cpp RigidBody objects
  //DEBUG
  //cout << "rigidBodyManipulatormex:constructModel: Parsing contact points " << endl;
  //cout << "rigidBodyManipulatormex:constructModel: Get struct" << endl;
  //END_DEBUG
  mxArray* contact_pts_struct[1];
  if (~mexCallMATLAB(1,contact_pts_struct,1,const_cast<mxArray**>(&pRBM),"getTerrainContactPoints")) {
    //DEBUG
    //cout << "rigidBodyManipulatormex:constructModel: Got struct" << endl;
    //if (contact_pts_struct) {
    //cout << "rigidBodyManipulatormex:constructModel: Struct pointer: " << contact_pts_struct << endl;
    //} else {
    //cout << "rigidBodyManipulatormex:constructModel: Struct pointer NULL" << endl;
    //}
    //cout << "rigidBodyManipulatormex:constructModel: Get numel of struct" << endl;
    //END_DEBUG
    const int n_bodies_w_contact_pts = static_cast<int>(mxGetNumberOfElements(contact_pts_struct[0]));
    //DEBUG
    //cout << "rigidBodyManipulatormex:constructModel: Got numel of struct:" << n_bodies_w_contact_pts << endl;
    //END_DEBUG
    mxArray* pPts;
    int body_idx;
    int n_pts;
    for (int j=0; j < n_bodies_w_contact_pts; j++) {
      //DEBUG
      //cout << "rigidBodyManipulatormex:constructModel: Loop: Iteration " << j << endl;
      //cout << "rigidBodyManipulatormex:constructModel: Get body_idx" << endl;
      //END_DEBUG
      body_idx = (int) mxGetScalar(mxGetField(contact_pts_struct[0],j,"idx")) - 1;
      //DEBUG
      //cout << "rigidBodyManipulatormex:constructModel: Got body_idx: " << body_idx << endl;
      //cout << "rigidBodyManipulatormex:constructModel: Get points" << endl;
      //END_DEBUG
      pPts = mxGetField(contact_pts_struct[0],j,"pts");
      //DEBUG
      //cout << "rigidBodyManipulatormex:constructModel: Get points" << endl;
      //cout << "rigidBodyManipulatormex:constructModel: Get number of points" << endl;
      //END_DEBUG
      n_pts = static_cast<int>(mxGetN(pPts));
      //DEBUG
      //cout << "rigidBodyManipulatormex:constructModel: Got number of points: " << n_pts << endl;
      //cout << "rigidBodyManipulatormex:constructModel: Set contact_pts of body" << endl;
      //END_DEBUG
      Map<MatrixXd> pts(mxGetPr(pPts),3,n_pts);
      model->bodies[body_idx]->contact_pts.resize(4,n_pts);
      model->bodies[body_idx]->contact_pts << pts, MatrixXd::Ones(1,n_pts);
      //DEBUG
      //cout << "rigidBodyManipulatormex:constructModel: Contact_pts of body: " << endl;
      //cout << model->bodies[body_idx]->contact_pts << endl;
      //END_DEBUG
    }
  }

  for (int i=0; i<model->num_frames; i++) {
    pm = mxGetProperty(pFrames,i,"name");
    mxGetString(pm,buf,100);
    model->frames[i].name.assign(buf,strlen(buf));

    pm = mxGetProperty(pFrames,i,"body_ind");
    model->frames[i].body_ind = (int) mxGetScalar(pm)-1;

    pm = mxGetProperty(pFrames,i,"T");
    memcpy(model->frames[i].Ttree.data(),mxGetPr(pm),sizeof(double)*4*4);
  }


  memcpy(model->joint_limit_min.data(), mxGetPr(mxGetProperty(pRBM,0,"joint_limit_min")), sizeof(double)*model->num_dof);
  memcpy(model->joint_limit_max.data(), mxGetPr(mxGetProperty(pRBM,0,"joint_limit_max")), sizeof(double)*model->num_dof);

  const mxArray* a_grav_array = mxGetProperty(pRBM,0,"gravity");
  if (a_grav_array && mxGetNumberOfElements(a_grav_array)==3) {
    double* p = mxGetPr(a_grav_array);
    model->a_grav[3] = p[0];
    model->a_grav[4] = p[1];
    model->a_grav[5] = p[2];
  } else {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:constructModel:BadGravity","Couldn't find a 3 element gravity vector in the object.");
  }

  mxLogical* use_new_kinsol = mxGetLogicals(mxGetProperty(pRBM,0,"use_new_kinsol"));
  model->use_new_kinsol = (bool) use_new_kinsol[0];

  model->compile();

  mxArray* args[1];  args[0]=mxCreateDoubleScalar(DELETE_MODEL_COMMAND);
  plhs[0] = createDrakeMexPointer((void*)model,"RigidBodyManipulator",1,args);
}

void HandC( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) {
  if (nrhs<1) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:HandC:NotEnoughInputs","Usage [H,C,dH,dC] = rigidBodyManipulatormex(H_AND_C_COMMAND,model_ptr,q,qd[,f_ext,df_ext]).");
  }

  if (nrhs==4 && nlhs>2) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:HandC:NotEnoughInputs","You need to provide df_ext if you request dH or dC while supplying f_ext");
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  double *q,*qd;
  Map<MatrixXd> *f_ext=NULL;
  Map<MatrixXd> *df_ext=NULL;
  if (static_cast<int>(mxGetNumberOfElements(prhs[1]))!=model->num_dof || static_cast<int>(mxGetNumberOfElements(prhs[2]))!=model->num_dof)
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:HandC:BadInputs","q and qd must be size %d x 1",model->num_dof);
  q = mxGetPr(prhs[1]);
  qd = mxGetPr(prhs[2]);
  if (nrhs>3) {
    if (!mxIsEmpty(prhs[3])) {
      f_ext = new Map<MatrixXd>(mxGetPr(prhs[3]),6,model->NB);
    }
  }
  if (nrhs>4) {
    if (!mxIsEmpty(prhs[4])) {
      df_ext = new Map<MatrixXd>(mxGetPr(prhs[4]),6*model->NB,2*model->num_dof);
    }
  }

  Map<MatrixXd> *dH=NULL, *dC=NULL;

  plhs[0] = mxCreateDoubleMatrix(model->num_dof,model->num_dof,mxREAL);
  Map<MatrixXd> H(mxGetPr(plhs[0]),model->num_dof,model->num_dof);

  plhs[1] = mxCreateDoubleMatrix(model->num_dof,1,mxREAL);
  Map<VectorXd> C(mxGetPr(plhs[1]),model->num_dof);

  if (nlhs>2) {
    plhs[2] = mxCreateDoubleMatrix(model->num_dof*model->num_dof,model->num_dof,mxREAL);
    dH = new Map<MatrixXd>(mxGetPr(plhs[2]),model->num_dof*model->num_dof,model->num_dof);
  }
  if (nlhs>3) {
    plhs[3] = mxCreateDoubleMatrix(model->num_dof,2*model->num_dof,mxREAL);
    dC = new Map<MatrixXd>(mxGetPr(plhs[3]),model->num_dof,2*model->num_dof);
  }

  model->HandC(q,qd,f_ext,H,C,dH,dC,df_ext);

  // destroy dynamically allocated Map<MatrixXd> (but not the underlying data!)
  if (f_ext) delete f_ext;
  if (df_ext) delete df_ext;
  if (nlhs>2) delete dH;
  if (nlhs>3) delete dC;
}

void doKinematics( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) {
  if (nrhs != 4) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:doKinematics:NotEnoughInputs", "Usage rigidBodyManipulatormex(DO_KINEMATICS_COMMAND,model_ptr,q,b_compute_second_derivatives,qd)");
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  double *q, *qd=NULL;
  if (mxGetNumberOfElements(prhs[1])!=model->NB)
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:doKinematics:BadInputs", "q must be size %d x 1", model->NB);
  q = mxGetPr(prhs[1]);
  bool b_compute_second_derivatives = (mxGetScalar(prhs[2])!=0.0);
  if (mxGetNumberOfElements(prhs[3])>0)
    qd = mxGetPr(prhs[3]);

  model->doKinematics(q,b_compute_second_derivatives,qd);
}

void forwardKin( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {

  if (nrhs < 5) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:forwardKin:NotEnoughInputs","Usage rigidBodyManipulatormex(FORWARD_KIN_COMMAND,model_ptr,q_cache,0,robotnum,b_jacdot) for center of mass, or rigidBodyManipulatormex(4,model_pts,q_cache,body_ind,pts,rotation_type,b_jacdot)");
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  double* q = mxGetPr(prhs[1]);
  for (int i = 0; i < model->num_dof; i++) {
    if (q[i] - model->cached_q[i] > 1e-8 || q[i] - model->cached_q[i] < -1e-8) {
      mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:forwardKin:InvalidKinematics","This kinsol is no longer valid.  Somebody has called doKinematics with a different q since the solution was computed.");
    }
  }

  int body_ind = ((int) mxGetScalar(prhs[2])) - 1;  // note: this is body_ind-1 (so 0 to num_bodies-1)
  bool b_jacdot;
  if(body_ind != -1)
  {
    b_jacdot = nrhs>5 && (mxGetScalar(prhs[5])!=0.0);
  }
  else
  {
    b_jacdot = (mxGetScalar(prhs[4])!=0.0);
  }

  if (body_ind==-1) {  // compute center of mass
    int num_robot = static_cast<int>(mxGetNumberOfElements(prhs[3]));
    set<int> robotnum_set;
    double* probotnum = mxGetPr(prhs[3]);
    for(int i = 0;i<num_robot;i++)
    {
      robotnum_set.insert((int) probotnum[i]-1);
    }
    if (b_jacdot && nlhs>0) {
      plhs[0] = mxCreateDoubleMatrix(3,model->num_dof,mxREAL);
      Map<MatrixXd> Jdot(mxGetPr(plhs[0]),3,model->num_dof);
      model->getCOMJacDot(Jdot,robotnum_set);
      return;
    }

    if (nlhs>0) {
      plhs[0] = mxCreateDoubleMatrix(3,1,mxREAL);
      Map<Vector3d> x(mxGetPr(plhs[0]));
      model->getCOM(x,robotnum_set);
    }
    if (nlhs>1) {
      plhs[1] = mxCreateDoubleMatrix(3,model->num_dof,mxREAL);
      Map<MatrixXd> J(mxGetPr(plhs[1]),3,model->num_dof);
      model->getCOMJac(J,robotnum_set);
    }

    if (nlhs>2) {
      plhs[2] = mxCreateDoubleMatrix(3,model->num_dof*model->num_dof,mxREAL);
      Map<MatrixXd> dJ(mxGetPr(plhs[2]),3,model->num_dof*model->num_dof);
      model->getCOMdJac(dJ,robotnum_set);
    }

    return;
  } else if (body_ind<-(model->num_frames+1) || body_ind>=model->num_bodies) {
      mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:forwardKin:BadInputs","body_ind must be -1 (for com) or between -num_frames-1 and num_bodies-1");
  }

  if (nrhs != 5 &&nrhs !=6) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:forwardKin:NotEnoughInputs", "Usage rigidBodyManipulatormex(4,model_ptr,q_cache,body_index,pts,rotation_type,b_jacdot)");
  }

  int n_pts = static_cast<int>(mxGetN(prhs[3]));
  int dim = static_cast<int>(mxGetM(prhs[3])), dim_with_rot=dim;

//  if (dim != 2 && dim != 3)
//    mexErrMsgIdAndTxt("rigidBodyManipulatormex:forwardKin", "number of rows in pts must be 2 or 3");
  if (dim != 3)
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:forwardKin:BadInputs", "number of rows in pts must be 3");

  int rotation_type = (int)mxGetScalar(prhs[4]);
  if (rotation_type==1) dim_with_rot += 3;
  else if (rotation_type==2) dim_with_rot += 4;

  Map<MatrixXd> pts_tmp(mxGetPr(prhs[3]),dim,n_pts);
  MatrixXd pts(dim+1,n_pts);
  pts << pts_tmp, MatrixXd::Ones(1,n_pts);

  if (b_jacdot) {
    if (rotation_type>1) mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:forwardKin:NotImplemented","Jacobian dot of quaternions are not implemented yet");

    plhs[0] = mxCreateDoubleMatrix(dim_with_rot*n_pts,model->num_dof,mxREAL);
    Map<MatrixXd> Jdot(mxGetPr(plhs[0]),dim_with_rot*n_pts,model->num_dof);
    model->forwardJacDot(body_ind,pts,rotation_type,Jdot);
    return;
  } else {
    if (nlhs>0) {
      plhs[0] = mxCreateDoubleMatrix(dim_with_rot,n_pts,mxREAL);
      Map<MatrixXd> x(mxGetPr(plhs[0]),dim_with_rot,n_pts);
      model->forwardKin(body_ind,pts,rotation_type,x);
    }
    if (nlhs>1) {
      plhs[1] = mxCreateDoubleMatrix(dim_with_rot*n_pts,model->num_dof,mxREAL);
      Map<MatrixXd> J(mxGetPr(plhs[1]),dim_with_rot*n_pts,model->num_dof);
      model->forwardJac(body_ind,pts,rotation_type,J);
    }

    if (nlhs>2) {
      if (rotation_type>0) mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:forwardKin:NotImplemented","Second derivatives of rotations are not implemented yet");
      plhs[2] = mxCreateDoubleMatrix(dim*n_pts,model->num_dof*model->num_dof,mxREAL);
      Map<MatrixXd> dJ(mxGetPr(plhs[2]),dim*n_pts,model->num_dof*model->num_dof);
      model->forwarddJac(body_ind,pts,dJ);
    }
  }
}

void bodyKin( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {

  if (nrhs < 3) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:bodyKin:NotEnoughInputs","Usage [x,J,P]=rigidBodyManupulatormex(BODY_KIN_COMMAND,model_ptr,q_cache,body_ind,pts)");
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  double* q = mxGetPr(prhs[1]);
  for (int i = 0; i < model->num_dof; i++) {
    if (q[i] - model->cached_q[i] > 1e-8 || q[i] - model->cached_q[i] < -1e-8) {
      mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:bodyKin:InvalidKinematics","This kinsol is no longer valid.  Somebody has called doKinematics with a different q since the solution was computed.");
    }
  }

  int body_ind = ((int) mxGetScalar(prhs[2])) - 1;  // note: this is body_ind-1 (so 0 to num_bodies-1)

  if (body_ind<-(model->num_frames+1) || body_ind>=model->num_bodies) {
      mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:bodyKin:BadInputs","body_ind must be -1 (for com) or between -num_frames-1 and num_bodies-1");
  }

  int n_pts = static_cast<int>(mxGetN(prhs[3]));
  int dim = static_cast<int>(mxGetM(prhs[3]));

  if (dim != 3)
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:bodyKin:BadInputs", "number of rows in pts must be 3");

  Map<MatrixXd> pts_tmp(mxGetPr(prhs[3]),dim,n_pts);
  MatrixXd pts(dim+1,n_pts);
  pts << pts_tmp, MatrixXd::Ones(1,n_pts);

  plhs[0] = mxCreateDoubleMatrix(dim,n_pts,mxREAL);
  Map<MatrixXd> x(mxGetPr(plhs[0]),dim,n_pts);

  Map<MatrixXd> *J=NULL, *P=NULL;

  if (nlhs>1) {
    plhs[1] = mxCreateDoubleMatrix(dim*n_pts,model->num_dof,mxREAL);
    J = new Map<MatrixXd>(mxGetPr(plhs[1]),dim*n_pts,model->num_dof);
  }
  if (nlhs>2) {
    plhs[2] = mxCreateDoubleMatrix(dim*n_pts,dim*n_pts,mxREAL);
    P = new Map<MatrixXd>(mxGetPr(plhs[2]),dim*n_pts,dim*n_pts);
  }

  model->bodyKin(body_ind,pts,x,J,P);

  if (nlhs>1) delete J;
  if (nlhs>2) delete P;

}

void collisionDetect( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] )
{
  if (nrhs < 1) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:collisionDetect:NotEnoughInputs","Usage rigidBodyManipulatormex(COLLISION_DETECT_COMMAND,model_ptr)");
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  // Parse `active_collision_options`
  vector<int> active_bodies_idx;
  set<string> active_group_names;
  // First get the list of body indices for which to compute distances
  const mxArray* active_collision_options = prhs[2];
  const mxArray* body_idx = mxGetField(active_collision_options,0,"body_idx");
  if (body_idx != NULL) {
    //DEBUG
    //cout << "collisionDetectmex: Received body_idx" << endl;
    //END_DEBUG
    int n_active_bodies = static_cast<int>(mxGetNumberOfElements(body_idx));
    //DEBUG
    //cout << "collisionDetectmex: n_active_bodies = " << n_active_bodies << endl;
    //END_DEBUG
    active_bodies_idx.resize(n_active_bodies);
    memcpy(active_bodies_idx.data(),(int*) mxGetData(body_idx),
           sizeof(int)*n_active_bodies);
    transform(active_bodies_idx.begin(),active_bodies_idx.end(),
              active_bodies_idx.begin(),
              [](int i){return --i;});
  }

  // Then get the group names for which to compute distances
  const mxArray* collision_groups = mxGetField(active_collision_options,0,
                                               "collision_groups");
  if (collision_groups != NULL) {
    int num = static_cast<int>(mxGetNumberOfElements(collision_groups));
    for (int i=0; i<num; i++) {
      const mxArray *ptr = mxGetCell(collision_groups,i);
      int buflen = static_cast<int>(mxGetN(ptr)*sizeof(mxChar))+1;
      char* str = (char*)mxMalloc(buflen);
      mxGetString(ptr, str, buflen);
      active_group_names.insert(str);
      mxFree(str);
    }
  }

  vector<int> bodyA_idx, bodyB_idx;
  MatrixXd ptsA, ptsB, normals, JA, JB, Jd;
  VectorXd dist;
  if (active_bodies_idx.size() > 0) {
    if (active_group_names.size() > 0) {
      model-> collisionDetect(dist, normals, ptsA, ptsB, bodyA_idx, bodyB_idx,
                              active_bodies_idx,active_group_names);
    } else {
      model-> collisionDetect(dist, normals, ptsA, ptsB, bodyA_idx, bodyB_idx,
                              active_bodies_idx);
    }
  } else {
    if (active_group_names.size() > 0) {
      model-> collisionDetect(dist, normals, ptsA, ptsB, bodyA_idx, bodyB_idx,
                             active_group_names);
    } else {
      model-> collisionDetect(dist, normals, ptsA, ptsB, bodyA_idx, bodyB_idx);
    }
  }

  vector<int32_T> idxA(bodyA_idx.size());
  transform(bodyA_idx.begin(),bodyA_idx.end(),idxA.begin(),
      [](int i){return ++i;});
  vector<int32_T> idxB(bodyB_idx.size());
  transform(bodyB_idx.begin(),bodyB_idx.end(),idxB.begin(),
      [](int i){return ++i;});

  if (nlhs>0) {
    plhs[0] = mxCreateDoubleMatrix(3,static_cast<int>(ptsA.cols()),mxREAL);
    memcpy(mxGetPr(plhs[0]),ptsA.data(),sizeof(double)*3*ptsA.cols());
  }
  if (nlhs>1) {
    plhs[1] = mxCreateDoubleMatrix(3,static_cast<int>(ptsB.cols()),mxREAL);
    memcpy(mxGetPr(plhs[1]),ptsB.data(),sizeof(double)*3*ptsB.cols());
  }
  if (nlhs>2) {
    plhs[2] = mxCreateDoubleMatrix(3,static_cast<int>(normals.cols()),mxREAL);
    memcpy(mxGetPr(plhs[2]),normals.data(),sizeof(double)*3*normals.cols());
  }
  if (nlhs>3) {
    plhs[3] = mxCreateDoubleMatrix(1,static_cast<int>(dist.size()),mxREAL);
    memcpy(mxGetPr(plhs[3]),dist.data(),sizeof(double)*dist.size());
  }
  if (nlhs>4) {
    plhs[4] = mxCreateNumericMatrix(1,static_cast<int>(idxA.size()),mxINT32_CLASS,mxREAL);
    memcpy(mxGetPr(plhs[4]),idxA.data(),sizeof(int32_T)*idxA.size());
  }
  if (nlhs>5) {
    plhs[5] = mxCreateNumericMatrix(1,static_cast<int>(idxB.size()),mxINT32_CLASS,mxREAL);
    memcpy(mxGetPr(plhs[5]),idxB.data(),sizeof(int32_T)*idxB.size());
  }
}

void
smoothDistancePenalty(double& c, MatrixXd& dc,
                      RigidBodyManipulator* robot,
                      const double min_distance,
                      const VectorXd& dist,
                      const MatrixXd& normal,
                      const MatrixXd& xA,
                      const MatrixXd& xB,
                      const vector<int>& idxA,
                      const vector<int>& idxB)
{
  VectorXd scaled_dist, pairwise_costs;
  MatrixXd ddist_dq, dscaled_dist_ddist, dpairwise_costs_dscaled_dist;

  int num_pts = static_cast<int>(xA.cols());
  ddist_dq = MatrixXd::Zero(num_pts,robot->num_dof);

  // Scale distance
  int nd = static_cast<int>(dist.size());
  double recip_min_dist = 1/min_distance;
  scaled_dist = recip_min_dist*dist - VectorXd::Ones(nd,1);
  dscaled_dist_ddist = recip_min_dist*MatrixXd::Identity(nd,nd);
  // Compute penalties
  pairwise_costs = VectorXd::Zero(nd,1);
  dpairwise_costs_dscaled_dist = MatrixXd::Zero(nd,nd);
  for (int i = 0; i < nd; ++i) {
    if (scaled_dist(i) < 0) {
      double exp_recip_scaled_dist = exp(1/scaled_dist(i));
      pairwise_costs(i) = -scaled_dist(i)*exp_recip_scaled_dist;
      dpairwise_costs_dscaled_dist(i,i) = exp_recip_scaled_dist*(1/scaled_dist(i) - 1);
    }
  }

  // Compute Jacobian of closest distance vector
  std::vector< std::vector<int> > orig_idx_of_pt_on_bodyA(robot->num_bodies);
  std::vector< std::vector<int> > orig_idx_of_pt_on_bodyB(robot->num_bodies);
  for (int k = 0; k < num_pts; ++k) {
    //DEBUG
    //std::cout << "MinDistanceConstraint::eval: First loop: " << k << std::endl;
    //std::cout << "pairwise_costs.size() = " << pairwise_costs.size() << std::endl;
    //std::cout << "pairwise_costs.size() = " << pairwise_costs.size() << std::endl;
    //END_DEBUG
    if (pairwise_costs(k) > 0) {
      orig_idx_of_pt_on_bodyA.at(idxA.at(k)).push_back(k);
      orig_idx_of_pt_on_bodyB.at(idxB.at(k)).push_back(k);
    }
  }
  for (int k = 0; k < robot->num_bodies; ++k) {
    //DEBUG
    //std::cout << "MinDistanceConstraint::eval: Second loop: " << k << std::endl;
    //END_DEBUG
    int l = 0;
    int numA = static_cast<int>(orig_idx_of_pt_on_bodyA.at(k).size());
    int numB = static_cast<int>(orig_idx_of_pt_on_bodyB.at(k).size());
    if(numA+numB == 0)
    {
      continue;
    }
    MatrixXd x_k(3, numA + numB);
    for (; l < numA; ++l) {
      //DEBUG
      //std::cout << "MinDistanceConstraint::eval: Third loop: " << l << std::endl;
      //END_DEBUG
      x_k.col(l) = xA.col(orig_idx_of_pt_on_bodyA.at(k).at(l));
    }
    for (; l < numA + numB; ++l) {
      //DEBUG
      //std::cout << "MinDistanceConstraint::eval: Fourth loop: " << l << std::endl;
      //END_DEBUG
      x_k.col(l) = xB.col(orig_idx_of_pt_on_bodyB.at(k).at(l-numA));
    }
    MatrixXd x_k_1(4,x_k.cols());
    MatrixXd J_k(3*x_k.cols(),robot->num_dof);
    x_k_1 << x_k, MatrixXd::Ones(1,x_k.cols());
    robot->forwardJac(k,x_k_1,0,J_k);
    l = 0;
    for (; l < numA; ++l) {
      //DEBUG
      //std::cout << "MinDistanceConstraint::eval: Fifth loop: " << l << std::endl;
      //END_DEBUG
      ddist_dq.row(orig_idx_of_pt_on_bodyA.at(k).at(l)) += normal.col(orig_idx_of_pt_on_bodyA.at(k).at(l)).transpose() * J_k.block(3*l,0,3,robot->num_dof);
    }
    for (; l < numA+numB; ++l) {
      //DEBUG
      //std::cout << "MinDistanceConstraint::eval: Sixth loop: " << l << std::endl;
      //END_DEBUG
      ddist_dq.row(orig_idx_of_pt_on_bodyB.at(k).at(l-numA)) += -normal.col(orig_idx_of_pt_on_bodyB.at(k).at(l-numA)).transpose() * J_k.block(3*l,0,3,robot->num_dof);
    }
  }
  MatrixXd dcost_dscaled_dist(dpairwise_costs_dscaled_dist.colwise().sum());
  c = pairwise_costs.sum();
  dc = dcost_dscaled_dist*dscaled_dist_ddist*ddist_dq;
};


/*
 * mex interface for evaluating a smooth-penalty on violations of a
 * minimum-distance constraint. For each eligible pair of collision geometries,
 * a penalty is computed according to
 *
 * \f[
 * c =
 * \begin{cases}
 *   -de^{\frac{1}{d}}, & d <   0  \\
 *   0,                & d \ge 0.
 * \end{cases}
 * \f]
 *
 * where $d$ is the normalized violation of the minimum distance, $d_{min}$
 *
 * \f[
 * d = \frac{(\phi - d_{min})}{d_{min}}
 * \f]
 *
 * for a signed distance of $\phi$ between the geometries. These pairwise costs
 * are summed to yield a single penalty which is zero if and only if all
 * eligible pairs of collision geometries are separated by at least $d_{min}$.
 *
 * MATLAB signature:
 *
 * [penalty, dpenalty] = ...
 *    rigidBodyManipulator(7, mex_model_ptr, min_distance, allow_multiple_contacts,
 *                        active_collision_options);
 */

void smoothDistancePenalty( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {

  if (nrhs < 2) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:smoothDistancePenalty:NotEnoughInputs","Usage rigidBodyManipulatormex(SMOOTH_DISTANCE_PENALTY_COMMAND,model_ptr,min_distance)");
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  // Get the minimum allowable distance
  double min_distance = mxGetScalar(prhs[1]);

  // Parse `active_collision_options`
  vector<int> active_bodies_idx;
  set<string> active_group_names;
  // First get the list of body indices for which to compute distances
  const mxArray* active_collision_options = prhs[3];
  const mxArray* body_idx = mxGetField(active_collision_options,0,"body_idx");
  if (body_idx != NULL) {
    int n_active_bodies = static_cast<int>(mxGetNumberOfElements(body_idx));
    active_bodies_idx.resize(n_active_bodies);
    if (mxGetClassID(body_idx) == mxINT32_CLASS) {
      memcpy(active_bodies_idx.data(),(int*) mxGetData(body_idx),
          sizeof(int)*n_active_bodies);
    } else if(mxGetClassID(body_idx) == mxDOUBLE_CLASS) {
    double* ptr = mxGetPr(body_idx);
    for (int i=0; i<n_active_bodies; i++)
    active_bodies_idx[i] = static_cast<int>(ptr[i]);
    } else {
      mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:smoothDistancePenalty:WrongInputClass","active_collision_options.body_idx must be an int32 or a double array");
    }
    transform(active_bodies_idx.begin(),active_bodies_idx.end(),
              active_bodies_idx.begin(),
              [](int i){return --i;});
  }

  // Then get the group names for which to compute distances
  const mxArray* collision_groups = mxGetField(active_collision_options,0,
                                               "collision_groups");
  if (collision_groups != NULL) {
  int num = static_cast<int>(mxGetNumberOfElements(collision_groups));
  for (int i=0; i<num; i++) {
      const mxArray *ptr = mxGetCell(collision_groups,i);
    int buflen = static_cast<int>(mxGetN(ptr)*sizeof(mxChar))+1;
    char* str = (char*)mxMalloc(buflen);
    mxGetString(ptr, str, buflen);
    active_group_names.insert(str);
    mxFree(str);
    }
  }

  double penalty;
  vector<int> bodyA_idx, bodyB_idx;
  MatrixXd ptsA, ptsB, normals, JA, JB, Jd, dpenalty;
  VectorXd dist;
  if (active_bodies_idx.size() > 0) {
    if (active_group_names.size() > 0) {
      model-> collisionDetect(dist, normals, ptsA, ptsB, bodyA_idx, bodyB_idx,
                              active_bodies_idx,active_group_names);
    } else {
      model-> collisionDetect(dist, normals, ptsA, ptsB, bodyA_idx, bodyB_idx,
                              active_bodies_idx);
    }
  } else {
    if (active_group_names.size() > 0) {
      model-> collisionDetect(dist, normals, ptsA, ptsB, bodyA_idx, bodyB_idx,
                             active_group_names);
    } else {
      model-> collisionDetect(dist, normals, ptsA, ptsB, bodyA_idx, bodyB_idx);
    }
  }

  smoothDistancePenalty(penalty, dpenalty, model, min_distance, dist, normals, ptsA, ptsB, bodyA_idx, bodyB_idx);

  vector<int32_T> idxA(bodyA_idx.size());
  transform(bodyA_idx.begin(),bodyA_idx.end(),idxA.begin(),
      [](int i){return ++i;});
  vector<int32_T> idxB(bodyB_idx.size());
  transform(bodyB_idx.begin(),bodyB_idx.end(),idxB.begin(),
      [](int i){return ++i;});

  if (nlhs>0) {
    plhs[0] = mxCreateDoubleScalar(penalty);
  }
  if (nlhs>1) {
    plhs[1] = mxCreateDoubleMatrix(static_cast<int>(dpenalty.rows()),static_cast<int>(dpenalty.cols()),mxREAL);
    memcpy(mxGetPr(plhs[1]),dpenalty.data(),sizeof(double)*dpenalty.size());
  }
}

void collisionRaycast( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {

  if (nrhs < 1) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:collisionRaycast:NotEnoughInputs","Usage rigidBodyManipulatormex(COLLISION_RAYCAST_COMMAND,model_ptr, origin_vector, ray_endpoint)");
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);


  Matrix3Xd origins(3, mxGetN(prhs[1])), ray_endpoints(3, mxGetN(prhs[2]));

  if (mxIsNumeric(prhs[1]) != true) {
      mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:collisionRaycast:InputNotNumeric","Expected a numeric value, got something else.");
  }

  if (mxIsNumeric(prhs[2]) != true) {
      mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:collisionRaycast:InputNotNumeric","Expected a numeric value, got something else.");
  }


  if (mxGetM(prhs[1]) != 3) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:collisionRaycast:InputSizeWrong","Expected a 3 x N matrix, got %d x %d.", mxGetM(prhs[1]), mxGetN(prhs[1]));
  }

  if (mxGetM(prhs[2]) != 3) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:collisionRaycast:InputSizeWrong","Expected a 3-element vector for ray_endpoint, got %d elements", mxGetNumberOfElements(prhs[2]));
  }


  memcpy(origins.data(), mxGetPr(prhs[1]), sizeof(double)*mxGetNumberOfElements(prhs[1]));
  memcpy(ray_endpoints.data(), mxGetPr(prhs[2]), sizeof(double)*mxGetNumberOfElements(prhs[2]));
  bool use_margins = (mxGetScalar(prhs[3])!=0.0);
  VectorXd distances;

  model->collisionRaycast(origins, ray_endpoints, distances, use_margins);

  if (nlhs>0) {
    plhs[0] = mxCreateDoubleMatrix(static_cast<int>(distances.size()),1,mxREAL);
    memcpy(mxGetPr(plhs[0]), distances.data(), sizeof(double)*distances.size());
  }

}

void allCollisions( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] )
{
  if (nrhs < 1) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:allCollisions:NotEnoughInputs","Usage rigidBodyManipulatormex(ALL_COLLISIONS_COMMAND,model_ptr)");
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  // Now get the list of body indices for which to compute distances
  vector<int> active_bodies_idx;
  const mxArray* active_collision_options = prhs[2];
  //DEBUG
  //cout << "collisionDetectmex: Num fields in active_collision_options" << mxGetNumberOfFields(active_collision_options) << endl;
  //for (int i = 0; i < mxGetNumberOfFields(active_collision_options); ++i) {
    //const char* fieldname;
    //fieldname = mxGetFieldNameByNumber(active_collision_options,i);
    //cout << *fieldname << endl;
  //}
  //END_DEBUG
  const mxArray* body_idx = mxGetField(active_collision_options,0,"body_idx");
  if (body_idx != NULL) {
    //DEBUG
    //cout << "collisionDetectmex: Received body_idx" << endl;
    //END_DEBUG
    int n_active_bodies = mxGetNumberOfElements(body_idx);
    //DEBUG
    //cout << "collisionDetectmex: n_active_bodies = " << n_active_bodies << endl;
    //END_DEBUG
    active_bodies_idx.resize(n_active_bodies);
    memcpy(active_bodies_idx.data(),(int*) mxGetData(body_idx),sizeof(int)*n_active_bodies);
    transform(active_bodies_idx.begin(),active_bodies_idx.end(),active_bodies_idx.begin(),
        [](int i){return --i;});
  }

  vector<int> bodyA_idx, bodyB_idx;
  MatrixXd ptsA, ptsB;
  model->allCollisions(bodyA_idx,bodyB_idx,ptsA,ptsB);
  vector<int32_T> idxA(bodyA_idx.size());
  transform(bodyA_idx.begin(),bodyA_idx.end(),idxA.begin(),
      [](int i){return ++i;});
  vector<int32_T> idxB(bodyB_idx.size());
  transform(bodyB_idx.begin(),bodyB_idx.end(),idxB.begin(),
      [](int i){return ++i;});

  if (nlhs>0) {
    plhs[0] = mxCreateDoubleMatrix(3,ptsA.cols(),mxREAL);
    memcpy(mxGetPr(plhs[0]),ptsA.data(),sizeof(double)*3*ptsA.cols());
  }
  if (nlhs>1) {
    plhs[1] = mxCreateDoubleMatrix(3,ptsB.cols(),mxREAL);
    memcpy(mxGetPr(plhs[1]),ptsB.data(),sizeof(double)*3*ptsB.cols());
  }
  if (nlhs>2) {
    plhs[2] = mxCreateNumericMatrix(1,idxA.size(),mxINT32_CLASS,mxREAL);
    memcpy(mxGetPr(plhs[2]),idxA.data(),sizeof(int32_T)*idxA.size());
  }
  if (nlhs>3) {
    plhs[3] = mxCreateNumericMatrix(1,idxB.size(),mxINT32_CLASS,mxREAL);
    memcpy(mxGetPr(plhs[3]),idxB.data(),sizeof(int32_T)*idxB.size());
  }
}

void getCMM( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] )
{
  if (nrhs < 2) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:getCMM:NotEnoughInputs","Usage rigidBodyManipulatormex(GET_CMM_COMMAND,model_ptr,q_cache) or rigidBodyManipulatormex(GET_CMM_COMMAND,model_ptr,q_cache,qd)");
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  double* q = mxGetPr(prhs[1]);
  for (int i = 0; i < model->num_dof; i++) {
    if (q[i] - model->cached_q[i] > 1e-8 || q[i] - model->cached_q[i] < -1e-8) {
      mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:getCMM:InvalidKinematics","This kinsol is no longer valid.  Somebody has called doKinematics with a different q since the solution was computed.");
    }
  }

  plhs[0] = mxCreateDoubleMatrix(6,model->num_dof,mxREAL);
  Map<MatrixXd> A(mxGetPr(plhs[0]),6,model->num_dof);
  plhs[1] = mxCreateDoubleMatrix(6,model->num_dof,mxREAL);
  Map<MatrixXd> Adot(mxGetPr(plhs[1]),6,model->num_dof);

  double* qd;
  if (nrhs > 2) {
    qd = mxGetPr(prhs[2]);
  }
  else {
    qd = new double[model->num_dof];
    for (int i = 0; i < model->num_dof; i++) {
      qd[i] = 0;
    }
  }

  model->getCMM(q,qd,A,Adot);
  return;
}

void findKinematicPath( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] )
{
  std::string usage = "Usage [body_path, joint_path, signs] = rigidBodyManipulator(FIND_KINEMATIC_PATH_COMMAND,model_ptr, start_body_or_frame_idx, end_body_or_frame_idx)";
  if (nrhs != 3) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:findKinematicPath:WrongNumberOfInputs", usage.c_str());
  }

  if (nlhs > 3) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:findKinematicPath:WrongNumberOfOutputs", usage.c_str());
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  int start_body_or_frame_idx = ((int) mxGetScalar(prhs[1])) - 1; // base 1 to base 0
  int end_body_or_frame_idx = ((int) mxGetScalar(prhs[2])) - 1; // base 1 to base 0

  KinematicPath path;
  model->findKinematicPath(path, start_body_or_frame_idx, end_body_or_frame_idx);

  if (nlhs > 0) {
    baseZeroToBaseOne(path.body_path);
    plhs[0] = stdVectorToMatlab(path.body_path);
  }
  if (nlhs > 1) {
    baseZeroToBaseOne(path.joint_path);
    plhs[1] = stdVectorToMatlab(path.joint_path);
  }
  if (nlhs > 2) {
    plhs[2] = stdVectorToMatlab(path.joint_direction_signs);
  }
}

void geometricJacobian( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] )
{

  std::string usage = "Usage [J, vIndices] = rigidBodyManipulatormex(GEOMETRIC_JACOBIAN_COMMAND,model_ptr, base_body_or_frame_ind, end_effector_body_or_frame_ind, expressed_in_body_or_frame_ind, in_terms_of_qdot)";
  if (nrhs != 5) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:geometricJacobian:WrongNumberOfInputs", usage.c_str());
  }

  if (nlhs > 3) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:geometricJacobian:WrongNumberOfOutputs", usage.c_str());
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  int base_body_or_frame_ind = ((int) mxGetScalar(prhs[1])) - 1; // base 1 to base 0
  int end_effector_body_or_frame_ind = ((int) mxGetScalar(prhs[2])) - 1; // base 1 to base 0
  int expressed_in_body_or_frame_ind = ((int) mxGetScalar(prhs[3])) - 1; // base 1 to base 0
  bool in_terms_of_qdot = (bool) (mxGetLogicals(prhs[4]))[0];

  Eigen::Matrix<double, TWIST_SIZE, Eigen::Dynamic> J(TWIST_SIZE, 1);

  int gradient_order = nlhs > 2 ? 1 : 0;
  if (nlhs > 1) {
    std::vector<int> v_indices;
    auto J_gradientVar = model->geometricJacobian<double>(base_body_or_frame_ind, end_effector_body_or_frame_ind, expressed_in_body_or_frame_ind, gradient_order, in_terms_of_qdot, &v_indices);
    plhs[0] = eigenToMatlab(J_gradientVar.value());

    baseZeroToBaseOne(v_indices);
    plhs[1] = stdVectorToMatlab(v_indices);
    if (nlhs > 2) {
      plhs[2] = eigenToMatlab(J_gradientVar.gradient().value());
    }
  }
  else if (nlhs > 0){
    auto J_gradientVar = model->geometricJacobian<double>(base_body_or_frame_ind, end_effector_body_or_frame_ind, expressed_in_body_or_frame_ind, gradient_order, in_terms_of_qdot, nullptr);
    plhs[0] = eigenToMatlab(J_gradientVar.value());
  }

}

void doKinematicsNew(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs != 5) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:doKinematicsNew:NotEnoughInputs", "Usage rigidBodyManipulatormex(DO_KINEMATICS_NEW_COMMAND,model_ptr,q,compute_gradients,v,compute_JdotV)");
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  double *q, *v = nullptr;
  if (mxGetNumberOfElements(prhs[1]) != model->num_dof)
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:doKinematicsNew:BadInputs", "q must be size %d x 1", model->num_dof);
  q = mxGetPr(prhs[1]);
  bool compute_gradients = (bool) (mxGetLogicals(prhs[2]))[0];
  if (mxGetNumberOfElements(prhs[3]) > 0) {
    if (mxGetNumberOfElements(prhs[3]) != model->num_velocities)
      mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:doKinematicsNew:BadInputs", "v must be size %d x 1", model->num_velocities);
    v = mxGetPr(prhs[3]);
  }
  bool compute_Jdotv = (bool) (mxGetLogicals(prhs[4]))[0];

  model->doKinematicsNew(q, compute_gradients, v, compute_Jdotv);
}

void forwardKinV(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[]) {

  std::string usage = "Usage [x, J, dJ] = rigidBodyManipulatormex(FORWARD_KIN_V_COMMAND,model_ptr, body_or_frame_ind, points, rotation_type, base_or_frame_ind, compute_analytic_jacobian)";
  if (nrhs != 6) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulator:forwardKinV:WrongNumberOfInputs", usage.c_str());
  }

  if (nlhs > 3) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulator:forwardKinV:WrongNumberOfOutputs", usage.c_str());
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  int body_or_frame_ind = ((int) mxGetScalar(prhs[1])) - 1; // base 1 to base 0
  auto points = matlabToEigen<SPACE_DIMENSION, Eigen::Dynamic>(prhs[2]);
  int rotation_type = (int) mxGetScalar(prhs[3]);
  int base_or_frame_ind = ((int) mxGetScalar(prhs[4])) - 1; // base 1 to base 0
  bool compute_analytic_jacobian = (bool) (mxGetLogicals(prhs[5]))[0];

  if (compute_analytic_jacobian) {
    int gradient_order = nlhs - 1; // position gradient order
    auto x = model->forwardKinNew(points, body_or_frame_ind, base_or_frame_ind, rotation_type, gradient_order);

    plhs[0] = eigenToMatlab(x.value());
    if (gradient_order > 0)
      plhs[1] = eigenToMatlab(x.gradient().value());
    if (gradient_order > 1)
      plhs[2] = eigenToMatlab(x.gradient().gradient().value());
  }
  else {
    int gradient_order = nlhs > 1 ? nlhs - 2 : 0; // Jacobian gradient order
    auto x = model->forwardKinNew(points, body_or_frame_ind, base_or_frame_ind, rotation_type, gradient_order);
    plhs[0] = eigenToMatlab(x.value());
    if (nlhs > 1) {
      auto J = model->forwardJacV(x, body_or_frame_ind, base_or_frame_ind, rotation_type, compute_analytic_jacobian, gradient_order);
      plhs[1] = eigenToMatlab(J.value());
      if (nlhs > 2) {
        plhs[2] = eigenToMatlab(J.gradient().value());
      }
    }
  }
}

void forwardKinPositionGradient(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[])
{
  std::string usage = "Usage [P, dP] = rigidBodyManipulatormex(FORWARD_KIN_POSITION_GRADIENT_COMMAND,model_ptr, npoints, current_body_or_frame_ind, new_body_or_frame_ind)";
  if (nrhs != 4) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:forwardKinPositionGradient:WrongNumberOfInputs", usage.c_str());
  }

  if (nlhs > 2) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:forwardKinPositionGradient:WrongNumberOfOutputs", usage.c_str());
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  int npoints =  ((int) mxGetScalar(prhs[1]));
  int current_body_or_frame_ind = ((int) mxGetScalar(prhs[2])) - 1; // base 1 to base 0
  int new_body_or_frame_ind = ((int) mxGetScalar(prhs[3])) - 1; // base 1 to base 0
  int gradient_order = nlhs - 1;
  auto P = model->forwardKinPositionGradient<double>(npoints, current_body_or_frame_ind, new_body_or_frame_ind, gradient_order);
  plhs[0] = eigenToMatlab(P.value());

  if (P.hasGradient()) {
    plhs[1] = eigenToMatlab(P.gradient().value());
  }
}

void centerOfMass(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[])
{
  std::string usage = "Usage [x, J, dJ] = rigidBodyManipulatormex(CENTER_OF_MASS_COMMAND,robotnum)";
  if (nrhs != 2) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:centerOfMass:WrongNumberOfInputs", usage.c_str());
  }

  if (nlhs > 3) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:centerOfMass:WrongNumberOfOutputs", usage.c_str());
  }

  RigidBodyManipulator *model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  set<int> robotnum_set;
  int num_robot = static_cast<int>(mxGetNumberOfElements(prhs[1]));
  double* robotnum = mxGetPr(prhs[1]);
  for (int i = 0; i < num_robot; i++) {
    robotnum_set.insert((int) robotnum[i] - 1);
  }
  int gradient_order = nlhs - 1;
  auto x = model->centerOfMass<double>(gradient_order, robotnum_set);

  plhs[0] = eigenToMatlab(x.value());
  if (gradient_order > 0)
    plhs[1] = eigenToMatlab(x.gradient().value());
  if (gradient_order > 1)
    plhs[2] = eigenToMatlab(x.gradient().gradient().value());
}

void centroidalMomentumMatrix(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[])
{
  std::string usage = "Usage [A, dA] = rigidBodyManipulatormex(CENTROIDAL_MOMENTUM_MATRIX_COMMAND)";
  if (nrhs != 1) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:centroidalMomentumMatrix:WrongNumberOfInputs", usage.c_str());
  }

  if (nlhs > 2) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:centroidalMomentumMatrix:WrongNumberOfOutputs", usage.c_str());
  }

  RigidBodyManipulator *model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  int gradient_order = nlhs - 1;
  auto A = model->centroidalMomentumMatrix<double>(gradient_order);

  plhs[0] = eigenToMatlab(A.value());
  if (gradient_order > 0)
    plhs[1] = eigenToMatlab(A.gradient().value());
}

void massMatrix(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[])
{
  string usage = "Usage [M, dM] = rigidBodyManipulatormex(MASS_MATRIX_COMMAND,model_ptr)";
  if (nrhs != 1) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:massMatrix:WrongNumberOfInputs", usage.c_str());
  }

  if (nlhs > 2) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:massMatrix:WrongNumberOfOutputs", usage.c_str());
  }

  int gradient_order = nlhs - 1;


  RigidBodyManipulator *model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  auto ret = model->massMatrix<double>(gradient_order);

  plhs[0] = eigenToMatlab(ret.value());
  if (gradient_order > 0)
    plhs[1] = eigenToMatlab(ret.gradient().value());
}

void inverseDynamics(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[])
{
  string usage = "Usage [C, dC] = rigidBodyDynamicsmex(INVERSE_DYNAMICS_COMMAND,model_ptr, f_ext, vd, df_ext, dvd)";
  if (nrhs < 1 || nrhs > 5) {
    mexErrMsgIdAndTxt("Drake:rigidBodyDynamicsmex:inverseDynamics:WrongNumberOfInputs", usage.c_str());
  }

  if (nlhs > 2) {
    mexErrMsgIdAndTxt("Drake:rigidBodyDynamicsmex:inverseDynamics:WrongNumberOfOutputs", usage.c_str());
  }

  int gradient_order = nlhs - 1;


  RigidBodyManipulator *model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  int nq = model->num_dof;
  int nv = model->num_velocities;
  const mxArray* f_ext_matlab = nullptr;
  const mxArray* vd_matlab = nullptr;
  const mxArray* df_ext_matlab = nullptr;
  const mxArray* dvd_matlab = nullptr;
  if (nrhs > 1)
    f_ext_matlab = prhs[1];
  if (nrhs > 2)
    vd_matlab = prhs[2];
  if (nrhs > 3)
    df_ext_matlab = prhs[3];
  if (nrhs > 4)
    dvd_matlab = prhs[4];

  map<int, unique_ptr<GradientVar<double, TWIST_SIZE, 1> > > f_ext;
  if (f_ext_matlab != nullptr) {
    if (!mxIsEmpty(f_ext_matlab)) {
      if (mxIsCell(f_ext_matlab)) {
        int rows = mxGetM(f_ext_matlab);
        int cols = mxGetN(f_ext_matlab);
        if (rows != 1)
          throw runtime_error("f_ext cell array has number of rows not equal to 1");
        if (cols != model->num_bodies)
          throw runtime_error("f_ext cell array has number of columns not equal to number of rigid bodies in manipulator");

        for (int i = 0; i < model->num_bodies; i++) {
          mxArray* f_ext_cell = mxGetCell(f_ext_matlab, i);
          if (!mxIsEmpty(f_ext_cell)) {
            f_ext[i] = unique_ptr< GradientVar<double, TWIST_SIZE, 1> >(new GradientVar<double, TWIST_SIZE, 1>(TWIST_SIZE, 1, nq + nv, gradient_order));
            // TODO: could save a copy with a two-argument matlabToEigen.
            f_ext[i]->value() = matlabToEigen<TWIST_SIZE, Dynamic>(f_ext_cell);
            if (gradient_order > 0 && df_ext_matlab != nullptr) {
              mxArray* df_ext_cell = mxGetCell(df_ext_matlab, i);
              f_ext[i]->gradient().value() = matlabToEigen<Dynamic, Dynamic>(df_ext_cell);
            }
          }
        }
      }
      else if (mxIsNumeric(f_ext_matlab)) {
        int rows = mxGetM(f_ext_matlab);
        int cols = mxGetN(f_ext_matlab);

        if (rows != TWIST_SIZE)
          throw runtime_error("f_ext matrix has number of rows not equal to 6");
        if (cols != model->num_bodies)
          throw runtime_error("f_ext matrix has number of columns not equal to number of rigid bodies in manipulator");


        GradientVar<double, TWIST_SIZE, Dynamic> f_ext_matrix(TWIST_SIZE, model->num_bodies, nq + nv, gradient_order);
        f_ext_matrix.value() = matlabToEigen<TWIST_SIZE, Dynamic>(f_ext_matlab);
        if (gradient_order > 0) {
          if (df_ext_matlab == nullptr) {
            throw runtime_error("df_ext must be passed in if you pass in f_ext and want gradient output");
          }
          f_ext_matrix.gradient().value() = matlabToEigen<Dynamic, Dynamic>(df_ext_matlab);
        }

        for (int i = 0; i < model->num_bodies; i++) {
          f_ext[i] = unique_ptr< GradientVar<double, TWIST_SIZE, 1> >(new GradientVar<double, TWIST_SIZE, 1>(TWIST_SIZE, 1, nq + nv, gradient_order));
          f_ext[i]->value() = f_ext_matrix.value().col(i);
          if (f_ext_matrix.hasGradient()) {
            f_ext[i]->gradient().value() = f_ext_matrix.gradient().value().middleRows<TWIST_SIZE>(TWIST_SIZE * i);
          }
        }
      }
    }
  }
  else
    throw runtime_error("data type of f_ext not recognized");

  unique_ptr<GradientVar<double, Eigen::Dynamic, 1> > vd_ptr;
  if (vd_matlab != nullptr) {
    if (!mxIsEmpty(vd_matlab)) {
      int nv = model->num_velocities;
      vd_ptr = unique_ptr<GradientVar<double, Eigen::Dynamic, 1>>(new GradientVar<double, Eigen::Dynamic, 1>(nv, 1, nq + nv, gradient_order));
      vd_ptr->value() = matlabToEigen<Dynamic, 1>(vd_matlab);
      if (gradient_order > 0) {
        if (dvd_matlab == nullptr) {
          throw runtime_error("dvd must be passed in if you pass in vd and want gradient output");
        }
        vd_ptr->gradient().value() = matlabToEigen<Dynamic, Dynamic>(dvd_matlab);
      }
    }
  }

  auto ret = model->inverseDynamics(f_ext, vd_ptr.get(), gradient_order);

  plhs[0] = eigenToMatlab(ret.value());
  if (gradient_order > 0)
    plhs[1] = eigenToMatlab(ret.gradient().value());
}

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )
{
  if (nrhs<1) {
    mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:BadInputs","Usage RigidBodyManipulatormex(command,...)");
  }

  if (isa(prhs[0],"DrakeMexPointer")) {  // then it's calling the destructor
    destroyDrakeMexPointer<RigidBodyManipulator*>(prhs[0]);
    return;
  }

  int command = mxGetScalar(prhs[0]);
  nrhs--; prhs++;  // remove command from the inputs before passing to the sub-methods

  switch (command) {
    case DELETE_MODEL_COMMAND:
      destroyDrakeMexPointer<RigidBodyManipulator*>(prhs[0]);
      break;
    case CONSTRUCT_MODEL_COMMAND:
      constructModel(nlhs,plhs,nrhs,prhs);
      break;
    case H_AND_C_COMMAND:
      HandC(nlhs,plhs,nrhs,prhs);
      break;
    case DO_KINEMATICS_COMMAND:
      doKinematics(nlhs,plhs,nrhs,prhs);
      break;
    case FORWARD_KIN_COMMAND:
      forwardKin(nlhs,plhs,nrhs,prhs);
      break;
    case BODY_KIN_COMMAND:
      bodyKin(nlhs,plhs,nrhs,prhs);
      break;
    case COLLISION_DETECT_COMMAND:
      collisionDetect(nlhs,plhs,nrhs,prhs);
      break;
    case SMOOTH_DISTANCE_PENALTY_COMMAND:
      smoothDistancePenalty(nlhs,plhs,nrhs,prhs);
      break;
    case COLLISION_RAYCAST_COMMAND:
      collisionRaycast(nlhs,plhs,nrhs,prhs);
      break;
    case ALL_COLLISIONS_COMMAND:
      allCollisions(nlhs,plhs,nrhs,prhs);
      break;
    case GET_CMM_COMMAND:
      getCMM(nlhs,plhs,nrhs,prhs);
      break;
    case FIND_KINEMATIC_PATH_COMMAND:
      findKinematicPath(nlhs,plhs,nrhs,prhs);
      break;
    case GEOMETRIC_JACOBIAN_COMMAND:
      geometricJacobian(nlhs,plhs,nrhs,prhs);
      break;
    case DO_KINEMATICS_NEW_COMMAND:
      doKinematicsNew(nlhs,plhs,nrhs,prhs);
      break;
    case FORWARD_KIN_V_COMMAND:
      forwardKinV(nlhs,plhs,nrhs,prhs);
      break;
    case FORWARD_KIN_POSITION_GRADIENT_COMMAND:
      forwardKinPositionGradient(nlhs,plhs,nrhs,prhs);
      break;
    case CENTER_OF_MASS_COMMAND:
      centerOfMass(nlhs,plhs,nrhs,prhs);
      break;
    case CENTROIDAL_MOMENTUM_MATRIX_COMMAND:
      centroidalMomentumMatrix(nlhs,plhs,nrhs,prhs);
      break;
    case MASS_MATRIX_COMMAND:
      massMatrix(nlhs,plhs,nrhs,prhs);
      break;
    case INVERSE_DYNAMICS_COMMAND:
      inverseDynamics(nlhs,plhs,nrhs,prhs);
      break;
    default:
      mexErrMsgIdAndTxt("Drake:rigidBodyManipulatormex:UnrecognizedCommand","unknown command");
      break;
  }
}
