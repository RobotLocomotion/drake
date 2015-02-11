#include "mex.h"
#include <iostream>
#include <cmath>
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"
#include <stdexcept>

#include "joints/drakeJointUtil.h"

using namespace Eigen;
using namespace std;

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )
{
  //DEBUG
  //cout << "constructModelmex: START" << endl;
  //END_DEBUG
  char buf[100];
  mxArray *pm;

  if (nrhs!=1) {
    mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","Usage model_ptr = constructModelmex(obj)");
  }

  if (isa(prhs[0],"DrakeMexPointer")) {  // then it's calling the destructor
    destroyDrakeMexPointer<RigidBodyManipulator*>(prhs[0]);
    return;
  }

  const mxArray* pRBM = prhs[0];
  RigidBodyManipulator *model=NULL;

//  model->robot_name = get_strings(mxGetProperty(pRBM,0,"name"));

  const mxArray* featherstone = mxGetProperty(pRBM,0,"featherstone");
  if (!featherstone) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs", "the featherstone array is invalid");

  const mxArray* pBodies = mxGetProperty(pRBM,0,"body");
  if (!pBodies) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","the body array is invalid");
  int num_bodies = static_cast<int>(mxGetNumberOfElements(pBodies));

  const mxArray* pFrames = mxGetProperty(pRBM,0,"frame");
  if (!pFrames) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","the frame array is invalid");
  int num_frames = static_cast<int>(mxGetNumberOfElements(pFrames));

  // set up the model
  pm = mxGetField(featherstone,0,"NB");
  if (!pm) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","can't find field model.featherstone.NB.  Are you passing in the correct structure?");
  model = new RigidBodyManipulator((int) mxGetScalar(pm), (int) mxGetScalar(pm), num_bodies, num_frames);

  pm = mxGetField(featherstone,0,"parent");
  if (!pm) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","can't find field model.featherstone.parent.");
  double* parent = mxGetPr(pm);

  pm = mxGetField(featherstone,0,"pitch");
  if (!pm) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","can't find field model.featherstone.pitch.");
  double* pitch = mxGetPr(pm);

  pm = mxGetField(featherstone,0,"position_num");
  if (!pm) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","can't find field model.featherstone.position_num.");
  double* dofnum = mxGetPr(pm);
//  throw runtime_error("here");

  pm = mxGetField(featherstone,0,"damping");
  if (!pm) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","can't find field model.featherstone.damping.");
  memcpy(model->damping.data(),mxGetPr(pm),sizeof(double)*model->NB);

  pm = mxGetField(featherstone,0,"coulomb_friction");
  if (!pm) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","can't find field model.featherstone.coulomb_friction.");
  memcpy(model->coulomb_friction.data(),mxGetPr(pm),sizeof(double)*model->NB);

  pm = mxGetField(featherstone,0,"static_friction");
  if (!pm) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","can't find field model.featherstone.static_friction.");
  memcpy(model->static_friction.data(),mxGetPr(pm),sizeof(double)*model->NB);

  pm = mxGetField(featherstone,0,"coulomb_window");
  if (!pm) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","can't find field model.featherstone.coulomb_window.");
  memcpy(model->coulomb_window.data(),mxGetPr(pm),sizeof(double)*model->NB);

  mxArray* pXtree = mxGetField(featherstone,0,"Xtree");
  if (!pXtree) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","can't find field model.featherstone.Xtree.");

  mxArray* pI = mxGetField(featherstone,0,"I");
  if (!pI) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","can't find field model.featherstone.I.");

  for (int i=0; i<model->NB; i++) {
    model->parent[i] = ((int) parent[i]) - 1;  // since it will be used as a C index
    model->pitch[i] = (int) pitch[i];
    model->dofnum[i] = (int) dofnum[i] - 1; // zero-indexed

    mxArray* pXtreei = mxGetCell(pXtree,i);
    if (!pXtreei) mexErrMsgIdAndTxt("Drake:HandCpmex:BadInputs","can't access model.featherstone.Xtree{%d}",i);

    // todo: check that the size is 6x6
    memcpy(model->Xtree[i].data(),mxGetPr(pXtreei),sizeof(double)*6*6);

    mxArray* pIi = mxGetCell(pI,i);
    if (!pIi) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","can't access model.featherstone.I{%d}",i);

    // todo: check that the size is 6x6
    memcpy(model->I[i].data(),mxGetPr(pIi),sizeof(double)*6*6);
  }

  int num_velocities = 0;
  for (int i=0; i<model->num_bodies; i++) {
    //DEBUG
    //cout << "constructModelmex: body " << i << endl;
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
        num_velocities += model->bodies[i]->getJoint().getNumVelocities();
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
    //cout << "constructModelmex: About to parse collision geometry"  << endl;
    //END_DEBUG
    pm = mxGetProperty(pBodies,i,"collision_geometry");
    Matrix4d T;
    if (!mxIsEmpty(pm)){
      for (int j=0; j<mxGetNumberOfElements(pm); j++) {
        //DEBUG
        //cout << "constructModelmex: Body " << i << ", Element " << j << endl;
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
      //cout << "constructModelmex: Group: " << *group << endl;
      //cout << "constructModelmex: Mask " << *mask << endl;
      //END_DEBUG
      model->setCollisionFilter(i,*group,*mask);
    }
  }
  model->num_velocities = num_velocities;

  // THIS IS UGLY: I'm sending the terrain contact points into the
  // contact_pts field of the cpp RigidBody objects
  //DEBUG
  //cout << "constructModelmex: Parsing contact points " << endl;
  //cout << "constructModelmex: Get struct" << endl;
  //END_DEBUG
  mxArray* contact_pts_struct[1];
  if (~mexCallMATLAB(1,contact_pts_struct,1,const_cast<mxArray**>(&pRBM),"getTerrainContactPoints")) {
    //DEBUG
    //cout << "constructModelmex: Got struct" << endl;
    //if (contact_pts_struct) {
    //cout << "constructModelmex: Struct pointer: " << contact_pts_struct << endl;
    //} else {
    //cout << "constructModelmex: Struct pointer NULL" << endl;
    //}
    //cout << "constructModelmex: Get numel of struct" << endl;
    //END_DEBUG
    const int n_bodies_w_contact_pts = static_cast<int>(mxGetNumberOfElements(contact_pts_struct[0]));
    //DEBUG
    //cout << "constructModelmex: Got numel of struct:" << n_bodies_w_contact_pts << endl;
    //END_DEBUG
    mxArray* pPts;
    int body_idx;
    int n_pts;
    for (int j=0; j < n_bodies_w_contact_pts; j++) {
      //DEBUG
      //cout << "constructModelmex: Loop: Iteration " << j << endl;
      //cout << "constructModelmex: Get body_idx" << endl;
      //END_DEBUG
      body_idx = (int) mxGetScalar(mxGetField(contact_pts_struct[0],j,"idx")) - 1;
      //DEBUG
      //cout << "constructModelmex: Got body_idx: " << body_idx << endl;
      //cout << "constructModelmex: Get points" << endl;
      //END_DEBUG
      pPts = mxGetField(contact_pts_struct[0],j,"pts");
      //DEBUG
      //cout << "constructModelmex: Get points" << endl;
      //cout << "constructModelmex: Get number of points" << endl;
      //END_DEBUG
      n_pts = static_cast<int>(mxGetN(pPts));
      //DEBUG
      //cout << "constructModelmex: Got number of points: " << n_pts << endl;
      //cout << "constructModelmex: Set contact_pts of body" << endl;
      //END_DEBUG
      Map<MatrixXd> pts(mxGetPr(pPts),3,n_pts);
      model->bodies[body_idx]->contact_pts.resize(4,n_pts);
      model->bodies[body_idx]->contact_pts << pts, MatrixXd::Ones(1,n_pts);
      //DEBUG
      //cout << "constructModelmex: Contact_pts of body: " << endl;
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
    mexErrMsgIdAndTxt("Drake:constructModelmex:BadGravity","Couldn't find a 3 element gravity vector in the object.");
  }

  mxLogical* use_new_kinsol = mxGetLogicals(mxGetProperty(pRBM,0,"use_new_kinsol"));
  model->use_new_kinsol = (bool) use_new_kinsol[0];

  model->compile();

  plhs[0] = createDrakeMexPointer((void*)model,"RigidBodyManipulator");
}
