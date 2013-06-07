#include "mex.h"
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "RigidBodyManipulator.h"

#define INF -2147483648

using namespace Eigen;
using namespace std;

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) {

  if (nrhs!=1) {
    mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","Usage model_ptr = constructModelmex(obj)");
  }

  RigidBodyManipulator *model=NULL;

  const mxArray* featherstone = mxGetProperty(prhs[0],0,"featherstone");
  if (!featherstone) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs", "the featherstone array is invalid");

  const mxArray* pBodies = mxGetProperty(prhs[0],0,"body");
  if (!pBodies) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","the body array is invalid");
  int num_bodies = mxGetNumberOfElements(pBodies);

  // set up the model
  mxArray *pm;
  int dim=3;
  
  pm = mxGetField(featherstone,0,"NB");
  if (!pm) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","can't find field model.featherstone.NB.  Are you passing in the correct structure?");
  model = new RigidBodyManipulator((int) mxGetScalar(pm), (int) mxGetScalar(pm), num_bodies);
  
  pm = mxGetField(featherstone,0,"parent");
  if (!pm) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","can't find field model.featherstone.parent.");
  double* parent = mxGetPr(pm);
  
  pm = mxGetField(featherstone,0,"pitch");
  if (!pm) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","can't find field model.featherstone.pitch.");
  double* pitch = mxGetPr(pm);
  
  pm = mxGetField(featherstone,0,"dofnum");
  if (!pm) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","can't find field model.featherstone.dofnum.");
  double* dofnum = mxGetPr(pm);

  pm = mxGetField(featherstone,0,"damping");
  if (!pm) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","can't find field model.featherstone.damping.");
  memcpy(model->damping,mxGetPr(pm),sizeof(double)*model->NB);
  
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

  char buf[100];
  for (int i=0; i<model->num_bodies; i++) {
    pm = mxGetProperty(pBodies,i,"linkname");
    mxGetString(pm,buf,100);
    model->bodies[i].linkname.assign(buf,strlen(buf));

    pm = mxGetProperty(pBodies,i,"jointname");
    mxGetString(pm,buf,100);
    model->bodies[i].jointname.assign(buf,strlen(buf));

    pm = mxGetProperty(pBodies,i,"mass");
    model->bodies[i].mass = (double) mxGetScalar(pm);

    pm = mxGetProperty(pBodies,i,"contact_pts");
    if (!mxIsEmpty(pm)) {
      Map<MatrixXd> pts_tmp(mxGetPr(pm),mxGetM(pm),mxGetN(pm));
      model->bodies[i].contact_pts.resize(4,mxGetN(pm));
      model->bodies[i].contact_pts << pts_tmp, MatrixXd::Ones(1,mxGetN(pm));
    }
    
    pm = mxGetProperty(pBodies,i,"com");
    if (!mxIsEmpty(pm)) memcpy(model->bodies[i].com.data(),mxGetPr(pm),sizeof(double)*3);

    pm = mxGetProperty(pBodies,i,"dofnum");
    model->bodies[i].dofnum = (int) mxGetScalar(pm) - 1;  //zero-indexed
    
    pm = mxGetProperty(pBodies,i,"floating");
    model->bodies[i].floating = (int) mxGetScalar(pm);

    pm = mxGetProperty(pBodies,i,"pitch");
    model->bodies[i].pitch = (int) mxGetScalar(pm);
    
    {  // lookup parent index
      pm = mxGetProperty(pBodies,i,"parent");
      if (!pm || mxIsEmpty(pm)) {
      	model->bodies[i].parent = -1;
      } else {
      	pm = mxGetProperty(pm,0,"dofnum");
      	if (!pm) { mexPrintf("i=%d\n",i); mexErrMsgTxt("couldn't get dofnum property"); }
      	int parent_dofnum = mxGetScalar(pm) - 1;
      	bool b_found=false;
      	for (int j=0; j<i; j++) {
      		if (model->bodies[j].dofnum == parent_dofnum) {
      			b_found = true;
      			model->bodies[i].parent = j;
      			break;
      		}
      	}
      	if (!b_found) {
      		mexPrintf("i=%d\n");
  				mexErrMsgTxt("oops! parent link was expected but not found.");  // note that this assumed that the parent comes before the child in pBodies, which should be enforced before this gets called.
      	}
      }
    }
    
//     if (model->bodies[i].dofnum>=0) {
//       pm = mxGetProperty(pBodies,i,"joint_limit_min");
//       model->joint_limit_min[model->bodies[i].dofnum] = mxGetScalar(pm);
//       pm = mxGetProperty(pBodies,i,"joint_limit_max");
//       model->joint_limit_max[model->bodies[i].dofnum] = mxGetScalar(pm);
//     }    
    
    pm = mxGetProperty(pBodies,i,"Ttree");
    // todo: check that the size is 4x4
    memcpy(model->bodies[i].Ttree.data(),mxGetPr(pm),sizeof(double)*4*4);
    
    pm = mxGetProperty(pBodies,i,"T_body_to_joint");
    memcpy(model->bodies[i].T_body_to_joint.data(),mxGetPr(pm),sizeof(double)*4*4);

#ifdef BULLET_COLLISION
    pm = mxGetProperty(pBodies,i,"contact_shapes");
    for (int j=0; j<mxGetNumberOfElements(pm); j++) {
    	// construct bullet collision object and transform
    	RigidBody::CollisionObject co;
    	double* params = mxGetPr(mxGetField(pm,j,"params"));
    	int type = (int)mxGetScalar(mxGetField(pm,j,"type"));
    	switch (type) {
    	case 1: // BOX
    		co.bt_shape = new btBoxShape( btVector3(params[0]/2,params[1]/2,params[2]/2) );
    		break;
    	case 2: // SPHERE
    		co.bt_shape = new btSphereShape(params[0]) ;
    		break;
    	case 3: // CYLINDER
    		co.bt_shape = new btCylinderShapeZ( btVector3(params[0],params[0],params[1]/2) );
    		break;
    	case 4: // MESH
    		co.bt_shape = new btConvexHullShape( (btScalar*) params, (int) mxGetNumberOfElements(mxGetField(pm,j,"params")) ,(int) 3*sizeof(double) );
    		break;
    	default:
    		mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs", "Body %d collision shape %d has an unknown type %d", i+1,j+1,type);
    		break;
    	}
    	co.bt_obj = new btCollisionObject();
    	co.bt_obj->setCollisionShape(co.bt_shape);
    	memcpy(co.T.data(), mxGetPr(mxGetField(pm,j,"T")), sizeof(double)*4*4);

    	// add to the manipulator's collision world
    	model->bt_collision_world.addCollisionObject(co.bt_obj);

    	if (model->bodies[i].parent>=0) {
    		co.bt_obj->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
    		co.bt_obj->activate();
    	} else {
    		co.bt_obj->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT);
    	}

    	// add to the body
    	model->bodies[i].collision_objects.push_back(co);
    }
    if (model->bodies[i].parent<0)
    	model->updateCollisionObjects(i);  // update static objects only once - right here on load
#endif
  }
  
  double *min = mxGetPr(mxGetProperty(prhs[0],0,"joint_limit_min"));
  double *max = mxGetPr(mxGetProperty(prhs[0], 0, "joint_limit_max"));
  
  memcpy(model->joint_limit_min, mxGetPr(mxGetProperty(prhs[0],0,"joint_limit_min")), sizeof(double)*model->num_dof);
  memcpy(model->joint_limit_max, mxGetPr(mxGetProperty(prhs[0],0,"joint_limit_max")), sizeof(double)*model->num_dof);
  
  const mxArray* a_grav_array = mxGetProperty(prhs[0],0,"gravity");
  if (a_grav_array && mxGetNumberOfElements(a_grav_array)==3) {
    double* p = mxGetPr(a_grav_array);
    model->a_grav[3] = p[0];
    model->a_grav[4] = p[1];
    model->a_grav[5] = p[2];
  } else {
    mexErrMsgIdAndTxt("Drake:constructModelmex:BadGravity","Couldn't find a 3 element gravity vector in the object.");
  }

  model->compile();
  
  if (nlhs>0) {  // return a pointer to the model
    mxClassID cid;
    if (sizeof(model)==4) cid = mxUINT32_CLASS;
    else if (sizeof(model)==8) cid = mxUINT64_CLASS;
    else mexErrMsgIdAndTxt("Drake:constructModelmex:PointerSize","Are you on a 32-bit machine or 64-bit machine??");
    
    plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
    memcpy(mxGetData(plhs[0]),&model,sizeof(model));
  }

}
