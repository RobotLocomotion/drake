#include "mex.h"
#include <iostream>
#include <cmath>
#include "drakeMexUtil.h"
#include "RigidBodyManipulator.h"
#include <stdexcept>
#include <RevoluteJoint.h>
#include <PrismaticJoint.h>
#include <HelicalJoint.h>
#include <RollPitchYawFloatingJoint.h>
#include <QuaternionFloatingJoint.h>

using namespace Eigen;
using namespace std;

bool isMxArrayVector( const mxArray* array )
{
  size_t num_rows = mxGetM(array);
  size_t num_cols = mxGetN(array);
  return (num_rows <= 1) || (num_cols <= 1);
}

template <typename Derived>
void setDynamics(const mxArray *pBodies, int i, FixedAxisOneDoFJoint<Derived>* fixed_axis_one_dof_joint) {
  double damping = mxGetScalar(mxGetProperty(pBodies, i, "damping"));
  double coulomb_friction = mxGetScalar(mxGetProperty(pBodies, i, "coulomb_friction"));
  double coulomb_window = mxGetScalar(mxGetProperty(pBodies, i, "coulomb_window"));
  fixed_axis_one_dof_joint->setDynamics(damping, coulomb_friction, coulomb_window);
}

template <typename Derived>
void setLimits(const mxArray *pBodies, int i, FixedAxisOneDoFJoint<Derived>* fixed_axis_one_dof_joint) {
  double joint_limit_min = mxGetScalar(mxGetProperty(pBodies,i,"joint_limit_min"));
  double joint_limit_max = mxGetScalar(mxGetProperty(pBodies,i,"joint_limit_max"));
  fixed_axis_one_dof_joint->setJointLimits(joint_limit_min,joint_limit_max);
}


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

  const mxArray* pBodies = mxGetProperty(pRBM,0,"body");
  if (!pBodies) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","the body array is invalid");
  int num_bodies = static_cast<int>(mxGetNumberOfElements(pBodies));

  const mxArray* pFrames = mxGetProperty(pRBM,0,"frame");
  if (!pFrames) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","the frame array is invalid");
  int num_frames = static_cast<int>(mxGetNumberOfElements(pFrames));

  pm = mxGetProperty(pRBM, 0, "num_positions");
  if (!pm) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","model should have a num_positions field");
  int num_positions = static_cast<int>(*mxGetPrSafe(pm));
  model = new RigidBodyManipulator(num_positions, num_bodies, num_frames);

  for (int i=0; i<model->num_bodies; i++) {
    //DEBUG
    //cout << "constructModelmex: body " << i << endl;
    //END_DEBUG
    model->bodies[i]->body_index = i;

    pm = mxGetProperty(pBodies,i,"linkname");
    mxGetString(pm,buf,100);
    model->bodies[i]->linkname.assign(buf,strlen(buf));

    pm = mxGetProperty(pBodies,i,"robotnum");
    model->bodies[i]->robotnum = (int) mxGetScalar(pm)-1;

    pm = mxGetProperty(pBodies,i,"mass");
    model->bodies[i]->mass = mxGetScalar(pm);

    pm = mxGetProperty(pBodies,i,"com");
    if (!mxIsEmpty(pm)) memcpy(model->bodies[i]->com.data(),mxGetPrSafe(pm),sizeof(double)*3);

    pm = mxGetProperty(pBodies,i,"I");
    if (!mxIsEmpty(pm)) memcpy(model->bodies[i]->I.data(),mxGetPrSafe(pm),sizeof(double)*6*6);

    pm = mxGetProperty(pBodies,i,"position_num");
    model->bodies[i]->position_num_start = (int) mxGetScalar(pm) - 1;  //zero-indexed

    pm = mxGetProperty(pBodies,i,"velocity_num");
    model->bodies[i]->velocity_num_start = (int) mxGetScalar(pm) - 1;  //zero-indexed

    pm = mxGetProperty(pBodies,i,"parent");
    if (!pm || mxIsEmpty(pm))
      model->bodies[i]->parent = nullptr;
    else {
      int parent_ind = static_cast<int>(mxGetScalar(pm))-1;
      if (parent_ind >= static_cast<int>(model->bodies.size()))
        mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","bad body.parent %d (only have %d bodies)",parent_ind,model->bodies.size());
      if (parent_ind>=0)
        model->bodies[i]->parent = model->bodies[parent_ind];
    }

    {
      mxGetString(mxGetProperty(pBodies, i, "jointname"), buf, 100);
      string joint_name;
      joint_name.assign(buf, strlen(buf));

      pm = mxGetProperty(pBodies, i, "transform_to_parent_body");
      // todo: check that the size is 4x4
      Isometry3d transform_to_parent_body;
      memcpy(transform_to_parent_body.data(), mxGetPrSafe(pm), sizeof(double) * 4 * 4);

      int floating = (int) mxGetScalar(mxGetProperty(pBodies, i, "floating"));

      Eigen::Vector3d joint_axis;
      pm = mxGetProperty(pBodies, i, "joint_axis");
      memcpy(joint_axis.data(), mxGetPrSafe(pm), sizeof(double) * 3);

      double pitch = mxGetScalar(mxGetProperty(pBodies, i, "pitch"));

      if (model->bodies[i]->hasParent()) {
        std::unique_ptr<DrakeJoint> joint;
        switch (floating) {
          case 0: {
            if (pitch == 0.0) {
              RevoluteJoint *revolute_joint = new RevoluteJoint(joint_name, transform_to_parent_body, joint_axis);
              joint = std::unique_ptr<RevoluteJoint>(revolute_joint);
              setLimits(pBodies, i, revolute_joint);
              setDynamics(pBodies, i, revolute_joint);
            } else if (isInf(pitch)) {
              PrismaticJoint *prismatic_joint = new PrismaticJoint(joint_name, transform_to_parent_body, joint_axis);
              joint = std::unique_ptr<PrismaticJoint>(prismatic_joint);
              setLimits(pBodies, i, prismatic_joint);
              setDynamics(pBodies, i, prismatic_joint);
            } else {
              joint = std::unique_ptr<HelicalJoint>(new HelicalJoint(joint_name, transform_to_parent_body, joint_axis, pitch));
            }
            break;
          }
          case 1: {
            joint = std::unique_ptr<RollPitchYawFloatingJoint>(new RollPitchYawFloatingJoint(joint_name, transform_to_parent_body));
            break;
          }
          case 2: {
            joint = std::unique_ptr<QuaternionFloatingJoint>(new QuaternionFloatingJoint(joint_name, transform_to_parent_body));
            break;
          }
          default: {
            std::ostringstream stream;
            stream << "floating type " << floating << " not recognized.";
            throw std::runtime_error(stream.str());
          }
        }

        model->bodies[i]->setJoint(move(joint));
      }
    }

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
        memcpy(T.data(), mxGetPrSafe(mxGetProperty(pShape,0,"T")), sizeof(double)*4*4);
        auto shape = (DrakeShapes::Shape)static_cast<int>(mxGetScalar(mxGetProperty(pShape,0,"drake_shape_id")));
        vector<double> params_vec;
        RigidBody::CollisionElement element(T, model->bodies[i]);
        switch (shape) {
          case DrakeShapes::BOX:
          {
            double* params = mxGetPrSafe(mxGetProperty(pShape,0,"size"));
            element.setGeometry(DrakeShapes::Box(Vector3d(params[0],params[1],params[2])));
          }
            break;
          case DrakeShapes::SPHERE:
          {
            double r(*mxGetPrSafe(mxGetProperty(pShape,0,"radius")));
            element.setGeometry(DrakeShapes::Sphere(r));
          }
            break;
          case DrakeShapes::CYLINDER:
          {
            double r(*mxGetPrSafe(mxGetProperty(pShape,0,"radius")));
            double l(*mxGetPrSafe(mxGetProperty(pShape,0,"len")));
            element.setGeometry(DrakeShapes::Cylinder(r, l));
          }
            break;
          case DrakeShapes::MESH:
          {
            string filename(mxArrayToString(mxGetProperty(pShape,0,"filename")));
            element.setGeometry(DrakeShapes::Mesh(filename, filename));
          }
            break;
          case DrakeShapes::MESH_POINTS:
          {
            mxArray* pPoints;
            mexCallMATLAB(1,&pPoints,1,&pShape,"getPoints");
            int n_pts = static_cast<int>(mxGetN(pPoints));
            Map<Matrix3Xd> pts(mxGetPrSafe(pPoints),3,n_pts);
            element.setGeometry(DrakeShapes::MeshPoints(pts));
            mxDestroyArray(pPoints);
            // The element-to-link transform is applied in
            // RigidBodyMesh/getPoints - don't apply it again!
            T = Matrix4d::Identity();
          }
            break;
          case DrakeShapes::CAPSULE:
          {
            double r(*mxGetPrSafe(mxGetProperty(pShape,0,"radius")));
            double l(*mxGetPrSafe(mxGetProperty(pShape,0,"len")));
            element.setGeometry(DrakeShapes::Capsule(r, l));
          }
            break;
          default:
            // intentionally do nothing..
            
            //DEBUG
            //cout << "constructModelmex: SHOULD NOT GET HERE" << endl;
            //END_DEBUG
            break;
        }
        //DEBUG
        //cout << "constructModelmex: geometry = " << geometry.get() << endl;
        //END_DEBUG
        model->addCollisionElement(element, model->bodies[i], group_name);
      }
      if (!model->bodies[i]->hasParent()) {
        model->updateCollisionElements(model->bodies[i]);  // update static objects only once - right here on load
      }



      // Set collision filtering bitmasks
      pm = mxGetProperty(pBodies,i,"collision_filter");
      DrakeCollision::bitmask group, mask;

      mxArray* belongs_to = mxGetField(pm,0,"belongs_to");
      mxArray* ignores = mxGetField(pm,0,"ignores");
      if (!(mxIsLogical(belongs_to)) || !isMxArrayVector(belongs_to)) {
        cout << "is logical: " << mxIsLogical(belongs_to) << endl;
        cout << "number of dimensions: " << mxGetNumberOfDimensions(belongs_to) << endl;
        mexErrMsgIdAndTxt("Drake:constructModelmex:BadCollisionFilterStruct",
                          "The 'belongs_to' field of the 'collision_filter' "
                          "struct must be a logical vector.");
      }
      if (!(mxIsLogical(ignores)) || !isMxArrayVector(ignores)) {
        mexErrMsgIdAndTxt("Drake:constructModelmex:BadCollisionFilterStruct",
                          "The 'ignores' field of the 'collision_filter' "
                          "struct must be a logical vector.");
      }
      size_t numel_belongs_to(mxGetNumberOfElements(belongs_to));
      size_t numel_ignores(mxGetNumberOfElements(ignores));
      size_t num_collision_filter_groups = max(numel_belongs_to, numel_ignores);
      if (num_collision_filter_groups > MAX_NUM_COLLISION_FILTER_GROUPS) {
        mexErrMsgIdAndTxt("Drake:constructModelmex:TooManyCollisionFilterGroups",
                          "The total number of collision filter groups (%d) "
                          "exceeds the maximum allowed number (%d)", 
                          num_collision_filter_groups, 
                          MAX_NUM_COLLISION_FILTER_GROUPS);
      }

      mxLogical* logical_belongs_to = mxGetLogicals(belongs_to);
      for (int j = 0; j < numel_belongs_to; ++j) {
        if (static_cast<bool>(logical_belongs_to[j])) {
          group.set(j);
        }
      }

      mxLogical* logical_ignores = mxGetLogicals(ignores);
      for (int j = 0; j < numel_ignores; ++j) {
        if (static_cast<bool>(logical_ignores[j])) {
          mask.set(j);
        }
      }
      model->bodies[i]->setCollisionFilter(group,mask);
    }
  }

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
      Map<Matrix3Xd> pts(mxGetPrSafe(pPts),3,n_pts);
      model->bodies[body_idx]->contact_pts = pts;
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
    memcpy(model->frames[i].Ttree.data(),mxGetPrSafe(pm),sizeof(double)*4*4);
  }

  const mxArray* a_grav_array = mxGetProperty(pRBM,0,"gravity");
  if (a_grav_array && mxGetNumberOfElements(a_grav_array)==3) {
    double* p = mxGetPrSafe(a_grav_array);
    model->a_grav[3] = p[0];
    model->a_grav[4] = p[1];
    model->a_grav[5] = p[2];
  } else {
    mexErrMsgIdAndTxt("Drake:constructModelmex:BadGravity","Couldn't find a 3 element gravity vector in the object.");
  }

  //  LOOP CONSTRAINTS
  const mxArray* pLoops = mxGetProperty(pRBM,0,"loop");
  int num_loops = static_cast<int>(mxGetNumberOfElements(pLoops));
  model->loops.clear();
  for (int i=0; i<num_loops; i++)
  {
    pm = mxGetProperty(pLoops,i,"body1");
    int body_A_ind = static_cast<int>(mxGetScalar(pm)-1);
    pm = mxGetProperty(pLoops,i,"body2");
    int body_B_ind = static_cast<int>(mxGetScalar(pm)-1);
    pm = mxGetProperty(pLoops,i,"pt1");
    Vector3d pA;
    memcpy(pA.data(), mxGetPrSafe(pm), 3*sizeof(double));
    pm = mxGetProperty(pLoops,i,"pt2");
    Vector3d pB;
    memcpy(pB.data(), mxGetPrSafe(pm), 3*sizeof(double));
    model->loops.push_back(RigidBodyLoop(model->bodies[body_A_ind], pA, model->bodies[body_B_ind], pB));
  }

  //ACTUATORS
  const mxArray* pActuators = mxGetProperty(pRBM,0,"actuator");
  int num_actuators = static_cast<int>(mxGetNumberOfElements(pActuators));
  model->actuators.clear();
  for (int i=0; i<num_actuators; i++)
  {
    pm = mxGetProperty(pActuators,i,"name");
    mxGetString(pm,buf,100);
    pm = mxGetProperty(pActuators,i,"joint");
    int joint = static_cast<int>(mxGetScalar(pm)-1);
    pm = mxGetProperty(pActuators,i, "reduction");
    model->actuators.push_back(RigidBodyActuator(std::string(buf), model->bodies[joint], static_cast<double>(mxGetScalar(pm))));
  }  

  model->compile();

  plhs[0] = createDrakeMexPointer((void*)model,"RigidBodyManipulator");
  //DEBUG
  //cout << "constructModelmex: END" << endl;
  //END_DEBUG
}