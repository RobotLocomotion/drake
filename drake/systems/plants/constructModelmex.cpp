#include <mex.h>

#include <iostream>
#include <cmath>
#include "drake/matlab/util/drakeMexUtil.h"
#include "rigidBodyTreeMexConversions.h"
#include <stdexcept>
#include "drake/systems/plants/joints/DrakeJoints.h"

using namespace Eigen;
using namespace std;

bool isMxArrayVector(const mxArray* array) {
  size_t num_rows = mxGetM(array);
  size_t num_cols = mxGetN(array);
  return (num_rows <= 1) || (num_cols <= 1);
}

template <typename Derived>
void setDynamics(const mxArray* pBodies, int i,
                 FixedAxisOneDoFJoint<Derived>* fixed_axis_one_dof_joint) {
  double damping = mxGetScalar(mxGetPropertySafe(pBodies, i, "damping"));
  double coulomb_friction =
      mxGetScalar(mxGetPropertySafe(pBodies, i, "coulomb_friction"));
  double coulomb_window =
      mxGetScalar(mxGetPropertySafe(pBodies, i, "coulomb_window"));
  fixed_axis_one_dof_joint->setDynamics(damping, coulomb_friction,
                                        coulomb_window);
}

template <typename Derived>
void setLimits(const mxArray* pBodies, int i,
               FixedAxisOneDoFJoint<Derived>* fixed_axis_one_dof_joint) {
  double joint_limit_min =
      mxGetScalar(mxGetPropertySafe(pBodies, i, "joint_limit_min"));
  double joint_limit_max =
      mxGetScalar(mxGetPropertySafe(pBodies, i, "joint_limit_max"));
  fixed_axis_one_dof_joint->setJointLimits(joint_limit_min, joint_limit_max);
}

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  // DEBUG
  // mexPrintf("constructModelmex: START\n");
  // END_DEBUG
  mxArray* pm;

  if (nrhs != 1) {
    mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs",
                      "Usage model_ptr = constructModelmex(obj)");
  }

  if (isa(prhs[0], "DrakeMexPointer")) {  // then it's calling the destructor
    destroyDrakeMexPointer<RigidBodyTree*>(prhs[0]);
    return;
  }

  const mxArray* pRBM = prhs[0];
  RigidBodyTree* model = new RigidBodyTree();
  model->bodies.clear();  // a little gross:  the default constructor makes a
                          // body "world".  zap it because we will construct one
                          // again below

  //  model->robot_name = get_strings(mxGetPropertySafe(pRBM, 0,"name"));

  const mxArray* pBodies = mxGetPropertySafe(pRBM, 0, "body");
  if (!pBodies)
    mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs",
                      "the body array is invalid");
  int num_bodies = static_cast<int>(mxGetNumberOfElements(pBodies));

  const mxArray* pFrames = mxGetPropertySafe(pRBM, 0, "frame");
  if (!pFrames)
    mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs",
                      "the frame array is invalid");
  int num_frames = static_cast<int>(mxGetNumberOfElements(pFrames));

  for (int i = 0; i < num_bodies; i++) {
    // DEBUG
    // mexPrintf("constructModelmex: body %d\n", i);
    // END_DEBUG
    std::unique_ptr<RigidBody> b(new RigidBody());
    b->set_body_index(i);

    b->set_name(mxGetStdString(mxGetPropertySafe(pBodies, i, "linkname")));

    pm = mxGetPropertySafe(pBodies, i, "robotnum");
    b->set_model_instance_id((int)mxGetScalar(pm) - 1);

    pm = mxGetPropertySafe(pBodies, i, "mass");
    b->set_mass(mxGetScalar(pm));

    pm = mxGetPropertySafe(pBodies, i, "com");
    Eigen::Vector3d com;
    if (!mxIsEmpty(pm)) {
      memcpy(com.data(), mxGetPrSafe(pm), sizeof(double) * 3);
      b->set_center_of_mass(com);
    }

    pm = mxGetPropertySafe(pBodies, i, "I");
    if (!mxIsEmpty(pm)) {
      drake::SquareTwistMatrix<double> I;
      memcpy(I.data(), mxGetPrSafe(pm), sizeof(double) * 6 * 6);
      b->set_spatial_inertia(I);
    }

    pm = mxGetPropertySafe(pBodies, i, "position_num");
    b->set_position_start_index((int)mxGetScalar(pm) - 1);  // zero-indexed

    pm = mxGetPropertySafe(pBodies, i, "velocity_num");
    b->set_velocity_start_index((int)mxGetScalar(pm) - 1);  // zero-indexed

    pm = mxGetPropertySafe(pBodies, i, "parent");
    if (!pm || mxIsEmpty(pm)) {
      b->set_parent(nullptr);
    } else {
      int parent_ind = static_cast<int>(mxGetScalar(pm)) - 1;
      if (parent_ind >= static_cast<int>(model->bodies.size()))
        mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs",
                          "bad body.parent %d (only have %d bodies)",
                          parent_ind, model->bodies.size());
      if (parent_ind >= 0) b->set_parent(model->bodies[parent_ind].get());
    }

    if (b->has_parent_body()) {
      string joint_name =
          mxGetStdString(mxGetPropertySafe(pBodies, i, "jointname"));
      // mexPrintf("adding joint %s\n", joint_name.c_str());

      pm = mxGetPropertySafe(pBodies, i, "Ttree");
      // todo: check that the size is 4x4
      Isometry3d transform_to_parent_body;
      memcpy(transform_to_parent_body.data(), mxGetPrSafe(pm),
             sizeof(double) * 4 * 4);

      int floating =
          (int)mxGetScalar(mxGetPropertySafe(pBodies, i, "floating"));

      Eigen::Vector3d joint_axis;
      pm = mxGetPropertySafe(pBodies, i, "joint_axis");
      memcpy(joint_axis.data(), mxGetPrSafe(pm), sizeof(double) * 3);

      double pitch = mxGetScalar(mxGetPropertySafe(pBodies, i, "pitch"));

      std::unique_ptr<DrakeJoint> joint;
      switch (floating) {
        case 0: {
          if (pitch == 0.0) {
            RevoluteJoint* revolute_joint = new RevoluteJoint(
                joint_name, transform_to_parent_body, joint_axis);
            joint = std::unique_ptr<RevoluteJoint>(revolute_joint);
            setLimits(pBodies, i, revolute_joint);
            setDynamics(pBodies, i, revolute_joint);
          } else if (std::isinf(static_cast<double>(pitch))) {
            PrismaticJoint* prismatic_joint = new PrismaticJoint(
                joint_name, transform_to_parent_body, joint_axis);
            joint = std::unique_ptr<PrismaticJoint>(prismatic_joint);
            setLimits(pBodies, i, prismatic_joint);
            setDynamics(pBodies, i, prismatic_joint);
          } else {
            joint = std::unique_ptr<HelicalJoint>(new HelicalJoint(
                joint_name, transform_to_parent_body, joint_axis, pitch));
          }
          break;
        }
        case 1: {
          joint = std::unique_ptr<RollPitchYawFloatingJoint>(
              new RollPitchYawFloatingJoint(joint_name,
                                            transform_to_parent_body));
          break;
        }
        case 2: {
          joint = std::unique_ptr<QuaternionFloatingJoint>(
              new QuaternionFloatingJoint(joint_name,
                                          transform_to_parent_body));
          break;
        }
        default: {
          std::ostringstream stream;
          stream << "floating type " << floating << " not recognized.";
          throw std::runtime_error(stream.str());
        }
      }

      b->setJoint(std::move(joint));
    }

    // DEBUG
    // mexPrintf("constructModelmex: About to parse collision geometry\n");
    // END_DEBUG
    pm = mxGetPropertySafe(pBodies, i, "collision_geometry");
    Isometry3d T;
    if (!mxIsEmpty(pm)) {
      for (int j = 0; j < mxGetNumberOfElements(pm); j++) {
        // DEBUG
        // cout << "constructModelmex: Body " << i << ", Element " << j << endl;
        // END_DEBUG
        mxArray* pShape = mxGetCell(pm, j);
        char* group_name_cstr =
            mxArrayToString(mxGetPropertySafe(pShape, 0, "name"));
        string group_name;
        if (group_name_cstr) {
          group_name = group_name_cstr;
        } else {
          group_name = "default";
        }

        // Get element-to-link transform from MATLAB object
        memcpy(T.data(), mxGetPrSafe(mxGetPropertySafe(pShape, 0, "T")),
               sizeof(double) * 4 * 4);
        auto shape = (DrakeShapes::Shape) static_cast<int>(
            mxGetScalar(mxGetPropertySafe(pShape, 0, "drake_shape_id")));
        vector<double> params_vec;
        DrakeCollision::Element element(T, b.get());
        switch (shape) {
          case DrakeShapes::BOX: {
            double* params = mxGetPrSafe(mxGetPropertySafe(pShape, 0, "size"));
            element.setGeometry(
                DrakeShapes::Box(Vector3d(params[0], params[1], params[2])));
          } break;
          case DrakeShapes::SPHERE: {
            double r(*mxGetPrSafe(mxGetPropertySafe(pShape, 0, "radius")));
            element.setGeometry(DrakeShapes::Sphere(r));
          } break;
          case DrakeShapes::CYLINDER: {
            double r(*mxGetPrSafe(mxGetPropertySafe(pShape, 0, "radius")));
            double l(*mxGetPrSafe(mxGetPropertySafe(pShape, 0, "len")));
            element.setGeometry(DrakeShapes::Cylinder(r, l));
          } break;
          case DrakeShapes::MESH: {
            string filename(
                mxArrayToString(mxGetPropertySafe(pShape, 0, "filename")));
            element.setGeometry(DrakeShapes::Mesh(filename, filename));
          } break;
          case DrakeShapes::MESH_POINTS: {
            mxArray* pPoints;
            mexCallMATLAB(1, &pPoints, 1, &pShape, "getPoints");
            int n_pts = static_cast<int>(mxGetN(pPoints));
            Map<Matrix3Xd> pts(mxGetPrSafe(pPoints), 3, n_pts);
            element.setGeometry(DrakeShapes::MeshPoints(pts));
            mxDestroyArray(pPoints);
            // The element-to-link transform is applied in
            // RigidBodyMesh/getPoints - don't apply it again!
            T = Matrix4d::Identity();
          } break;
          case DrakeShapes::CAPSULE: {
            double r(*mxGetPrSafe(mxGetPropertySafe(pShape, 0, "radius")));
            double l(*mxGetPrSafe(mxGetPropertySafe(pShape, 0, "len")));
            element.setGeometry(DrakeShapes::Capsule(r, l));
          } break;
          default:
            // intentionally do nothing..

            // DEBUG
            // cout << "constructModelmex: SHOULD NOT GET HERE" << endl;
            // END_DEBUG
            break;
        }
        // DEBUG
        // cout << "constructModelmex: geometry = " << geometry.get() << endl;
        // END_DEBUG
        model->addCollisionElement(element, *b, group_name);
      }
      // NOTE: the following should not be necessary since the same thing is
      // being done in RigidBodyTree::compile, which is called below.
      //      if (!model->bodies[i]->has_parent_body()) {
      //        model->updateCollisionElements(model->bodies[i], cache);  //
      //        update static objects only once - right here on load
      //      }

      // Set collision filtering bitmasks
      pm = mxGetPropertySafe(pBodies, i, "collision_filter");
      DrakeCollision::bitmask group, mask;

      mxArray* belongs_to = mxGetField(pm, 0, "belongs_to");
      mxArray* ignores = mxGetField(pm, 0, "ignores");
      if (!(mxIsLogical(belongs_to)) || !isMxArrayVector(belongs_to)) {
        cout << "is logical: " << mxIsLogical(belongs_to) << endl;
        cout << "number of dimensions: " << mxGetNumberOfDimensions(belongs_to)
             << endl;
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
        mexErrMsgIdAndTxt(
            "Drake:constructModelmex:TooManyCollisionFilterGroups",
            "The total number of collision filter groups (%d) "
            "exceeds the maximum allowed number (%d)",
            num_collision_filter_groups, MAX_NUM_COLLISION_FILTER_GROUPS);
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
      b->setCollisionFilter(group, mask);
    }

    model->bodies.push_back(std::move(b));
  }

  // THIS IS UGLY: I'm sending the terrain contact points into the
  // contact_pts field of the cpp RigidBody objects
  // DEBUG
  // mexPrintf("constructModelmex: Parsing contact points (calling
  // getTerrainContactPoints)\n");
  // cout << "constructModelmex: Get struct" << endl;
  // END_DEBUG
  mxArray* contact_pts_struct[1];
  if (!mexCallMATLAB(1, contact_pts_struct, 1, const_cast<mxArray**>(&pRBM),
                     "getTerrainContactPoints")) {
    // DEBUG
    // mexPrintf("constructModelmex: Got terrain contact points struct\n");
    // if (contact_pts_struct) {
    // cout << "constructModelmex: Struct pointer: " << contact_pts_struct <<
    // endl;
    //} else {
    // cout << "constructModelmex: Struct pointer NULL" << endl;
    //}
    // cout << "constructModelmex: Get numel of struct" << endl;
    // END_DEBUG
    const int n_bodies_w_contact_pts =
        static_cast<int>(mxGetNumberOfElements(contact_pts_struct[0]));
    // DEBUG
    // cout << "constructModelmex: Got numel of struct:" <<
    // n_bodies_w_contact_pts << endl;
    // END_DEBUG
    mxArray* pPts;
    int body_idx;
    int n_pts;
    for (int j = 0; j < n_bodies_w_contact_pts; j++) {
      // DEBUG
      // cout << "constructModelmex: Loop: Iteration " << j << endl;
      // cout << "constructModelmex: Get body_idx" << endl;
      // END_DEBUG
      body_idx =
          (int)mxGetScalar(mxGetField(contact_pts_struct[0], j, "idx")) - 1;
      // DEBUG
      // cout << "constructModelmex: Got body_idx: " << body_idx << endl;
      // cout << "constructModelmex: Get points" << endl;
      // END_DEBUG
      pPts = mxGetField(contact_pts_struct[0], j, "pts");
      // DEBUG
      // cout << "constructModelmex: Get points" << endl;
      // cout << "constructModelmex: Get number of points" << endl;
      // END_DEBUG
      n_pts = static_cast<int>(mxGetN(pPts));
      // DEBUG
      // cout << "constructModelmex: Got number of points: " << n_pts << endl;
      // cout << "constructModelmex: Set contact_pts of body" << endl;
      // END_DEBUG
      Map<Matrix3Xd> pts(mxGetPrSafe(pPts), 3, n_pts);
      model->bodies[body_idx]->set_contact_points(pts);
      // DEBUG
      // mexPrintf("constructModelmex: created %d contact points for body %d\n",
      // n_pts, body_idx);
      // cout << "constructModelmex: Contact_pts of body: " << endl;
      // cout << model->bodies[body_idx]->contact_pts << endl;
      // END_DEBUG
    }
  }

  //  FRAMES
  // DEBUG
  // mexPrintf("constructModelmex: Parsing frames\n");
  // END_DEBUG
  for (int i = 0; i < num_frames; i++) {
    shared_ptr<RigidBodyFrame> fr(new RigidBodyFrame());

    fr->set_name(mxGetStdString(mxGetPropertySafe(pFrames, i, "name")));

    pm = mxGetPropertySafe(pFrames, i, "body_ind");
    fr->set_rigid_body(model->bodies[(int)mxGetScalar(pm) - 1].get());

    pm = mxGetPropertySafe(pFrames, i, "T");
    memcpy(fr->get_mutable_transform_to_body()->data(), mxGetPrSafe(pm),
           sizeof(double) * 4 * 4);

    fr->set_frame_index(-i - 2);
    model->frames.push_back(fr);
  }

  const mxArray* a_grav_array = mxGetPropertySafe(pRBM, 0, "gravity");
  if (a_grav_array && mxGetNumberOfElements(a_grav_array) == 3) {
    double* p = mxGetPrSafe(a_grav_array);
    model->a_grav[3] = p[0];
    model->a_grav[4] = p[1];
    model->a_grav[5] = p[2];
  } else {
    mexErrMsgIdAndTxt(
        "Drake:constructModelmex:BadGravity",
        "Couldn't find a 3 element gravity vector in the object.");
  }

  //  LOOP CONSTRAINTS
  // DEBUG
  // mexPrintf("constructModelmex: Parsing loop constraints\n");
  // END_DEBUG
  const mxArray* pLoops = mxGetPropertySafe(pRBM, 0, "loop");
  int num_loops = static_cast<int>(mxGetNumberOfElements(pLoops));
  model->loops.clear();
  for (int i = 0; i < num_loops; i++) {
    pm = mxGetPropertySafe(pLoops, i, "frameA");
    int frame_A_ind = static_cast<int>(-mxGetScalar(pm) - 1);
    if (frame_A_ind < 0 || frame_A_ind >= model->frames.size())
      mexErrMsgIdAndTxt(
          "Drake:constructModelmex:BadFrameNumber",
          "Something is wrong, this doesn't point to a valid frame");
    pm = mxGetPropertySafe(pLoops, i, "frameB");
    int frame_B_ind = static_cast<int>(-mxGetScalar(pm) - 1);
    if (frame_B_ind < 0 || frame_B_ind >= model->frames.size())
      mexErrMsgIdAndTxt(
          "Drake:constructModelmex:BadFrameNumber",
          "Something is wrong, this doesn't point to a valid frame");
    pm = mxGetPropertySafe(pLoops, i, "axis");
    Vector3d axis;
    memcpy(axis.data(), mxGetPrSafe(pm), 3 * sizeof(double));
    //    cout << "loop " << i << ": frame_A = " <<
    //    model->frames[frame_A_ind]->name << ", frame_B = " <<
    //    model->frames[frame_B_ind]->name << endl;
    model->loops.push_back(RigidBodyLoop(model->frames[frame_A_ind],
                                         model->frames[frame_B_ind], axis));
  }

  // ACTUATORS
  //  LOOP CONSTRAINTS
  // DEBUG
  // mexPrintf("constructModelmex: Parsing actuators\n");
  // END_DEBUG
  const mxArray* pActuators = mxGetPropertySafe(pRBM, 0, "actuator");
  int num_actuators = static_cast<int>(mxGetNumberOfElements(pActuators));
  model->actuators.clear();
  for (int i = 0; i < num_actuators; i++) {
    string name = mxGetStdString(mxGetPropertySafe(pActuators, i, "name"));
    pm = mxGetPropertySafe(pActuators, i, "joint");
    size_t joint_index = static_cast<size_t>(mxGetScalar(pm) - 1);
    double reduction =
        mxGetScalar(mxGetPropertySafe(pActuators, i, "reduction"));
    double effort_min =
        mxGetScalar(mxGetPropertySafe(pBodies, joint_index, "effort_min"));
    double effort_max =
        mxGetScalar(mxGetPropertySafe(pBodies, joint_index, "effort_max"));
    model->actuators.push_back(
        RigidBodyActuator(name, model->bodies[joint_index].get(), reduction,
                          effort_min, effort_max));
  }

  //  LOOP CONSTRAINTS
  // DEBUG
  // mexPrintf("constructModelmex: Calling compile\n");
  // END_DEBUG
  model->compile();

  // mexPrintf("constructModelmex: Creating DrakeMexPointer\n");
  plhs[0] = createDrakeMexPointer((void*)model, "RigidBodyTree",
                                  DrakeMexPointerTypeId<RigidBodyTree>::value);
  // DEBUG
  // mexPrintf("constructModelmex: END\n");
  // END_DEBUG
}
