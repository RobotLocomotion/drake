#include "drakeMexUtil.h"
#include "RigidBodyManipulator.h"
#include <typeinfo>

using namespace Eigen;
using namespace std;

template <typename Scalar>
mxArray* createKinematicsCache(RigidBodyManipulator& model) {
  KinematicsCache<Scalar>* cache = new KinematicsCache<Scalar>(model.bodies);
  return createDrakeMexPointer((void*)cache, typeid(KinematicsCache<Scalar>).name());
}

template <typename Scalar>
void destructKinematicsCache(const mxArray* mex) {
  destroyDrakeMexPointer<KinematicsCache<Scalar>*>(mex);
}

/**
 * usage:
 * * createKinematicsCacheAutoDiffmex(model_ptr, num_derivs) to construct
 * * createKinematicsCacheAutoDiffmex(cache_ptr) to destruct
 */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nlhs == 0 && nrhs == 1) {
    // if a KinematicsCache is passed in, then assume the destructor is being called
    auto name = mxGetStdString(mxGetPropertySafe(prhs[0], "name"));
    if (name == typeid(KinematicsCache<AutoDiffScalar<VectorXd>>).name()) {
      destructKinematicsCache<AutoDiffScalar<VectorXd>>(prhs[0]);
    }
    else if (name == typeid(KinematicsCache<typename DrakeJoint::AutoDiffFixedMaxSize>).name()) {
      destructKinematicsCache<typename DrakeJoint::AutoDiffFixedMaxSize>(prhs[0]);
    }
    else
      mexErrMsgTxt("unrecognized KinematicsCache type");
  }
  else if (nlhs == 1 && nrhs == 2) {
    RigidBodyManipulator *model = static_cast<RigidBodyManipulator*>(getDrakeMexPointer(prhs[0]));
    int num_derivs = static_cast<int>(mxGetScalar(prhs[1]));
    if (num_derivs <= DrakeJoint::AutoDiffFixedMaxSize::DerType::MaxRowsAtCompileTime) {
      plhs[0] = createKinematicsCache<typename DrakeJoint::AutoDiffFixedMaxSize>(*model);
    }
    else {
      plhs[0] = createKinematicsCache<AutoDiffScalar<VectorXd>>(*model);
    }
  }
  else
    mexErrMsgTxt("couldn't parse input");
}
