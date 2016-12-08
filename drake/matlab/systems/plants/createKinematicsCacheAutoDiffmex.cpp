#include "drake/matlab/util/drakeMexUtil.h"
#include "drake/matlab/systems/plants/rigidBodyTreeMexConversions.h"

#include <memory>

using namespace Eigen;
using namespace std;

template <typename Scalar>
mxArray *createKinematicsCache(RigidBodyTreeWithAlternates<double> &model) {
  auto cache = std::make_unique<KinematicsCache<Scalar>>(
      model.CreateKinematicsCacheWithType<Scalar>());
  return createDrakeMexPointer(
      (void *)cache.release(), typeid(KinematicsCache<Scalar>).name(),
      DrakeMexPointerTypeId<KinematicsCache<Scalar>>::value);
}

template <typename Scalar>
void destructKinematicsCache(const mxArray *mex) {
  destroyDrakeMexPointer<KinematicsCache<Scalar> *>(mex);
}

/**
 * usage:
 * * createKinematicsCacheAutoDiffmex(model_ptr, num_derivs) to construct
 * * createKinematicsCacheAutoDiffmex(cache_ptr) to destruct
 */
DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nlhs == 0 && nrhs == 1) {
    // if a KinematicsCache is passed in, then assume the destructor is being
    // called
    int type_id =
        static_cast<int>(mxGetScalar(mxGetProperty(prhs[0], 0, "type_id")));

    switch (type_id) {
      case DrakeMexPointerTypeId<
          KinematicsCache<AutoDiffScalar<VectorXd>>>::value:
        destructKinematicsCache<AutoDiffScalar<VectorXd>>(prhs[0]);
        break;
      case DrakeMexPointerTypeId<
          KinematicsCache<DrakeJoint::AutoDiffFixedMaxSize>>::value:
        destructKinematicsCache<typename DrakeJoint::AutoDiffFixedMaxSize>(
            prhs[0]);
        break;
      default:
        mexErrMsgTxt("unrecognized KinematicsCache type");
    }
  } else if (nlhs == 1 && nrhs == 2) {
    RigidBodyTreeWithAlternates<double> *model =
        static_cast<RigidBodyTreeWithAlternates<double> *>(
            getDrakeMexPointer(prhs[0]));
    int num_derivs = static_cast<int>(mxGetScalar(prhs[1]));
    if (num_derivs <=
        DrakeJoint::AutoDiffFixedMaxSize::DerType::MaxRowsAtCompileTime) {
      plhs[0] =
          createKinematicsCache<typename DrakeJoint::AutoDiffFixedMaxSize>(
              *model);
    } else {
      plhs[0] = createKinematicsCache<AutoDiffScalar<VectorXd>>(*model);
    }
  } else {
    mexErrMsgTxt("couldn't parse input");
  }
}
