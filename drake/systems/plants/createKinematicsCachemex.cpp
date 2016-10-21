#include "drake/matlab/util/drakeMexUtil.h"
#include "rigidBodyTreeMexConversions.h"

using namespace Eigen;
using namespace std;

/**
 * usage:
 * * createKinematicsCachemex(model_ptr) to construct
 * * createKinematicsCachemex(cache_ptr) to destruct
 */
DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nlhs == 0 && nrhs == 1) {
    // if no output arguments, then assume the destructor is being called
    destroyDrakeMexPointer<KinematicsCache<double> *>(prhs[0]);
  } else if (nlhs == 1 && nrhs == 1) {
    RigidBodyTree *model =
        static_cast<RigidBodyTree *>(getDrakeMexPointer(prhs[0]));
    KinematicsCache<double> *cache = new KinematicsCache<double>(model->bodies);
    plhs[0] = createDrakeMexPointer(
        (void *)cache, typeid(KinematicsCache<double>).name(),
        DrakeMexPointerTypeId<KinematicsCache<double>>::value);
  } else {
    mexErrMsgTxt("couldn't parse input");
  }
}
