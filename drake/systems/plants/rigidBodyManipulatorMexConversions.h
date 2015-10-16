#ifndef DRAKE_RIGIDBODYMANIPULATORMEXCONVERSIONS_H
#define DRAKE_RIGIDBODYMANIPULATORMEXCONVERSIONS_H

#include "RigidBodyManipulator.h"
#include "KinematicsCache.h"
#include "KinematicPath.h"
#include "mexify.h"

/**
 * fromMex specializations
 */
RigidBodyManipulator &fromMex(const mxArray *source, RigidBodyManipulator *) {
  return *static_cast<RigidBodyManipulator *>(getDrakeMexPointer(source));
}

template<typename Scalar>
KinematicsCache<Scalar> &fromMex(const mxArray *mex, KinematicsCache<Scalar> *) {
  if (!mxIsClass(mex, "DrakeMexPointer")) {
    throw MexToCppConversionError("Expected DrakeMexPointer containing KinematicsCache");
  }
  auto name = mxGetStdString(mxGetPropertySafe(mex, "name"));
  if (name != typeid(KinematicsCache<Scalar>).name()) {
    std::ostringstream buf;
    buf << "Expected KinematicsCache of type " << typeid(KinematicsCache<Scalar>).name() << ", but got " << name;
    throw MexToCppConversionError(buf.str());
  }
  return *static_cast<KinematicsCache<Scalar> *>(getDrakeMexPointer(mex));
}

/**
 * toMex specializations
 */
void toMex(const KinematicPath &path, mxArray *dest[], int nlhs) {
  if (nlhs > 0) {
    dest[0] = stdVectorToMatlab(path.body_path);
  }
  if (nlhs > 1) {
    dest[1] = stdVectorToMatlab(path.joint_path);
  }
  if (nlhs > 2) {
    dest[2] = stdVectorToMatlab(path.joint_direction_signs);
  }
}

#endif //DRAKE_RIGIDBODYMANIPULATORMEXCONVERSIONS_H
