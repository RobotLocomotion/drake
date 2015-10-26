#ifndef DRAKE_RIGIDBODYMANIPULATORMEXCONVERSIONS_H
#define DRAKE_RIGIDBODYMANIPULATORMEXCONVERSIONS_H

#include "RigidBodyManipulator.h"
#include "KinematicsCache.h"
#include "KinematicPath.h"
#include "mexify.h"

/**
 * fromMex specializations
 */
bool isConvertibleFromMex(const mxArray *source, RigidBodyManipulator*, std::ostream* log) NOEXCEPT {
  if (!mxIsClass(source, "DrakeMexPointer")) {
    if (log)
      *log << "Expected DrakeMexPointer containing RigidBodyManipulator";
    return false;
  }
  return true;
}

RigidBodyManipulator &fromMexUnsafe(const mxArray *source, RigidBodyManipulator *) {
  return *static_cast<RigidBodyManipulator *>(getDrakeMexPointer(source));
}

template<typename Scalar>
bool isConvertibleFromMex(const mxArray *mex, KinematicsCache<Scalar>*, std::ostream* log) NOEXCEPT {
  if (!mxIsClass(mex, "DrakeMexPointer")) {
    if (log)
      *log << "Expected DrakeMexPointer containing KinematicsCache";
    return false;
  }
  auto name = mxGetStdString(mxGetPropertySafe(mex, "name"));
  if (name != typeid(KinematicsCache<Scalar>).name()) {
    if (log)
      *log << "Expected KinematicsCache of type " << typeid(KinematicsCache<Scalar>).name() << ", but got " << name;
    return false;
  }
  return true;
}

template<typename Scalar>
KinematicsCache<Scalar> &fromMexUnsafe(const mxArray *mex, KinematicsCache<Scalar> *) {
  return *static_cast<KinematicsCache<Scalar> *>(getDrakeMexPointer(mex));
}

// set<int>: not of sufficient quality yet
bool isConvertibleFromMex(const mxArray* source, std::set<int>*, std::ostream* log) NOEXCEPT {
  return true; // TODO: improve
}

std::set<int> fromMexUnsafe(const mxArray *source, std::set<int> *) {
  std::set<int> ret;
  int num_robot = static_cast<int>(mxGetNumberOfElements(source));
  double *data = mxGetPrSafe(source);
  for (int i = 0; i < num_robot; i++) {
    ret.insert((int) data[i]);
  }
  return ret;
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
