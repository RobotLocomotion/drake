#pragma once

#include "drake/matlab/util/mexify.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/KinematicsCache.h"
#include "drake/systems/plants/KinematicPath.h"

template <typename T>
struct DrakeMexPointerTypeId {
  enum { value = -1 };
};
template <>
struct DrakeMexPointerTypeId<RigidBodyTree> {
  enum { value = 1 };
};
template <>
struct DrakeMexPointerTypeId<KinematicsCache<double>> {
  enum { value = 2 };
};
template <>
struct DrakeMexPointerTypeId<
    KinematicsCache<Eigen::AutoDiffScalar<Eigen::VectorXd>>> {
  enum { value = 3 };
};
template <>
struct DrakeMexPointerTypeId<
    KinematicsCache<DrakeJoint::AutoDiffFixedMaxSize>> {
  enum { value = 4 };
};

template <typename T>
bool isDrakeMexPointerOfCorrectType(const mxArray *source, T *,
                                    std::ostream *log) NOEXCEPT {
  if (!mxIsClass(source, "DrakeMexPointer")) {
    if (log) *log << "Expected DrakeMexPointer";
    return false;
  }
  int type_id =
      static_cast<int>(mxGetScalar(mxGetProperty(source, 0, "type_id")));
  if (type_id != DrakeMexPointerTypeId<T>::value) {
    if (log) *log << "Expected DrakeMexPointer to " << typeid(T).name() << ".";
    return false;
  }
  return true;
}

/**
 * fromMex specializations
 */
bool isConvertibleFromMex(const mxArray *source, RigidBodyTree *ptr,
                          std::ostream *log) NOEXCEPT {
  return isDrakeMexPointerOfCorrectType(source, ptr, log);
}

RigidBodyTree &fromMexUnsafe(const mxArray *source, RigidBodyTree *) {
  return *static_cast<RigidBodyTree *>(getDrakeMexPointer(source));
}

template <typename Scalar>
bool isConvertibleFromMex(const mxArray *mex, KinematicsCache<Scalar> *ptr,
                          std::ostream *log) NOEXCEPT {
  return isDrakeMexPointerOfCorrectType(mex, ptr, log);
}

template <typename Scalar>
KinematicsCache<Scalar> &fromMexUnsafe(const mxArray *mex,
                                       KinematicsCache<Scalar> *) {
  return *static_cast<KinematicsCache<Scalar> *>(getDrakeMexPointer(mex));
}

// set<int>: not of sufficient quality yet
bool isConvertibleFromMex(const mxArray *source, std::set<int> *,
                          std::ostream *log) NOEXCEPT {
  return true;  // TODO(tkoolen): improve
}

std::set<int> fromMexUnsafe(const mxArray *source, std::set<int> *) {
  std::set<int> ret;
  int num_robot = static_cast<int>(mxGetNumberOfElements(source));
  double *data = mxGetPrSafe(source);
  for (int i = 0; i < num_robot; i++) {
    ret.insert((int)data[i]);
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
