#include "drake/util/drakeGeometryUtil.h"

#include <Eigen/Sparse>
#include <stdexcept>

#include "drake/math/axis_angle.h"
#include "drake/math/random_rotation.h"

using namespace Eigen;

double angleDiff(double phi1, double phi2) {
  double d = phi2 - phi1;
  if (d > 0.0) {
    d = fmod(d + M_PI, 2 * M_PI) - M_PI;
  } else {
    d = fmod(d - M_PI, 2 * M_PI) + M_PI;
  }
  return d;
}

DRAKE_EXPORT int rotationRepresentationSize(int rotation_type) {
  switch (rotation_type) {
    case 0:
      return 0;
      break;
    case 1:
      return 3;
      break;
    case 2:
      return 4;
      break;
    default:
      throw std::runtime_error("rotation representation type not recognized");
  }
}
