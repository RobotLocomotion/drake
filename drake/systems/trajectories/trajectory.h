#pragma once

#include <Eigen/Core>
#include "drake/common/drake_export.h"

namespace drake {

class DRAKE_EXPORT Trajectory {
 public:
  virtual Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> value(
      double t) const = 0;
};

}  // namespace drake
