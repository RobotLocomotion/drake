#pragma once

#include "drake/common/autodiff.h"
#include "drake/common/symbolic.h"

namespace drake {

// Makes the derviatives of the recipient coherent with respect to those of the
// donor variable.  See drake/common/autodiffxd.h for further information.

inline static void make_coherent(const AutoDiffXd& donor,
                                 AutoDiffXd* recipient) {
  Eigen::internal::make_coherent(recipient->derivatives(), donor.derivatives());
}

inline static void make_coherent(const double&, double*) {}

inline static void make_coherent(const symbolic::Expression&,
                                 symbolic::Expression*) {}

}  // namespace drake
