#pragma once

#include "drake/common/autodiff.h"
#include "drake/common/symbolic.h"

namespace drake {

/// Makes the derviatives of the recipient coherent with respect to those of the
/// donor variable (see drake/common/autodiffxd.h).  If the recipient's
/// derivatives are already populated with a vector of the same size as that of
/// the donor, variables pass through unchanged.  An exception is thrown when
/// there are nonempty vectors of different sizes.
inline void autodiffxd_make_coherent(const AutoDiffXd& donor,
                                     AutoDiffXd* recipient) {
  DRAKE_ASSERT(recipient != nullptr);
  if (recipient->derivatives().size() == 0) {
    recipient->derivatives() =
        Eigen::VectorXd::Zero(donor.derivatives().size());
  } else if (recipient->derivatives().size() != donor.derivatives().size()) {
    throw std::runtime_error("Nonempty recipient derivatives must match the "
                             "size of the donor derivatives.");
  }
}

inline void autodiffxd_make_coherent(const double&, double*) {}

inline void autodiffxd_make_coherent(const symbolic::Expression&,
                                     symbolic::Expression*) {}

}  // namespace drake
