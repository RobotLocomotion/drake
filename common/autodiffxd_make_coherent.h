#pragma once

#include "drake/common/autodiff.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/symbolic/expression.h"

namespace drake {

/// Makes the derviatives of the recipient coherent with respect to those of the
/// donor variable (see drake/common/autodiffxd.h).  If the recipient's
/// derivatives are already populated with a vector of the same size as that of
/// the donor, variables pass through unchanged.  An exception is thrown when
/// there are nonempty vectors of different sizes.
DRAKE_DEPRECATED("2022-11-01",
    "This function is being removed; "
    "if you still need it you may copy the code into your own project.")
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

DRAKE_DEPRECATED("2022-11-01",
    "This function is being removed; "
    "if you still need it you may copy the code into your own project.")
inline void autodiffxd_make_coherent(const double&, double*) {}

DRAKE_DEPRECATED("2022-11-01",
    "This function is being removed; "
    "if you still need it you may copy the code into your own project.")
inline void autodiffxd_make_coherent(const symbolic::Expression&,
                                     symbolic::Expression*) {}

}  // namespace drake
