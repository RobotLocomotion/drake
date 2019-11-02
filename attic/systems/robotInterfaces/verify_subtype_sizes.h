#pragma once

#include "drake/common/drake_deprecated.h"
#include "drake/lcmt_qp_controller_input.hpp"

DRAKE_DEPRECATED("2020-02-01", "The robotInterfaces package is being removed.")
void verifySubtypeSizes(
    // NOLINTNEXTLINE(runtime/references)
    drake::lcmt_support_data& support_data);

DRAKE_DEPRECATED("2020-02-01", "The robotInterfaces package is being removed.")
void verifySubtypeSizes(
    // NOLINTNEXTLINE(runtime/references)
    drake::lcmt_qp_controller_input& qp_input);
