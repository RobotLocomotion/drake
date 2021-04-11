#pragma once

#include "drake/common/drake_deprecated.h"
#include "drake/lcmt_header.hpp"
#include "drake/lcmt_image.hpp"
#include "drake/lcmt_image_array.hpp"
#include "drake/lcmt_point.hpp"
#include "drake/lcmt_quaternion.hpp"

// @file
//
// To ease the deprecation process for the @lcmtypes_robotlocomotion external,
// we provide this header that calling code may temporarily #include in order
// to alias the robotlocomotion message names to their new Drake equivalent.
//
// This file will be removed on 2021-08-01.

namespace robotlocomotion {

using header_t DRAKE_DEPRECATED("2021-08-01",
    "The @lcmtypes_robotlocomotion repository is being removed from Drake. "
    "Downstream projects should add it to their own WORKSPACE if needed.")
    = drake::lcmt_header;

using image_t DRAKE_DEPRECATED("2021-08-01",
    "The @lcmtypes_robotlocomotion repository is being removed from Drake. "
    "Downstream projects should add it to their own WORKSPACE if needed.")
    = drake::lcmt_image;

using image_array_t DRAKE_DEPRECATED("2021-08-01",
    "The @lcmtypes_robotlocomotion repository is being removed from Drake. "
    "Downstream projects should add it to their own WORKSPACE if needed.")
    = drake::lcmt_image_array;

using point_t DRAKE_DEPRECATED("2021-08-01",
    "The @lcmtypes_robotlocomotion repository is being removed from Drake. "
    "Downstream projects should add it to their own WORKSPACE if needed.")
    = drake::lcmt_point;

using quaternion_t DRAKE_DEPRECATED("2021-08-01",
    "The @lcmtypes_robotlocomotion repository is being removed from Drake. "
    "Downstream projects should add it to their own WORKSPACE if needed.")
    = drake::lcmt_quaternion;

}  // namespace robotlocomotion
