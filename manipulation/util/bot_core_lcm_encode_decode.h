#pragma once

#include "bot_core/position_3d_t.hpp"
#include "bot_core/quaternion_t.hpp"
#include "bot_core/twist_t.hpp"
#include "bot_core/vector_3d_t.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"

DRAKE_DEPRECATED("2020-10-01",
    "This function is unused in Drake, so is deprecated with no replacement; "
    "you may copy it into your own project, if you still need it.")
void EncodeVector3d(
    const Eigen::Ref<const Eigen::Vector3d>& vec,
    // NOLINTNEXTLINE(runtime/references)
    bot_core::vector_3d_t& msg);

DRAKE_DEPRECATED("2020-10-01",
    "This function is unused in Drake, so is deprecated with no replacement; "
    "you may copy it into your own project, if you still need it.")
Eigen::Vector3d DecodeVector3d(
    const bot_core::vector_3d_t& msg);

DRAKE_DEPRECATED("2020-10-01",
    "This function is unused in Drake, so is deprecated with no replacement; "
    "you may copy it into your own project, if you still need it.")
void EncodeQuaternion(const Eigen::Quaterniond& q,
    // NOLINTNEXTLINE(runtime/references)
    bot_core::quaternion_t& msg);

DRAKE_DEPRECATED("2020-10-01",
    "This function is unused in Drake, so is deprecated with no replacement; "
    "you may copy it into your own project, if you still need it.")
Eigen::Vector4d DecodeQuaternion(
    const bot_core::quaternion_t& msg);

// Note that bot_core::position_3d_t is badly named.
DRAKE_DEPRECATED("2020-10-01",
    "This function is unused in Drake, so is deprecated with no replacement; "
    "you may copy it into your own project, if you still need it.")
void EncodePose(
    const Eigen::Isometry3d& pose,
    // NOLINTNEXTLINE(runtime/references)
    bot_core::position_3d_t& msg);

// Note that bot_core::position_3d_t is badly named.
DRAKE_DEPRECATED("2020-10-01",
    "This function is unused in Drake, so is deprecated with no replacement; "
    "you may copy it into your own project, if you still need it.")
Eigen::Isometry3d DecodePose(
    const bot_core::position_3d_t& msg);

DRAKE_DEPRECATED("2020-10-01",
    "This function is unused in Drake, so is deprecated with no replacement; "
    "you may copy it into your own project, if you still need it.")
void EncodeTwist(
    const Eigen::Ref<const drake::TwistVector<double>>& twist,
    // NOLINTNEXTLINE(runtime/references)
    bot_core::twist_t& msg);

DRAKE_DEPRECATED("2020-10-01",
    "This function is unused in Drake, so is deprecated with no replacement; "
    "you may copy it into your own project, if you still need it.")
drake::TwistVector<double> DecodeTwist(
    const bot_core::twist_t& msg);
