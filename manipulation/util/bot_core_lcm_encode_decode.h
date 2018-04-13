#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "bot_core/position_3d_t.hpp"
#include "bot_core/quaternion_t.hpp"
#include "bot_core/twist_t.hpp"
#include "bot_core/vector_3d_t.hpp"

#include "drake/common/eigen_types.h"

void EncodeVector3d(
    const Eigen::Ref<const Eigen::Vector3d>& vec,
    // NOLINTNEXTLINE(runtime/references)
    bot_core::vector_3d_t& msg);

Eigen::Vector3d DecodeVector3d(
    const bot_core::vector_3d_t& msg);

void EncodeQuaternion(const Eigen::Quaterniond& q,
    // NOLINTNEXTLINE(runtime/references)
    bot_core::quaternion_t& msg);

Eigen::Vector4d DecodeQuaternion(
    const bot_core::quaternion_t& msg);

// Note that bot_core::position_3d_t is badly named.
void EncodePose(
    const Eigen::Isometry3d& pose,
    // NOLINTNEXTLINE(runtime/references)
    bot_core::position_3d_t& msg);

// Note that bot_core::position_3d_t is badly named.
Eigen::Isometry3d DecodePose(
    const bot_core::position_3d_t& msg);

void EncodeTwist(
    const Eigen::Ref<const drake::TwistVector<double>>& twist,
    // NOLINTNEXTLINE(runtime/references)
    bot_core::twist_t& msg);

drake::TwistVector<double> DecodeTwist(
    const bot_core::twist_t& msg);
