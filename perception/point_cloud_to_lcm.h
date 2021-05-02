#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace perception {

/// Converts PointCloud inputs to lcmt_point_cloud output messages.  The
/// message can be transmitted other processes using LcmPublisherSystem.
///
/// @system
/// name: PointCloudToLcm
/// input_ports:
/// - point_cloud
/// output_ports:
/// - lcmt_point_cloud
/// @endsystem
///
/// The input cloud must contain the kXYZs data channel.  If the kRGBs data
/// channel is present, it will also be converted.  Any other channels will
/// be ignored.
///
/// Only the finite points from the cloud are copied into the message
/// (too-close or too-far points from a depth sensor are omitted).
///
/// @note The output message's header field `seq` is always zero.
class PointCloudToLcm final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PointCloudToLcm)

  PointCloudToLcm();
  ~PointCloudToLcm() final;
};

}  // namespace perception
}  // namespace drake
