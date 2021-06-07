#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace perception {

/// Converts PointCloud inputs to lcmt_point_cloud output messages.  The
/// message can be transmitted to other processes using LcmPublisherSystem.
///
/// @system
/// name: PointCloudToLcm
/// input_ports:
/// - point_cloud
/// output_ports:
/// - lcmt_point_cloud
/// @endsystem
///
/// Any descriptor channels of the PointCloud will currently be ignored,
/// though may be added in a future revision.
///
/// Only the finite points from the cloud are copied into the message
/// (too-close or too-far points from a depth sensor are omitted).
class PointCloudToLcm final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PointCloudToLcm)

  /// Constructs a system that outputs messages using the given `frame_name`.
  explicit PointCloudToLcm(std::string frame_name = {});
  ~PointCloudToLcm() final;

 private:
  const std::string frame_name_;
};

}  // namespace perception
}  // namespace drake
