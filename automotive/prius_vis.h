#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/automotive/car_vis.h"
#include "drake/common/drake_copyable.h"
#include "drake/lcmt_viewer_link_data.hpp"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/rendering/pose_bundle.h"

namespace drake {
namespace automotive {

/// PriusVis displays a visualization of a 2015 Toyota Prius. It relies on
/// `drake/automotive/models/prius/prius_with_lidar.sdf` and requires that this
/// SDF file only contain one model instance that is not connected to the world
/// (i.e., the root body of the SDF model must not be named
/// RigidBodyTreeConstants::kWorldName).
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
template <typename T>
class PriusVis : public CarVis<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PriusVis)

  /// Defines the distance between the visual model's origin and the middle of
  /// the rear axle.
  static constexpr double kVisOffset{1.40948};

  PriusVis(int id, const std::string& name);

  const std::vector<lcmt_viewer_link_data>& GetVisElements() const override;

  systems::rendering::PoseBundle<T> CalcPoses(
      const Isometry3<T>& X_WM) const override;

 private:
  std::unique_ptr<RigidBodyTree<T>> tree_;
  std::vector<lcmt_viewer_link_data> vis_elements_;
};

}  // namespace automotive
}  // namespace drake
