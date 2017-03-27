#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/automotive/car_vis.h"
#include "drake/common/drake_copyable.h"
#include "drake/lcmtypes/drake/lcmt_viewer_link_data.hpp"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/rendering/pose_bundle.h"

namespace drake {
namespace automotive {

/// PriusVis displays a visualization of a 2015 Toyota Prius.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_systems
///
template <typename T>
class PriusVis : public CarVis<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PriusVis)

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
