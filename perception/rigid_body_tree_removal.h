#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/perception/point_cloud.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace perception {

/// A filtering algorithm for point clouds that removes known geometries.
/// Given a RigidBodyTree object, the algorithm takes a point cloud and a vector
/// of tree positions as input, and outputs a filtered point cloud from which
/// all points
/// expected to belong to the rigid bodies comprising the RigidBodyTree have
/// been removed.
class RigidBodyTreeRemoval final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RigidBodyTreeRemoval)

  /// Constructs the filter given a RigidBodyTree.
  ///
  /// @param[in] tree The RigidBodyTree containing the geometric configuration
  /// of the world.
  explicit RigidBodyTreeRemoval(const RigidBodyTree<double>& tree);

  /// Returns the RigidBodyTree for which known geometries are calculated.
  //  const RigidBodyTree<double>& tree() const { return tree_; }

  /// Returns a descriptor of the input port containing the generalized
  /// positions of
  /// the RigidBodyTree.
  const systems::InputPortDescriptor<double>&
  get_rigid_body_tree_positions_input_port() const {
    return this->get_input_port(input_port_index_tree_positions_);
  }

  /// Returns a descriptor of the input port containing the point cloud.
  const systems::InputPortDescriptor<double>& get_point_cloud_input_port()
      const {
    return this->get_input_port(input_port_index_point_cloud_);
  }

  const systems::OutputPort<double>& point_cloud_output_port() const {
    return LeafSystem<double>::get_output_port(0);
  }

 private:
  PointCloud MakeOutputPointCloud() const;

  void FilterPointCloud(const systems::Context<double>& context,
                        PointCloud* output) const;

  const RigidBodyTree<double>& tree_;

  int input_port_index_point_cloud_{0};
  int input_port_index_tree_positions_{1};

  double collision_threshold_;
};

}  // namespace perception
}  // namespace drake
