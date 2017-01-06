#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

// TODO(david-german-tri, SeanCurtis-TRI): Evolve PoseAggregator into
// GeometrySystem after GeometryWorld becomes available.

/// PoseAggregator is a multiplexer for heterogeneous sources of poses.
/// Supported sources are:
/// - The state output of a RigidBodyPlant.
/// - An abstract PoseBundle input.
///
/// PoseAggregator is stateless.
///
/// The output is a flat PoseBundle containing all poses in the inputs.
/// The output poses are named in the form "<source>::<pose>".
/// - For poses derived from a RigidBodyPlant input, <source> is the name of
/// the RigidBodyTree model, concatenated with the instance ID, and <pose>
/// is the name of the link.
/// - For poses derived from a generic input, <source> is the bundle name
/// provided at construction time, and <pose> is the name found in the input
/// PoseBundle at output evaluation time.
///
/// In typical usage, Diagrams should contain just one PoseAggregator, and
/// every pose in the Diagram should appear as an input to it. Then,
/// Systems that need to ingest every pose in the universe, such as renderers,
/// can simply depend on the output.
///
/// This class is explicitly instantiated for the following scalar types. No
/// other scalar types are supported.
/// - double
/// - AutoDiffXd
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class PoseAggregator : public LeafSystem<T> {
 public:
  PoseAggregator();
  ~PoseAggregator() override;

  /// Adds an input for the state of a RigidBodyTree. Stores an alias of @p
  /// tree, which must live at least as long as this system.
  const InputPortDescriptor<T>& AddRigidBodyInput(
      const RigidBodyTree<double>& tree);

  /// Adds an input for a PoseBundle containing @num_poses poses, which must
  /// be expressed in the world frame.
  const InputPortDescriptor<T>& AddGenericInput(const std::string& bundle_name,
                                                int num_poses);

  /// Aggregates the input poses into the output PoseBundle, in the order
  /// the input ports were added. Aborts if any inputs have an unexpected
  /// dimension.
  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const override;

  /// Allocates a PoseBundle of length equal to the concatenation of all inputs.
  std::unique_ptr<AbstractValue> AllocateOutputAbstract(
      const OutputPortDescriptor<T>& descriptor) const override;

 private:
  enum PoseInputType {
    kGeneric = 1,
    kRigidBody = 2,
  };

  // Returns the total number of poses from all inputs.
  int GetNumPoses() const;

  // Returns the tree corresponding to the given input @p port_index, or aborts
  // if that port is not a rigid body input.
  const RigidBodyTree<double>& GetTreeOrDie(int port_index) const;

  // Returns the model names corresponding to the given input @p port_index, or
  // aborts if that port is not a generic input.
  const std::string& GetBundleNameOrDie(int port_index) const;

  // Returns a canonical name for the body in the given @p tree at the given
  // @p body_index.
  const std::string GetBodyName(const RigidBodyTree<double>& tree,
                                int body_index) const;

  // The type of each input port.
  std::vector<PoseInputType> input_types_;
  // The number of poses expected in each input port.
  std::vector<int> num_poses_;
  // A map from input port number to RigidBodyTree for all rigid-body inputs.
  std::map<int, const RigidBodyTree<double>*> trees_;
  // A map from input port number to string identifier for all generic inputs.
  std::map<int, std::string> bundle_names_;
};

}  // namespace systems
}  // namespace drake
