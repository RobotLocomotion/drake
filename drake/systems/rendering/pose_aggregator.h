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
namespace rendering {

// TODO(david-german-tri, SeanCurtis-TRI): Evolve PoseAggregator into
// GeometrySystem after GeometryWorld becomes available.

/// PoseAggregator is a multiplexer for heterogeneous sources of poses.
/// Supported sources are:
/// - The state output of a RigidBodyPlant.
/// - An abstract PoseBundle input.
///
/// PoseAggregator is stateless.
///
/// The output is a flat PoseBundle that contains all the poses from all the
/// inputs, except for the "world" bodies in a RigidBodyTree. By convention,
/// each aggregated pose is in the same world frame of reference.
///
/// The output poses are named in the form "<source>::<pose>".
/// - For poses derived from a RigidBodyPlant input, <source> is the name of
/// the RigidBodyTree model, concatenated with the instance ID, and <pose>
/// is the name of the body.  For instance, if the model is named "model", the
/// pose of the link named "foo" in the first insance of that model is
/// "model_0::foo".
///
/// - For poses derived from a generic input, <source> is the bundle name
/// provided at construction time, and <pose> is the name found in the input
/// PoseBundle at output evaluation time. In any sane use case, the input
/// names will be stable during a simulation, but PoseAggregator is stateless
/// and therefore can't check whether this is actually true.
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
///           Only double and AutoDiffXd are supported.
template <typename T>
class PoseAggregator : public LeafSystem<T> {
 public:
  PoseAggregator();
  ~PoseAggregator() override;

  /// Adds an input for the state of a RigidBodyPlant. Stores an alias of @p
  /// tree, which must live at least as long as this system.
  const InputPortDescriptor<T>& AddRigidBodyPlantInput(
      const RigidBodyTree<double> &tree);

  /// Adds an input for a PoseBundle containing @p num_poses poses.
  const InputPortDescriptor<T>& AddBundleInput(const std::string& bundle_name,
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
    kUnknown = 0,
    kBundle = 1,
    kRigidBodyTree = 2,
  };

  struct InputRecord {
    // Constructs an InputRecord for a RigidBodyTree input. The number of poses
    // excludes the world "body".
    explicit InputRecord(const RigidBodyTree<double>* tree_in)
        : type(kRigidBodyTree),
          num_poses(tree_in->get_num_bodies() - 1),
          tree(tree_in) {}

    // Constructs an InputRecord for a generic PoseBundle input.
    InputRecord(const std::string& bundle_name_in, const int num_poses_in)
        : type(kBundle),
          num_poses(num_poses_in),
          bundle_name(bundle_name_in) {}

    PoseInputType type{kUnknown};
    int num_poses{0};
    // bundle_name is only valid if type is kBundle.
    std::string bundle_name{};
    // tree is only valid if type is kRigidBodyTree.
    const RigidBodyTree<double>* tree{};
  };

  // Returns the total number of poses from all inputs.
  int CountNumPoses() const;

  // Returns a name for the body in the given @p tree at the given
  // @p body_index, which contains the model name, the model instance ID, and
  // the body name. See class comment for details.
  const std::string MakeBodyName(const RigidBodyTree<double>& tree,
                                int body_index) const;

  // The type, size, and source of each input port.
  std::vector<InputRecord> input_records_;
};

}  // namespace rendering
}  // namespace systems
}  // namespace drake
