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
/// - A PoseVector input, which is a single pose {R, p}, and vector-valued.
/// - A PoseBundle input, which is a collection of poses, and abstract-valued.
///
/// PoseAggregator is stateless.
///
/// The output is a flat PoseBundle that contains all the poses from all the
/// inputs, except for the "world" bodies in a RigidBodyTree. By convention,
/// each aggregated pose is in the same world frame of reference.
///
/// The output poses are named in the form `<source>` or `<source>::<pose>`.
/// - For poses derived from a RigidBodyPlant input, `<source>` is the name of
///   the RigidBodyTree model, and `<pose>` is the name of the body.
/// - For poses derived from a PoseVector input, <source> is the bundle name
///   provided at construction time, and "::<pose>" is omitted.
/// - For poses derived from a PoseBundle input, <source> is the bundle name
///   provided at construction time, and <pose> is the name found in the input
///   PoseBundle at output evaluation time. In any sane use case, the input
///   names will be stable during a simulation, but PoseAggregator is stateless
///   and therefore can't check whether this is actually true.
///
/// The output poses are also each assigned a model instance ID, which must be
/// an integer that is greater than or equal to zero. All poses with the same
/// model instance ID must have unique names. This enables PoseAggregator to
/// aggregate multiple instances of the same model.
/// - For poses derived from a RigidBodyPlant input, the instance ID is obtained
///   from the RigidBodyTree.
/// - For poses derived from a PoseVector input, the instance ID is specified
///   when the input is declared.
/// - For poses derived from a PoseBundle input, the instance ID is obtained
///   directly from the PoseBundle.
///
/// In typical usage, Diagrams should contain just one PoseAggregator, and
/// every pose in the Diagram should appear as an input to it. Then,
/// Systems that need to ingest every pose in the universe, such as renderers
/// or sensor models, can simply depend on the output.
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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PoseAggregator)

  PoseAggregator();
  ~PoseAggregator() override;

  /// Adds an input for the state of a RigidBodyPlant. Stores an alias of @p
  /// tree, which must live at least as long as this system.
  const InputPortDescriptor<T>& AddRigidBodyPlantInput(
      const RigidBodyTree<double> &tree);

  /// Adds an input for a PoseVector. @p name must be unique for all inputs with
  /// the same @p model_instance_id.
  const InputPortDescriptor<T>& AddSingleInput(const std::string& name,
                                               int model_instance_id);

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
    kSingle = 1,
    kBundle = 2,
    kRigidBodyTree = 3,
  };

  struct InputRecord {
    PoseInputType type{kUnknown};
    int num_poses{0};
    // name is only valid if type is kSingle or kBundle.
    std::string name{};
    // model_instance_id is only valid if type is kSingle.
    int model_instance_id{-1};
    // tree is only valid if type is kRigidBodyTree.
    const RigidBodyTree<double>* tree{};
  };

  // Returns an InputRecord for a RigidBodyTree input. The number of poses
  // excludes the world "body".
  static InputRecord MakeRigidBodyTreeInputRecord(
      const RigidBodyTree<double>* tree);

  // Returns an InputRecord for a generic single pose input.
  static InputRecord MakeSinglePoseInputRecord(const std::string& name,
                                               int model_instance_id);

  // Returns an InputRecord for a generic PoseBundle input.
  static InputRecord MakePoseBundleInputRecord(const std::string& bundle_name,
                                               int num_poses);

  // Returns the total number of poses from all inputs.
  int CountNumPoses() const;

  // Returns a name for the given @p body. See class comment for details.
  const std::string MakeBodyName(const RigidBody<double>& body) const;

  // The type, size, and source of each input port.
  std::vector<InputRecord> input_records_;
};

}  // namespace rendering
}  // namespace systems
}  // namespace drake
