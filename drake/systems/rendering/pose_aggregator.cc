#include "drake/systems/rendering/pose_aggregator.h"

#include <string>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/systems/rendering/pose_bundle.h"

using std::to_string;

namespace drake {
namespace systems {
namespace rendering {

template <typename T>
PoseAggregator<T>::PoseAggregator() {
  this->DeclareAbstractOutputPort();
}

template <typename T>
PoseAggregator<T>::~PoseAggregator() {}

template <typename T>
const InputPortDescriptor<T>& PoseAggregator<T>::AddRigidBodyPlantInput(
    const RigidBodyTree<double> &tree) {
  input_records_.emplace_back(&tree);
  const int num_states = tree.get_num_positions() + tree.get_num_velocities();
  const InputPortDescriptor<T>& descriptor =
      this->DeclareInputPort(kVectorValued, num_states);
  return descriptor;
}

template <typename T>
const InputPortDescriptor<T>& PoseAggregator<T>::AddBundleInput(
    const std::string &bundle_name, int num_poses) {
  input_records_.emplace_back(bundle_name, num_poses);
  const InputPortDescriptor<T>& descriptor = this->DeclareAbstractInputPort();
  return descriptor;
}

template <typename T>
void PoseAggregator<T>::DoCalcOutput(const Context<T>& context,
                                     SystemOutput<T>* output) const {
  PoseBundle<T>& bundle =
      output->GetMutableData(0)->template GetMutableValue<PoseBundle<T>>();
  int pose_index = 0;

  const int num_ports = this->get_num_input_ports();
  for (int port_index = 0; port_index < num_ports; ++port_index) {
    const InputRecord& record = input_records_[port_index];
    const int num_poses = record.num_poses;
    switch (record.type) {
      case kRigidBodyTree: {
        // Extract just the positions from the state of the RigidBodyTree, and
        // use them to compute the kinematics.
        const auto value = this->EvalEigenVectorInput(context, port_index);
        const RigidBodyTree<double>& tree = *record.tree;
        const int num_q = tree.get_num_positions();
        const int num_v = tree.get_num_velocities();
        DRAKE_ASSERT(value.size() == num_q + num_v);
        const VectorX<T> q = value.head(num_q);
        KinematicsCache<T> cache = tree.doKinematics(q);

        // Obtain the world transform for every body in the tree, except for
        // the world body at index 0. Concatenate the RigidBodyTree poses into
        // the output bundle.
        for (int i = 0; i < num_poses; ++i) {
          const int body_index = i + 1;
          DRAKE_ASSERT(pose_index < bundle.get_num_poses());
          bundle.set_pose(pose_index,
                          cache.get_element(body_index).transform_to_world);
          bundle.set_name(pose_index, MakeBodyName(tree, body_index));
          pose_index++;
        }
        break;
      }
      case kBundle: {
        // Concatenate the poses of the input pose bundle into the output.
        // TODO(david-german-tri): Accept PoseBundles of variable width, with
        // variable names.
        const PoseBundle<T>* value =
            this->template EvalInputValue<PoseBundle<T>>(context, port_index);
        DRAKE_ASSERT(num_poses == value->get_num_poses());
        const std::string& bundle_name = record.bundle_name;
        for (int j = 0; j < num_poses; ++j) {
          DRAKE_ASSERT(pose_index < bundle.get_num_poses());
          bundle.set_pose(pose_index, value->get_pose(j));
          bundle.set_name(pose_index, bundle_name + "::" + value->get_name(j));
          pose_index++;
        }
        break;
      }
      default: {
        DRAKE_ABORT_MSG("Unknown PoseInputType.");
      }
    }
  }
  return;
}

template <typename T>
std::unique_ptr<AbstractValue> PoseAggregator<T>::AllocateOutputAbstract(
    const OutputPortDescriptor<T>& descriptor) const {
  return AbstractValue::Make(PoseBundle<T>(this->CountNumPoses()));
}

template <typename T>
int PoseAggregator<T>::CountNumPoses() const {
  int num_poses = 0;
  for (const InputRecord& record : input_records_) {
    num_poses += record.num_poses;
  }
  return num_poses;
}

template <typename T>
const std::string PoseAggregator<T>::MakeBodyName(
    const RigidBodyTree<double>& tree, int body_index) const {
  const RigidBody<double>& body = tree.get_body(body_index);
  const std::string model_instance = to_string(body.get_model_instance_id());
  return body.get_model_name() + "_" + model_instance + "::" + body.get_name();
}

template class PoseAggregator<double>;
template class PoseAggregator<AutoDiffXd>;

}  // namespace rendering
}  // namespace systems
}  // namespace drake
