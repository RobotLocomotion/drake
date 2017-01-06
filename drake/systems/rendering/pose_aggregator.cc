#include "drake/systems/rendering/pose_aggregator.h"

#include <string>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/systems/rendering/pose_bundle.h"

using std::to_string;

namespace drake {
namespace systems {

template <typename T>
PoseAggregator<T>::PoseAggregator() {
  this->DeclareAbstractOutputPort();
}

template <typename T>
PoseAggregator<T>::~PoseAggregator() {}

template <typename T>
const InputPortDescriptor<T>& PoseAggregator<T>::AddRigidBodyInput(
    const RigidBodyTree<double>& tree) {
  input_types_.push_back(kRigidBody);
  num_poses_.push_back(tree.bodies.size());

  const int num_states = tree.get_num_positions() + tree.get_num_velocities();
  const InputPortDescriptor<T>& descriptor =
      this->DeclareInputPort(kVectorValued, num_states);
  trees_[descriptor.get_index()] = &tree;
  return descriptor;
}

template <typename T>
const InputPortDescriptor<T>& PoseAggregator<T>::AddGenericInput(
    const std::string& bundle_name, int num_poses) {
  input_types_.push_back(kGeneric);
  num_poses_.push_back(num_poses);

  const InputPortDescriptor<T>& descriptor = this->DeclareAbstractInputPort();
  bundle_names_[descriptor.get_index()] = bundle_name;
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
    const int num_poses = num_poses_[port_index];
    switch (input_types_[port_index]) {
      case kRigidBody: {
        // Extract just the positions from the state of the RigidBodyTree, and
        // use them to compute the kinematics.
        const auto value = this->EvalEigenVectorInput(context, port_index);
        const RigidBodyTree<double>& tree = GetTreeOrDie(port_index);
        const VectorX<T> q = value.head(tree.get_num_positions());
        KinematicsCache<T> cache = tree.doKinematics(q);

        // Obtain the world transform for every body in the tree, and
        // concatenate the RigidBodyTree poses into the output bundle.
        DRAKE_DEMAND(num_poses <= bundle.get_num_poses());
        for (int i = 0; i < num_poses; ++i) {
          *bundle.get_mutable_pose(pose_index) =
              cache.get_element(i).transform_to_world;
          bundle.set_name(pose_index, GetBodyName(tree, i));
          pose_index++;
        }
        break;
      }
      case kGeneric: {
        // Concatenate the poses of the input pose bundle into the output.
        const PoseBundle<T>* value =
            this->template EvalInputValue<PoseBundle<T>>(context, port_index);
        DRAKE_DEMAND(num_poses == value->get_num_poses());
        DRAKE_DEMAND(num_poses <= bundle.get_num_poses());
        const std::string& bundle_name = GetBundleNameOrDie(port_index);
        for (int j = 0; j < num_poses; ++j) {
          *bundle.get_mutable_pose(pose_index) = value->get_pose(j);
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
  return AbstractValue::Make(PoseBundle<T>(this->GetNumPoses()));
}

template <typename T>
int PoseAggregator<T>::GetNumPoses() const {
  int num_poses = 0;
  for (const int n : num_poses_) {
    num_poses += n;
  }
  return num_poses;
}

template <typename T>
const std::string PoseAggregator<T>::GetBodyName(
    const RigidBodyTree<double>& tree, int body_index) const {
  const RigidBody<double>& body = tree.get_body(body_index);
  const std::string model_instance = to_string(body.get_model_instance_id());
  return body.get_model_name() + model_instance + "::" + body.get_name();
}

template <typename T>
const RigidBodyTree<double>& PoseAggregator<T>::GetTreeOrDie(
    int port_index) const {
  auto it = trees_.find(port_index);
  DRAKE_DEMAND(it != trees_.end());
  return *(it->second);
}

template <typename T>
const std::string& PoseAggregator<T>::GetBundleNameOrDie(int port_index) const {
  auto it = bundle_names_.find(port_index);
  DRAKE_DEMAND(it != bundle_names_.end());
  return it->second;
}

template class PoseAggregator<double>;
template class PoseAggregator<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
