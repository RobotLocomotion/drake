#include "drake/systems/rendering/pose_aggregator.h"

#include <string>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/systems/rendering/pose_vector.h"

using std::to_string;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

namespace drake {
namespace systems {
namespace rendering {

template <typename T>
PoseAggregator<T>::PoseAggregator()
    : LeafSystem<T>(SystemTypeTag<PoseAggregator>{}) {
  // Declare the output port and provide an allocator for a PoseBundle of length
  // equal to the concatenation of all inputs. This can't be done with a model
  // value because we don't know at construction how big the output will be.
  this->DeclareAbstractOutputPort(kUseDefaultName,
                                  &PoseAggregator::CalcPoseBundle);
}

template <typename T>
template <typename U>
PoseAggregator<T>::PoseAggregator(const PoseAggregator<U>& other)
    : PoseAggregator() {
  for (const auto& record : other.input_records_) {
    this->DeclareInput(record);
  }
}

template <typename T>
PoseAggregator<T>::~PoseAggregator() {}

template <typename T>
const InputPort<T>& PoseAggregator<T>::AddSingleInput(
    const std::string& name, int model_instance_id) {
  return DeclareInput(MakeSinglePoseInputRecord(name, model_instance_id));
}

template <typename T>
PoseVelocityInputPorts<T>
PoseAggregator<T>::AddSinglePoseAndVelocityInput(const std::string& name,
                                                 int model_instance_id) {
  // Add an input for the pose.
  const auto& pose_input_port =
      DeclareInput(MakeSinglePoseInputRecord(name, model_instance_id));
  // Add an input for the velocity.
  const auto& velocity_input_port =
      DeclareInput(MakeSingleVelocityInputRecord(name, model_instance_id));
  return {pose_input_port, velocity_input_port};
}

template <typename T>
const InputPort<T>& PoseAggregator<T>::AddBundleInput(
    const std::string& bundle_name, int num_poses) {
  return DeclareInput(MakePoseBundleInputRecord(bundle_name, num_poses));
}

template <typename T>
void PoseAggregator<T>::CalcPoseBundle(const Context<T>& context,
                                       PoseBundle<T>* output) const {
  PoseBundle<T>& bundle = *output;

  const int total_num_poses = this->CountNumPoses();
  if (bundle.get_num_poses() != total_num_poses) {
    bundle = PoseBundle<T>(total_num_poses);
  }

  int pose_index = 0;
  const int num_ports = this->num_input_ports();
  for (int port_index = 0; port_index < num_ports; ++port_index) {
    const auto& port = this->get_input_port(port_index);
    const InputRecord& record = input_records_[port_index];
    const int num_poses = record.num_poses;
    switch (record.type) {
      case InputRecord::kSinglePose: {
        const auto& value = port.template Eval<PoseVector<T>>(context);
        DRAKE_ASSERT(pose_index < bundle.get_num_poses());
        bundle.set_name(pose_index, record.name);
        bundle.set_transform(pose_index, value.get_transform());
        bundle.set_model_instance_id(pose_index, record.model_instance_id);
        pose_index++;
        continue;
      }
      case InputRecord::kSingleVelocity: {
        // Single velocities are associated with the single pose that must
        // immediately precede.
        DRAKE_ASSERT(port_index > 0);
        DRAKE_ASSERT(input_records_[port_index - 1].type ==
                     InputRecord::kSinglePose);

        const auto& value = port.template Eval<FrameVelocity<T>>(context);
        const int prev_pose_index = pose_index - 1;
        DRAKE_ASSERT(bundle.get_name(prev_pose_index) == record.name);
        const int last_model_instance_id =
            bundle.get_model_instance_id(prev_pose_index);
        DRAKE_ASSERT(last_model_instance_id == record.model_instance_id);

        // Write the velocity to the previous pose_index, and do not increment
        // the pose_index, because this input was not a pose.
        bundle.set_velocity(prev_pose_index, value);
        continue;
      }
      case InputRecord::kBundle: {
        // Concatenate the poses of the input pose bundle into the output.
        // TODO(david-german-tri): Accept PoseBundles of variable width, with
        // variable names.
        const auto& value = port.template Eval<PoseBundle<T>>(context);
        DRAKE_ASSERT(num_poses == value.get_num_poses());
        const std::string& bundle_name = record.name;
        for (int j = 0; j < num_poses; ++j) {
          DRAKE_ASSERT(pose_index < bundle.get_num_poses());
          bundle.set_transform(pose_index, value.get_transform(j));
          bundle.set_velocity(pose_index, value.get_velocity(j));
          bundle.set_name(pose_index, bundle_name + "::" + value.get_name(j));
          bundle.set_model_instance_id(pose_index,
                                       value.get_model_instance_id(j));
          pose_index++;
        }
        continue;
      }
    }
    DRAKE_UNREACHABLE();
  }
  DRAKE_DEMAND(pose_index == total_num_poses);
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
typename PoseAggregator<T>::InputRecord
PoseAggregator<T>::MakeSinglePoseInputRecord(const std::string& name,
                                             int model_instance_id) {
  InputRecord rec;
  rec.type = InputRecord::kSinglePose;
  rec.num_poses = 1;
  rec.name = name;
  rec.model_instance_id = model_instance_id;
  return rec;
}

template <typename T>
typename PoseAggregator<T>::InputRecord
PoseAggregator<T>::MakeSingleVelocityInputRecord(const std::string& name,
                                                 int model_instance_id) {
  InputRecord rec;
  rec.type = InputRecord::kSingleVelocity;
  rec.num_poses = 0;  // A velocity is not a pose.
  rec.name = name;
  rec.model_instance_id = model_instance_id;
  return rec;
}

template <typename T>
typename PoseAggregator<T>::InputRecord
PoseAggregator<T>::MakePoseBundleInputRecord(const std::string& bundle_name,
                                             int num_poses) {
  InputRecord rec;
  rec.type = InputRecord::kBundle;
  rec.num_poses = num_poses;
  rec.name = bundle_name;
  return rec;
}

template <typename T>
const InputPort<T>&
PoseAggregator<T>::DeclareInput(const InputRecord& record) {
  input_records_.push_back(record);
  switch (record.type) {
    case InputRecord::kSinglePose:
      return this->DeclareVectorInputPort(kUseDefaultName, PoseVector<T>());
    case InputRecord::kSingleVelocity:
      return this->DeclareVectorInputPort(kUseDefaultName, FrameVelocity<T>());
    case InputRecord::kBundle:
      return this->DeclareAbstractInputPort(
          kUseDefaultName, Value<PoseBundle<T>>());
  }
  DRAKE_UNREACHABLE();
}

}  // namespace rendering
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::rendering::PoseAggregator)

#pragma GCC diagnostic pop
