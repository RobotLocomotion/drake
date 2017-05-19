#include "drake/systems/rendering/pose_aggregator.h"

#include <string>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/systems/rendering/pose_bundle.h"
#include "drake/systems/rendering/pose_vector.h"

using std::to_string;

namespace drake {
namespace systems {
namespace rendering {

template <typename T>
PoseAggregator<T>::PoseAggregator() {
  // Declare the output port and provide an allocator for a PoseBundle of length
  // equal to the concatenation of all inputs. This can't be done with a model
  // value because we don't know at construction how big the output will be.
  this->DeclareAbstractOutputPort(&PoseAggregator::MakePoseBundle,
                                  &PoseAggregator::CalcPoseBundle);
}

template <typename T>
PoseAggregator<T>::~PoseAggregator() {}

template <typename T>
const InputPortDescriptor<T>& PoseAggregator<T>::AddSingleInput(
    const std::string& name, int model_instance_id) {
  return DeclareInput(MakeSinglePoseInputRecord(name, model_instance_id));
}

template <typename T>
std::pair<const InputPortDescriptor<T>&, const InputPortDescriptor<T>&>
PoseAggregator<T>::AddSinglePoseAndVelocityInput(const std::string& name,
                                                 int model_instance_id) {
  // Add an input for the pose.
  const auto& pose_descriptor =
      DeclareInput(MakeSinglePoseInputRecord(name, model_instance_id));
  // Add an input for the velocity.
  const auto& velocity_descriptor =
      DeclareInput(MakeSingleVelocityInputRecord(name, model_instance_id));
  return std::pair<const InputPortDescriptor<T>&,
                   const InputPortDescriptor<T>&>(pose_descriptor,
                                                  velocity_descriptor);
}

template <typename T>
const InputPortDescriptor<T>& PoseAggregator<T>::AddBundleInput(
    const std::string& bundle_name, int num_poses) {
  return DeclareInput(MakePoseBundleInputRecord(bundle_name, num_poses));
}

template <typename T>
void PoseAggregator<T>::CalcPoseBundle(const Context<T>& context,
                                       PoseBundle<T>* output) const {
  PoseBundle<T>& bundle = *output;
  int pose_index = 0;

  const int num_ports = this->get_num_input_ports();
  for (int port_index = 0; port_index < num_ports; ++port_index) {
    const InputRecord& record = input_records_[port_index];
    const int num_poses = record.num_poses;
    switch (record.type) {
      case InputRecord::kSinglePose: {
        const PoseVector<T>* value =
            this->template EvalVectorInput<PoseVector>(context, port_index);
        DRAKE_ASSERT(value != nullptr);
        DRAKE_ASSERT(pose_index < bundle.get_num_poses());
        bundle.set_name(pose_index, record.name);
        bundle.set_pose(pose_index, value->get_isometry());
        bundle.set_model_instance_id(pose_index, record.model_instance_id);
        pose_index++;
        break;
      }
      case InputRecord::kSingleVelocity: {
        // Single velocities are associated with the single pose that must
        // immediately precede.
        DRAKE_ASSERT(port_index > 0);
        DRAKE_ASSERT(input_records_[port_index - 1].type ==
                     InputRecord::kSinglePose);

        const FrameVelocity<T>* value =
            this->template EvalVectorInput<FrameVelocity>(context, port_index);
        DRAKE_ASSERT(value != nullptr);
        const int prev_pose_index = pose_index - 1;
        DRAKE_ASSERT(bundle.get_name(prev_pose_index) == record.name);
        const int last_model_instance_id =
            bundle.get_model_instance_id(prev_pose_index);
        DRAKE_ASSERT(last_model_instance_id == record.model_instance_id);

        // Write the velocity to the previous pose_index, and do not increment
        // the pose_index, because this input was not a pose.
        bundle.set_velocity(prev_pose_index, *value);
        break;
      }
      case InputRecord::kBundle: {
        // Concatenate the poses of the input pose bundle into the output.
        // TODO(david-german-tri): Accept PoseBundles of variable width, with
        // variable names.
        const PoseBundle<T>* value =
            this->template EvalInputValue<PoseBundle<T>>(context, port_index);
        DRAKE_ASSERT(num_poses == value->get_num_poses());
        const std::string& bundle_name = record.name;
        for (int j = 0; j < num_poses; ++j) {
          DRAKE_ASSERT(pose_index < bundle.get_num_poses());
          bundle.set_pose(pose_index, value->get_pose(j));
          bundle.set_velocity(pose_index, value->get_velocity(j));
          bundle.set_name(pose_index, bundle_name + "::" + value->get_name(j));
          bundle.set_model_instance_id(pose_index,
                                       value->get_model_instance_id(j));
          pose_index++;
        }
        break;
      }
      default: { DRAKE_ABORT_MSG("Unknown PoseInputType."); }
    }
  }
}

template <typename T>
PoseBundle<T> PoseAggregator<T>::MakePoseBundle() const {
  return PoseBundle<T>(this->CountNumPoses());
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
PoseAggregator<AutoDiffXd>* PoseAggregator<T>::DoToAutoDiffXd() const {
  auto result = new PoseAggregator<AutoDiffXd>;
  for (const auto& record : input_records_) {
    result->DeclareInput(record);
  }
  return result;
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
const InputPortDescriptor<T>&
PoseAggregator<T>::DeclareInput(const InputRecord& record) {
  input_records_.push_back(record);
  switch (record.type) {
    case InputRecord::kSinglePose:
      return this->DeclareVectorInputPort(PoseVector<T>());
    case InputRecord::kSingleVelocity:
      return this->DeclareVectorInputPort(FrameVelocity<T>());
    case InputRecord::kBundle:
      return this->DeclareAbstractInputPort();
    case InputRecord::kUnknown:
      break;
  }
  DRAKE_ABORT_MSG("Invariant failure");
}

template class PoseAggregator<double>;
template class PoseAggregator<AutoDiffXd>;

}  // namespace rendering
}  // namespace systems
}  // namespace drake
