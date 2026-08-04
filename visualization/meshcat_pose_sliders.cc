#include "drake/visualization/meshcat_pose_sliders.h"

#include <algorithm>
#include <chrono>
#include <thread>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/common/scope_exit.h"
#include "drake/common/text_logging.h"
#include "drake/common/unused.h"
#include "drake/geometry/meshcat_graphviz.h"

namespace drake {
namespace visualization {

using math::RigidTransformd;
using math::RollPitchYawd;
using systems::Context;
using systems::System;

namespace {

std::string AddPrefix(std::string name, std::string prefix) {
  if (prefix.empty()) {
    return name;
  } else {
    return prefix + "_" + name;
  }
}

// Finds the value x + k*2π for some k which is in [lower_bound, upper_bound],
// if possible; otherwise it returns the closest bound.
double AdjustAngle(double x, double lower_bound, double upper_bound) {
  constexpr double TWO_PI = 2.0 * M_PI;
  // Adjust x by adding/subtracting multiples of 2π until it falls within the
  // range [lower_bound, upper_bound).
  while (x < lower_bound) {
    x += TWO_PI;
  }

  while (x >= upper_bound) {
    x -= TWO_PI;
  }

  // If after adjusting, x is outside of bounds, return the closest bound.
  if (x < lower_bound) {
    return lower_bound;
  } else if (x > upper_bound) {
    return upper_bound;
  }

  // Otherwise, return the adjusted angle.
  return x;
}

// Extracts roll, pitch, and yaw from `pose`, after unwrapping and clipping to
// the lower and upper limits.
Vector6d ExtractRpyXyz(const RigidTransformd& pose,
                       const Eigen::Ref<const Vector6d>& lower_limit,
                       const Eigen::Ref<const Vector6d>& upper_limit) {
  using std::clamp;
  using std::fmod;
  using std::min;
  RollPitchYawd rpy = pose.rotation().ToRollPitchYaw();
  Vector6d rpy_xyz;
  rpy_xyz << AdjustAngle(rpy.roll_angle(), lower_limit[0], upper_limit[0]),
      AdjustAngle(rpy.pitch_angle(), lower_limit[1], upper_limit[1]),
      AdjustAngle(rpy.yaw_angle(), lower_limit[2], upper_limit[2]),
      clamp(pose.translation()[0], lower_limit[3], upper_limit[3]),
      clamp(pose.translation()[1], lower_limit[4], upper_limit[4]),
      clamp(pose.translation()[2], lower_limit[5], upper_limit[5]);
  return rpy_xyz;
}

}  // namespace

template <typename T>
MeshcatPoseSliders<T>::MeshcatPoseSliders(
    std::shared_ptr<geometry::Meshcat> meshcat,
    const RigidTransformd& initial_pose,
    const Eigen::Ref<const Vector6d>& lower_limit,
    const Eigen::Ref<const Vector6d>& upper_limit,
    const Eigen::Ref<const Vector6d>& step,
    std::vector<std::string> decrement_keycodes,
    std::vector<std::string> increment_keycodes, std::string prefix,
    const Eigen::Ref<const Vector6<bool>>& visible)
    : meshcat_(std::move(meshcat)),
      nominal_pose_(initial_pose),
      is_registered_{true},
      lower_limit_(lower_limit),
      upper_limit_(upper_limit),
      visible_(visible) {
  DRAKE_THROW_UNLESS(meshcat_ != nullptr);
  DRAKE_THROW_UNLESS((lower_limit.array() <= upper_limit.array()).all());
  DRAKE_THROW_UNLESS((step.array().all() > 0.0));

  if (decrement_keycodes.size() != 6) {
    throw std::logic_error(
        fmt::format("Expected decrement_keycodes of size 6, but got "
                    "size {} instead",
                    decrement_keycodes.size()));
  }

  if (increment_keycodes.size() != 6) {
    throw std::logic_error(
        fmt::format("Expected increment_keycodes of size 6, but got "
                    "size {} instead",
                    increment_keycodes.size()));
  }

  Vector6d rpy_xyz = ExtractRpyXyz(initial_pose, lower_limit, upper_limit);
  slider_names_ = {AddPrefix("roll", prefix), AddPrefix("pitch", prefix),
                   AddPrefix("yaw", prefix),  AddPrefix("x", prefix),
                   AddPrefix("y", prefix),    AddPrefix("z", prefix)};

  // Add sliders.
  for (int i = 0; i < 6; ++i) {
    if (visible[i]) {
      meshcat_->AddSlider(slider_names_[i], lower_limit[i], upper_limit[i],
                          step[i], rpy_xyz[i], decrement_keycodes[i],
                          increment_keycodes[i]);
    }
  }

  // Declare (optional) input_port.
  this->DeclareAbstractInputPort("pose", Value<RigidTransformd>());

  // Declare the output port.
  auto& output = this->DeclareAbstractOutputPort(
      "pose", &MeshcatPoseSliders<T>::CalcOutput);

  // The port is always marked out-of-date, so that the slider values will be
  // fetched every time the output port is needed in a computation.
  output.disable_caching_by_default();

  // We use an initialization event to read the pose input port (if it's
  // connected).
  this->DeclareInitializationPublishEvent(
      &MeshcatPoseSliders<T>::OnInitialization);
}

template <typename T>
void MeshcatPoseSliders<T>::Delete() {
  const auto was_registered = is_registered_.exchange(false);
  if (was_registered) {
    for (int i = 0; i < 6; ++i) {
      if (visible_[i]) {
        meshcat_->DeleteSlider(slider_names_[i], /*strict = */ false);
      }
    }
  }
}

template <typename T>
MeshcatPoseSliders<T>::~MeshcatPoseSliders() {
  // Destructors are not allowed to throw. Ensure this by catching any
  // exceptions and failing fast.
  try {
    Delete();
  } catch (...) {
    DRAKE_UNREACHABLE();
  }
}

template <typename T>
void MeshcatPoseSliders<T>::CalcOutput(const Context<T>&,
                                       RigidTransformd* pose) const {
  if (is_registered_) {
    Vector6d rpy_xyz = ExtractRpyXyz(nominal_pose_, lower_limit_, upper_limit_);
    for (int i = 0; i < 6; ++i) {
      // TODO(jwnimmer-tri) If CalcOutput is in flight concurrently with a
      // call to Delete, we might race and ask for a deleted slider value.
      if (visible_[i]) {
        rpy_xyz[i] = meshcat_->GetSliderValue(slider_names_[i]);
      }
    }
    pose->set_rotation(RollPitchYawd(rpy_xyz.head<3>()));
    pose->set_translation(rpy_xyz.tail<3>());
  } else {
    *pose = nominal_pose_;
  }
}

template <typename T>
systems::EventStatus MeshcatPoseSliders<T>::OnInitialization(
    const Context<T>& context) const {
  if (this->get_input_port().HasValue(context)) {
    SetPose(this->get_input_port().template Eval<RigidTransformd>(context));
    return systems::EventStatus::Succeeded();
  }
  return systems::EventStatus::DidNothing();
}

template <typename T>
typename systems::LeafSystem<T>::GraphvizFragment
MeshcatPoseSliders<T>::DoGetGraphvizFragment(
    const typename systems::LeafSystem<T>::GraphvizFragmentParams& params)
    const {
  geometry::internal::MeshcatGraphviz meshcat_graphviz(
      /* path = */ std::nullopt,
      /* subscribe = */ true);
  return meshcat_graphviz.DecorateResult(
      systems::LeafSystem<T>::DoGetGraphvizFragment(
          meshcat_graphviz.DecorateParams(params)));
}

template <typename T>
RigidTransformd MeshcatPoseSliders<T>::Run(
    const System<T>& system, const Context<T>& context,
    std::optional<double> timeout, std::string stop_button_keycode) const {
  // Make a context and create reference shortcuts to some pieces of it.
  std::unique_ptr<Context<T>> root_context = context.Clone();
  const auto& system_context = *root_context;
  const auto& pose_context = this->GetMyContextFromRoot(*root_context);

  // Add the Stop button.
  constexpr char kButtonName[] = "Stop MeshcatPoseSliders";
  log()->info("Press the '{}' button in Meshcat{} to continue.", kButtonName,
              stop_button_keycode.empty()
                  ? ""
                  : fmt::format(" or press '{}'", stop_button_keycode));
  meshcat_->AddButton(kButtonName, std::move(stop_button_keycode));
  ScopeExit guard([this, kButtonName]() {
    meshcat_->DeleteButton(kButtonName);
  });

  // Grab the current time, to implement the timeout.
  using Clock = std::chrono::steady_clock;
  using Duration = std::chrono::duration<double>;
  const auto start_time = Clock::now();

  system.ExecuteInitializationEvents(root_context.get());

  RigidTransformd pose = nominal_pose_;

  // Loop until the button is clicked, or the timeout (when given) is reached.
  system.ForcedPublish(system_context);
  while (meshcat_->GetButtonClicks(kButtonName) < 1) {
    if (timeout.has_value()) {
      const auto elapsed = Duration(Clock::now() - start_time).count();
      if (elapsed >= timeout.value()) {
        break;
      }
    }

    // If the sliders have not changed, avoid invalidating the context.
    const RigidTransformd& new_pose =
        this->get_output_port().template Eval<RigidTransformd>(pose_context);
    if (new_pose.IsExactlyEqualTo(pose)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    pose = new_pose;
    system.ForcedPublish(system_context);
  }

  return pose;
}

template <typename T>
void MeshcatPoseSliders<T>::SetPose(const RigidTransformd& pose) const {
  nominal_pose_ = pose;
  if (is_registered_) {
    // For items with an associated slider, update the meshcat UI.
    // TODO(jwnimmer-tri) If SetPose is in flight concurrently with a
    // call to Delete, we might race and ask for a deleted slider value.
    Vector6d rpy_xyz = ExtractRpyXyz(nominal_pose_, lower_limit_, upper_limit_);
    for (int i = 0; i < 6; ++i) {
      // TODO(jwnimmer-tri) If CalcOutput is in flight concurrently with a
      // call to Delete, we might race and ask for a deleted slider value.
      if (visible_[i]) {
        meshcat_->SetSliderValue(slider_names_[i], rpy_xyz[i]);
      }
    }
  }
}

}  // namespace visualization
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::visualization::MeshcatPoseSliders);
