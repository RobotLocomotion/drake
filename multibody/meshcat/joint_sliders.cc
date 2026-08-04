#include "drake/multibody/meshcat/joint_sliders.h"

#include <algorithm>
#include <chrono>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/common/overloaded.h"
#include "drake/common/scope_exit.h"
#include "drake/common/text_logging.h"
#include "drake/common/unused.h"
#include "drake/geometry/meshcat_graphviz.h"

namespace drake {
namespace multibody {
namespace meshcat {

using Eigen::VectorXd;
using internal::SliderDetail;
using systems::BasicVector;
using systems::Context;
using systems::Diagram;

namespace {

// Returns a vector of size num_positions, based on the given value variant.
// If a VectorXd is given, it's checked for size and then returned unchanged.
// If a double is given, it's broadcast to size and returned.
// If no value is given, then the default_value is broadcast instead.
VectorXd Broadcast(
    const char* diagnostic_name, double default_value, int num_positions,
    const std::variant<std::monostate, double, VectorXd>& value) {
  return std::visit<VectorXd>(
      overloaded{
          [num_positions, default_value](std::monostate) {
            return VectorXd::Constant(num_positions, default_value);
          },
          [num_positions](double arg) {
            return VectorXd::Constant(num_positions, arg);
          },
          [num_positions, diagnostic_name](const VectorXd& arg) {
            if (arg.size() != num_positions) {
              throw std::logic_error(
                  fmt::format("Expected {} of size {}, but got size {} instead",
                              diagnostic_name, num_positions, arg.size()));
            }
            return arg;
          },
      },
      value);
}

// Returns the plant's default positions.
template <typename T>
VectorXd GetDefaultPositions(const MultibodyPlant<T>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  const int nq = plant->num_positions();
  VectorXd result(nq);
  auto context = plant->CreateDefaultContext();
  auto positions = plant->GetPositions(*context);
  for (int i = 0; i < nq; ++i) {
    result[i] = ExtractDoubleOrThrow(positions[i]);
  }
  return result;
}

// Returns true iff slider_details has any duplicated names.
bool HasAnyDuplicatedNames(const std::vector<SliderDetail>& slider_details) {
  std::unordered_set<std::string_view> values_seen;
  for (const auto& slider_detail : slider_details) {
    const bool inserted = values_seen.insert(slider_detail.name).second;
    if (!inserted) {
      return true;
    }
  }
  return false;
}

// Returns vector of slider details indexed the same way the plant's positions.
// (The result.size() == nq and the slider for the i'th position is result[i].)
// When use_model_instance_name is set, both the joint name and model name will
// be used to to form the slider name.
template <typename T>
std::vector<SliderDetail> MakeSliderDetails(
    const MultibodyPlant<T>* plant,
    const std::optional<VectorXd>& initial_value,
    const std::variant<std::monostate, double, Eigen::VectorXd>& lower_limit,
    const std::variant<std::monostate, double, Eigen::VectorXd>& upper_limit,
    const std::variant<std::monostate, double, Eigen::VectorXd>& step,
    bool use_model_instance_name) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  const int nq = plant->num_positions();

  // Default any missing arguments; check (or widen) them to be of size == nq.
  if (initial_value.has_value() && initial_value->size() != nq) {
    throw std::logic_error(fmt::format(
        "Expected initial_value of size {}, but got size {} instead", nq,
        initial_value->size()));
  }
  const VectorXd nominal_positions =
      initial_value.has_value() ? *initial_value : GetDefaultPositions(plant);
  const VectorXd lower_broadcast =
      Broadcast("lower_limit", -10.0, nq, lower_limit);
  const VectorXd upper_broadcast =
      Broadcast("upper_limit", 10.0, nq, upper_limit);
  const VectorXd step_broadcast = Broadcast("step", 0.01, nq, step);

  // Map all joints' positions into the result.
  std::vector<SliderDetail> result(nq);
  VectorX<bool> seen = VectorX<bool>::Constant(nq, false);
  for (JointIndex joint_index : plant->GetJointIndices()) {
    const Joint<T>& joint = plant->get_joint(joint_index);
    for (int j = 0; j < joint.num_positions(); ++j) {
      const int position_index = joint.position_start() + j;
      std::string description;
      if (joint.num_positions() > 1) {
        description =
            fmt::format("{}_{}", joint.name(), joint.position_suffix(j));
      } else {
        description = joint.name();
      }
      if (use_model_instance_name) {
        description += fmt::format(
            "/{}", plant->GetModelInstanceName(joint.model_instance()));
      }
      result[position_index].name = std::move(description);
      seen[position_index] = true;
    }
  }

  // Cross-check the plant invariant that all positions are covered by joints.
  DRAKE_DEMAND(seen == VectorX<bool>::Constant(nq, true));

  // Populate the limits for each slider.
  const VectorXd lower_plant = plant->GetPositionLowerLimits();
  const VectorXd upper_plant = plant->GetPositionUpperLimits();
  for (int i = 0; i < nq; ++i) {
    SliderDetail& detail = result[i];
    detail.min = std::max(lower_broadcast[i], lower_plant[i]);
    detail.max = std::min(upper_broadcast[i], upper_plant[i]);
    detail.step = step_broadcast[i];
    detail.nominal_value = nominal_positions[i];
  }

  // TODO(RussTedrake) For positions associated with quaternions, clamping the
  // min/max to [-1,1] would be dramatically more sensible. Ideally that should
  // be written down upstream as joint limits to MbP itself, but currently MbP
  // has asserts that say joint limits are not allowed except for single dof
  // mobilizers. If we can't fix it in the MbP, then we should add it here.

  return result;
}

// Same contract as MakeSliderDetails() above, but tries to avoid using the
// model instance name unless required.
template <typename T>
std::vector<SliderDetail> MakeBestSliderDetails(
    const MultibodyPlant<T>* plant,
    const std::optional<VectorXd>& initial_value,
    const std::variant<std::monostate, double, Eigen::VectorXd>& lower_limit,
    const std::variant<std::monostate, double, Eigen::VectorXd>& upper_limit,
    const std::variant<std::monostate, double, Eigen::VectorXd>& step) {
  bool use_model_instance_name = false;
  std::vector<SliderDetail> result =
      MakeSliderDetails(plant, initial_value, lower_limit, upper_limit, step,
                        use_model_instance_name);
  if (!HasAnyDuplicatedNames(result)) {
    return result;
  }
  // We must blend in the the model_instance_name to obtain unique names.
  use_model_instance_name = true;
  result = MakeSliderDetails(plant, initial_value, lower_limit, upper_limit,
                             step, use_model_instance_name);
  DRAKE_DEMAND(!HasAnyDuplicatedNames(result));
  return result;
}

}  // namespace

template <typename T>
JointSliders<T>::JointSliders(
    std::shared_ptr<geometry::Meshcat> meshcat, const MultibodyPlant<T>* plant,
    const std::optional<VectorXd>& initial_value,
    const std::variant<std::monostate, double, VectorXd>& lower_limit,
    const std::variant<std::monostate, double, VectorXd>& upper_limit,
    const std::variant<std::monostate, double, VectorXd>& step,
    std::vector<std::string> decrement_keycodes,
    std::vector<std::string> increment_keycodes)
    : meshcat_(std::move(meshcat)),
      plant_(plant),
      slider_details_(MakeBestSliderDetails(plant, initial_value, lower_limit,
                                            upper_limit, step)),
      is_registered_{true} {
  DRAKE_THROW_UNLESS(meshcat_ != nullptr);
  DRAKE_THROW_UNLESS(plant_ != nullptr);
  const int nq = plant->num_positions();
  if (!decrement_keycodes.empty() && ssize(decrement_keycodes) != nq) {
    throw std::logic_error(
        fmt::format("Expected decrement_keycodes of size zero or {}, but got "
                    "size {} instead",
                    nq, decrement_keycodes.size()));
  }
  if (!increment_keycodes.empty() && ssize(increment_keycodes) != nq) {
    throw std::logic_error(
        fmt::format("Expected increment_keycodes of size zero or {}, but got "
                    "size {} instead",
                    nq, increment_keycodes.size()));
  }

  // Add one slider for each plant position.
  for (int i = 0; i < nq; ++i) {
    const auto& slider_detail = slider_details_[i];
    std::string decrement_keycode = !decrement_keycodes.empty()
                                        ? std::move(decrement_keycodes[i])
                                        : std::string{};
    std::string increment_keycode = !increment_keycodes.empty()
                                        ? std::move(increment_keycodes[i])
                                        : std::string{};
    meshcat_->AddSlider(
        slider_detail.name, slider_detail.min, slider_detail.max,
        slider_detail.step, slider_detail.nominal_value,
        std::move(decrement_keycode), std::move(increment_keycode));
  }

  // Declare the output port.
  auto& output = this->DeclareVectorOutputPort("positions", nq,
                                               &JointSliders<T>::CalcOutput);

  // The port is always marked out-of-date, so that the slider values will be
  // fetched every time the output port is needed in a computation.
  output.disable_caching_by_default();
}

template <typename T>
void JointSliders<T>::Delete() {
  const auto was_registered = is_registered_.exchange(false);
  if (was_registered) {
    for (const auto& slider_detail : slider_details_) {
      meshcat_->DeleteSlider(slider_detail.name, /*strict = */ false);
    }
  }
}

template <typename T>
JointSliders<T>::~JointSliders() {
  // Destructors are not allowed to throw. Ensure this by catching any
  // exceptions and failing fast.
  try {
    Delete();
  } catch (...) {
    DRAKE_UNREACHABLE();
  }
}

template <typename T>
void JointSliders<T>::CalcOutput(const Context<T>&,
                                 BasicVector<T>* output) const {
  const int nq = plant_->num_positions();
  DRAKE_DEMAND(output->size() == nq);
  if (is_registered_) {
    for (int i = 0; i < nq; ++i) {
      const auto& slider_detail = slider_details_[i];
      // TODO(jwnimmer-tri) If CalcOutput is in flight concurrently with a
      // call to Delete, we might race and ask for a deleted slider value.
      (*output)[i] = meshcat_->GetSliderValue(slider_detail.name);
    }
  } else {
    for (int i = 0; i < nq; ++i) {
      const auto& slider_detail = slider_details_[i];
      (*output)[i] = slider_detail.nominal_value;
    }
  }
}

template <typename T>
typename systems::LeafSystem<T>::GraphvizFragment
JointSliders<T>::DoGetGraphvizFragment(
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
Eigen::VectorXd JointSliders<T>::Run(const Diagram<T>& diagram,
                                     std::optional<double> timeout,
                                     std::string stop_button_keycode) const {
  // Make a context and create reference shortcuts to some pieces of it.
  // TODO(jwnimmer-tri) If the user has forgotten to add the plant or sliders
  // to the diagram, our error message here is awful. Ideally, we should be
  // asking the diagram if the systems have been added already, but as of
  // when this code was written, there was no function available to ask that
  // question.
  std::unique_ptr<Context<T>> root_context = diagram.CreateDefaultContext();
  const auto& diagram_context = *root_context;
  const auto& sliders_context = this->GetMyContextFromRoot(*root_context);
  auto& plant_context = plant_->GetMyMutableContextFromRoot(&*root_context);

  // Add the Stop button.
  constexpr char kButtonName[] = "Stop JointSliders";
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

  diagram.ExecuteInitializationEvents(root_context.get());

  // Set the context to the initial slider values.
  plant_->SetPositions(&plant_context,
                       this->get_output_port().Eval(sliders_context));

  // Loop until the button is clicked, or the timeout (when given) is reached.
  diagram.ForcedPublish(diagram_context);
  while (meshcat_->GetButtonClicks(kButtonName) < 1) {
    if (timeout.has_value()) {
      const auto elapsed = Duration(Clock::now() - start_time).count();
      if (elapsed >= timeout.value()) {
        break;
      }
    }

    // If the sliders have not changed, avoid invalidating the context.
    const auto& old_positions = plant_->GetPositions(plant_context);
    const auto& new_positions = this->get_output_port().Eval(sliders_context);
    if (new_positions == old_positions) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    // Publish the new positions.
    plant_->SetPositions(&plant_context, new_positions);
    diagram.ForcedPublish(diagram_context);
  }

  return ExtractDoubleOrThrow(plant_->GetPositions(plant_context).eval());
}

template <typename T>
void JointSliders<T>::SetPositions(const Eigen::VectorXd& q) {
  const int nq = plant_->num_positions();
  if (q.size() != nq) {
    throw std::logic_error(fmt::format(
        "Expected q of size {}, but got size {} instead", nq, q.size()));
  }
  /* For *all* positions provided in q, update their value in nominal_value_. */
  for (int i = 0; i < nq; ++i) {
    slider_details_[i].nominal_value = q[i];
  }
  if (is_registered_) {
    // For items with an associated slider, update the meshcat UI.
    // TODO(jwnimmer-tri) If SetPositions is in flight concurrently with a
    // call to Delete, we might race and ask for a deleted slider value.
    for (int i = 0; i < nq; ++i) {
      meshcat_->SetSliderValue(slider_details_[i].name, q[i]);
    }
  }
}

}  // namespace meshcat
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::meshcat::JointSliders);
