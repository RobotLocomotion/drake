#include "drake/multibody/meshcat/joint_sliders.h"

#include <chrono>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/scope_exit.h"
#include "drake/common/unused.h"

namespace {
// Boilerplate for std::visit.
template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;
}  // namespace

namespace drake {
namespace multibody {
namespace meshcat {

using Eigen::VectorXd;
using systems::BasicVector;
using systems::Context;
using systems::Diagram;

namespace {

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

// Returns true iff data has any duplicated values.
bool HasAnyDuplicatedValues(const std::map<int, std::string>& data) {
  std::unordered_set<std::string_view> values_seen;
  for (const auto& iter : data) {
    const std::string& one_value = iter.second;
    const bool inserted = values_seen.insert(one_value).second;
    if (!inserted) {
      return true;
    }
  }
  return false;
}

// Returns a mapping from an index within the plant's position vector to the
// slider name that refers to it.  Note that the map is sparse; positions
// without joints (e.g., a floating base) are not represented here.
//
// When use_model_instance_name is set, both the joint name and model name will
// be used to to form the slider name; otherwise, only the joint name is used,
// unless there are duplicate joint names in which case this is forced to be
// "true".
template <typename T>
std::map<int, std::string> GetPositionNames(
    const MultibodyPlant<T>* plant,
    bool use_model_instance_name = false) {
  DRAKE_THROW_UNLESS(plant != nullptr);

  // Map all joints into the positions-to-name result.
  std::map<int, std::string> result;
  for (JointIndex i{0}; i < plant->num_joints(); ++i) {
    const Joint<T>& joint = plant->get_joint(i);
    for (int j = 0; j < joint.num_positions(); ++j) {
      const int position_index = joint.position_start() + j;
      std::string description;
      if (joint.num_positions() > 1) {
        description = fmt::format("{}_{}",
            joint.name(), joint.position_suffix(j));
      } else {
        description = joint.name();
      }
      if (use_model_instance_name) {
        description += fmt::format("/{}",
            plant->GetModelInstanceName(joint.model_instance()));
      }
      const bool inserted = result.insert({
          position_index, std::move(description)}).second;
      DRAKE_DEMAND(inserted);
    }
  }

  // Check for duplicate names.  If we had a name collision, then we'll need
  // need to use the model_instance_name to make them unique.
  if (HasAnyDuplicatedValues(result)) {
    DRAKE_DEMAND(use_model_instance_name == false);
    return GetPositionNames(plant, true);
  }

  return result;
}

// Returns a vector of size num_positions, based on the given value variant.
// If a VectorXd is given, it's checked for size and then returned unchanged.
// If a double is given, it's broadcast to size and returned.
// If no value is given, then the default_value is broadcast instead.
VectorXd Broadcast(
    const char* diagnostic_name, double default_value, int num_positions,
    std::variant<std::monostate, double, VectorXd> value) {
  return std::visit(overloaded{
    [num_positions, default_value](std::monostate) -> VectorXd {
      return VectorXd::Constant(num_positions, default_value);
    },
    [num_positions](double arg) -> VectorXd {
      return VectorXd::Constant(num_positions, arg);
    },
    [num_positions, diagnostic_name](VectorXd&& arg) -> VectorXd {
      if (arg.size() != num_positions) {
        throw std::logic_error(fmt::format(
            "Expected {} of size {}, but got size {} instead",
            diagnostic_name, num_positions, arg.size()));
      }
      return std::move(arg);
    },
  }, std::move(value));
}

}  // namespace

template <typename T>
JointSliders<T>::JointSliders(
    std::shared_ptr<geometry::Meshcat> meshcat, const MultibodyPlant<T>* plant,
    std::optional<VectorXd> initial_value,
    std::variant<std::monostate, double, VectorXd> lower_limit,
    std::variant<std::monostate, double, VectorXd> upper_limit,
    std::variant<std::monostate, double, VectorXd> step,
    std::vector<std::string> decrement_keycodes,
    std::vector<std::string> increment_keycodes)
    : meshcat_(std::move(meshcat)),
      plant_(plant),
      position_names_(GetPositionNames(plant)),
      initial_value_(
          std::move(initial_value).value_or(GetDefaultPositions(plant))),
      is_registered_{true} {
  DRAKE_THROW_UNLESS(meshcat_ != nullptr);
  DRAKE_THROW_UNLESS(plant_ != nullptr);

  const int nq = plant->num_positions();
  if (initial_value_.size() != nq) {
    throw std::logic_error(fmt::format(
        "Expected initial_value of size {}, but got size {} instead",
        nq, initial_value_.size()));
  }

  // Default any missing arguments; check (or widen) them to be of size == nq.
  const VectorXd lower_broadcast = Broadcast(
      "lower_limit", -10.0,  nq, std::move(lower_limit));
  const VectorXd upper_broadcast = Broadcast(
      "upper_limit",  10.0,  nq, std::move(upper_limit));
  const VectorXd step_broadcast = Broadcast(
      "step",          0.01, nq, std::move(step));

  if (decrement_keycodes.size() &&
      static_cast<int>(decrement_keycodes.size()) != nq) {
    throw std::logic_error(
        fmt::format("Expected decrement_keycodes of size zero or {}, but got "
                    "size {} instead",
                    nq, decrement_keycodes.size()));
  }

  if (increment_keycodes.size() &&
      static_cast<int>(increment_keycodes.size()) != nq) {
    throw std::logic_error(
        fmt::format("Expected increment_keycodes of size zero or {}, but got "
                    "size {} instead",
                    nq, increment_keycodes.size()));
  }

  // Add one slider per joint position.
  const VectorXd lower_plant = plant_->GetPositionLowerLimits();
  const VectorXd upper_plant = plant_->GetPositionUpperLimits();
  for (const auto& [position_index, slider_name] : position_names_) {
    DRAKE_DEMAND(position_index >= 0);
    DRAKE_DEMAND(position_index < nq);
    const double one_min = std::max(
        lower_broadcast[position_index],
        lower_plant[position_index]);
    const double one_max = std::min(
        upper_broadcast[position_index],
        upper_plant[position_index]);
    const double one_step = step_broadcast[position_index];
    const double one_value = initial_value_[position_index];
    const std::string one_decrement_keycode =
        decrement_keycodes.size()
            ? std::move(decrement_keycodes[position_index])
            : "";
    const std::string one_increment_keycode =
        increment_keycodes.size()
            ? std::move(increment_keycodes[position_index])
            : "";
    meshcat_->AddSlider(slider_name, one_min, one_max, one_step, one_value,
                        one_decrement_keycode, one_increment_keycode);
  }

  // Declare the output port.
  auto& output = this->DeclareVectorOutputPort(
      "positions", nq, &JointSliders<T>::CalcOutput);

  // The port is always marked out-of-date, so that the slider values will be
  // fetched every time the output port is needed in a computation.
  output.disable_caching_by_default();
}

template <typename T>
void JointSliders<T>::Delete() {
  const auto was_registered = is_registered_.exchange(false);
  if (was_registered) {
    for (const auto& [position_index, slider_name] : position_names_) {
      unused(position_index);
      meshcat_->DeleteSlider(slider_name);
    }
  }
}

template <typename T>
JointSliders<T>::~JointSliders() {
  Delete();
}

template <typename T>
void JointSliders<T>::CalcOutput(
    const Context<T>&, BasicVector<T>* output) const {
  const int nq = plant_->num_positions();
  DRAKE_DEMAND(output->size() == nq);
  for (int i = 0; i < nq; ++i) {
    (*output)[i] = initial_value_[i];
  }
  if (is_registered_) {
    for (const auto& [position_index, slider_name] : position_names_) {
      // TODO(jwnimmer-tri) If CalcOutput is in flight concurrently with a
      // call to Delete, we might race and ask for a deleted slider value.
      (*output)[position_index] = meshcat_->GetSliderValue(slider_name);
    }
  }
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

  // Grab the current time, to implement the timout.
  using Clock = std::chrono::steady_clock;
  using Duration = std::chrono::duration<double>;
  const auto start_time = Clock::now();

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
        "Expected q of size {}, but got size {} instead",
        nq, q.size()));
  }
  /* For all positions provided in q, update their value in initial_value_
    including items without an associated slider (e.g., a floating base). */
  initial_value_ = q;
  if (is_registered_) {
    // For items with an associated slider, update the meshcat UI.
    // TODO(jwnimmer-tri) If SetPositions is in flight concurrently with a
    // call to Delete, we might race and ask for a deleted slider value.
    for (const auto& [position_index, slider_name] : position_names_) {
      meshcat_->SetSliderValue(slider_name, q[position_index]);
    }
  }
}

}  // namespace meshcat
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::meshcat::JointSliders)
