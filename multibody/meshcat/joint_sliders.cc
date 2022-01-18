#include "drake/multibody/meshcat/joint_sliders.h"

#include <utility>

#include <fmt/format.h>

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

// Returns a mapping from an index within the plant's position vector to the
// slider name that refers to it.  Note that the map is sparse; positions
// without joints (e.g., a floating base) are not represented here.
template <typename T>
std::map<int, std::string> GetPositionNames(const MultibodyPlant<T>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);
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
      const bool inserted = result.insert({
          position_index, std::move(description)}).second;
      DRAKE_DEMAND(inserted);
    }
  }
  return result;
}

// Returns a vector of size num_positions, drawing from the given value when
// provided.
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
      return arg;
    },
  }, std::move(value));
}

}  // namespace

template <typename T>
JointSliders<T>::JointSliders(
    std::shared_ptr<geometry::Meshcat> meshcat,
    const MultibodyPlant<T>* plant,
    std::optional<VectorXd> initial_value,
    std::variant<std::monostate, double, VectorXd> lower_limit,
    std::variant<std::monostate, double, VectorXd> upper_limit,
    std::variant<std::monostate, double, VectorXd> step)
    : meshcat_(std::move(meshcat)),
      plant_(plant),
      position_names_(GetPositionNames(plant)),
      initial_value_(std::move(initial_value).value_or(
          GetDefaultPositions(plant))) {
  DRAKE_THROW_UNLESS(meshcat_ != nullptr);
  DRAKE_THROW_UNLESS(plant_ != nullptr);

  // Default any missing arguments; check (or widen) them to be of size == nq.
  const int nq = plant->num_positions();
  const VectorXd lower_broadcast = Broadcast(
      "lower_limit", -10.0,  nq, std::move(lower_limit));
  const VectorXd upper_broadcast = Broadcast(
      "upper_limit",  10.0,  nq, std::move(upper_limit));
  const VectorXd step_broadcast = Broadcast(
      "step",          0.01, nq, std::move(step));

  // Add one slider per joint position.
  const VectorXd lower_plant = plant_->GetPositionLowerLimits();
  const VectorXd upper_plant = plant_->GetPositionUpperLimits();
  for (const auto& [position_index, slider_name] : position_names_) {
    DRAKE_DEMAND(position_index >= 0);
    DRAKE_DEMAND(position_index < nq);
    const double slider_min = std::max(
        lower_broadcast[position_index],
        lower_plant[position_index]);
    const double slider_max = std::min(
        upper_broadcast[position_index],
        upper_plant[position_index]);
    const double slider_step = step_broadcast[position_index];
    const double value = initial_value_[position_index];
    meshcat->AddSlider(slider_name, slider_min, slider_max, slider_step, value);
  }

  // Declare the output port.
  auto& output = this->DeclareVectorOutputPort(
      "positions", nq, &JointSliders<T>::CalcOutput);
  output.disable_caching_by_default();
}

template <typename T>
JointSliders<T>::~JointSliders() = default;

template <typename T>
void JointSliders<T>::CalcOutput(
    const Context<T>&, BasicVector<T>* output) const {
  const int nq = plant_->num_positions();
  DRAKE_DEMAND(output->size() == nq);
  for (int i = 0; i < nq; ++i) {
    (*output)[i] = initial_value_[i];
  }
  for (const auto& [position_index, slider_name] : position_names_) {
    (*output)[position_index] = meshcat_->GetSliderValue(slider_name);
  }
}

}  // namespace meshcat
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::meshcat::JointSliders)
