#include "drake/multibody/meshcat/joint_sliders.h"

#include <chrono>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/overloaded.h"
#include "drake/common/scope_exit.h"
#include "drake/common/text_logging.h"
#include "drake/common/unused.h"
#include "drake/geometry/meshcat_graphviz.h"
#include "drake/multibody/inverse_kinematics/add_multibody_plant_constraints.h"
#include "drake/solvers/choose_best_solver.h"

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
// slider name that refers to it. The map only includes positions associated
// with *joints*. Positions without joints (e.g., deformable vertex positions)
// are not represented here.
//
// When use_model_instance_name is set, both the joint name and model name will
// be used to to form the slider name; otherwise, only the joint name is used,
// unless there are duplicate joint names in which case this is forced to be
// "true".
template <typename T>
std::map<int, std::string> GetPositionNames(
    const MultibodyPlant<T>* plant, bool use_model_instance_name = false) {
  DRAKE_THROW_UNLESS(plant != nullptr);

  // Map all joints into the positions-to-name result.
  std::map<int, std::string> result;
  for (JointIndex i : plant->GetJointIndices()) {
    const Joint<T>& joint = plant->get_joint(i);
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
      const bool inserted =
          result.insert({position_index, std::move(description)}).second;
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
VectorXd Broadcast(const char* diagnostic_name, double default_value,
                   int num_positions,
                   std::variant<std::monostate, double, VectorXd> value) {
  return std::visit<VectorXd>(
      overloaded{
          [num_positions, default_value](std::monostate) {
            return VectorXd::Constant(num_positions, default_value);
          },
          [num_positions](double arg) {
            return VectorXd::Constant(num_positions, arg);
          },
          [num_positions, diagnostic_name](VectorXd&& arg) {
            if (arg.size() != num_positions) {
              throw std::logic_error(
                  fmt::format("Expected {} of size {}, but got size {} instead",
                              diagnostic_name, num_positions, arg.size()));
            }
            return std::move(arg);
          },
      },
      std::move(value));
}

template <typename T>
std::shared_ptr<const MultibodyPlant<double>> MakeDoublePlant(
    const MultibodyPlant<T>* plant) {
  return systems::System<T>::template ToScalarType<double>(*plant);
}

template <>
std::shared_ptr<const MultibodyPlant<double>> MakeDoublePlant(
    const MultibodyPlant<double>* plant) {
  return std::shared_ptr<const MultibodyPlant<double>>{
      /* managed object = */ std::shared_ptr<void>{},
      /* stored pointer = */ plant};
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
    std::vector<std::string> increment_keycodes, double time_step)
    : meshcat_(std::move(meshcat)),
      plant_(plant),
      double_plant_(MakeDoublePlant(plant)),
      position_names_(GetPositionNames(plant)),
      is_registered_{true} {
  DRAKE_THROW_UNLESS(meshcat_ != nullptr);
  DRAKE_THROW_UNLESS(plant_ != nullptr);

  const int nq = plant->num_positions();
  Eigen::VectorXd nominal_value =
      std::move(initial_value).value_or(GetDefaultPositions(plant));
  if (nominal_value.size() != nq) {
    throw std::logic_error(fmt::format(
        "Expected initial_value of size {}, but got size {} instead", nq,
        nominal_value.size()));
  }

  // Default any missing arguments; check (or widen) them to be of size == nq.
  const VectorXd lower_broadcast =
      Broadcast("lower_limit", -10.0, nq, std::move(lower_limit));
  const VectorXd upper_broadcast =
      Broadcast("upper_limit", 10.0, nq, std::move(upper_limit));
  step_ = Broadcast("step", 0.01, nq, std::move(step));

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
  lower_ = plant_->GetPositionLowerLimits();
  upper_ = plant_->GetPositionUpperLimits();
  for (const auto& [position_index, slider_name] : position_names_) {
    DRAKE_DEMAND(position_index >= 0);
    DRAKE_DEMAND(position_index < nq);
    lower_[position_index] =
        std::max(lower_broadcast[position_index], lower_[position_index]);
    upper_[position_index] =
        std::min(upper_broadcast[position_index], upper_[position_index]);
    const double one_value = nominal_value[position_index];
    const std::string one_decrement_keycode =
        decrement_keycodes.size()
            ? std::move(decrement_keycodes[position_index])
            : "";
    const std::string one_increment_keycode =
        increment_keycodes.size()
            ? std::move(increment_keycodes[position_index])
            : "";
    meshcat_->AddSlider(slider_name, lower_[position_index],
                        upper_[position_index], step_[position_index],
                        one_value, one_decrement_keycode,
                        one_increment_keycode);
  }

  slider_values_index_ = this->DeclareDiscreteState(nominal_value.cast<T>());

  this->DeclarePeriodicDiscreteUpdateEvent(time_step, 0.0,
                                           &JointSliders<T>::Update);

  // Setup a mathematical program to resolve constraints.
  double_plant_context_ = double_plant_->CreateDefaultContext();
  q_ = prog_.NewContinuousVariables(nq, "q");
  auto bindings = AddMultibodyPlantConstraints(double_plant_, q_, &prog_,
                                               double_plant_context_.get());
  // AddMultibodyPlantConstraints always added joint limit constraints. If the
  // only constraint added is the joint limits, then we don't need to call
  // Solve() in Update(). The sliders will trivially respect the joint limits.
  has_constraints_to_solve_ = bindings.size() > 1;

  if (has_constraints_to_solve_) {
    // Replace the joint limits from AddMultibodyPlantConstraints with the
    // potentially tighter limits from the sliders.
    bool removed_joint_limits = false;
    for (const auto& binding : prog_.bounding_box_constraints()) {
      if (binding.evaluator()->get_description() == "Joint limits") {
        prog_.RemoveConstraint(binding);
        removed_joint_limits = true;
        break;
      }
    }
    DRAKE_DEMAND(removed_joint_limits);
    prog_.AddBoundingBoxConstraint(lower_, upper_, q_)
        .evaluator()
        ->set_description("Slider limits");

    // Make the solver.
    const solvers::SolverId solver_id = solvers::ChooseBestSolver(prog_);
    drake::log()->debug("JointSliders will use solver_id {}", solver_id);
    solver_ = solvers::MakeSolver(solver_id);

    // Solve the problem once.
    Eigen::VectorXd constrained_values(nq);
    solvers::SolutionResult result;
    std::tie(constrained_values, result) =
        ResolveConstraints(nominal_value, nominal_value, nominal_value);
    if (result != solvers::SolutionResult::kSolutionFound) {
      throw std::runtime_error(
          "JointSliders failed to resolve the constraints at startup.");
    }
    constrained_values_index_ =
        this->DeclareDiscreteState(constrained_values.cast<T>());
    result_index_ =
        this->DeclareDiscreteState(Vector1<T>{static_cast<T>(result)});
    this->DeclareStateOutputPort("positions", constrained_values_index_);

    // We only need to publish updated values if ResolveConstraints() is making
    // changes.
    this->DeclarePeriodicPublishEvent(time_step, 0.0,
                                      &JointSliders<T>::Publish);
    this->DeclareForcedPublishEvent(&JointSliders<T>::Publish);
  } else {
    this->DeclareStateOutputPort("positions", slider_values_index_);
  }
}

template <typename T>
std::pair<Eigen::VectorXd, solvers::SolutionResult>
JointSliders<T>::ResolveConstraints(
    const Eigen::Ref<const VectorXd>& target_values,
    const Eigen::Ref<const VectorXd>& previous_values,
    const Eigen::Ref<const VectorXd>& initial_guess) const {
  // To resolve constraints, we'll use an IK problem formulated as:
  // min_q |q - q_sliders|^2
  // s.t. q[just_updated_index] == q_sliders[just_updated_index]
  //      additional constraints from AddMultibodyPlantConstraints already
  //      added to prog_ in the constructor.
  solvers::Binding<solvers::QuadraticCost> cost =
      prog_.AddQuadraticErrorCost(1.0, target_values, q_);
  std::vector<solvers::Binding<solvers::Constraint>> constraints;
  for (int i = 0; i < plant_->num_positions(); ++i) {
    if (target_values[i] != previous_values[i]) {
      constraints.push_back(prog_.AddBoundingBoxConstraint(
          target_values[i], target_values[i], q_[i]));
    }
  }
  solvers::MathematicalProgramResult result{};
  solver_->Solve(prog_, initial_guess, {}, &result);
  prog_.RemoveCost(cost);
  for (const auto& constraint : constraints) {
    prog_.RemoveConstraint(constraint);
  }
  return {result.GetSolution(q_), result.get_solution_result()};
}

template <typename T>
Eigen::VectorXd JointSliders<T>::RoundSliderValues(
    const Eigen::Ref<const Eigen::VectorXd>& values) const {
  Eigen::VectorXd result = values;
  // This should match the logic in meshcat_->SetSliderValue.
  for (int i = 0; i < plant_->num_positions(); ++i) {
    result[i] = std::max(result[i], lower_[i]);
    result[i] = std::min(result[i], upper_[i]);
    result[i] = std::round(result[i] / step_[i]) * step_[i];
  }
  return result;
}

template <typename T>
void JointSliders<T>::Delete() {
  const auto was_registered = is_registered_.exchange(false);
  if (was_registered) {
    for (const auto& [position_index, slider_name] : position_names_) {
      unused(position_index);
      meshcat_->DeleteSlider(slider_name, /*strict = */ false);
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
systems::EventStatus JointSliders<T>::Update(
    const systems::Context<T>& context,
    systems::DiscreteValues<T>* updates) const {
  const VectorX<T>& slider_values =
      context.get_discrete_state().value(slider_values_index_);
  Eigen::VectorBlock<VectorX<T>> new_slider_values =
      updates->get_mutable_value(slider_values_index_);

  new_slider_values = slider_values;
  if (is_registered_) {
    for (const auto& [position_index, slider_name] : position_names_) {
      // TODO(jwnimmer-tri) If CalcOutput is in flight concurrently with a
      // call to Delete, we might race and ask for a deleted slider value.
      new_slider_values[position_index] = meshcat_->GetSliderValue(slider_name);
    }
  }

  if (has_constraints_to_solve_) {
    const VectorX<T>& constrained_values =
        context.get_discrete_state().value(constrained_values_index_);
    Eigen::VectorBlock<VectorX<T>> new_constrained_values =
        updates->get_mutable_value(constrained_values_index_);
    VectorXd solved_values(new_constrained_values.size());

    if (new_slider_values != slider_values) {
      // Then something changed in the gui. Resolve the constraints.
      solvers::SolutionResult result;
      std::tie(solved_values, result) =
          ResolveConstraints(ExtractDoubleOrThrow(new_slider_values.eval()),
                             ExtractDoubleOrThrow(slider_values),
                             ExtractDoubleOrThrow(constrained_values));
      T previous_result = context.get_discrete_state().value(result_index_)[0];
      // We only log a warning if we've switched from success to failure.
      if (static_cast<T>(result) != previous_result &&
          result != solvers::SolutionResult::kSolutionFound) {
        log()->warn("JointSliders failed to resolve the constraints.");
      }
      updates->set_value(result_index_, Vector1<T>{static_cast<T>(result)});
      new_constrained_values = solved_values;
      new_slider_values = RoundSliderValues(solved_values);
    } else {
      new_constrained_values = constrained_values;
    }
  }
  return systems::EventStatus::Succeeded();
}

template <typename T>
systems::EventStatus JointSliders<T>::Publish(
    const systems::Context<T>& context) const {
  if (is_registered_) {
    const VectorX<T>& slider_values =
        context.get_discrete_state().value(slider_values_index_);
    for (const auto& [position_index, slider_name] : position_names_) {
      // TODO(russt): I considered publishing only the slider values which did
      // _not_ change in the last Update(), reasoning that the changed values
      // are the ones being actively modified by the user in the gui. I don't
      // want to fight with the user. But preliminary testing suggests that
      // setting all values is fine; and it's certainly simpler.
      meshcat_->SetSliderValue(
          slider_name, ExtractDoubleOrThrow(slider_values[position_index]));
    }
  }
  return systems::EventStatus::Succeeded();
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
  auto& sliders_context = this->GetMyMutableContextFromRoot(root_context.get());
  auto& plant_context = plant_->GetMyMutableContextFromRoot(root_context.get());

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
    const systems::DiscreteValues<T>& updated =
        this->EvalUniquePeriodicDiscreteUpdate(sliders_context);
    sliders_context.SetDiscreteState(updated);
    const auto& new_positions = this->get_output_port().Eval(sliders_context);
    if (new_positions == old_positions) {
      std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / 32.0));
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
  if (is_registered_) {
    // For items with an associated slider, update the meshcat UI.
    // TODO(jwnimmer-tri) If SetPositions is in flight concurrently with a
    // call to Delete, we might race and ask for a deleted slider value.
    for (const auto& [position_index, slider_name] : position_names_) {
      const double rounded_value =
          meshcat_->SetSliderValue(slider_name, q[position_index]);
      // This is a sanity check to make sure that the rounding logic in
      // RoundSliderValues() stays in sync with the rounding in
      // Meshcat::SetSliderValue().
      DRAKE_ASSERT(rounded_value == q[position_index]);
    }
  }
}

template <typename T>
void JointSliders<T>::SetPositions(systems::Context<T>* context,
                                   const Eigen::VectorXd& q) {
  this->ValidateContext(context);
  const int nq = plant_->num_positions();
  if (q.size() != nq) {
    throw std::logic_error(fmt::format(
        "Expected q of size {}, but got size {} instead", nq, q.size()));
  }
  Eigen::VectorBlock<VectorX<T>> slider_values =
      context->get_mutable_discrete_state().get_mutable_value(
          slider_values_index_);
  if (has_constraints_to_solve_) {
    Eigen::VectorBlock<VectorX<T>> constrained_values =
        context->get_mutable_discrete_state().get_mutable_value(
            constrained_values_index_);
    VectorXd solved_values(nq);
    solvers::SolutionResult result;
    std::tie(solved_values, result) = ResolveConstraints(
        q, q, ExtractDoubleOrThrow(constrained_values.eval()));
    if (result != solvers::SolutionResult::kSolutionFound) {
      throw std::runtime_error(
          "JointSliders failed to resolve the constraints.");
    }
    constrained_values = solved_values;
    slider_values = RoundSliderValues(solved_values);
  } else {
    slider_values = RoundSliderValues(q);
  }
  Publish(*context);
}

}  // namespace meshcat
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::meshcat::JointSliders);
