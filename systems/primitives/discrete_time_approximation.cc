#include "drake/systems/primitives/discrete_time_approximation.h"

#include <memory>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_config_functions.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

namespace {

template <typename T>
class DiscreteTimeSystem final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteTimeSystem);

  DiscreteTimeSystem(std::unique_ptr<System<T>> system, double time_period,
                     double time_offset,
                     const SimulatorConfig& integrator_config)
      : LeafSystem<T>(SystemTypeTag<DiscreteTimeSystem>{}),
        continuous_system_(std::move(system)),
        continuous_context_(continuous_system_->CreateDefaultContext()),
        time_period_(time_period),
        time_offset_(time_offset),
        integrator_config_(integrator_config),
        simulator__(Simulator(*continuous_system_)) {
    this->Initialize();
  }

  void SetRandomParameters(const Context<T>& context, Parameters<T>* parameters,
                           RandomGenerator* generator) const override {
    continuous_system_->SetRandomParameters(
        *continuous_context_, &(continuous_context_->get_mutable_parameters()),
        generator);
    parameters->SetFrom(continuous_context_->get_parameters());
  }

  void SetRandomState(const Context<T>& context, State<T>* state,
                      RandomGenerator* generator) const override {
    continuous_system_->SetRandomState(
        *continuous_context_, &(continuous_context_->get_mutable_state()),
        generator);
    state->get_mutable_discrete_state().get_mutable_vector().SetFromVector(
        continuous_context_->get_continuous_state_vector().CopyToVector());
  }

  // Scalar-converting copy constructor. See @ref system_scalar_conversion.
  template <typename U>
  explicit DiscreteTimeSystem(const DiscreteTimeSystem<U>& other)
      : DiscreteTimeSystem<T>(
            other.continuous_system_->template ToScalarType<T>(),
            other.time_period_, other.time_offset_, other.integrator_config_) {}

 private:
  template <typename U>
  friend class DiscreteTimeSystem;

  void Initialize() {
    // Configure integrator.
    ApplySimulatorConfig(integrator_config_, &simulator__);
    integrator_ = &(simulator__.get_mutable_integrator());
    integrator_->reset_context(continuous_context_.get());
    integrator_->Initialize();

    // Set name.
    const std::string& name = continuous_system_->get_name();
    this->set_name(name.empty() ? "discrete-time approximation"
                                : "discrete-time approximated " + name);

    // Declare input ports.
    for (int i = 0; i < continuous_system_->num_input_ports(); ++i) {
      const InputPort<T>& port = continuous_system_->get_input_port(i);
      this->DeclareInputPort(port.get_name(), port.get_data_type(), port.size(),
                             port.get_random_type());
    }

    // Declare output ports.
    for (int i = 0; i < continuous_system_->num_output_ports(); ++i) {
      const OutputPort<T>& port = continuous_system_->get_output_port(i);
      if (port.get_data_type() == PortDataType::kVectorValued) {
        this->DeclareVectorOutputPort(
            port.get_name(), port.size(),
            [this, &port](const Context<T>& discrete_context,
                          BasicVector<T>* out) {
              this->CopyDiscreteContextToContinuousContext(discrete_context);
              out->SetFromVector(port.Eval(*(this->continuous_context_)));
            });
      } else if (port.get_data_type() == PortDataType::kAbstractValued) {
        this->DeclareAbstractOutputPort(
            port.get_name(),
            [&port] {
              return port.Allocate();
            },
            [this, &port](const Context<T>& discrete_context,
                          AbstractValue* out) {
              this->CopyDiscreteContextToContinuousContext(discrete_context);
              port.Calc(*(this->continuous_context_), out);
            });
      }
    }

    // Declare parameters.
    for (int i = 0; i < continuous_context_->num_numeric_parameter_groups();
         ++i) {
      this->DeclareNumericParameter(
          continuous_context_->get_numeric_parameter(i));
    }
    for (int i = 0; i < continuous_context_->num_abstract_parameters(); ++i) {
      this->DeclareAbstractParameter(
          continuous_context_->get_abstract_parameter(i));
    }

    // Declare state.
    this->DeclareDiscreteState(continuous_system_->num_continuous_states());
    this->DeclarePeriodicDiscreteUpdateEvent(
        time_period_, time_offset_, &DiscreteTimeSystem<T>::DiscreteUpdate);
  }

  void DiscreteUpdate(const Context<T>& discrete_context,
                      DiscreteValues<T>* out) const {
    this->CopyDiscreteContextToContinuousContext(discrete_context);
    this->integrator_->IntegrateWithMultipleStepsToTime(
        continuous_context_->get_time() + time_period_);
    out->get_mutable_vector().SetFromVector(
        continuous_context_->get_continuous_state_vector().CopyToVector());
  }

  void CopyDiscreteContextToContinuousContext(
      const Context<T>& discrete_context) const {
    // Copy time.
    continuous_context_->SetTime(discrete_context.get_time());
    // Copy state.
    continuous_context_->SetContinuousState(
        discrete_context.get_discrete_state_vector().value());
    // Copy parameters.
    continuous_context_->get_mutable_parameters().SetFrom(
        discrete_context.get_parameters());
    // Copy accuracy.
    continuous_context_->SetAccuracy(discrete_context.get_accuracy());
    // Copy fixed input port values.
    for (int i = 0; i < continuous_system_->num_input_ports(); ++i) {
      continuous_context_->FixInputPort(
          i, *(this->EvalAbstractInput(discrete_context, i)));
    }
  }

  const std::unique_ptr<const System<T>> continuous_system_;
  const std::unique_ptr<Context<T>> continuous_context_;
  const double time_period_;
  const double time_offset_;
  const SimulatorConfig integrator_config_;
  Simulator<T> simulator__;
  IntegratorBase<T>* integrator_;
};

}  // namespace

namespace scalar_conversion {
template <>
struct Traits<DiscreteTimeSystem> : public NonSymbolicTraits {};
}  // namespace scalar_conversion

template <typename T>
std::unique_ptr<System<T>> DiscreteTimeApproximation(
    const System<T>& system, double time_period, double time_offset,
    const SimulatorConfig& integrator_config) {
  // Check that the original system is continuous.
  DRAKE_THROW_UNLESS(system.IsDifferentialEquationSystem());
  // Check that the discrete time_period is greater than zero.
  DRAKE_THROW_UNLESS(time_period > 0);

  return std::make_unique<DiscreteTimeSystem<T>>(
      system.Clone(), time_period, time_offset, integrator_config);
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&DiscreteTimeApproximation<T>));

}  // namespace systems
}  // namespace drake
