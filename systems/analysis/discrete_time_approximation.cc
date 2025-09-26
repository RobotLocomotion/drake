#include "drake/systems/analysis/discrete_time_approximation.h"

#include <functional>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <unsupported/Eigen/MatrixFunctions>

#include "drake/common/nice_type_name.h"
#include "drake/systems/analysis/simulator_config_functions.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

namespace {

struct DummyValue {};

template <typename T>
class DiscreteTimeSystem final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteTimeSystem);

  DiscreteTimeSystem(std::shared_ptr<const System<T>> system,
                     double time_period,
                     const SimulatorConfig& integrator_config)
      : LeafSystem<T>(SystemTypeTag<DiscreteTimeSystem>{}),
        continuous_system_(std::move(system)),
        time_period_(time_period),
        integrator_config_(integrator_config) {
    DRAKE_THROW_UNLESS(continuous_system_ != nullptr);
    DRAKE_THROW_UNLESS(continuous_system_->num_continuous_states() &&
                       !continuous_system_->num_discrete_state_groups() &&
                       !continuous_system_->num_abstract_states());
    DRAKE_THROW_UNLESS(time_period_ > 0);

    this->get_mutable_system_scalar_converter().RemoveUnlessAlsoSupportedBy(
        continuous_system_->get_system_scalar_converter());
    if (!IsScalarTypeSupportedByIntegrator<symbolic::Expression>(
            integrator_config_.integration_scheme)) {
      this->get_mutable_system_scalar_converter()
          .template Remove<symbolic::Expression, T>();
    }

    this->Initialize();
  }

  // Scalar-converting copy constructor. See @ref system_scalar_conversion.
  template <typename U>
  explicit DiscreteTimeSystem(const DiscreteTimeSystem<U>& other)
      : DiscreteTimeSystem<T>(
            other.continuous_system_->template ToScalarType<T>(),
            other.time_period_, other.integrator_config_) {}

  ~DiscreteTimeSystem() override = default;

 private:
  template <typename U>
  friend class DiscreteTimeSystem;

  // This function is called by the constructor.
  void Initialize() {
    // Set name.
    const std::string& name = continuous_system_->get_name();
    this->set_name(name.empty() ? "DiscreteTimeApproximation"
                                : "DiscreteTimeApproximated_" + name);

    // Declare input ports.
    const int num_input_ports = continuous_system_->num_input_ports();
    for (int i = 0; i < num_input_ports; ++i) {
      const InputPort<T>& port = continuous_system_->get_input_port(i);

      if (port.get_data_type() == PortDataType::kVectorValued) {
        this->DeclareVectorInputPort(port.get_name(), port.size(),
                                     port.get_random_type());
      } else if (port.get_data_type() == PortDataType::kAbstractValued) {
        this->DeclareAbstractInputPort(port.get_name(),
                                       /* model value = */ *port.Allocate());
      }
      // The discrete system input port value will be used only by the the
      // continuous system input port, so no need to explicit specify the
      // "deprecate" attribute for this discrete system input port.
    }

    // Find the input dependencies of all output ports.
    const int num_output_ports = continuous_system_->num_output_ports();
    auto direct_feedthroughs = continuous_system_->GetDirectFeedthroughs();
    std::vector<std::vector<InputPortIndex>> all_input_dependencies(
        num_output_ports);
    for (const auto [input_index, output_index] : direct_feedthroughs) {
      all_input_dependencies[output_index].emplace_back(input_index);
    }

    // Declare output ports.
    for (int j = 0; j < num_output_ports; ++j) {
      const OutputPort<T>& port = continuous_system_->get_output_port(j);
      std::vector<InputPortIndex>& input_dependencies =
          all_input_dependencies[j];

      std::set<DependencyTicket> prerequisites_of_calc{
          SystemBase::all_sources_except_input_ports_ticket()};
      for (InputPortIndex input_index : input_dependencies) {
        prerequisites_of_calc.insert(this->input_port_ticket(input_index));
      }

      if (port.get_data_type() == PortDataType::kVectorValued) {
        this->DeclareVectorOutputPort(
            port.get_name(), port.size(),
            [this, &port, input_dep = std::move(input_dependencies)](
                const Context<T>& context, BasicVector<T>* out) {
              const Context<T>& continuous_context =
                  this->get_uptodate_continuous_context(context, &input_dep);
              out->SetFromVector(port.Eval(continuous_context));
            },
            prerequisites_of_calc);
      } else if (port.get_data_type() == PortDataType::kAbstractValued) {
        this->DeclareAbstractOutputPort(
            port.get_name(),
            [&port] {
              return port.Allocate();
            },
            [this, &port, input_dep = std::move(input_dependencies)](
                const Context<T>& context, AbstractValue* out) {
              const Context<T>& continuous_context =
                  this->get_uptodate_continuous_context(context, &input_dep);
              out->SetFrom(
                  port.template Eval<AbstractValue>(continuous_context));
            },
            prerequisites_of_calc);
      }
      // The discrete system output port relays from the continuous system
      // output port, so no need to explicit specify the "deprecate" attribute
      // for this discrete system output port.
    }

    // Declare state.
    auto continuous_context = continuous_system_->CreateDefaultContext();
    this->DeclareDiscreteState(/* model vector = */
                               continuous_context->get_continuous_state()
                                   .CopyToVector());
    this->DeclarePeriodicDiscreteUpdateEvent(
        time_period_, 0.0, &DiscreteTimeSystem<T>::DiscreteUpdate);

    // Declare parameters.
    for (int k = 0; k < continuous_context->num_numeric_parameter_groups();
         ++k) {
      this->DeclareNumericParameter(
          continuous_context->get_numeric_parameter(k));
    }
    for (int k = 0; k < continuous_context->num_abstract_parameters(); ++k) {
      this->DeclareAbstractParameter(
          continuous_context->get_abstract_parameter(k));
    }

    // Create an integrator based on the integrator config and store the mutable
    // integrator in a cache.
    std::unique_ptr<IntegratorBase<T>> integrator = CreateIntegratorFromConfig(
        continuous_system_.get(), integrator_config_);
    integrator->reset_context(std::move(continuous_context));
    integrator->Initialize();
    integrator_cache_entry_ = &this->DeclareCacheEntry(
        "integrator", ValueProducer(*integrator, &ValueProducer::NoopCalc),
        {SystemBase::nothing_ticket()});

    // These dummy-valued cache entries update the continuous system context.
    time_updater_ = &this->DeclareCacheEntry(
        "update time",
        UpdateFuncWrapper([this](const Context<T>& context, DummyValue*) {
          this->get_mutable_continuous_context(context).SetTime(
              context.get_time());
        }),
        {this->time_ticket()});
    state_updater_ = &this->DeclareCacheEntry(
        "update state",
        UpdateFuncWrapper([this](const Context<T>& context, DummyValue*) {
          this->get_mutable_continuous_context(context).SetContinuousState(
              context.get_discrete_state_vector().value());
        }),
        {this->xd_ticket()});
    parameters_updater_ = &this->DeclareCacheEntry(
        "update parameters",
        UpdateFuncWrapper([this](const Context<T>& context, DummyValue*) {
          this->get_mutable_continuous_context(context)
              .get_mutable_parameters()
              .SetFrom(context.get_parameters());
        }),
        {this->all_parameters_ticket()});
    accuracy_updater_ = &this->DeclareCacheEntry(
        "update accuracy",
        UpdateFuncWrapper([this](const Context<T>& context, DummyValue*) {
          this->get_mutable_continuous_context(context).SetAccuracy(
              context.get_accuracy());
        }),
        {this->accuracy_ticket()});

    input_port_value_updaters_.resize(num_input_ports);
    for (int i = 0; i < num_input_ports; ++i) {
      input_port_value_updaters_[i] = &this->DeclareCacheEntry(
          fmt::format("update input port {} value", i),
          UpdateFuncWrapper([this, i](const Context<T>& context, DummyValue*) {
            const InputPort<T>& input_port = this->get_input_port(i);
            if (input_port.HasValue(context)) {
              this->get_mutable_continuous_context(context).FixInputPort(
                  i, input_port.template Eval<AbstractValue>(context));
            }
          }),
          {this->input_port_ticket(InputPortIndex(i))});
    }
  }

  // Callback performing the periodic discrete update.
  void DiscreteUpdate(const Context<T>& context, DiscreteValues<T>* out) const {
    auto& continuous_context = this->get_uptodate_continuous_context(context);

    IntegratorBase<T>& integrator = this->get_mutable_integrator(context);
    integrator.IntegrateWithMultipleStepsToTime(continuous_context.get_time() +
                                                time_period_);

    // Although we have now updated the continuous time and state to where we
    // eventually want the discrete time and state to end up, our promise for
    // `get_uptodate_continuous_context(context)` is that it will reflect the
    // current values of the discrete `context`. Below, we are returning the
    // updated state into `DiscreteValues<T>* out`, but that doesn't yet update
    // the discrete state. Consequently we must mark the continuous time and
    // state updaters out of date in case `get_uptodate_continuous_context()` is
    // called before the time and discrete state in `context` are updated.
    time_updater_->get_mutable_cache_entry_value(context).mark_out_of_date();
    state_updater_->get_mutable_cache_entry_value(context).mark_out_of_date();

    Eigen::VectorBlock<VectorX<T>> discrete_state_vector =
        out->get_mutable_value();
    continuous_context.get_continuous_state_vector().CopyToPreSizedVector(
        &discrete_state_vector);
  }

  // Get the continuous context with up-to-date time, state, parameters, and
  // fixed input port values (only ones contained in `input_dependencies`,
  // `nullptr` means all inputs are required).
  const Context<T>& get_uptodate_continuous_context(
      const ContextBase& context,
      const std::vector<InputPortIndex>* input_dependencies = nullptr) const {
    time_updater_->EvalAbstract(context);
    state_updater_->EvalAbstract(context);
    parameters_updater_->EvalAbstract(context);
    accuracy_updater_->EvalAbstract(context);
    if (input_dependencies) {
      for (InputPortIndex i : *input_dependencies) {
        input_port_value_updaters_[i]->EvalAbstract(context);
      }
    } else {
      for (int i = 0; i < ssize(input_port_value_updaters_); ++i) {
        input_port_value_updaters_[i]->EvalAbstract(context);
      }
    }
    return get_mutable_continuous_context(context);
  }

  Context<T>& get_mutable_continuous_context(const ContextBase& context) const {
    return *get_mutable_integrator(context).get_mutable_context();
  }

  IntegratorBase<T>& get_mutable_integrator(const ContextBase& context) const {
    return integrator_cache_entry_->get_mutable_cache_entry_value(context)
        .template GetMutableValueOrThrow<IntegratorBase<T>>();
  }

  template <typename Func>
  static ValueProducer UpdateFuncWrapper(Func&& lambda) {
    return ValueProducer(std::function<void(const Context<T>&, DummyValue*)>(
        std::forward<Func>(lambda)));
  }

  std::string GetUnsupportedScalarConversionMessage(
      const std::type_info& source_type,
      const std::type_info& destination_type) const override {
    // Start with the default message for this system.
    std::stringstream result;
    result << LeafSystem<T>::GetUnsupportedScalarConversionMessage(
        source_type, destination_type);

    // Append extra details.
    std::vector<std::string> causes;
    if (!continuous_system_->get_system_scalar_converter().IsConvertible(
            destination_type, source_type)) {
      causes.push_back(fmt::format(
          "the continuous-time system of type {} does not support "
          "scalar conversion to type {}",
          // continuous_system_ contains source_type in its template type.
          NiceTypeName::Get(*continuous_system_),
          NiceTypeName::Get(destination_type)));
    }
    if (destination_type == typeid(symbolic::Expression) &&
        !IsScalarTypeSupportedByIntegrator<symbolic::Expression>(
            integrator_config_.integration_scheme)) {
      causes.push_back(fmt::format(
          "the integration scheme '{}' does not support scalar type {}",
          integrator_config_.integration_scheme,
          NiceTypeName::Get<symbolic::Expression>()));
    }
    if (!causes.empty()) {
      result << fmt::format(" (because {})", fmt::join(causes, " and "));
    }

    return std::move(result).str();
  }

  const std::shared_ptr<const System<T>> continuous_system_;
  const double time_period_;
  const SimulatorConfig integrator_config_;

  const CacheEntry* integrator_cache_entry_{};

  const CacheEntry* time_updater_{};
  const CacheEntry* state_updater_{};
  const CacheEntry* parameters_updater_{};
  const CacheEntry* accuracy_updater_{};
  std::vector<const CacheEntry*> input_port_value_updaters_{};
};

}  // namespace

template <typename T>
std::unique_ptr<LinearSystem<T>> DiscreteTimeApproximation(
    const LinearSystem<T>& system, double time_period) {
  // Check that the original system is continuous.
  DRAKE_THROW_UNLESS(system.IsDifferentialEquationSystem());
  // Check that the discrete time_period is greater than zero.
  DRAKE_THROW_UNLESS(time_period > 0);

  const int ns = system.num_states();
  const int ni = system.num_inputs();

  Eigen::MatrixXd M(ns + ni, ns + ni);
  M << system.A(), system.B(), Eigen::MatrixXd::Zero(ni, ns + ni);

  Eigen::MatrixXd Md = (M * time_period).exp();

  auto Ad = Md.block(0, 0, ns, ns);
  auto Bd = Md.block(0, ns, ns, ni);
  auto& Cd = system.C();
  auto& Dd = system.D();

  return std::make_unique<LinearSystem<T>>(Ad, Bd, Cd, Dd, time_period);
}

template <typename T>
std::unique_ptr<AffineSystem<T>> DiscreteTimeApproximation(
    const AffineSystem<T>& system, double time_period) {
  // Check that the original system is continuous.
  DRAKE_THROW_UNLESS(system.IsDifferentialEquationSystem());
  // Check that the discrete time_period is greater than zero.
  DRAKE_THROW_UNLESS(time_period > 0);

  const int ns = system.num_states();
  const int ni = system.num_inputs();

  Eigen::MatrixXd M(ns + ni + 1, ns + ni + 1);
  M << system.A(), system.B(), system.f0(),
      Eigen::MatrixXd::Zero(ni + 1, ns + ni + 1);

  Eigen::MatrixXd Md = (M * time_period).exp();

  auto Ad = Md.block(0, 0, ns, ns);
  auto Bd = Md.block(0, ns, ns, ni);
  auto f0d = Md.block(0, ns + ni, ns, 1);
  auto& Cd = system.C();
  auto& Dd = system.D();
  auto& y0d = system.y0();

  return std::make_unique<AffineSystem<T>>(Ad, Bd, f0d, Cd, Dd, y0d,
                                           time_period);
}

template <typename T>
std::unique_ptr<System<T>> DiscreteTimeApproximation(
    std::shared_ptr<const System<T>> system, double time_period,
    const SimulatorConfig& integrator_config) {
  return std::make_unique<DiscreteTimeSystem<T>>(std::move(system), time_period,
                                                 integrator_config);
}

template <typename T>
std::unique_ptr<System<T>> DiscreteTimeApproximation(
    const System<T>& system, double time_period,
    const SimulatorConfig& integrator_config) {
  return std::make_unique<DiscreteTimeSystem<T>>(
      std::shared_ptr<const System<T>>(
          /* managed object = */ std::shared_ptr<void>{},
          /* stored pointer = */ &system),
      time_period, integrator_config);
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    static_cast<std::unique_ptr<::drake::systems::LinearSystem<T>> (*)(
        const ::drake::systems::LinearSystem<T>&, double)>(
        &::drake::systems::DiscreteTimeApproximation<T>),
    static_cast<std::unique_ptr<::drake::systems::AffineSystem<T>> (*)(
        const ::drake::systems::AffineSystem<T>&, double)>(
        &::drake::systems::DiscreteTimeApproximation<T>),
    static_cast<std::unique_ptr<System<T>> (*)(std::shared_ptr<const System<T>>,
                                               double, const SimulatorConfig&)>(
        &DiscreteTimeApproximation<T>),
    static_cast<std::unique_ptr<System<T>> (*)(const System<T>&, double,
                                               const SimulatorConfig&)>(
        &DiscreteTimeApproximation<T>)));

}  // namespace systems
}  // namespace drake
