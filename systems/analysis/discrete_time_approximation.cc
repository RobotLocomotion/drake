#include "drake/systems/analysis/discrete_time_approximation.h"

#include <functional>
#include <memory>

#include <fmt/core.h>
#include <unsupported/Eigen/MatrixFunctions>

#include "drake/systems/analysis/simulator_config_functions.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

template <typename T>
std::unique_ptr<LinearSystem<T>> DiscreteTimeApproximation(
    const LinearSystem<T>& system, double time_period) {
  DRAKE_THROW_UNLESS(system.time_period() == 0.0);
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
  DRAKE_THROW_UNLESS(system.time_period() == 0.0);
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

namespace {

struct DummyValue {};

template <typename T>
class DiscreteTimeSystem final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteTimeSystem);

  DiscreteTimeSystem(std::shared_ptr<const System<T>> system,
                     double time_period, double time_offset,
                     const SimulatorConfig& integrator_config)
      : LeafSystem<T>(SystemTypeTag<DiscreteTimeSystem>{}),
        continuous_system_(std::move(system)),
        time_period_(time_period),
        time_offset_(time_offset),
        integrator_config_(integrator_config) {
    DRAKE_THROW_UNLESS(continuous_system_ != nullptr);
    DRAKE_THROW_UNLESS(continuous_system_->num_continuous_states() &&
                       !continuous_system_->num_discrete_state_groups() &&
                       !continuous_system_->num_abstract_states());
    DRAKE_THROW_UNLESS(time_period_ > 0);

    this->get_mutable_system_scalar_converter().RemoveUnlessAlsoSupportedBy(
        continuous_system_->get_system_scalar_converter());

    this->Initialize();
  }

  // Scalar-converting copy constructor. See @ref system_scalar_conversion.
  template <typename U>
  explicit DiscreteTimeSystem(const DiscreteTimeSystem<U>& other)
      : DiscreteTimeSystem<T>(
            other.continuous_system_->template ToScalarType<T>(),
            other.time_period_, other.time_offset_, other.integrator_config_) {}

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
    }

    // Declare output ports.
    for (int j = 0; j < continuous_system_->num_output_ports(); ++j) {
      const OutputPort<T>& port = continuous_system_->get_output_port(j);
      std::vector<bool> input_dependency_mask = FindInputDependencies(j);

      std::set<DependencyTicket> prerequisites_of_calc{
          SystemBase::all_sources_except_input_ports_ticket()};
      for (int i = 0; i < num_input_ports; ++i) {
        if (input_dependency_mask[i]) {
          prerequisites_of_calc.insert(
              this->input_port_ticket(InputPortIndex(i)));
        }
      }

      if (port.get_data_type() == PortDataType::kVectorValued) {
        this->DeclareVectorOutputPort(
            port.get_name(), port.size(),
            [this, &port, input_dependency_mask](const Context<T>& context,
                                                 BasicVector<T>* out) {
              const Context<T>& continuous_context =
                  this->get_uptodate_continuous_context(context,
                                                        input_dependency_mask);
              out->SetFromVector(port.Eval(continuous_context));
            },
            prerequisites_of_calc);
      } else if (port.get_data_type() == PortDataType::kAbstractValued) {
        this->DeclareAbstractOutputPort(
            port.get_name(),
            [&port] {
              return port.Allocate();
            },
            [this, &port, input_dependency_mask](const Context<T>& context,
                                                 AbstractValue* out) {
              const Context<T>& continuous_context =
                  this->get_uptodate_continuous_context(context,
                                                        input_dependency_mask);
              out->SetFrom(
                  port.template Eval<AbstractValue>(continuous_context));
            },
            prerequisites_of_calc);
      }
    }

    // Declare state.
    this->DeclareDiscreteState(continuous_system_->num_continuous_states());
    this->DeclarePeriodicDiscreteUpdateEvent(
        time_period_, time_offset_, &DiscreteTimeSystem<T>::DiscreteUpdate);

    // Declare parameters.
    auto continuous_context = continuous_system_->CreateDefaultContext();
    for (int k = 0; k < continuous_context->num_numeric_parameter_groups();
         ++k) {
      this->DeclareNumericParameter(
          continuous_context->get_numeric_parameter(k));
    }
    for (int k = 0; k < continuous_context->num_abstract_parameters(); ++k) {
      this->DeclareAbstractParameter(
          continuous_context->get_abstract_parameter(k));
    }

    // Create a integrator based on the integrator config.
    Simulator<T> simulator(*continuous_system_);
    ApplySimulatorConfig(integrator_config_, &simulator);
    IntegratorBase<T>& integrator = simulator.get_mutable_integrator();
    integrator.reset_context(std::move(continuous_context));
    integrator.Initialize();

    // Store the mutable integrator in a cache.
    integrator_cache_entry_ = &this->DeclareCacheEntry(
        "integrator",
        ValueProducer(integrator, &ValueProducer::NoopCalc),
        {SystemBase::nothing_ticket()});

    // These dummy-valued cache entries update the continuous system context.
    time_updater_ = &this->DeclareCacheEntry(
        "update time",
        UpdateFuncWrapper([this](const Context<T>& context, DummyValue*) {
          this->get_continuous_context(context).SetTime(context.get_time());
        }),
        {this->time_ticket()});
    state_updater_ = &this->DeclareCacheEntry(
        "update state",
        UpdateFuncWrapper([this](const Context<T>& context, DummyValue*) {
          this->get_continuous_context(context).SetContinuousState(
              context.get_discrete_state_vector().value());
        }),
        {this->xd_ticket()});
    parameters_updater_ = &this->DeclareCacheEntry(
        "update parameters",
        UpdateFuncWrapper([this](const Context<T>& context, DummyValue*) {
          this->get_continuous_context(context)
              .get_mutable_parameters()
              .SetFrom(context.get_parameters());
        }),
        {this->all_parameters_ticket()});
    accuracy_updater_ = &this->DeclareCacheEntry(
        "update accuracy",
        UpdateFuncWrapper([this](const Context<T>& context, DummyValue*) {
          this->get_continuous_context(context).SetAccuracy(
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
              this->get_continuous_context(context).FixInputPort(
                  i, input_port.template Eval<AbstractValue>(context));
            }
          }),
          {this->input_port_ticket(InputPortIndex(i))});
    }
  }

  // Callback performing the periodic discrete update.
  void DiscreteUpdate(const Context<T>& context, DiscreteValues<T>* out) const {
    Context<T>& continuous_context =
        this->get_uptodate_continuous_context(context);
    const T current_time = continuous_context.get_time();

    IntegratorBase<T>& integrator = this->get_integrator(context);
    integrator.IntegrateWithMultipleStepsToTime(current_time + time_period_);
    continuous_context.SetTime(current_time);

    Eigen::VectorBlock<VectorX<T>> discrete_state_vector =
        out->get_mutable_value();
    continuous_context.get_continuous_state_vector().CopyToPreSizedVector(
        &discrete_state_vector);
  }

  // Get the continuous context with up-to-date time, state, parameters, and
  // fixed input port values (only ones where input_dependency_mask[i]==true,
  // std::nullopt means all inputs are required).
  Context<T>& get_uptodate_continuous_context(
      const ContextBase& context,
      const std::optional<std::vector<bool>>& input_dependency_mask =
          std::nullopt) const {
    time_updater_->EvalAbstract(context);
    state_updater_->EvalAbstract(context);
    parameters_updater_->EvalAbstract(context);
    accuracy_updater_->EvalAbstract(context);
    for (int i = 0; i < ssize(input_port_value_updaters_); ++i) {
      if (input_dependency_mask && !(*input_dependency_mask)[i]) continue;
      input_port_value_updaters_[i]->EvalAbstract(context);
    }
    return get_continuous_context(context);
  }

  Context<T>& get_continuous_context(const ContextBase& context) const {
    return *get_integrator(context).get_mutable_context();
  }

  IntegratorBase<T>& get_integrator(const ContextBase& context) const {
    return integrator_cache_entry_->get_mutable_cache_entry_value(context)
        .template GetMutableValueOrThrow<IntegratorBase<T>>();
  }

  std::vector<bool> FindInputDependencies(int output_port_index) {
    DRAKE_ASSERT(0 <= output_port_index &&
                 output_port_index < continuous_system_->num_output_ports());
    const int num_input_ports = continuous_system_->num_input_ports();
    std::vector<bool> input_dependency_mask(num_input_ports);
    for (int i = 0; i < num_input_ports; ++i) {
      input_dependency_mask[i] =
          continuous_system_->HasDirectFeedthrough(i, output_port_index);
    }
    return input_dependency_mask;
  }

  template <typename Func>
  static ValueProducer UpdateFuncWrapper(Func&& lambda) {
    return ValueProducer(std::function<void(const Context<T>&, DummyValue*)>(
        std::forward<Func>(lambda)));
  }

  const std::shared_ptr<const System<T>> continuous_system_;
  const double time_period_;
  const double time_offset_;
  const SimulatorConfig integrator_config_;

  const CacheEntry* integrator_cache_entry_;

  const CacheEntry* time_updater_;
  const CacheEntry* state_updater_;
  const CacheEntry* parameters_updater_;
  const CacheEntry* accuracy_updater_;
  std::vector<const CacheEntry*> input_port_value_updaters_;
};

}  // namespace

namespace scalar_conversion {
template <> struct Traits<DiscreteTimeSystem> : public NonSymbolicTraits {};
}  // namespace scalar_conversion

template <typename T>
std::unique_ptr<System<T>> DiscreteTimeApproximation(
    std::shared_ptr<const System<T>> system, double time_period,
    double time_offset, const SimulatorConfig& integrator_config) {
  return std::make_unique<DiscreteTimeSystem<T>>(
      system, time_period, time_offset, integrator_config);
}

template <typename T>
std::unique_ptr<System<T>> DiscreteTimeApproximation(
    const System<T>& system, double time_period, double time_offset,
    const SimulatorConfig& integrator_config) {
  return std::make_unique<DiscreteTimeSystem<T>>(
      std::shared_ptr<const System<T>>(
          /* managed object = */ std::shared_ptr<void>{},
          /* stored pointer = */ &system),
      time_period, time_offset, integrator_config);
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    static_cast<std::unique_ptr<LinearSystem<T>> (*)(const LinearSystem<T>&,
                                                     double)>(
        &DiscreteTimeApproximation<T>),
    static_cast<std::unique_ptr<AffineSystem<T>> (*)(const AffineSystem<T>&,
                                                     double)>(
        &DiscreteTimeApproximation<T>)
));

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS((
    static_cast<std::unique_ptr<System<T>> (*)(std::shared_ptr<const System<T>>,
                                               double, double,
                                               const SimulatorConfig&)>(
        &DiscreteTimeApproximation<T>),
    static_cast<std::unique_ptr<System<T>> (*)(const System<T>&, double, double,
                                               const SimulatorConfig&)>(
        &DiscreteTimeApproximation<T>)
));

}  // namespace systems
}  // namespace drake
