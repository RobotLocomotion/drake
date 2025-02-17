#include "drake/systems/analysis/discrete_time_approximation.h"

#include <functional>
#include <memory>

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
        integrator_config_(integrator_config),
        depend_on_all_input_mask_(continuous_system_->num_input_ports(), true) {
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
    // Set the name of this system.
    const std::string& name = continuous_system_->get_name();
    this->set_name(name.empty() ? "DiscreteTimeApproximation"
                                : "DiscreteTimeApproximated " + name);

    // Create a continuous system context model value.
    auto continuous_context = continuous_system_->CreateDefaultContext();

    // Store the mutable continuous system context in a cache.
    std::function<void(const Context<T>&, Context<T>*)> calc =
        [](const Context<T>& context, Context<T>* cont_context) {
          cont_context->SetTime(context.get_time());
          cont_context->SetAccuracy(context.get_accuracy());
          cont_context->SetContinuousState(
              context.get_discrete_state_vector().value());
              cont_context->get_mutable_parameters().SetFrom(
              context.get_parameters());
        };
    continuous_context_cache_entry_ = &this->DeclareCacheEntry(
        "continuous system context", ValueProducer(*continuous_context, calc),
        {SystemBase::all_sources_except_input_ports_ticket()});

    // Create a integrator based on the integrator config.
    Simulator<T> simulator(*continuous_system_);
    ApplySimulatorConfig(integrator_config_, &simulator);
    auto& integrator = simulator.get_mutable_integrator();
    integrator.reset_context(nullptr);
    // Store the mutable integrator in a cache.
    integrator_cache_entry_ = &this->DeclareCacheEntry(
        "integrator", ValueProducer(integrator, &ValueProducer::NoopCalc),
        {SystemBase::nothing_ticket()});

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
              const Context<T>& cont_context =
                  this->get_continuous_context(context, input_dependency_mask);
              out->SetFromVector(port.Eval(cont_context));
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
              const Context<T>& cont_context =
                  this->get_continuous_context(context, input_dependency_mask);
              out->SetFrom(
                  port.template Eval<AbstractValue>(cont_context));
            },
            prerequisites_of_calc);
      }
    }

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

    // Declare state.
    this->DeclareDiscreteState(continuous_system_->num_continuous_states());
    this->DeclarePeriodicDiscreteUpdateEvent(
        time_period_, time_offset_, &DiscreteTimeSystem<T>::DiscreteUpdate);

    // Create a vector<bool> in cache tracking if the inputs port values copied
    // to the continuous context are up-to-date.
    std::function<void(const Context<T>&, std::vector<bool>*)> reset_all_flags =
        [](const Context<T>&, std::vector<bool>* flags) {
          std::fill(flags->begin(), flags->end(), false);
        };
    input_uptodate_flags_cache_entry_ = &this->DeclareCacheEntry(
        "input values up-to-date flags",
        ValueProducer(std::vector<bool>(num_input_ports, false),
                      reset_all_flags),
        {SystemBase::all_input_ports_ticket()});
  }

  // Callback performing the periodic discrete update.
  void DiscreteUpdate(const Context<T>& context, DiscreteValues<T>* out) const {
    IntegratorBase<T>& integrator = this->get_integrator(context);
    Context<T>* continuous_context = this->get_mutable_continuous_context(
        context, depend_on_all_input_mask_);

    // Attach continuous context to the integrator (this will only run once).
    if (integrator.get_mutable_context() == nullptr) {
      integrator.reset_context(continuous_context);
      integrator.Initialize();
    }

    integrator.IntegrateWithMultipleStepsToTime(continuous_context->get_time() +
                                                time_period_);

    Eigen::VectorBlock<VectorX<T>> discrete_state_vector =
        out->get_mutable_value();
    continuous_context->get_continuous_state_vector().CopyToPreSizedVector(
        &discrete_state_vector);
  }

  // Returns the continuous context with up-to-date time, state, parameters, and
  // input port values (only ones where input_dependency_mask[i] == true).
  const Context<T>& get_continuous_context(
      const Context<T>& context,
      const std::vector<bool>& input_dependency_mask) const {
    // Make time, state, and parameters up-to-date.
    continuous_context_cache_entry_->Eval<Context<T>>(context);

    // Get the continuous context so we can copy the input port values to it.
    auto& cache_entry_value =
        continuous_context_cache_entry_->get_mutable_cache_entry_value(context);
    cache_entry_value.mark_out_of_date();
    auto& continuous_context =
        cache_entry_value.template GetMutableValueOrThrow<Context<T>>();
    cache_entry_value.mark_up_to_date();

    // Copy only the input port values where it is
    // 1) requested as in input_dependency_mask[i]==true, but
    // 2) the input port values on the continuous context is not up-to-date as
    //    in input_uptodate_flags[i]==false.
    auto& input_uptodate_flags = this->get_input_uptodate_flags(context);
    for (int i = 0; i < context.num_input_ports(); ++i) {
      if (input_dependency_mask[i] && !input_uptodate_flags[i]) {
        const AbstractValue* input_val = this->EvalAbstractInput(context, i);
        if (input_val != nullptr) {
          continuous_context.FixInputPort(i, *input_val);
        }
      }
    }
    this->bitwiseor_assign_input_uptodate_flags(context, input_dependency_mask);

    return continuous_context;
  }

  // Returns the mutable continuous context with up-to-date time, state,
  // parameters, and input port values.
  Context<T>* get_mutable_continuous_context(
      const Context<T>& context,
      const std::vector<bool>& input_dependency_mask) const {
    get_continuous_context(context, input_dependency_mask);
    CacheEntryValue& cache_entry_value =
        continuous_context_cache_entry_->get_mutable_cache_entry_value(context);
    cache_entry_value.mark_out_of_date();
    return &cache_entry_value.template GetMutableValueOrThrow<Context<T>>();
  }

  IntegratorBase<T>& get_integrator(const Context<T>& context) const {
    return integrator_cache_entry_->get_mutable_cache_entry_value(context)
        .template GetMutableValueOrThrow<IntegratorBase<T>>();
  }

  const std::vector<bool>& get_input_uptodate_flags(
      const Context<T>& context) const {
    return input_uptodate_flags_cache_entry_->Eval<std::vector<bool>>(context);
  }

  void bitwiseor_assign_input_uptodate_flags(
      const Context<T>& context, const std::vector<bool>& other) const {
    auto& cache_entry_value =
        input_uptodate_flags_cache_entry_->get_mutable_cache_entry_value(
            context);
    cache_entry_value.mark_out_of_date();
    std::vector<bool>& flags =
        cache_entry_value.template GetMutableValueOrThrow<std::vector<bool>>();
    DRAKE_ASSERT(flags.size() == other.size());
    std::transform(flags.cbegin(), flags.cend(), other.cbegin(), flags.begin(),
                   std::bit_or<bool>());
    cache_entry_value.mark_up_to_date();
  }

  std::vector<bool> FindInputDependencies(int output_port_index) {
    DRAKE_ASSERT(0 <= output_port_index &&
                 output_port_index < continuous_system_->num_output_ports());
    const int num_inputs = continuous_system_->num_input_ports();
    std::vector<bool> input_dependency_mask(num_inputs);
    for (int i = 0; i < num_inputs; ++i) {
      input_dependency_mask[i] =
          continuous_system_->HasDirectFeedthrough(i, output_port_index);
    }
    return input_dependency_mask;
  }

  const std::shared_ptr<const System<T>> continuous_system_;
  const double time_period_;
  const double time_offset_;
  const SimulatorConfig integrator_config_;

  const std::vector<bool> depend_on_all_input_mask_;

  const CacheEntry* continuous_context_cache_entry_;
  const CacheEntry* integrator_cache_entry_;
  const CacheEntry* input_uptodate_flags_cache_entry_;
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

template <typename T>
std::unique_ptr<System<T>> DiscreteTimeApproximation(
    System<T>&& system, double time_period, double time_offset,
    const SimulatorConfig& integrator_config) {
  return std::make_unique<DiscreteTimeSystem<T>>(
      system.Clone(), time_period, time_offset, integrator_config);
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
        &DiscreteTimeApproximation<T>),
    static_cast<std::unique_ptr<System<T>> (*)(System<T>&&, double, double,
                                               const SimulatorConfig&)>(
        &DiscreteTimeApproximation<T>)
));

}  // namespace systems
}  // namespace drake
