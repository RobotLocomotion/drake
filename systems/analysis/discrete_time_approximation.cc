#include "drake/systems/analysis/discrete_time_approximation.h"

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

  DiscreteTimeSystem(std::unique_ptr<System<T>> system, double time_period,
                     double time_offset,
                     const SimulatorConfig& integrator_config)
      : LeafSystem<T>(SystemTypeTag<DiscreteTimeSystem>{}),
        continuous_system_(std::move(system)),
        time_period_(time_period),
        time_offset_(time_offset),
        integrator_config_(integrator_config) {
    this->Initialize();
  }

  ~DiscreteTimeSystem() override = default;

  // Scalar-converting copy constructor. See @ref system_scalar_conversion.
  template <typename U>
  explicit DiscreteTimeSystem(const DiscreteTimeSystem<U>& other)
      : DiscreteTimeSystem<T>(
            other.continuous_system_->template ToScalarType<T>(),
            other.time_period_, other.time_offset_, other.integrator_config_) {}

 private:
  template <typename U>
  friend class DiscreteTimeSystem;

  // This function is called by the constructor.
  void Initialize() {
    // Set name of this system.
    const std::string& name = continuous_system_->get_name();
    this->set_name(name.empty() ? "discrete-time approximation"
                                : "discrete-time approximated " + name);

    // Create a cache entry storing a flag. If false, do not evaluate the inputs
    // of this system.
    eval_input_flag_cache_entry_ = &this->DeclareCacheEntry(
        "evaluate input flag", ValueProducer(true, &ValueProducer::NoopCalc),
        {SystemBase::nothing_ticket()});

    // Create a continuous context context model value.
    auto continuous_context = continuous_system_->CreateDefaultContext();

    // Store the continuous system context in a cache where we can safely modify
    // it without a (non-thread-safe) mutable member.
    continuous_context_cache_entry_ = &this->DeclareCacheEntry(
        "continuous system context", *continuous_context,
        &DiscreteTimeSystem<T>::CopyToContinuousContext,
        {SystemBase::all_sources_ticket()});

    // Create a integrator based on the integration scheme.
    Simulator<T> simulator(*continuous_system_);
    ApplySimulatorConfig(integrator_config_, &simulator);
    auto& integrator = simulator.get_mutable_integrator();
    integrator.reset_context(nullptr);
    // Store the integrator in a cache to safely call integration procedures
    // without requiring a mutable member, as the integrator manages mutable
    // scratch memory.
    integrator_cache_entry_ = &this->DeclareCacheEntry(
        "integrator", ValueProducer(integrator, &ValueProducer::NoopCalc),
        {SystemBase::nothing_ticket()});

    // Declare input ports.
    for (int i = 0; i < continuous_system_->num_input_ports(); ++i) {
      const InputPort<T>& port = continuous_system_->get_input_port(i);

      if (port.get_data_type() == PortDataType::kVectorValued) {
        this->DeclareVectorInputPort(port.get_name(), port.size(),
                                     port.get_random_type());
      } else if (port.get_data_type() == PortDataType::kAbstractValued) {
        this->DeclareAbstractInputPort(port.get_name(),
                                       /* model value */ *port.Allocate());
      }
    }

    // Declare output ports.
    for (int i = 0; i < continuous_system_->num_output_ports(); ++i) {
      const OutputPort<T>& port = continuous_system_->get_output_port(i);

      bool input_dependent = continuous_system_->HasDirectFeedthrough(i);

      DependencyTicket prerequisites_of_calc =
          input_dependent ? SystemBase::all_sources_ticket()
                          : SystemBase::all_sources_except_input_ports_ticket();

      if (port.get_data_type() == PortDataType::kVectorValued) {
        this->DeclareVectorOutputPort(
            port.get_name(), port.size(),
            [this, &port, input_dependent](const Context<T>& context,
                                           BasicVector<T>* out) {
              out->SetFromVector(port.Eval(
                  this->get_continuous_context(context, input_dependent)));
            },
            {prerequisites_of_calc});
      } else if (port.get_data_type() == PortDataType::kAbstractValued) {
        this->DeclareAbstractOutputPort(
            port.get_name(),
            [&port] {
              return port.Allocate();
            },
            [this, &port, input_dependent](const Context<T>& context,
                                           AbstractValue* out) {
              out->SetFrom(port.template Eval<AbstractValue>(
                  this->get_continuous_context(context, input_dependent)));
            },
            {prerequisites_of_calc});
      }
    }

    // Declare parameters.
    for (int i = 0; i < continuous_context->num_numeric_parameter_groups();
         ++i) {
      this->DeclareNumericParameter(
          continuous_context->get_numeric_parameter(i));
    }
    for (int i = 0; i < continuous_context->num_abstract_parameters(); ++i) {
      this->DeclareAbstractParameter(
          continuous_context->get_abstract_parameter(i));
    }

    // Declare state.
    this->DeclareDiscreteState(continuous_system_->num_continuous_states());
    this->DeclarePeriodicDiscreteUpdateEvent(
        time_period_, time_offset_, &DiscreteTimeSystem<T>::DiscreteUpdate);
  }

  // Callback to perform the periodic discrete updates.
  void DiscreteUpdate(const Context<T>& context, DiscreteValues<T>* out) const {
    // Get the integrator from cache.
    IntegratorBase<T>* integrator = get_mutable_integrator(context);

    // Get the mutable up-to-date continuous-time context from cache so that the
    // integrator can integrate on it.
    Context<T>* continuous_context = get_mutable_continuous_context(context);

    // Attach continuous-time context to the integrator (will only run once).
    if (integrator->get_mutable_context() == nullptr) {
      integrator->reset_context(continuous_context);
      integrator->set_fixed_step_mode(true);
      integrator->Initialize();
    }

    // Perform the integration.
    integrator->IntegrateWithMultipleStepsToTime(
        continuous_context->get_time() + time_period_);

    out->get_mutable_vector().SetFromVector(
        continuous_context->get_continuous_state_vector().CopyToVector());
  }

  // Callback to make the continuous-context up-to-date. Avoids evaluating the
  // input ports of this system if eval-input flag is set to false.
  void CopyToContinuousContext(const Context<T>& context,
                               Context<T>* continuous_context) const {
    // Copy time.
    continuous_context->SetTime(context.get_time());
    // Copy state.
    continuous_context->SetContinuousState(
        context.get_discrete_state_vector().value());
    // Copy parameters.
    continuous_context->get_mutable_parameters().SetFrom(
        context.get_parameters());
    // Copy accuracy.
    continuous_context->SetAccuracy(context.get_accuracy());
    // Copy fixed input port values (if eval_input_flag is true).
    if (!get_eval_input_flag(context)) return;
    for (int i = 0; i < continuous_system_->num_input_ports(); ++i) {
      const AbstractValue* input_val = this->EvalAbstractInput(context, i);
      if (input_val != nullptr)
        continuous_context->FixInputPort(i, *input_val);
    }
  }

  // Returns the up-to-date contiunous context, and avoids evaluating the input
  // ports of this system if input_dependent==false.
  const Context<T>& get_continuous_context(const Context<T>& context,
                                           bool input_dependent) const {
    // If the continuous-time context was previously evaluated without input,
    // but we need the input here, then the cached continuous-time context is
    // out of date.
    if (!get_eval_input_flag(context) && input_dependent) {
      continuous_context_cache_entry_->get_mutable_cache_entry_value(context)
          .mark_out_of_date();
    }
    // Mark the eval-input flag so that CopyToContinuousContext() won't evaluate
    // the input ports of this system unnecessarily.
    if (get_eval_input_flag(context) != input_dependent) {
      set_eval_input_flag(context, input_dependent);
    }
    // Get the continuous-time context. This will either call
    // CopyToContinuousContext() or get the value directly from cache.
    return continuous_context_cache_entry_->Eval<Context<T>>(context);
  }

  // Returns the mutable up-to-date contiunous context, and avoids evaluating
  // the input ports of this system if input_dependent==false.
  Context<T>* get_mutable_continuous_context(
      const Context<T>& context, bool input_dependent = true) const {
    // Ensure that the contiuous-time context is up to date.
    get_continuous_context(context, input_dependent);
    // Return the mutable contiuous-time context.
    CacheEntryValue& cache_entry_value =
        continuous_context_cache_entry_->get_mutable_cache_entry_value(context);
    cache_entry_value.mark_out_of_date();
    return &cache_entry_value.template GetMutableValueOrThrow<Context<T>>();
  }

  // Returns the non-const integrator in the cache able to perform integration.
  IntegratorBase<T>* get_mutable_integrator(const Context<T>& context) const {
    return &integrator_cache_entry_->get_mutable_cache_entry_value(context)
                .template GetMutableValueOrThrow<IntegratorBase<T>>();
  }

  // Read and write the eval-input flag.
  bool get_eval_input_flag(const Context<T>& context) const {
    return eval_input_flag_cache_entry_->Eval<bool>(context);
  }
  void set_eval_input_flag(const Context<T>& context, bool flag) const {
    CacheEntryValue& cache_entry_value =
        eval_input_flag_cache_entry_->get_mutable_cache_entry_value(context);
    cache_entry_value.mark_out_of_date();
    cache_entry_value.set_value(flag);
  }

  const std::unique_ptr<const System<T>> continuous_system_;
  const double time_period_;
  const double time_offset_;
  const SimulatorConfig integrator_config_;

  const CacheEntry* continuous_context_cache_entry_;
  const CacheEntry* integrator_cache_entry_;
  const CacheEntry* eval_input_flag_cache_entry_;
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
  DRAKE_THROW_UNLESS(!system.num_discrete_state_groups() &&
                     !system.num_abstract_states());
  DRAKE_THROW_UNLESS(time_period > 0);

  return std::make_unique<DiscreteTimeSystem<T>>(
      system.Clone(), time_period, time_offset, integrator_config);
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    (static_cast<std::unique_ptr<LinearSystem<T>> (*)(const LinearSystem<T>&,
                                                      double)>(
        &DiscreteTimeApproximation<T>),
     static_cast<std::unique_ptr<AffineSystem<T>> (*)(const AffineSystem<T>&,
                                                      double)>(
        &DiscreteTimeApproximation<T>)
));

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS((
    static_cast<std::unique_ptr<System<T>> (*)(const System<T>&, double, double,
                                               const SimulatorConfig&)>(
        &DiscreteTimeApproximation<T>)
));

}  // namespace systems
}  // namespace drake
