#pragma once

#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/input_port_evaluator_interface.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace systems {

/// A description of a discrete-time event, which is passed from the Simulator
/// to the recipient System's appropriate event handler method.
template <typename T>
struct DiscreteEvent {
  typedef std::function<void(const Context<T>&)> PublishCallback;
  typedef std::function<void(const Context<T>&, DiscreteState<T>*)>
      CalcUpdateCallback;

  enum ActionType {
    kUnknownAction = 0,  // A default value that causes the handler to abort.
    kPublishAction = 1,  // On a publish action, state does not change.
    kUpdateAction = 2,   // On an update action, discrete state may change.
  };

  /// The type of action the system must take in response to the event.
  ActionType action{kUnknownAction};

  /// An optional callback, supplied by the recipient, to carry out a
  /// kPublishAction. If nullptr, Publish will be used.
  PublishCallback do_publish{nullptr};

  /// An optional callback, supplied by the recipient, to carry out a
  /// kUpdateAction. If nullptr, DoCalcDiscreteVariableUpdates() will be used.
  CalcUpdateCallback do_calc_update{nullptr};
};

/// A token that identifies the next sample time at which a System must
/// perform some actions, and the actions that must be performed.
template <typename T>
struct UpdateActions {
  /// When the System next requires a discrete action. If the System is
  /// not discrete, time should be set to infinity.
  T time{std::numeric_limits<T>::quiet_NaN()};

  /// The events that should occur when the sample time arrives.
  std::vector<DiscreteEvent<T>> events;
};

/// A superclass template for systems that receive input, maintain state, and
/// produce output of a given mathematical type T.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class System {
 public:
  virtual ~System() {}

  /// Evaluates to `true` if any of the inputs to the system is directly
  /// fed through to any of its outputs and `false` otherwise.
  ///
  /// By default we assume that there is direct feedthrough of values from
  /// every input port to every output port. This is a conservative assumption
  /// that ensures we detect and can prevent the formation of algebraic loops
  /// (implicit computations) in system diagrams. Any System for which none of
  /// the input ports ever feeds through to any of the output ports should
  /// override this method to return false.
  // TODO(amcastro-tri): Provide a more descriptive mechanism to specify
  // pairwise (input_port, output_port) feedthrough.
  virtual bool has_any_direct_feedthrough() const { return true; }

  /// Returns the number of input ports of the system.
  int get_num_input_ports() const {
    return static_cast<int>(input_ports_.size());
  }

  /// Returns the number of output ports of the system.
  int get_num_output_ports() const {
    return static_cast<int>(output_ports_.size());
  }

  /// Returns descriptors for all the input ports of this system.
  const std::vector<SystemPortDescriptor<T>>& get_input_ports() const {
    return input_ports_;
  }

  /// Returns the descriptor of the input port at index @p port_index.
  const SystemPortDescriptor<T>& get_input_port(int port_index) const {
    if (port_index >= get_num_input_ports()) {
      throw std::out_of_range("port number out of range.");
    }
    return input_ports_[port_index];
  }

  /// Returns the descriptor of the output port at index @p port_index.
  const SystemPortDescriptor<T>& get_output_port(int port_index) const {
    if (port_index >= get_num_output_ports()) {
      throw std::out_of_range("port number out of range.");
    }
    return output_ports_[port_index];
  }

  /// Returns descriptors for all the output ports of this system.
  const std::vector<SystemPortDescriptor<T>>& get_output_ports() const {
    return output_ports_;
  }

  /// Returns the total dimension of all of the input ports (as if they were
  /// muxed).
  int get_num_total_inputs() const {
    int count = 0;
    for (const auto& in : input_ports_) count += in.get_size();
    return count;
  }

  /// Returns the total dimension of all of the output ports (as if they were
  /// muxed).
  int get_num_total_outputs() const {
    int count = 0;
    for (const auto& out : output_ports_) count += out.get_size();
    return count;
  }

  /// Checks that @p output is consistent with the number and size of output
  /// ports declared by the system.
  /// @throw exception unless `output` is non-null and valid for this system.
  void CheckValidOutput(const SystemOutput<T>* output) const {
    DRAKE_THROW_UNLESS(output != nullptr);

    // Checks that the number of output ports in the system output is consistent
    // with the number of output ports declared by the System.
    DRAKE_THROW_UNLESS(output->get_num_ports() == get_num_output_ports());

    // Checks the validity of each output port.
    for (int i = 0; i < get_num_output_ports(); ++i) {
      // TODO(amcastro-tri): add appropriate checks for kAbstractValued ports
      // once abstract ports are implemented in 3164.
      if (get_output_port(i).get_data_type() == kVectorValued) {
        const VectorBase<T>* output_vector = output->get_vector_data(i);
        DRAKE_THROW_UNLESS(output_vector != nullptr);
        DRAKE_THROW_UNLESS(output_vector->size() ==
                           get_output_port(i).get_size());
      }
    }
  }

  /// Checks that @p context is consistent for this system.
  /// @throw exception unless `context` is valid for this system.
  void CheckValidContext(const Context<T>& context) const {
    // Checks that the number of input ports in the context is consistent with
    // the number of ports declared by the System.
    DRAKE_THROW_UNLESS(context.get_num_input_ports() ==
                       this->get_num_input_ports());

    // Checks that the size of the input ports in the context matches the
    // declarations made by the system.
    for (int i = 0; i < this->get_num_input_ports(); ++i) {
      context.VerifyInputPort(this->get_input_port(i));
    }
  }

  /// Returns a copy of the continuous state vector `xc` into an Eigen vector.
  VectorX<T> CopyContinuousStateVector(const Context<T>& context) const {
    DRAKE_ASSERT(context.get_continuous_state() != nullptr);
    return context.get_continuous_state()->CopyToVector();
  }

  /// Allocates a context, initialized with the correct numbers of concrete
  /// input ports and state variables for this System.  Since input port
  /// pointers are not owned by the context, they should simply be initialized
  /// to nullptr.
  virtual std::unique_ptr<Context<T>> AllocateContext() const = 0;

  /// Assigns default values to all elements of the state. Overrides must not
  /// change the number of state variables.
  virtual void SetDefaultState(const Context<T>& context,
                               State<T>* state) const = 0;

  // Sets Context fields to their default values.  User code should not
  // override.
  virtual void SetDefaults(Context<T>* context) const = 0;

  /// Allocates a context and sets its default values.
  std::unique_ptr<Context<T>> CreateDefaultContext() const {
    std::unique_ptr<Context<T>> context = AllocateContext();
    SetDefaults(context.get());
    return context;
  }

  /// Returns a default output, initialized with the correct number of
  /// concrete output ports for this System. @p context is provided as
  /// an argument to support some specialized use cases. Most typical
  /// System implementations should ignore it.
  virtual std::unique_ptr<SystemOutput<T>> AllocateOutput(
      const Context<T>& context) const = 0;

  /// Generates side-effect outputs such as terminal output, visualization,
  /// logging, plotting, and network messages. Other than computational cost,
  /// publishing has no effect on the progress of a simulation.
  /// Dispatches to DoPublish.
  ///
  /// This method is invoked by the Simulator at the start of each continuous
  /// integration step, after discrete variables have been updated to the values
  /// they will hold throughout the step. It will always be called at the start
  /// of the first step of a simulation (after initialization) and after the
  /// final simulation step (after a final update to discrete variables).
  void Publish(const Context<T>& context) const {
    DiscreteEvent<T> event;
    event.action = DiscreteEvent<T>::kPublishAction;
    Publish(context, event);
  }

  /// Generates the particular side-effect outputs requested by @p event,
  /// because the sample time requested by @p event has arrived. Dispatches to
  /// DoPublish by default, or to `event.do_publish` if provided.
  ///
  /// @note When publishing is scheduled at particular times, those times likely
  /// will not coincide with integrator step times. A Simulator may interpolate
  /// to generate a suitable Context, or it may adjust the integrator step size
  /// so that a step begins exactly at the next publication time. In the latter
  /// case the change in step size may affect the numerical result somewhat
  /// since a smaller integrator step produces a more accurate solution.
  void Publish(const Context<T>& context, const DiscreteEvent<T>& event) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    DRAKE_DEMAND(event.action == DiscreteEvent<T>::kPublishAction);
    if (event.do_publish == nullptr) {
      DoPublish(context);
    } else {
      event.do_publish(context);
    }
  }

  //----------------------------------------------------------------------------
  /// @name                        Evaluations
  /// Given the values in a Context, a Drake %System must be able to provide
  /// the results of particular computations needed for analysis and simulation
  /// of the %System. These results are maintained in a mutable cache within
  /// the Context so that a result need be computed only once, the first time
  /// it is requested after a change to one of its prerequisite values.
  ///
  /// The `Eval` methods in this group return a reference to the
  /// already-computed result in the given Context's cache. If the current value
  /// is out of date, they first update the cache entry using the corresponding
  /// `Calc` method from the "Calculations" group. Evaluations of input ports
  /// instead delegate to the containing Diagram, which arranges to have the
  /// appropriate subsystem evaluate the source output port.
  //@{

  //TODO(sherm1) EvalTimeDerivatives(), EvalDiscreteVariableUpdates(),
  //             EvalUnrestrictedUpdate() PR #4382, EvalOutputPort().

  // These are here as models of API-to-be.

  /// Returns a reference to the cached value of the potential energy. If
  /// necessary the cache will be updated first using CalcPotentialEnergy().
  /// @see CalcPotentialEnergy()
  const T& EvalPotentialEnergy(const Context<T>& context) const {
    // TODO(sherm1) Replace with an actual cache entry.
    static T fake_cache_entry;
    fake_cache_entry = CalcPotentialEnergy(context);
    return fake_cache_entry;
  }

  /// Returns a reference to the cached value of the kinetic energy. If
  /// necessary the cache will be updated first using CalcKineticEnergy().
  /// @see CalcKineticEnergy()
  const T& EvalKineticEnergy(const Context<T>& context) const {
    // TODO(sherm1) Replace with an actual cache entry.
    static T fake_cache_entry;
    fake_cache_entry = CalcKineticEnergy(context);
    return fake_cache_entry;
  }

  /// Returns a reference to the cached value of the conservative power. If
  /// necessary the cache will be updated first using CalcConservativePower().
  /// @see CalcConservativePower()
  const T& EvalConservativePower(const Context<T>& context) const {
    // TODO(sherm1) Replace with an actual cache entry.
    static T fake_cache_entry;
    fake_cache_entry = CalcConservativePower(context);
    return fake_cache_entry;
  }

  /// Returns a reference to the cached value of the non-conservative power. If
  /// necessary the cache will be updated first using
  /// CalcNonConservativePower().
  /// @see CalcNonConservativePower()
  const T& EvalNonConservativePower(const Context<T>& context) const {
    // TODO(sherm1) Replace with an actual cache entry.
    static T fake_cache_entry;
    fake_cache_entry = CalcNonConservativePower(context);
    return fake_cache_entry;
  }

  /// Causes the vector-valued input port with the given `port_index` to become
  /// up-to-date, delegating to our parent Diagram if necessary. Returns
  /// the port's value, or nullptr if the port is not connected.
  ///
  /// Throws std::bad_cast if the port is not vector-valued. Aborts if the port
  /// does not exist.
  const BasicVector<T>* EvalVectorInput(const Context<T>& context,
                                        int port_index) const {
    DRAKE_ASSERT(0 <= port_index && port_index < get_num_input_ports());
    return context.EvalVectorInput(parent_, get_input_port(port_index));
  }

  /// Causes the vector-valued input port with the given `port_index` to become
  /// up-to-date, delegating to our parent Diagram if necessary. Returns
  /// the port's value as an %Eigen expression.
  Eigen::VectorBlock<const VectorX<T>> EvalEigenVectorInput(
      const Context<T>& context, int port_index) const {
    const BasicVector<T>* input_vector = EvalVectorInput(context, port_index);
    DRAKE_ASSERT(input_vector != nullptr);
    DRAKE_ASSERT(input_vector->size() == get_input_port(port_index).get_size());
    return input_vector->get_value();
  }

  /// Causes the abstract-valued input port with the given `port_index` to
  /// become up-to-date, delegating to our parent Diagram if necessary. Returns
  /// the port's abstract value pointer, or nullptr if the port is not
  /// connected.
  const AbstractValue* EvalAbstractInput(const Context<T>& context,
                                         int port_index) const {
    DRAKE_ASSERT(0 <= port_index && port_index < get_num_input_ports());
    return context.EvalAbstractInput(parent_, get_input_port(port_index));
  }

  /// Causes the abstract-valued input port with the given `port_index` to
  /// become up-to-date, delegating to our parent Diagram if necessary. Returns
  /// the port's abstract value, or nullptr if the port is not connected.
  ///
  /// @tparam V The type of data expected.
  template <typename V>
  const V* EvalInputValue(const Context<T>& context, int port_index) const {
    DRAKE_ASSERT(0 <= port_index && port_index < get_num_input_ports());
    return context.template EvalInputValue<V>(parent_,
                                              get_input_port(port_index));
  }

  //@}

  //----------------------------------------------------------------------------
  /// @name                        Calculations
  /// A Drake %System defines a set of common computations that are understood
  /// by the framework. Most of these are embodied in a `Calc` method that
  /// unconditionally performs the calculation into an output argument of the
  /// appropriate type, using only values from the given Context. These are
  /// paired with an `Eval` method that returns a reference to an
  /// already-calculated result residing in the cache; if needed that result is
  /// first obtained using the `Calc` method. See the "Evaluations" group for
  /// more information.
  ///
  /// This group also includes additional %System-specific operations that
  /// depend on both Context and additional input arguments.
  //@{

  /// Calculates the time derivatives `xcdot` of the continuous state `xc`.
  /// The `derivatives` vector will correspond elementwise with the continuous
  /// state in the given Context. Thus, if the state in
  /// the Context has second-order structure `xc=[q v z]`, that same structure
  /// applies to the derivatives so we will have `xcdot=[qdot vdot zdot]`.
  ///
  /// @param context The Context whose time, input port, parameter, and state
  /// values are used to evaluate the derivatives.
  ///
  /// @param derivatives The time derivatives `xcdot`. Must be the same size as
  ///                    the continuous state vector in `context`.
  void CalcTimeDerivatives(const Context<T>& context,
                           ContinuousState<T>* derivatives) const {
    DRAKE_DEMAND(derivatives != nullptr);
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    DoCalcTimeDerivatives(context, derivatives);
  }

  /// This method is called to calculate the correct update `xd(n+1)` to
  /// discrete variables `xd` given a Context containing their current values
  /// `xd(n)`, because the given `event` has arrived.  Dispatches to
  /// DoCalcDiscreteVariableUpdates by default, or to `event.do_update` if
  /// provided.
  void CalcDiscreteVariableUpdates(const Context<T> &context,
                                   const DiscreteEvent<T> &event,
                                   DiscreteState<T> *discrete_state) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    DRAKE_DEMAND(event.action == DiscreteEvent<T>::kUpdateAction);
    if (event.do_calc_update == nullptr) {
      DoCalcDiscreteVariableUpdates(context, discrete_state);
    } else {
      event.do_calc_update(context, discrete_state);
    }
  }

  // TODO(edrumwri) CalcUnrestrictedUpdate() PR #4382.

  /// This method is called by a Simulator during its calculation of the size of
  /// the next continuous step to attempt. The System returns the next time at
  /// which some discrete action must be taken, and records what those actions
  /// ought to be in the given UpdateActions object, which must not be null.
  /// Upon reaching that time, the Simulator invokes either a publication
  /// action (with a const Context) or an update action (with a mutable
  /// Context). The UpdateAction object is retained and returned to the System
  /// when it is time to take the action.
  T CalcNextUpdateTime(const Context<T>& context,
                       UpdateActions<T>* actions) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    DRAKE_ASSERT(actions != nullptr);
    actions->events.clear();
    DoCalcNextUpdateTime(context, actions);
    return actions->time;
  }

  /// Computes the output values that should result from the current contents
  /// of the given Context. The result may depend on time and the current values
  /// of input ports, parameters, and state variables.
  void CalcOutput(const Context<T>& context, SystemOutput<T>* output) const {
    DRAKE_DEMAND(output != nullptr);
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    DRAKE_ASSERT_VOID(CheckValidOutput(output));
    DoCalcOutput(context, output);
  }

  /// Calculates and returns the potential energy current stored in the
  /// configuration provided in `context`. Non-physical Systems will return
  /// zero.
  /// @see EvalPotentialEnergy()
  T CalcPotentialEnergy(const Context<T>& context) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    return DoCalcPotentialEnergy(context);
  }

  /// Calculates and returns the kinetic energy currently present in the motion
  /// provided in the given Context. Non-physical Systems will return zero.
  /// @see EvalKineticEnergy()
  T CalcKineticEnergy(const Context<T>& context) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    return DoCalcKineticEnergy(context);
  }

  /// Calculates and returns the rate at which mechanical energy is being
  /// converted *from* potential energy *to* kinetic energy by this system in
  /// the given Context. This quantity will be positive when potential energy is
  /// decreasing. Note that kinetic energy will also be affected by
  /// non-conservative forces so we can't say whether it is increasing or
  /// decreasing in an absolute sense, only whether the conservative
  /// power is increasing or decreasing the kinetic energy. Power is in watts
  /// (J/s).Non-physical Systems will return zero.
  /// @see EvalConservativePower()
  T CalcConservativePower(const Context<T>& context) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    return DoCalcConservativePower(context);
  }

  /// Calculates and returns the rate at which mechanical energy is being
  /// generated (positive) or dissipated (negative) *other than* by conversion
  /// between potential and kinetic energy (in the given Context). Integrating
  /// this quantity yields work W, and the total energy `E=PE+KE-W` should be
  /// conserved by any physically-correct model, to within integration accuracy
  /// of W. Power is in watts (J/s). (Watts are abbreviated W but not to be
  /// confused with work!) This method is meaningful only for physical systems;
  /// others return zero.
  /// @see EvalNonConservativePower()
  T CalcNonConservativePower(const Context<T>& context) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    return DoCalcNonConservativePower(context);
  }


  /// Transforms a given generalized velocity `v` to the time derivative `qdot`
  /// of the generalized configuration `q` taken from the supplied Context.
  /// `v` and `qdot` are related linearly by `qdot = N(q) * v`, where `N` is a
  /// block diagonal matrix with a block for each tree joint. Note that `v` is
  /// *not* taken from the Context; it is given as an argument here.
  /// See the alternate signature if you already have the generalized
  /// velocity in an Eigen VectorX object; this signature will copy the
  /// VectorBase into an Eigen object before performing the computation.
  /// @see MapQDotToVelocity()
  void MapVelocityToQDot(const Context<T>& context,
                         const VectorBase<T>& generalized_velocity,
                         VectorBase<T>* qdot) const {
    MapVelocityToQDot(context, generalized_velocity.CopyToVector(), qdot);
  }

  /// Transforms the given generalized velocity to the time derivative of
  /// generalized configuration. See the other signature of MapVelocityToQDot()
  /// for more information.
  void MapVelocityToQDot(
      const Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& generalized_velocity,
      VectorBase<T>* qdot) const {
    DoMapVelocityToQDot(context, generalized_velocity, qdot);
  }

  /// Transforms the time derivative `qdot` of the generalized configuration `q`
  /// to generalized velocities `v`. `v` and `qdot` are related linearly by
  /// `qdot = N(q) * v`, where `N` is a block diagonal matrix with a block for
  /// each tree joint. Although `N` is not necessarily square,
  /// its left pseudo-inverse `N+` can be used to invert that relationship
  /// without residual error. Using the configuration `q` from the given
  /// context this method calculates `v = N+ * qdot` (where `N+=N+(q)`) for
  /// a given `qdot`. This method does not take `qdot` from the context. See the
  /// alternate signature if you already have `qdot` in an %Eigen VectorX
  /// object; this signature will copy the VectorBase into an %Eigen object
  /// before performing the computation.
  /// @see MapVelocityToQDot()
  void MapQDotToVelocity(const Context<T>& context, const VectorBase<T>& qdot,
                         VectorBase<T>* generalized_velocity) const {
    MapQDotToVelocity(context, qdot.CopyToVector(), generalized_velocity);
  }

  /// Transforms the given time derivative `qdot` of generalized configuration
  /// `q` to generalized velocity `v`. This signature takes `qdot` as an %Eigen
  /// VectorX object for faster speed. See the other signature of
  /// MapQDotToVelocity() for additional information.
  void MapQDotToVelocity(const Context<T>& context,
                         const Eigen::Ref<const VectorX<T>>& qdot,
                         VectorBase<T>* generalized_velocity) const {
    DoMapQDotToVelocity(context, qdot, generalized_velocity);
  }

  // TODO(edrumwri): MapAccelerationToQDotDot.

  //@}

  /// Returns a ContinuousState of the same size as the continuous_state
  /// allocated in CreateDefaultContext. The simulator will provide this state
  /// as the output argument to EvalTimeDerivatives.
  ///
  /// By default, allocates no derivatives. Systems with continuous state
  /// variables should override.
  virtual std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const {
    return nullptr;
  }

  /// Returns a DiscreteState of the same dimensions as the discrete_state
  /// allocated in CreateDefaultContext. The simulator will provide this state
  /// as the output argument to Update.
  /// By default, allocates nothing. Systems with discrete state variables
  /// should override.
  virtual std::unique_ptr<DiscreteState<T>> AllocateDiscreteVariables()
      const {
    return nullptr;
  }

  // Sets the name of the system. It is recommended that the name not include
  // the character ':', since that the path delimiter. is "::".
  void set_name(const std::string& name) { name_ = name; }
  std::string get_name() const { return name_; }

  /// Writes the full path of this System in the tree of Systems to @p output.
  /// The path has the form (::ancestor_system_name)*::this_system_name.
  void GetPath(std::stringstream* output) const {
    // If this System has a parent, that parent's path is a prefix to this
    // System's path. Otherwise, this is the root system and there is no prefix.
    if (parent_ != nullptr) {
      parent_->GetPath(output);
    }
    *output << "::";
    *output << (get_name().empty() ? "<unnamed System>" : get_name());
  }

  // Returns the full path of the System in the tree of Systems.
  std::string GetPath() const {
    std::stringstream path;
    GetPath(&path);
    return path.str();
  }

  /// Creates a deep copy of @p from, transmogrified to use the autodiff
  /// scalar type, with a dynamic-sized vector of partial derivatives. Returns
  /// nullptr if the template parameter S is not the type of the concrete
  /// system, or a superclass thereof.
  ///
  /// Usage: @code
  ///   MySystem<double> plant;
  ///   std::unique_ptr<MySystem<AutoDiffXd>> ad_plant =
  ///       systems::System<double>::ToAutoDiffXd<MySystem>(plant);
  /// @endcode
  ///
  /// @tparam S The specific System pointer type to return.
  template <template <typename> class S = ::drake::systems::System>
  static std::unique_ptr<S<AutoDiffXd>> ToAutoDiffXd(
      const System<double>& from) {
    // Capture the copy as System<AutoDiffXd>.
    std::unique_ptr<System<AutoDiffXd>> clone(from.DoToAutoDiffXd());
    // Attempt to downcast to S<AutoDiffXd>.
    S<AutoDiffXd>* downcast = dynamic_cast<S<AutoDiffXd>*>(clone.get());
    // If the downcast fails, return nullptr, letting the copy be deleted.
    if (downcast == nullptr) {
      return nullptr;
    }
    // If the downcast succeeds, redo it, taking ownership this time.
    return std::unique_ptr<S<AutoDiffXd>>(
        dynamic_cast<S<AutoDiffXd>*>(clone.release()));
  }

  /// Creates a deep copy of this System, transmogrified to use the autodiff
  /// scalar type, with a dynamic-sized vector of partial derivatives.
  /// Concrete Systems may shadow this with a more specific return type.
  std::unique_ptr<System<AutoDiffXd>> ToAutoDiffXd() const {
    return std::unique_ptr<System<AutoDiffXd>>(DoToAutoDiffXd());
  }

  /// Declares that @p parent is the immediately enclosing Diagram. The
  /// enclosing Diagram is needed to evaluate inputs recursively. Aborts if
  /// the parent has already been set to something else.
  ///
  /// This is a dangerous implementation detail. Conceptually, a System
  /// ought to be completely ignorant of its parent Diagram. However, we
  /// need this pointer so that we can cause our inputs to be evaluated.
  /// See https://github.com/RobotLocomotion/drake/pull/3455.
  void set_parent(const detail::InputPortEvaluatorInterface<T>* parent) {
    DRAKE_DEMAND(parent_ == nullptr || parent_ == parent);
    parent_ = parent;
  }

 protected:
  /// @name                 System construction
  /// Authors of concrete %Systems can use these methods in the constructor
  /// for those %Systems.
  //@{
  /// Constructs an empty %System base class object.
  System() {}

  /// Adds a port with the specified @p descriptor to the input topology.
  void DeclareInputPort(const SystemPortDescriptor<T>& descriptor) {
    DRAKE_ASSERT(descriptor.get_index() == get_num_input_ports());
    DRAKE_ASSERT(descriptor.get_face() == kInputPort);
    input_ports_.emplace_back(descriptor);
  }

  /// Adds a port with the specified @p type and @p size to the input topology.
  /// @return descriptor of declared port.
  const SystemPortDescriptor<T>& DeclareInputPort(PortDataType type, int size) {
    int port_index = get_num_input_ports();
    input_ports_.emplace_back(this, kInputPort, port_index, type, size);
    return input_ports_.back();
  }

  /// Adds an abstract-valued port to the input topology.
  /// @return descriptor of declared port.
  const SystemPortDescriptor<T>& DeclareAbstractInputPort() {
    return DeclareInputPort(kAbstractValued, 0 /* size */);
  }

  /// Adds a port with the specified @p descriptor to the output topology.
  void DeclareOutputPort(const SystemPortDescriptor<T>& descriptor) {
    DRAKE_ASSERT(descriptor.get_index() == get_num_output_ports());
    DRAKE_ASSERT(descriptor.get_face() == kOutputPort);
    output_ports_.emplace_back(descriptor);
  }

  /// Adds a port with the specified @p type and @p size to the output topology.
  /// @return descriptor of declared port.
  const SystemPortDescriptor<T>& DeclareOutputPort(PortDataType type,
                                                   int size) {
    int port_index = get_num_output_ports();
    output_ports_.emplace_back(this, kOutputPort, port_index, type, size);
    return output_ports_.back();
  }

  /// Adds an abstract-valued port with to the output topology.
  /// @return descriptor of declared port.
  const SystemPortDescriptor<T>& DeclareAbstractOutputPort() {
    return DeclareOutputPort(kAbstractValued, 0 /* size */);
  }
  //@}

  //----------------------------------------------------------------------------
  /// @name               Virtual methods for calculations
  /// These virtuals allow concrete systems to implement the calculations
  /// defined by the `Calc` methods in the public interface. Most have default
  /// implementations that are usable for simple systems, but you are likely
  /// to need to override some or all of these in your concrete system to
  /// produce meaningful calculations.
  ///
  /// These methods are invoked by the corresponding method in the public
  /// interface that has the same name with `Do` removed. The public method
  /// performs error checking on the arguments so you do not need to do so in
  /// your implementation. Users cannot invoke these directly since they are
  /// protected. You should place your overrides in the protected or private
  /// sections of your concrete class.
  //@{

  /// You must override this method to calculate values for output ports into
  /// the supplied argument, based on the contents of the given Context.
  virtual void DoCalcOutput(const Context<T>& context,
                            SystemOutput<T>* output) const = 0;

  /// Override this if you have any continuous state variables `xc` in your
  /// concrete %System to calculate their time derivatives.
  /// The `derivatives` vector will correspond elementwise with the state
  /// vector Context.state.continuous_state.get_state(). Thus, if the state in
  /// the Context has second-order structure `xc=[q,v,z]`, that same structure
  /// applies to the derivatives.
  ///
  /// This method is called from the public non-virtual CalcTimeDerivatives()
  /// which will already have error-checked the parameters so you don't have to.
  /// In particular, implementations may assume that the given Context is valid
  /// for this %System; that the `derivatives` pointer is non-null, and that
  /// the given `derivatives` argument has the same constituent structure as was
  /// produced by AllocateTimeDerivatives().
  ///
  /// The default implementation does nothing if the `derivatives` vector is,
  /// size zero and aborts otherwise.
  virtual void DoCalcTimeDerivatives(const Context<T>& context,
                                     ContinuousState<T>* derivatives) const {
    // This default implementation is only valid for Systems with no continuous
    // state. Other Systems must override this method!
    DRAKE_DEMAND(derivatives->size() == 0);
    return;
  }

  /// Implement this in your concrete System if you want it to take some action
  /// when the Simulator calls the Publish() method. This can be used for
  /// sending messages, producing console output, debugging, logging, saving the
  /// trajectory to a file, etc. You may assume that the `context` has already
  /// been validated before it is passed to you here.
  virtual void DoPublish(const Context<T>& context) const {}

  /// Given a Context containing current values for discrete variables `xd`,
  /// this method should calculate new values that `xd` should have after the
  /// update is complete. You should override it, along with
  /// DoCalcNextUpdateTime() if you want your discrete
  /// variables updated by a Simulator call to EvalDiscreteVariableUpdates().
  ///
  /// `discrete_state` is not a pointer into `context`. It is a separate
  /// buffer, which the Simulator is responsible for writing back to
  /// `context` later. That ensures that all simultaneous subsystem updates see
  /// the same values in `context` when computing the updated values for their
  /// discrete variables.
  virtual void DoCalcDiscreteVariableUpdates(
      const Context<T>& context, DiscreteState<T>* discrete_state) const {}

  /// Computes the next time at which this System must perform a discrete
  /// action.
  ///
  /// Override this method if your System has any discrete actions which must
  /// interrupt the continuous simulation. You may assume that the context
  /// has already been validated and the `actions` pointer is not nullptr.
  ///
  /// The default implementation returns with `actions` having a next sample
  /// time of Infinity and no actions to take.  If you declare actions, you may
  /// specify custom do_publish and do_update handlers.  If you do not,
  /// DoPublish and DoEvalDifferenceUpdates will be used by default.
  virtual void DoCalcNextUpdateTime(const Context<T>& context,
                                    UpdateActions<T>* actions) const {
    actions->time = std::numeric_limits<T>::infinity();
  }


  /// Override this method for physical systems to calculate the potential
  /// energy currently stored in the configuration provided in the given
  /// Context. The default implementation returns 0 which is correct for
  /// non-physical systems.
  virtual T DoCalcPotentialEnergy(const Context<T>& context) const {
    return T(0);
  }

  /// Override this method for physical systems to calculate the kinetic
  /// energy currently present in the motion provided in the given
  /// Context. The default implementation returns 0 which is correct for
  /// non-physical systems.
  virtual T DoCalcKineticEnergy(const Context<T>& context) const {
    return T(0);
  }

  /// Override this method to return the rate at which mechanical energy is
  /// being converted *from* potential energy *to* kinetic energy by this system
  /// in the given Context. This quantity must be positive when potential energy
  /// is *decreasing*. Power is in watts (J/s).
  ///
  /// By default, returns zero. Continuous, physical systems should override.
  virtual T DoCalcConservativePower(const Context<T>& context) const {
    return T(0);
  }

  /// Override this method to return the rate at which mechanical energy is
  /// being generated (positive) or dissipated (negative) *other than* by
  /// conversion between potential and kinetic energy (in the given Context).
  /// Integrating this quantity yields work W, and the total energy `E=PE+KE-W`
  /// should be conserved by any physically-correct model, to within integration
  /// accuracy of W. Power is in watts (J/s). (Watts are abbreviated W but not
  /// to be confused with work!) This method is meaningful only for physical
  /// systems; others return zero.
  ///
  /// By default, returns zero. Continuous, physical systems should override.
  virtual T DoCalcNonConservativePower(const Context<T>& context) const {
    return T(0);
  }

  /// Provides the substantive implementation of MapQDotToVelocity().
  ///
  /// The default implementation uses the identity mapping. It throws
  /// std::runtime_error if the `generalized_velocity` and
  /// `qdot` are not the same size, but that is not enough to guarantee that
  /// the default implementation is adequate. Child classes must
  /// override this function if qdot != v (even if they are the same size).
  /// This occurs, for example, if a joint uses roll-pitch-yaw rotation angles
  /// for orientation but angular velocity for rotational rate rather than
  /// rotation angle derivatives.
  ///
  /// Implementations may assume that `qdot` has already been
  /// validated to be the same size as `q` in the given
  /// Context, and that `generalized_velocity` is non-null. Implementations that
  /// are not second-order systems may simply do nothing.
  virtual void DoMapQDotToVelocity(const Context<T>& context,
                                   const Eigen::Ref<const VectorX<T>>& qdot,
                                   VectorBase<T>* generalized_velocity) const {
    // In the particular case where generalized velocity and generalized
    // configuration are not even the same size, we detect this error and abort.
    // This check will thus not identify cases where the generalized velocity
    // and time derivative of generalized configuration are identically sized
    // but not identical!
    const int n = qdot.size();
    // You need to override System<T>::DoMapConfigurationDerivativestoVelocity!
    DRAKE_THROW_UNLESS(generalized_velocity->size() == n);
    generalized_velocity->SetFromVector(qdot);
  }

  /// Provides the substantive implementation of MapVelocityToQDot().

  /// The default implementation uses the identity mapping. It throws
  /// std::runtime_error if the `generalized_velocity` (`v`) and
  /// `qdot` are not the same size, but that is not enough to guarantee that
  /// the default implementation is adequate. Child classes must
  /// override this function if `qdot != v` (even if they are the same size).
  /// This occurs, for example, if a joint uses roll-pitch-yaw rotation angles
  /// for orientation but angular velocity for rotational rate rather than
  /// rotation angle derivatives.
  ///
  /// Implementations may assume that `generalized_velocity` has already been
  /// validated to be the same size as the generalized velocity in the given
  /// Context, and that `qdot` is non-null. Implementations that are not
  /// second-order systems may simply do nothing.
  virtual void DoMapVelocityToQDot(
      const Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& generalized_velocity,
      VectorBase<T>* qdot) const {
    // In the particular case where generalized velocity and generalized
    // configuration are not even the same size, we detect this error and abort.
    // This check will thus not identify cases where the generalized velocity
    // and time derivative of generalized configuration are identically sized
    // but not identical!
    const int n = generalized_velocity.size();
    // You need to override System<T>::DoMapVelocityToQDot!
    DRAKE_THROW_UNLESS(qdot->size() == n);
    qdot->SetFromVector(generalized_velocity);
  }

  /// NVI implementation of ToAutoDiffXd. Caller takes ownership of the returned
  /// pointer. Overrides should return a more specific covariant type.
  /// Templated overrides may assume that they are subclasses of System<double>.
  ///
  /// No default implementation is provided in LeafSystem, since the member data
  /// of a particular concrete leaf system is not knowable to the framework.
  /// A default implementation is provided in Diagram, which Diagram subclasses
  /// with member data should override.
  virtual System<AutoDiffXd>* DoToAutoDiffXd() const {
    DRAKE_ABORT_MSG("Override DoToAutoDiffXd before using ToAutoDiffXd.");
    return nullptr;
  }
  //@}

  /// @name                    Utility methods
  //@{
  /// Returns a mutable Eigen expression for a vector valued output port with
  /// index @p port_index in this system. All InputPorts that directly depend
  /// on this OutputPort will be notified that upstream data has changed, and
  /// may invalidate cache entries as a result.
  Eigen::VectorBlock<VectorX<T>> GetMutableOutputVector(SystemOutput<T>* output,
                                                        int port_index) const {
    DRAKE_ASSERT(0 <= port_index && port_index < get_num_output_ports());

    BasicVector<T>* output_vector = output->GetMutableVectorData(port_index);
    DRAKE_ASSERT(output_vector != nullptr);
    DRAKE_ASSERT(output_vector->size() ==
        get_output_port(port_index).get_size());

    return output_vector->get_mutable_value();
  }

  /// Causes an InputPort in the @p context to become up-to-date, delegating to
  /// the parent Diagram if necessary.
  ///
  /// This is a framework implementation detail. User code should never call it.
  void EvalInputPort(const Context<T>& context, int port_index) const {
    DRAKE_ASSERT(0 <= port_index && port_index < get_num_input_ports());
    context.EvalInputPort(parent_, get_input_port(port_index));
  }
  //@}

 private:
  // System objects are neither copyable nor moveable.
  System(const System<T>& other) = delete;
  System& operator=(const System<T>& other) = delete;
  System(System<T>&& other) = delete;
  System& operator=(System<T>&& other) = delete;

  std::string name_;
  std::vector<SystemPortDescriptor<T>> input_ports_;
  std::vector<SystemPortDescriptor<T>> output_ports_;
  const detail::InputPortEvaluatorInterface<T>* parent_{nullptr};
};

}  // namespace systems
}  // namespace drake
