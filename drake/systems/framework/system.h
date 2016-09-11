#pragma once

#include <limits>
#include <string>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace systems {


/** @defgroup systems Modeling Dynamical Systems
 * @{
 * @brief Drake uses a Simulink-inspired description of dynamical systems.
 *
 * Includes basic building blocks (adders, integrators, delays, etc),
 * physics models of mechanical systems, and a growing list of sensors,
 * actuators, controllers, planners, estimators.
 *
 * All dynamical systems derive from the System base class, and must
 * explicitly  declare all State, parameters, and noise/disturbances inputs.
 * The Diagram class permits modeling complex systems from libraries of parts.
 * @}
 */



/// This information is returned along with the next update/sample/event time
/// and is provided back to the System when that time is reached to ensure that
/// the correct sampling actions are taken.
struct SampleActions {
  /// When the next discrete action is required.
  double time{std::numeric_limits<double>::quiet_NaN()};
  // TODO(sherm1) Record (subsystem,eventlist) pairs indicating what is
  // supposed to happen at that time and providing O(1) access to the
  // appropriate update/sampler/event handler method.
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

  /// Returns the input port @p input_port.
  const SystemPortDescriptor<T>& get_input_port(int port_number) const {
    if (port_number >= get_num_input_ports()) {
      throw std::out_of_range("port number out of range.");
    }
    return input_ports_[port_number];
  }

  /// Returns the output port @p output_port.
  const SystemPortDescriptor<T>& get_output_port(int port_number) const {
    if (port_number >= get_num_output_ports()) {
      throw std::out_of_range("port number out of range.");
    }
    return output_ports_[port_number];
  }

  /// Returns descriptors for all the output ports of this system.
  const std::vector<SystemPortDescriptor<T>>& get_output_ports() const {
    return output_ports_;
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
      // TODO(amcastro-tri): add appropriate checks for kAbstractValued ports
      // once abstract ports are implemented in 3164.
      if (this->get_input_port(i).get_data_type() == kVectorValued) {
        const VectorBase<T>* input_vector = context.get_vector_input(i);
        DRAKE_THROW_UNLESS(input_vector != nullptr);
        DRAKE_THROW_UNLESS(input_vector->size() ==
                           get_input_port(i).get_size());
      }
    }
  }

  /// Returns an Eigen expression for a vector valued input port with index
  /// @p port_index in this system.
  Eigen::VectorBlock<const VectorX<T>> get_input_vector(
      const Context<T>& context, int port_index) const {
    DRAKE_ASSERT(0 <= port_index && port_index < get_num_input_ports());
    const BasicVector<T>* input_vector = context.get_vector_input(port_index);

    DRAKE_ASSERT(input_vector != nullptr);
    DRAKE_ASSERT(input_vector->size() == get_input_port(port_index).get_size());

    return input_vector->get_value();
  }

  // Returns a copy of the continuous state vector into an Eigen vector.
  VectorX<T> CopyContinuousStateVector(const Context<T>& context) const {
    return context.get_state().continuous_state->get_state().CopyToVector();
  }

  /// Returns a default context, initialized with the correct
  /// numbers of concrete input ports and state variables for this System.
  /// Since input port pointers are not owned by the context, they should
  /// simply be initialized to nullptr.
  virtual std::unique_ptr<Context<T>> CreateDefaultContext() const = 0;

  /// Returns a default output, initialized with the correct number of
  /// concrete output ports for this System. @p context is provided as
  /// an argument to support some specialized use cases. Most typical
  /// System implementations should ignore it.
  virtual std::unique_ptr<SystemOutput<T>> AllocateOutput(
      const Context<T>& context) const = 0;

  /// This method is invoked by a Simulator at designated meaningful points
  /// along a trajectory, to allow the executing System a chance to take some
  /// kind of output action. Typical actions may include terminal output,
  /// visualization, logging, plotting, and sending messages. Other than
  /// computational cost, publishing has no effect on the progress of a
  /// simulation (but see note below).
  ///
  /// By default Publish() will be called at the start of each continuous
  /// integration step, after discrete variables have been updated to the values
  /// they will hold throughout the step. However, a Simulator may allow control
  /// over the publication rate. Publish() will always be called at the start of
  /// the first step of a simulation (after initialization) and after the final
  /// simulation step (after a final update to discrete variables).
  ///
  /// @note When publishing is scheduled at particular times, those times likely
  /// will not coincide with integrator step times. A Simulator may interpolate
  /// to generate a suitable Context, or it may adjust the integrator step size
  /// so that a step begins exactly at the next publication time. In the latter
  /// case the change in step size may affect the numerical result somewhat
  /// since a smaller integrator step produces a more accurate solution.
  // TODO(sherm1) Provide sample rate option for Publish().
  void Publish(const Context<T>& context) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    DoPublish(context);
  }

  /// This method is called to perform discrete updates to the Context, with
  /// the particular actions to take supplied in `actions`.
  // TODO(sherm1) Per great suggestion from David German in #3202, this method
  // should be modified to take a const context and provide a non-const
  // reference to only the part that is allowed to change. This will likely
  // require splitting into several APIs since different sample/update/event
  // actions permit different modifications.
  void Update(Context<T>* context, const SampleActions& actions) const {
    DRAKE_ASSERT_VOID(CheckValidContext(*context));
    DoUpdate(context, actions);
  }

  /// This method is called by a Simulator during its calculation of the size of
  /// the next continuous step to attempt. The System returns the next time at
  /// which some discrete action must be taken, and records what those actions
  /// ought to be in the given SampleActions object, which must not be null.
  /// Upon reaching that time, the Simulator invokes either a publication
  /// action (with a const Context) or an update action (with a mutable
  /// Context). The SampleAction object is retained and returned to the System
  /// when it is time to take the action.
  double CalcNextSampleTime(const Context<T>& context,
                            SampleActions* actions) const {
    // TODO(sherm1) Validate context (at least in Debug).
    DRAKE_ASSERT(actions != nullptr);
    DoCalcNextSampleTime(context, actions);
    return actions->time;
  }

  /// Computes the output for the given context, possibly updating values
  /// in the cache.
  virtual void EvalOutput(const Context<T>& context,
                          SystemOutput<T>* output) const = 0;

  /// Returns the potential energy currently stored in the configuration
  /// provided in the given @p context. Non-physical Systems will return 0.
  virtual T EvalPotentialEnergy(const Context<T>& context) const {
    return T(0);
  }

  /// Returns the kinetic energy currently present in the motion provided in
  /// the given @p context. Non-physical Systems will return 0.
  virtual T EvalKineticEnergy(const Context<T>& context) const { return T(0); }

  /// Returns the rate at which mechanical energy is being converted *from*
  /// potential energy *to* kinetic energy by this system in the given Context.
  /// This quantity will be positive when potential energy is decreasing. Note
  /// that kinetic energy will also be affected by non-conservative forces so we
  /// can't say which direction it is moving, only whether the conservative
  /// power is increasing or decreasing the kinetic energy. Power is in watts
  /// (J/s).
  ///
  /// By default, returns zero. Continuous, physical systems should override.
  virtual T EvalConservativePower(const Context<T>& context) const {
    return T(0);
  }

  /// Returns the rate at which mechanical energy is being generated (positive)
  /// or dissipated (negative) *other than* by conversion between potential and
  /// kinetic energy (in the given Context). Integrating this quantity yields
  /// work W, and the total energy `E=PE+KE-W` should be conserved by any
  /// physically-correct model, to within integration accuracy of W. Power is in
  /// watts (J/s). (Watts are abbreviated W but not to be confused with work!)
  /// This method is meaningful only for physical systems; others return 0.
  ///
  /// By default, returns zero. Continuous, physical systems should override.
  virtual T EvalNonConservativePower(const Context<T>& context) const {
    return T(0);
  }

  /// Returns a ContinuousState of the same size as the continuous_state
  /// allocated in CreateDefaultContext. Solvers will provide this state as the
  /// output argument to EvalTimeDerivatives.
  ///
  /// By default, allocates no derivatives. Systems with continuous state
  /// variables should override.
  virtual std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const {
    return nullptr;
  }

  /// Produces the derivatives of the continuous state xc with respect to time.
  /// The @p derivatives vector will correspond elementwise with the state
  /// vector Context.state.continuous_state.get_state(). Thus, if the state in
  /// the Context has second-order structure, that same structure applies to
  /// the derivatives.
  ///
  /// Implementations may assume that the given @p derivatives argument has the
  /// same constituent structure as was produced by AllocateTimeDerivatives.
  ///
  /// @param context The context in which to evaluate the derivatives.
  ///
  /// @param derivatives The output vector. Will be the same length as the
  ///                    state vector Context.state.continuous_state.
  virtual void EvalTimeDerivatives(const Context<T>& context,
                                   ContinuousState<T>* derivatives) const {
    return;
  }

  /// Transforms the velocity (v) in the given Context state to the derivative
  /// of the configuration (qdot). The transformation must be linear in velocity
  /// (qdot = N(q) * v), and it must require no more than O(N) time to compute
  /// in the number of generalized velocity states.
  ///
  /// The default implementation uses the identity mapping. It throws
  /// std::out_of_range if the @p generalized_velocity and
  /// @p configuration_derivatives are not the same size. Child classes must
  /// override this function if qdot != v (even if they are the same size).
  ///
  /// Implementations may assume that @p configuration_derivatives is of
  /// the same size as the generalized position allocated in
  /// CreateDefaultContext()->continuous_state.get_generalized_position(),
  /// and should populate it with elementwise-corresponding derivatives of
  /// position. Implementations that are not second-order systems may simply
  /// do nothing.
  virtual void MapVelocityToConfigurationDerivatives(
      const Context<T>& context, const VectorBase<T>& generalized_velocity,
      VectorBase<T>* configuration_derivatives) const {
    if (generalized_velocity.size() != configuration_derivatives->size()) {
      throw std::out_of_range(
          "generalized_velocity.size() " +
          std::to_string(generalized_velocity.size()) +
          " != configuration_derivatives.size() " +
          std::to_string(configuration_derivatives->size()) +
          ". Do you need to override the default implementation of " +
          "MapVelocityToConfigurationDerivatives()?");
    }

    for (int i = 0; i < generalized_velocity.size(); ++i) {
      configuration_derivatives->SetAtIndex(i,
                                            generalized_velocity.GetAtIndex(i));
    }
  }

  // TODO(david-german-tri): Add MapConfigurationDerivativesToVelocity
  // and MapAccelerationToConfigurationSecondDerivatives.

  virtual void set_name(const std::string& name) { name_ = name; }
  virtual std::string get_name() const { return name_; }

 protected:
  System() {}

  /// Adds a port with the specified @p descriptor to the input topology.
  void DeclareInputPort(const SystemPortDescriptor<T>& descriptor) {
    DRAKE_ASSERT(descriptor.get_index() == get_num_input_ports());
    DRAKE_ASSERT(descriptor.get_face() == kInputPort);
    input_ports_.emplace_back(descriptor);
  }

  /// Adds a port with the specified @p type, @p size, and @p sampling
  /// to the input topology.
  /// @return descriptor of declared port.
  const SystemPortDescriptor<T>& DeclareInputPort(PortDataType type, int size,
                                                  SamplingSpec sampling) {
    int port_number = get_num_input_ports();
    input_ports_.emplace_back(this, kInputPort, port_number, kVectorValued,
                              size, sampling);
    return input_ports_.back();
  }

  /// Adds an abstract-valued port with the specified @p sampling to the
  /// input topology.
  /// @return descriptor of declared port.
  const SystemPortDescriptor<T>& DeclareAbstractInputPort(
      SamplingSpec sampling) {
    return DeclareInputPort(kAbstractValued, 0 /* size */, sampling);
  }

  /// Adds a port with the specified @p descriptor to the output topology.
  void DeclareOutputPort(const SystemPortDescriptor<T>& descriptor) {
    DRAKE_ASSERT(descriptor.get_index() == get_num_output_ports());
    DRAKE_ASSERT(descriptor.get_face() == kOutputPort);
    output_ports_.emplace_back(descriptor);
  }

  /// Adds a port with the specified @p type, @p size, and @p sampling
  /// to the output topology.
  /// @return descriptor of declared port.
  const SystemPortDescriptor<T>& DeclareOutputPort(PortDataType type, int size,
                                                   SamplingSpec sampling) {
    int port_number = get_num_output_ports();
    output_ports_.emplace_back(this, kOutputPort, port_number, kVectorValued,
                               size, sampling);
    return output_ports_.back();
  }

  /// Adds an abstract-valued port with the specified @p sampling to the
  /// output topology.
  /// @return descriptor of declared port.
  const SystemPortDescriptor<T>& DeclareAbstractOutputPort(
      SamplingSpec sampling) {
    return DeclareOutputPort(kAbstractValued, 0 /* size */, sampling);
  }

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

  /// Implement this in your concrete System if you want it to take some action
  /// when the Simulator calls the Publish() method. This can be used for
  /// sending messages, producing console output, debugging, logging, saving the
  /// trajectory to a file, etc. You may assume that the `context` has already
  /// been validated before it is passed to you here.
  virtual void DoPublish(const Context<T>& context) const {}

  /// Implement this method if your System has any difference variables xd,
  /// mode variables, or sampled input or output ports. The `actions` argument
  /// specifies what to do and may include difference variable updates, port
  /// sampling, or execution of event handlers that may make arbitrary changes
  /// to the Context.
  virtual void DoUpdate(Context<T>* context,
                        const SampleActions& actions) const {}

  /// Implement this method if your System has any discrete actions which must
  /// interrupt the continuous simulation. You may assume that the context
  /// has already been validated and the `actions` pointer is not null.
  /// The default implemention returns with `actions` having a next sample
  /// time of Infinity and no actions to take.
  virtual void DoCalcNextSampleTime(const Context<T>& context,
                                    SampleActions* actions) const {
    actions->time = std::numeric_limits<double>::infinity();
  }

 private:
  // SystemInterface objects are neither copyable nor moveable.
  System(const System<T>& other) = delete;
  System& operator=(const System<T>& other) = delete;
  System(System<T>&& other) = delete;
  System& operator=(System<T>&& other) = delete;

  std::string name_;
  std::vector<SystemPortDescriptor<T>> input_ports_;
  std::vector<SystemPortDescriptor<T>> output_ports_;
};

}  // namespace systems
}  // namespace drake
