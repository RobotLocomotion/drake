#pragma once

#include <string>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/context_base.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace systems {

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
  virtual bool has_any_direct_feedthrough() const { return true;}

  /// Returns the number of input ports of the system.
  int get_num_input_ports() const { return input_ports_.size(); }

  /// Returns the number of output ports of the system.
  int get_num_output_ports() const { return output_ports_.size(); }

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
  /// @returns `true` if @p output is valid for this system and `false`
  /// otherwise.
  bool IsValidOutput(const SystemOutput<T>& output) const {
    // Checks that the number of output ports in the system output is consistent
    // with the number of output ports declared by the System.
    if (output.get_num_ports() != get_num_output_ports()) return false;

    // Checks the validity of each output port.
    for (int i = 0; i < get_num_output_ports(); ++i) {
      // TODO(amcastro-tri): add appropriate checks for kAbstractValued ports
      // once abstract ports are implemented in 3164.
      if (get_output_port(i).get_data_type() == kVectorValued) {
        const VectorInterface<T>* output_vector =
            output.get_port(i).get_vector_data();
        if (output_vector == nullptr) return false;
        if (output_vector->get_value().rows() !=
            get_output_port(i).get_size()) return false;
      }
    }

    // All checks passed.
    return true;
  }

  /// Checks that @p context is consistent for this system.
  /// @returns `true` if @p context is valid for this system and `false`
  /// otherwise.
  bool IsValidContext(const ContextBase<T>& context) const {
    // Checks that the number of input ports in the context is consistent with
    // the number of ports declared by the System.
    if (context.get_num_input_ports() != this->get_num_input_ports())
      return false;

    // Checks that the size of the input ports in the context matches the
    // declarations made by the system.
    for (int i = 0; i < this->get_num_input_ports(); ++i) {
      // TODO(amcastro-tri): add appropriate checks for kAbstractValued ports
      // once abstract ports are implemented in 3164.
      if (this->get_input_port(i).get_data_type() == kVectorValued) {
        const VectorInterface<T>* input_vector = context.get_vector_input(i);
        if (input_vector == nullptr) return false;
        if (input_vector->get_value().rows() !=
            get_input_port(i).get_size()) return false;
      }
    }

    // All checks passed.
    return true;
  }

  /// Returns a default context, initialized with the correct
  /// numbers of concrete input ports and state variables for this System.
  /// Since input port pointers are not owned by the context, they should
  /// simply be initialized to nullptr.
  virtual std::unique_ptr<ContextBase<T>> CreateDefaultContext() const = 0;

  /// Returns a default output, initialized with the correct number of
  /// concrete output ports for this System. @p context is provided as
  /// an argument to support some specialized use cases. Most typical
  /// System implementations should ignore it.
  virtual std::unique_ptr<SystemOutput<T>> AllocateOutput(
      const ContextBase<T>& context) const = 0;

  /// Computes the output for the given context, possibly updating values
  /// in the cache.
  virtual void EvalOutput(const ContextBase<T>& context,
                          SystemOutput<T>* output) const = 0;

  /// Returns the potential energy currently stored in the configuration
  /// provided in the given @p context. Non-physical Systems will return 0.
  virtual T EvalPotentialEnergy(const ContextBase<T>& context) const {
    return T(0);
  }

  /// Returns the kinetic energy currently present in the motion provided in
  /// the given @p context. Non-physical Systems will return 0.
  virtual T EvalKineticEnergy(const ContextBase<T>& context) const {
    return T(0);
  }

  /// Returns the rate at which mechanical energy is being converted *from*
  /// potential energy *to* kinetic energy by this system in the given Context.
  /// This quantity will be positive when potential energy is decreasing. Note
  /// that kinetic energy will also be affected by non-conservative forces so we
  /// can't say which direction it is moving, only whether the conservative
  /// power is increasing or decreasing the kinetic energy. Power is in watts
  /// (J/s).
  ///
  /// By default, returns zero. Continuous, physical systems should override.
  virtual T EvalConservativePower(const ContextBase<T>& context) const {
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
  virtual T EvalNonConservativePower(const ContextBase<T>& context) const {
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
  virtual void EvalTimeDerivatives(const ContextBase<T>& context,
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
  /// @p configuration_derivatives are not the same size. Child classes should
  /// override this function if qdot != v.
  ///
  /// Implementations may assume that @p configuration_derivatives is of
  /// the same size as the generalized position allocated in
  /// CreateDefaultContext()->continuous_state.get_generalized_position(),
  /// and should populate it with elementwise-corresponding derivatives of
  /// position. Implementations that are not second-order systems may simply
  /// do nothing.
  virtual void MapVelocityToConfigurationDerivatives(
      const ContextBase<T>& context, const StateVector<T>& generalized_velocity,
      StateVector<T>* configuration_derivatives) const {
    if (generalized_velocity.size() != configuration_derivatives->size()) {
      throw std::out_of_range(
          "generalized_velocity.size() " +
          std::to_string(generalized_velocity.size()) +
          " != configuration_derivatives.size() " +
          std::to_string(configuration_derivatives->size()) +
          ". Do you need to override the default implementation of " +
          "MapVelocityToConfigurationDerivatives?");
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
    DRAKE_ASSERT(descriptor.get_index() ==
                 static_cast<int>(input_ports_.size()));
    DRAKE_ASSERT(descriptor.get_face() == kInputPort);
    input_ports_.emplace_back(descriptor);
  }

  /// Adds a port with the specified @p type, @p size, and @p sampling
  /// to the input topology.
  void DeclareInputPort(PortDataType type, int size, SamplingSpec sampling) {
    input_ports_.emplace_back(this, kInputPort, input_ports_.size(),
                                 kVectorValued, size, sampling);
  }

  /// Adds a port with the specified @p descriptor to the output topology.
  void DeclareOutputPort(const SystemPortDescriptor<T>& descriptor) {
    DRAKE_ASSERT(descriptor.get_index() ==
                 static_cast<int>(output_ports_.size()));
    DRAKE_ASSERT(descriptor.get_face() == kOutputPort);
    output_ports_.emplace_back(descriptor);
  }

  /// Adds a port with the specified @p type, @p size, and @p sampling
  /// to the output topology.
  void DeclareOutputPort(PortDataType type, int size, SamplingSpec sampling) {
    output_ports_.emplace_back(this, kOutputPort, output_ports_.size(),
                                  kVectorValued, size, sampling);
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
