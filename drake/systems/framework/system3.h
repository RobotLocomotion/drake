#pragma once

#include <functional>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/context3.h"
#include "drake/systems/framework/system3_input.h"
#include "drake/systems/framework/system3_output.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

class AbstractContext3;
template <typename T>
class Context3;

// TODO(sherm1) These should probably be local classes so API users don't
// have to look at them.

/** A system diagram may inherit an input port from one of its contained
subsystems, which will know that port by a different port number than we will
use here. We record which subsystem and its port number here. Also, it may
turn out that the subsystem also inherited that port from one of its subsystems,
so the actual port may be arbitrarily deep. We'll find it and keep a direct
pointer to it for convenience. **/
struct InputPortFinder {
  InputPortFinder(int subsystem_num, int port_num, InputPort3* port)
      : subsystem_num(subsystem_num), port_num(port_num), port(port) {}
  int subsystem_num{-1};  ///< -1 means this system.
  int port_num{-1};       ///< Port number as understood by the subsystem.
  InputPort3* port{};      ///< Convenience pointer to the actual port.
};

/** See InputPortFinder for information. **/
struct OutputPortFinder {
  OutputPortFinder(int subsystem_num, int port_num, OutputPort3* port)
      : subsystem_num(subsystem_num), port_num(port_num), port(port) {}
  int subsystem_num{-1};  ///< -1 means this system.
  int port_num{-1};       ///< Port number as understood by the subsystem.
  OutputPort3* port{};     ///< Convenience pointer to the actual port.
};

/** An abstract superclass for dynamical systems, encapsulating functionality
that is independent of the numerical scalar type in use. That includes the
concepts of input and output, containment of subsystems, and connections
among the subsystems' inputs and outputs. A system that contains subsystems
is called a "system diagram", one without subsystems is called a "leaf system",
and the topmost system diagram is the "root system".

This class is intended to be paired with a compatible AbstractContext object
whose internal tree structure of subcontexts is identical to the structure
of the tree of subsystems.

Input values are presented to a System through zero or more InputPort objects;
output values are delivered through zero or more OutputPort objects. Systems are
composed by connecting compatible OutputPort and InputPort objects together into
a system diagram. These connections are established during extended
construction of a system diagram and must be completed prior to run time
execution (and are hence independent of Context). InputPort objects contain
information about their requirements for OutputPort mates.

Most systems should not inherit directly from %AbstractSystem. If your system
involves any numerical computations it should instead, inherit from
`System<T>`. **/
class AbstractSystem3 {
 public:
  virtual ~AbstractSystem3() {}

  /** Returns the name of this System. This is just an arbitrary string with
  no particular meaning attached. **/

  // TODO(sherm1) Consider requiring this to be unique among sibling subsystems
  // so that we can construct a unique pathname to any subsystem.
  std::string get_name() const {return name_;}

  //----------------------------------------------------------------------------
  /** @name            Methods useful for any System 

  These provide the ability to specify input and output ports, which are common
  to all dynamic systems. **/
  /**@{**/
  /** Add an input port that is to be owned by this System. The assigned port
  number is returned and can be used to retrieve this port later, and to locate
  its value in a compatible Context. **/
  int AddInputPort(std::unique_ptr<InputPort3> port) {
    const int my_port_num = (int)input_ports_.size();
    port->set_owner(this, my_port_num);
    input_ports_.emplace_back(-1, my_port_num, port.get());
    owned_input_ports_.push_back(std::move(port));
    return my_port_num;
  }

  /** Add an output port that is to be owned by this System. The assigned port
  number is returned and can be used to retrieve this port later, and to locate
  its value in a compatible Context. **/
  int AddOutputPort(std::unique_ptr<OutputPort3> port) {
    const int my_port_num = (int)output_ports_.size();
    port->set_owner(this, my_port_num);
    output_ports_.emplace_back(-1, my_port_num, port.get());
    owned_output_ports_.push_back(std::move(port));
    return my_port_num;
  }

  /** Returns the current number of input ports in this System. This is also the
  port number that will be assigned by the next call to `AddInputPort()`. **/
  int get_num_input_ports() const { return (int)input_ports_.size(); }

  /** Returns a const reference to the InputPort with the given port number. **/
  const InputPort3& get_input_port(int port_num) const {
    return *get_input_port_finder(port_num).port;
  }

  /** Returns a mutable pointer to the InputPort with the given port number. **/
  InputPort3* get_mutable_input_port(int port_num) {
    return get_input_port_finder(port_num).port;
  }

  /** Returns the current number of output ports in this System. This is also
  the port number that will be assigned by the next call to
  `AddOutputPort()`. **/
  int get_num_output_ports() const { return (int)output_ports_.size(); }

  /** Returns a const reference to the OutputPort with the given port
  number. **/
  const OutputPort3& get_output_port(int port_num) const {
    return *get_output_port_finder(port_num).port;
  }

  /** Returns a mutable pointer to the OutputPort with the given port
  number. **/
  OutputPort3* get_mutable_output_port(int port_num) {
    return get_output_port_finder(port_num).port;
  }
  /**@}**/


  /** Returns a default context, initialized with run time mutable memory for
  the correct number and type of InputPort, OutputPort, and state variable
  objects, as well as time, parameters, and basic computations. This will
  contain one subcontext corresponding to each subsystem of this System. **/
  DRAKESYSTEMFRAMEWORK_EXPORT std::unique_ptr<AbstractContext3>
  CreateDefaultContext() const;

  /** Obtain an up-to-date value for one of this System's output ports. (Output
  ports may have distinct sample times so must be individually evaluatable.) If
  necessary, computation of this value is initiated with the result written
  into the supplied Context's cache.

  @param[in,out] context A Context that is compatible with this System.
  @param[in] port_num The index number of the output port to be evaluated.
  @retval value A reference into `context` containing the current value for this
                output port. **/
  DRAKESYSTEMFRAMEWORK_EXPORT const AbstractValue& EvalOutputPort(
      const AbstractContext3& context, int port_num) const;

  //----------------------------------------------------------------------------
  /** @name   Methods for Systems with event detection and event handlers

  TODO(sherm1) Provide EvalWitnessFunctions() and a way to invoke handlers
  which are given writable access to Context so that mode variables (and
  any other state) can be updated. Handlers must be able to terminate a
  simulation and be able to indicate that a structural change is required. **/
  /**@{**/

  /**@}**/

  //----------------------------------------------------------------------------
  /** @name                System diagram methods

  These methods are useful for any System that contains subsystems.
  /**@{**/

  /** Takes ownership of the given System and returns an unowned, raw pointer to
  the concrete type for convenient access. This `subsystem` is assigned
  the next available index, as given by `get_num_subsystems()` prior to
  making this call. **/
  template <class ConcreteSystem>
  ConcreteSystem* AddSubsystem(std::unique_ptr<ConcreteSystem> subsystem) {
    ConcreteSystem* concrete = subsystem.get();
    subsystem->set_owner(this, get_num_subsystems());
    subsystems_.push_back(std::move(subsystem));
    return concrete;
  }

  /** The given subsystem's InputPort becomes the next InputPort of this
  system diagram. The diagram's input port number is returned; it will in
  general be different from the input port number in the subsystem from
  which it was inherited. **/
  int InheritInputPort(AbstractSystem3* child_subsystem, int input_port_num) {
    DRAKE_ABORT_UNLESS(child_subsystem->get_parent_system() == this);
    InputPort3* sub_port =
        child_subsystem->get_mutable_input_port(input_port_num);
    const int my_port_num = (int)input_ports_.size();
    input_ports_.emplace_back(child_subsystem->get_subsystem_num(),
                              input_port_num, sub_port);
    return my_port_num;
  }

  /** The given subsystem's OutputPort becomes the next OutputPort of this
  system diagram. The diagram's output port number is returned; it will in
  general be different from the output port number in the subsystem from
  which it was inherited. **/
  int InheritOutputPort(AbstractSystem3* child_subsystem, int output_port_num) {
    DRAKE_ABORT_UNLESS(child_subsystem->get_parent_system() == this);
    OutputPort3* sub_port =
        child_subsystem->get_mutable_output_port(output_port_num);
    const int my_port_num = (int)output_ports_.size();
    output_ports_.emplace_back(child_subsystem->get_subsystem_num(),
                               output_port_num, sub_port);
    return my_port_num;
  }

  /** Connect the given output port of subsystem 1 into the given input port
  of subsystem 2. Both subsystems must be immediate children of this system.
  The input port must be able to accept the type and sampling rate of the output
  port.
  @throws std::logic_error The input port does not accept the type or
                           sampling rate of the output port. **/
  void Connect(AbstractSystem3* source_subsystem, int output_port_num,
               AbstractSystem3* sink_subsystem, int input_port_num) {
    DRAKE_ABORT_UNLESS(source_subsystem->get_parent_system() == this &&
                       sink_subsystem->get_parent_system() == this);
    const OutputPort3& out = source_subsystem->get_output_port(output_port_num);
    InputPort3* in = sink_subsystem->get_mutable_input_port(input_port_num);
    in->ConnectTo(&out);
  }

  /** Returns the current number of subsystems contained in this system diagram.
  This is also the index that will be assigned to the next subsystem that
  is added here. **/
  int get_num_subsystems() const { return (int)subsystems_.size(); }

  /** Returns a const reference to one of the contained subsystems, using the
  index reflecting the order in which it was added. **/
  const AbstractSystem3& get_subsystem(int index) const {
    const AbstractSystem3* subsystem = subsystems_[index].get();
    DRAKE_ASSERT(subsystem && subsystem->get_parent_system() == this &&
                 subsystem->get_subsystem_num() == index);
    return *subsystem;
  }

  /** Returns a mutable pointer to one of the contained subsystems, using the
  index reflecting the order in which it was added. **/
  AbstractSystem3* get_mutable_subsystem(int index) {
    return const_cast<AbstractSystem3*>(&get_subsystem(index));
  }

  /** Returns a const pointer to the parent system that owns this one, or
  `nullptr` if this is a root system. **/
  const AbstractSystem3* get_parent_system() const { return owner_; }

  /** Returns a mutable pointer to the parent system that owns this one, or
  `nullptr` if this is a root system. **/
  AbstractSystem3* get_mutable_parent_system() { return owner_; }

  /** If this system is a subsystem of a parent system, return the subsystem
  number by which that parent knows us. Returns -1 if this is a root system. **/
  int get_subsystem_num() const { return subsystem_num_; }

  /** Determine the full path name of this subsystem in a form like
  `/rootname/parentname/myname`. This name will be unique if sibling subsystems
  have unique names within their parent system. **/
  std::string GetSubsystemPathName() const {
    std::string path_name;
    if (get_parent_system())
      path_name = get_parent_system()->GetSubsystemPathName();
    path_name += ("/" + get_name());
    return path_name;
  }

  /** Get const access to this subsystem's subcontext given const access to any
  other subsystem's subcontext in the same context diagram. **/
  const AbstractContext3& find_my_subcontext(
      const AbstractContext3& some_subcontext) const {
    // TODO(sherm1) This should use a prebuilt index rather than searching.
    std::vector<int> path = get_path_from_root_system();
    const AbstractContext3& my_subcontext =
        some_subcontext.get_root_context().find_subcontext(path);
    return my_subcontext;
  }

  /** Get mutable access to this subsystem's subcontext given mutable access to
  any other subsystem's subcontext in the same context diagram. Note that
  you don't need mutable access to the subsystem to get mutable access to its
  subcontext. **/
  AbstractContext3* find_my_mutable_subcontext(
      AbstractContext3* some_subcontext) const {
    return const_cast<AbstractContext3*>(&find_my_subcontext(*some_subcontext));
  }

  /** Find the root system of the tree of which this subsystem is a member.
  Searches up the tree so run time is roughly log(N) for an N-subsystem
  tree starting at a leaf.**/
  const AbstractSystem3& get_root_system() const {
    if (!get_parent_system()) return *this;
    return get_parent_system()->get_root_system();
  }

  /** Get a mutable pointer to the root system of the tree of which this 
  subsystem is a member. Never returns `nullptr`. See `get_root_system()` for
  more information. **/
  AbstractSystem3* get_mutable_root_system() {
    return const_cast<AbstractSystem3*>(&get_root_system());
  }

  /**@}**/

  const InputPortFinder& get_input_port_finder(int port_num) const {
    DRAKE_ASSERT(0 <= port_num && port_num < (int)input_ports_.size());
    return input_ports_[port_num];
  }

  const OutputPortFinder& get_output_port_finder(int port_num) const {
    DRAKE_ASSERT(0 <= port_num && port_num < (int)output_ports_.size());
    return output_ports_[port_num];
  }

  /** Given an output port number, return a function that knows how to
  calculate the value of that port when given a system and context. The
  signature is suitable for registering with a cache entry that can invoke this
  function when the entry is out of date and needs updating. **/
  CacheEntry::Calculator get_output_port_calculator(int port_num) const {
    using namespace std::placeholders;  // for _1, _2, _3...
    return std::bind(&AbstractSystem3::CalcOutputPort, _1, _2, port_num, _3);
  }

  /** Unconditionally calculate what would be the output port value into
  an already-allocated appropriate result object. **/
  void CalcOutputPort(const AbstractContext3& context, int port_num,
                      AbstractValue* result) const {
    const int n = get_num_output_ports();
    if (!(0 <= port_num && port_num < n))
      throw std::out_of_range("AbstractSystem3::CalcOutputPort(): port " +
                              std::to_string(port_num) +
                              " is out of range. There are " +
                              std::to_string(n) + " output ports.");
    DoCalcOutputPort(context, port_num, result);
  }

 protected:
  explicit AbstractSystem3(const std::string& name) : name_(name) {}

  /** Obtain an up-to-date value for one of this System's input ports. If 
  necessary this triggers the computation of whatever value source this input
  port is connected to, typically an output port.

  @param[in,out] context A Context that is compatible with this System.
  @param[in] port_num The index number of the input port to be evaluated.
  @retval value A reference into `context` containing the current value for this
                output port. **/
  DRAKESYSTEMFRAMEWORK_EXPORT const AbstractValue& EvalInputPort(
      const AbstractContext3& context, int port_num) const;

  /** Allocate a concrete Context type that is appropropriate for your
  concrete System. **/
  virtual std::unique_ptr<AbstractContext3> DoCreateEmptyContext() const = 0;

  /** Acquire any private Context resources your concrete System needs, and
  assign them default values. The supplied `context` is in the process of
  being constructed and you may assume it already contains resources for your
  input and output ports. The default implementation acquires no resources. **/
  virtual void DoAcquireContextResources(AbstractContext3* context) const {}

  /** Unconditionally compute the given output port's value. When implementing
  this method, you may assume that error checking has been performed to validate
  the Context and port number, and that the output value is of the same type
  as that port's model value. **/
  virtual void DoCalcOutputPort(const AbstractContext3& context, int port_num,
                                AbstractValue* value) const = 0;

 private:
  friend class AbstractContext3;

  // Create a context that has no content but whose tree structure matches
  // that of this system diagram, with the right kind of context in each node
  // as provided by DoCreateEmptyContext().
  DRAKESYSTEMFRAMEWORK_EXPORT std::unique_ptr<AbstractContext3>
  CreateEmptyContext() const;

  // Given an empty context with appropriate tree structure, allocate and
  // connect input and output ports in the context to match the wiring of this
  // system diagram.
  DRAKESYSTEMFRAMEWORK_EXPORT void AllocateOutputPorts(
      AbstractContext3* context) const;

  // Given an empty context with appropriate tree structure, allocate and
  // connect input and output ports in the context to match the wiring of this
  // system diagram.
  DRAKESYSTEMFRAMEWORK_EXPORT void AllocateAndConnectInputPorts(
      AbstractContext3* context) const;

  /** Given a path consisting of subsystem indices starting with this subsystem,
  trace the path down the tree and return a const reference to the indicated
  subsystem. If the path is empty we just return this subsystem. Example: if the
  path were 6,2 we would return the 3rd child of this system's 7th child. The 
  `start` parameter is for internal use; you shouldn't set it. **/
  const AbstractSystem3& find_subsystem(const std::vector<int>& path,
                                       int start = 0) const {
    if (start == (int)path.size()) return *this;
    return get_subsystem(path[start]).find_subsystem(path, start + 1);
  }

  /** Returns a mutable pointer to the found subsystem. See `find_subsystem()`
  for information. **/
  AbstractSystem3* find_mutable_subsystem(const std::vector<int>& path) {
    return const_cast<AbstractSystem3*>(&find_subsystem(path));
  }

  /** Returns the path from this tree's root system to this subsystem, by
  giving the subsystem number at each level. This is particularly useful for
  locating this subsystem's corresponding subcontext in a context diagram.
  
  An empty path is returned if this is the root system. Example: if the current
  subsystem is the 3rd child of the root system's 7th child, the path would
  be 6,2. **/
  std::vector<int> get_path_from_root_system() const {
    std::vector<int> path;
    if (get_parent_system()) {
      path = get_parent_system()->get_path_from_root_system();
      DRAKE_ASSERT(get_subsystem_num() >= 0);
      path.push_back(get_subsystem_num());
    }
    return path;
  }

  // AbstractSystem3 objects are neither copyable nor moveable.
  AbstractSystem3(const AbstractSystem3& other) = delete;
  AbstractSystem3& operator=(const AbstractSystem3& other) = delete;
  AbstractSystem3(AbstractSystem3&& other) = delete;
  AbstractSystem3& operator=(AbstractSystem3&& other) = delete;

  // Set backpointer to parent system and subsystem number by which we are
  // known there.
  void set_owner(AbstractSystem3* parent, int subsystem_num) {
    owner_ = parent;
    subsystem_num_ = subsystem_num;
  }

  std::string name_;

  // These are the actual ports indexed by port number. For leaf systems these
  // simply point to the corresponding entries in the "owned" lists. For
  // system diagrams they may point to inherited ports owned by an interior
  // subsystem, and the port numbers will be different than the indices of
  // the owned ports. If you need to know the difference, ask the port for
  // the System that owns it.
  std::vector<InputPortFinder> input_ports_;
  std::vector<OutputPortFinder> output_ports_;

  // These ports are owned by this system object.
  std::vector<std::unique_ptr<InputPort3>> owned_input_ports_;
  std::vector<std::unique_ptr<OutputPort3>> owned_output_ports_;

  // These are the immediate children owned by this system, indexed by
  // "subsystem number", in order of AddSubsystem calls.
  std::vector<std::unique_ptr<AbstractSystem3>> subsystems_;

  // If this system is a subsystem in a system diagram, this is set to the
  // owning parent system, with the subsystem number as known to the parent.
  AbstractSystem3* owner_{};
  int subsystem_num_{-1};
};

/** A superclass template for systems that use a specified scalar type `T` for
numerical values. These may be instantiated with different scalar types to use
the same code to obtain different kinds of results, such as derivatives or
structural analysis of dependencies.

At run time, a `System<T>` is asked to produce a compatible `Context<T>` object
that can hold all values that can change during a simulation, cache computations
that depend on those values, and manage dependencies to prevent access to
stale computations.

@tparam T The vector element type, which must be a valid Eigen scalar. **/
template <typename T>
class System3 : public AbstractSystem3 {
  // TODO(david-german-tri): Add static_asserts on T.
 public:
  //----------------------------------------------------------------------------
  /** @name            Methods common to all Systems 

  Every System can create a compatible context, and can provide values for
  its output ports. **/
  /**@{**/

  /** Get a default `Context<T>` compatible with this `System<T>`. **/
  std::unique_ptr<Context3<T>> CreateDefaultContext() const {
    auto abstract_context = AbstractSystem3::CreateDefaultContext();
    std::unique_ptr<Context3<T>> context(
        dynamic_cast<Context3<T>*>(abstract_context.release()));
    DRAKE_ABORT_UNLESS(context != nullptr);
    return context;
  }

  /** Convenience method for obtaining the up-to-date value for an output
  port which is known to be vector valued.
  @retval vector_value A reference into `context` containing the correct
                       value for this vector-valued output port. **/
  const VectorInterface<T>& EvalVectorOutputPort(const Context3<T>& context,
                                                 int port_num) const {
    const VectorInterface<T>& vector =
        to_vector_interface<T>(EvalOutputPort(context, port_num));
    return vector;
  }
  /**@}**/

  //----------------------------------------------------------------------------
  /** @name         Methods for all *physical* Systems

  Any System that models a physical system that can store energy or carry
  kinetic energy in moving masses should provide these methods so that
  conservation of energy can be monitored. In a conservative system (one with
  no losses or actuation), the sum of potential and kinetic energy should be
  constant. That is unchanged whether the system is modeled as continuous or
  discrete.
  
  For non-conservative systems additional information is required
  to track energy gained or lost, and that is handled differently for 
  continuous and discrete models. **/
  /**@{**/
  /** Returns the potential energy currently stored in the configuration
  provided in the given Context. Non-physical Systems will return 0. **/
  const T& EvalPotentialEnergy(const Context3<T>& context) const {
    // TODO(sherm1) Validate the context at least in Debug.
    return DoEvalPotentialEnergy(context);
  }

  /** Return the kinetic energy currently present in the motion provided in the
  given Context. Non-physical Systems will return 0. **/
  const T& EvalKineticEnergy(const Context3<T>& context) const {
    // TODO(sherm1) Validate the context at least in Debug.
    return DoEvalKineticEnergy(context);
  }
  /**@}**/

  //----------------------------------------------------------------------------
  /** @name   Methods for Systems with continuous state variables **/
  /**@{**/
  enum DerivativeBlock { kQdot = 0, kVdot = 1, kZdot = 2, kQdotDot = 3 };

  /** Returns time derivatives of one or all of the continuous state variable
  groups. We consider continuous state `xc=[q v z]` where q are the
  configuration variables (second order), v are the velocity variables, and
  z are arbitrary continuous variables. The time derivative is thus
  `xcdot=[qdot vdot zdot]`. The second time derivative qdotdot is also
  available. This method will initiate compuation of the
  requested derivative; typically the `vdot` calculation will be very
  expensive. **/
  Eigen::VectorBlock<const VectorX<T>> EvalTimeDerivatives(
      const Context3<T>& context, DerivativeBlock block) const;

  /** For continuous, *physical* systems only, returns the rate at which
  energy is being converted *from* potential energy *to* kinetic energy by this
  system in the given Context. This quantity will be positive when potential
  energy is decreasing. Note that kinetic energy will also be affected by
  non-conservative forces so we can't say which direction it is moving, only
  whether the conservative power is increasing or decreasing the kinetic energy.
  Power is in watts (J/s). Returns zero for systems where this doesn't make
  sense. **/
  const T& EvalConservativePower(const Context3<T>& context) const {
    return DoEvalConservativePower(context);
  }

  /** For continuous, *physical* systems only, returns the rate at which energy
  is being added to (positive) or dissipated from (negative) this sytem 
  *other than* by conversion between potential and kinetic energy (in the given
  Context). Integrating this quantity yields work W, and the total energy
  `E=PE+KE-W` should be conserved by any physically-correct model, to within
  integration accuracy of W. Power is in watts (J/s). (Watts are abbreviated W
  but not to be confused with work!) Returns zero for systems where this doesn't
  make sense. **/
  const T& EvalNonConservativePower(const Context3<T>& context) const {
    return DoEvalNonConservativePower(context);
  }

  /** Transforms a given generalized velocity v into qdot, the time derivative
  of the generalized configuration q. The current configuration is taken from
  the given Context. Note that the velocity is given explicitly as an argument;
  any velocity in the Context is ignored.

  The transformation is linear in velocity and requires no more than O(n) time
  to compute where n is the size of q. We are computing @verbatim
    qdot = N(q) * v
  @endverbatim
  where N is a block diagonal matrix with small blocks.
  
  @param[in] context 
    The complete evaluation Context, from which we obtain q. Some cache updates
    may occur.
  @param[in] generalized_velocity
      The input velocity v to transform.
  @param[out] configuration_derivatives 
      The output vector qdot. Must not be nullptr. **/
  void MapVelocityToConfigurationDerivatives(
      const Context3<T>& context,
      const Eigen::Ref<const VectorX<T>>& generalized_velocity,
      Eigen::Ref<VectorX<T>>* configuration_derivatives) const {
    // TODO(sherm1) Validate arguments.
    DoMapVelocityToConfigurationDerivatives(context, generalized_velocity,
                                            configuration_derivatives);
  }

  /** Transforms a given generalized acceleration vdot into qdotdot, the
  second time derivative of the generalized configuration q. The current
  configuration q and velcoity v are taken from the given Context. Note that
  generalized acceleration is given explicitly as an argument; any acceleration
  present in the Context is ignored.

  The transformation is linear in acceleration and requires no more than O(n)
  time to compute where n is the size of q. We are computing @verbatim
    qdotdot = N(q) * vdot + Ndot(q,v) * v
  @endverbatim
  where N is a block diagonal matrix with small blocks, and is the same N as
  described in `MapVelocityToConfigurationDerivatives()`.
  
  @param[in] context 
      The complete evaluation Context, from which we obtain q and v. Some cache 
      updates may occur.
  @param[in] generalized_acceleration
      The input acceleration vdot to transform.
  @param[out] configuration_second_derivatives 
      The output vector qdotdot. Must not be nullptr. **/
  void MapAccelerationToConfigurationSecondDerivatives(
      const Context3<T>& context,
      const Eigen::Ref<const VectorX<T>>& generalized_acceleration,
      Eigen::Ref<VectorX<T>>* configuration_second_derivatives) const {
    // TODO(sherm1) Validate arguments.
    DoMapAccelerationToConfigurationSecondDerivatives(
        context, generalized_acceleration, configuration_second_derivatives);
  }
  /**@}**/

  //----------------------------------------------------------------------------
  /** @name   Methods for Systems with discrete state variables **/
  /**@{**/
  /** TODO: update discrete variables. **/
  // TODO(sherm1) Only a subset needs to be updated at any given sampling
  // time. Probably makes more sense to pass a writable Context so the
  // System can perform the limited update.
  void UpdateDiscreteVariables(Context3<T>& context, int sample_key) const;

  /** Returns the next sample time required by any subsystem of this System.
  This is accompanied by a "sample key" which unambiguously identifies the
  subsystems, inputs, and outputs that need to be sampled. That key should
  be returned in a call to `UpdateDiscreteVariables()` **/
  // TODO(sherm1) This is half-baked and need to be thought through better.
  // Sampling doesn't just apply to discrete variables; many internal
  // computations need sampling too.
  std::pair<double,int> GetNextSampleTime(const Context3<T>& context) const;
  /**@}**/


 protected:
  /** Creates a System with no ports. **/
  explicit System3(const std::string& name) : AbstractSystem3(name) {}

  /** Convenience method for obtaining the up-to-date value for an input
  port which is known to be vector valued. Note that this will initiate
  computation of the value if necessary.

  @retval vector_value A reference into `context` containing the correct
                       value for this vector-valued input port. **/
  const VectorInterface<T>& EvalVectorInputPort(const Context3<T>& context,
                                                int port_num) const {
    const VectorInterface<T>& vector =
        to_vector_interface<T>(EvalInputPort(context, port_num));
    return vector;
  }

  /** If your system is capable of storing energy, implement this method to
  return the potential energy currently stored in the configuration provided in
  the given Context. Otherwise the default implementation will return zero. When
  implementing this method, you may assume that error checking has been
  performed to validate the Context. **/
  virtual const T& DoEvalPotentialEnergy(const Context3<T>& context) const {
    static const T zero(0);
    return zero;
  }

  /** If your system models energy of motion, implement this method to return
  the kinetic energy currently present in the motion provided in the
  given Context. Otherwise the default implementation will return zero. When
  implementing this method, you may assume that error checking has been
  performed to validate the Context. **/
  virtual const T& DoEvalKineticEnergy(const Context3<T>& context) const {
    static const T zero(0);
    return zero;
  }

  /** If your system has configuration and velocity variables, and the mapping
  from velocity v to configuration derivatives q is not identity, then you
  must implement this method. Otherwise the default method will check that
  the size of v and q match, and will simply set `qdot[i] = v[i]`. Otherwise,
  the transformation must be linear in velocity (qdot = N(q) * v), and it must
  require no more than O(n) time to compute where n is the size of q.
  
  Implementations may assume that `configuration_derivatives` is of the same
  size as the generalized position q present in the given Context, and that
  `generalized_velocity` is the same size as the generalized velocities v in
  that Context, and should populate `configuration_derivatives` with
  elementwise-corresponding derivatives of configuration.
  
  @param context The complete evaluation context.
  @param generalized_velocity The velocity to transform.
  @param configuration_derivatives The output vector.  Must not be nullptr. **/
  void DoMapVelocityToConfigurationDerivatives(
      const Context3<T>& context,
      const Eigen::Ref<const VectorX<T>>& generalized_velocity,
      Eigen::Ref<VectorX<T>>* configuration_derivatives) const;

  /** See above for when you have to implement this method. **/
  void DoMapAccelerationToConfigurationSecondDerivatives(
      const Context3<T>& context,
      const Eigen::Ref<const VectorX<T>>& generalized_acceleration,
      Eigen::Ref<VectorX<T>>* configuration_second_derivatives) const;

  // These two power methods should be present only for continuous
  // systems that represent some kind of physical system that can inject or
  // dissipate energy into the simulation.

  /** Return the rate at which mechanical energy is being converted *from*
  potential energy *to* kinetic energy by this system in the given Context.
  This quantity will be positive when potential energy is decreasing. Note
  that kinetic energy will also be affected by non-conservative forces so we
  can't say which direction it is moving, only whether the conservative
  power is increasing or decreasing the kinetic energy. Power is in watts
  (J/s). This method is meaningful only for *continuous*, *physical* systems;
  don't implement otherwise. The default implementation returns zero. **/
  virtual const T& DoEvalConservativePower(const Context3<T>& context) const {
    static const T zero(0);
    return zero;
  }

  /** Return the rate at which mechanical energy is being generated (positive)
  or dissipated (negative) *other than* by conversion between potential and
  kinetic energy (in the given Context). Integrating this quantity yields
  work W, and the total energy `E=PE+KE-W` should be conserved by any
  physically-correct model, to within integration accuracy of W. Power is in
  watts (J/s). (Watts are abbreviated W but not to be confused with work!)
  This method is meaningful only for *continuous*, *physical* systems;
  don't implement otherwise. The default implementation returns zero. **/
  virtual const T& DoEvalNonConservativePower(const Context3<T>& context) const {
    static const T zero(0);
    return zero;
  }

 private:
  // System3 objects are neither copyable nor moveable.
  System3(const System3<T>& other) = delete;
  System3& operator=(const System3<T>& other) = delete;
  System3(System3<T>&& other) = delete;
  System3& operator=(System3<T>&& other) = delete;
};

}  // namespace systems
}  // namespace drake
