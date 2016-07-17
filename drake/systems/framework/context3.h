#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/cache3.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

class AbstractSystem3;
class InputPort3;
class OutputPort3;

/** There is one of these corresponding to every InputPort in the System that
created the containing Context. If an input port is connected, this entry
will refer to the output port that provides its value. That output port may
be owned by a sibling context, supercontext, supersupercontext, etc. because
the input port may have been inherited by containing Systems. **/
struct InputEntryFinder {
  InputEntryFinder() = default;
  InputEntryFinder(int subcontext_num, int port_num, CacheEntry* entry)
      : subcontext_num(subcontext_num), port_num(port_num), entry(entry) {}
  InputEntryFinder(const InputEntryFinder& source) { *this = source; }
  InputEntryFinder& operator=(const InputEntryFinder& source) {
    subcontext_num = source.subcontext_num;
    port_num = source.port_num;
    entry = nullptr;
    return *this;
  }
  /** Move constructor preserves all values and clears the source. **/
  InputEntryFinder(InputEntryFinder&& source) { *this = std::move(source); }
  /** Move assignment preserves all values and clears the source. **/
  InputEntryFinder& operator=(InputEntryFinder&& source) {
    subcontext_num = source.subcontext_num;
    port_num = source.port_num;
    entry = source.entry;
    new (&source) InputEntryFinder(); // default re-construct source
    return *this;
  }
  int subcontext_num{-1};  ///< -1 means this context.
  int port_num{-1};        ///< Port number as understood by the subcontext.
  CacheEntry* entry{};     ///< Convenience pointer to output if connected.
};

/** There is one of these corresponding to every OutputPort in the System that
created the containing Context. This references a cache entry that has the
appropriate type and knows how to evaluate the output port. That entry may be
owned by this context or any contained subcontext, subsubcontext, etc. because
the output port may have been inherited by containing Systems. **/
struct OutputEntryFinder {
  OutputEntryFinder() = default;
  OutputEntryFinder(int subcontext_num, int location, CacheEntry* entry)
      : subcontext_num(subcontext_num), location(location), entry(entry) {
  
  }
  OutputEntryFinder(const OutputEntryFinder& source) { *this = source; }
  OutputEntryFinder& operator=(const OutputEntryFinder& source) {
    subcontext_num = source.subcontext_num;
    location = source.location;
    entry = nullptr;
    return *this;
  }
  /** Move constructor preserves all values and clears the source. **/
  OutputEntryFinder(OutputEntryFinder&& source) { *this = std::move(source); }
  /** Move assignment preserves all values and clears the source. **/
  OutputEntryFinder& operator=(OutputEntryFinder&& source) {
    subcontext_num = source.subcontext_num;
    location = source.location;
    entry = source.entry;
    new (&source) OutputEntryFinder(); // default re-construct source
    return *this;
  }
  int subcontext_num{-1};  ///< -1 means this context.
  int location{-1};        ///< Port number for subcontext, or entry if local.
  CacheEntry* entry{};     ///< Convenience pointer to the actual entry.
};

/** An abstract superclass for the Context3 objects for dynamical systems,
encapsulating functionality that is independent of the numerical scalar type in
use. That includes the concepts of input and output, whose values appear as
AbstractValue objects here, as well as the concept of values being out-of-date
and needing to be "realized" by opaque functions that understand the specific
types involved. **/
class AbstractContext3 {
 public:
  virtual ~AbstractContext3() {};

  /** Returns a deep copy of this Context3. The clone's time, input ports, and
  parameter ports will hold deep copies of the values that appeared on those
  ports at the time the clone was created. That ensures that the complete
  computation context is preserved. **/
  std::unique_ptr<AbstractContext3> Clone() const {
    return std::unique_ptr<AbstractContext3>(DoClone());
  }

  /** Allocate input port entries corresponding to a System's InputPort
  objects. These are initially unconnected. **/
  void SetNumInputPorts(int num_ports) {
    DRAKE_ABORT_UNLESS(num_ports >= 0);
    ClearInputPorts();
    inputs_.resize(num_ports);
  }

  /** Allocate output port entries corresponding to a System's OutputPort
  objects. These initially have no memory allocated for their values. **/
  void SetNumOutputPorts(int num_ports) {
    DRAKE_ABORT_UNLESS(num_ports >= 0);
    ClearOutputPorts();
    outputs_.resize(num_ports);
  }

  /** Set the input entry for this port number appropriately, given the
  corresponding InputPort in `system`. **/
  DRAKESYSTEMFRAMEWORK_EXPORT
  void SetInputPort(const AbstractSystem3& system, int port_num);

  /** Set the output entry for this port number appropriately, given the
  corresponding OutputPort in `system`. **/
  DRAKESYSTEMFRAMEWORK_EXPORT 
  void SetOutputPort(const AbstractSystem3& system, int port_num);

  int get_num_input_ports() const { return (int)inputs_.size(); }
  int get_num_output_ports() const { return (int)outputs_.size(); }
  int get_num_cache_entries() const { return (int)cache_.size(); }

  /** Return a const pointer to the cache entry that is providing the value for
  the given input port, if this port is connected. Otherwise returns
  `nullptr`. **/
  const CacheEntry* get_input_entry(int port_num) const {
    return get_input_entry_finder(port_num).entry;
  }

  /** Return a mutable pointer to the cache entry that is providing the value
  for the given input port, if this port is connected. Otherwise returns
  `nullptr`. **/
  CacheEntry* get_mutable_input_entry(int port_num) const {
    return get_input_entry_finder(port_num).entry;
  }

  const CacheEntry& get_output_entry(int port_num) const {
    return *get_output_entry_finder(port_num).entry;
  }

  CacheEntry* get_mutable_output_entry(int port_num) const {
    return get_output_entry_finder(port_num).entry;
  }

  /** @name            System diagram methods **/
  /**@{**/

  /** Add a subcontext as a child of this context. This must be done to 
  exactly mirror the system/subsystem structure of the compatible system and
  is intended only for internal use. **/
  int AddSubcontext(std::unique_ptr<AbstractContext3> subcontext) {
    const int subcontext_num = get_num_subcontexts();
    subcontext->set_owner(this, subcontext_num);
    subcontexts_.push_back(std::move(subcontext));
    return subcontext_num;
  }

  /** Returns the current number of subcontexts contained in this
  context. This is also the index that will be assigned to the next
  subcontext that is added here. **/
  int get_num_subcontexts() const { return (int)subcontexts_.size(); }

  /** Returns a const reference to one of the contained subcontexts, using the
  index reflecting the order in which it was added. Note that cache entries
  are still mutable in a const Context3. **/
  const AbstractContext3& get_subcontext(int index) const {
    return *subcontexts_[index];
  }

  /** Returns a mutable pointer to one of the contained subcontexts, using the
  index reflecting the order in which it was added. This does not cause any
  invalidation or notifications. **/
  AbstractContext3* get_mutable_subcontext(int index) {
    return subcontexts_[index].get();
  }

  /** Returns a const pointer to the parent context that owns this one, or
  `nullptr` if this is a root context. **/
  const AbstractContext3* get_parent_context() const { return owner_; }

  /** Returns a mutable pointer to the parent context that owns this one, or
  `nullptr` if this is a root context. **/
  AbstractContext3* get_mutable_parent_context() { return owner_; }

  /** If this context is a subcontext of a parent context, return the subcontext
  number by which that parent knows us. Returns -1 if this is a root
  context. **/
  int get_subcontext_num() const { return subcontext_num_; }


  /** Get const access to this subcontext's subsystem given const access to any
  other subcontext's subsystem in the same system diagram. **/
  const AbstractSystem3& find_my_subsystem(
      const AbstractSystem3& some_subsystem) const;

  /** Get mutable access to this subcontext's subsystem given mutable access to
  any other subcontext's subsystem in the same system diagram. Note that
  you don't need mutable access to the subcontext to get mutable access to its
  subsystem. **/
  AbstractSystem3* find_my_mutable_subsystem(
      AbstractSystem3* some_subsystem) const {
    return const_cast<AbstractSystem3*>(&find_my_subsystem(*some_subsystem));
  }


  /** Find the root context of the tree of which this subcontext is a member.
  Searches up the tree so run time is roughly log(N) for an N-subcontext
  tree starting at a leaf. **/
  const AbstractContext3& get_root_context() const {
    if (!get_parent_context()) return *this;
    return get_parent_context()->get_root_context();
  }

  /** Get a mutable pointer to the root context of the tree of which this 
  subcontext is a member. Never returns `nullptr`. See `get_root_context()` for
  more information. **/
  AbstractContext3* get_mutable_root_context() {
    return const_cast<AbstractContext3*>(&get_root_context());
  }

  /**@}**/

 protected:
   /** Create an empty %AbstractContext3. **/
   AbstractContext3() = default;

   /** Copy the base class data members and preserve the input and parameter
   port values. **/
   AbstractContext3(const AbstractContext3& source)
       : inputs_(source.inputs_),
         outputs_(source.outputs_),
         cache_(source.cache_) {
     // TODO(sherm1) input and output pointers are null now an need fixup.

     // TODO(sherm1) copy subcontexts
     //for (const auto& sub : source.subcontexts_)
     //  subcontexts_.emplace_back(sub->Clone());
   }

   /** Create a copy of the concrete Context3 object, using the %AbstractContext3
   copy constructor to deal with base class copying. **/
   virtual AbstractContext3* DoClone() const = 0;

 private:
  friend class AbstractSystem3;

  const InputEntryFinder& get_input_entry_finder(int port_num) const {
    return inputs_[port_num];
  }

  const OutputEntryFinder& get_output_entry_finder(int port_num) const {
    return outputs_[port_num];
  }

  InputEntryFinder* get_mutable_input_entry_finder(int port_num) {
    return &inputs_[port_num];
  }

  OutputEntryFinder* get_mutable_output_entry_finder(int port_num) {
    return &outputs_[port_num];
  }

  // Removes all the input ports, and deregisters them from the output ports
  // on which they depend.
  // TODO(sherm1) Actually do that.
  void ClearInputPorts() { inputs_.clear(); }

  // Removes all the output ports, and disconnects any input ports that might
  // have been plugged in to them.
  // TODO(sherm1) Actually do that.
  void ClearOutputPorts() { outputs_.clear(); }

  /** Given a path consisting of subcontext indices starting with this
  subcontext, trace the path down the tree and return a const reference to the
  indicated subcontext. If the path is empty we just return this subcontext.
  Example: if the path were 6,2 we would return the 3rd child of this context's
  7th child. The `start` parameter is for internal use; you shouldn't set
  it. **/
  const AbstractContext3& find_subcontext(const std::vector<int>& path,
                                       int start = 0) const {
    if (start == (int)path.size()) return *this;
    return get_subcontext(path[start]).find_subcontext(path, start + 1);
  }

  /** Returns a mutable pointer to the found subcontext. See `find_subcontext()`
  for information. **/
  AbstractContext3* find_mutable_subcontext(const std::vector<int>& path) {
    return const_cast<AbstractContext3*>(&find_subcontext(path));
  }

  /** Returns the path from this tree's root context to this subcontext, by
  giving the subcontext number at each level. This is particularly useful for
  locating this subcontext's corresponding subsystem in a system diagram.
  
  An empty path is returned if this is the root context. Example: if the current
  subcontext is the 3rd child of the root context's 7th child, the path would
  be 6,2. **/
  std::vector<int> get_path_from_root_context() const {
    std::vector<int> path;
    if (get_parent_context()) {
      path = get_parent_context()->get_path_from_root_context();
      DRAKE_ASSERT(get_subcontext_num() >= 0);
      path.push_back(get_subcontext_num());
    }
    return path;
  }

  // Set backpointer to parent context and subcontext number by which we are
  // known there.
  void set_owner(AbstractContext3* parent, int subcontext_num) {
    owner_ = parent;
    subcontext_num_ = subcontext_num;
  }

  std::vector<InputEntryFinder> inputs_;
  // TODO(sherm1) Add parameter entries.

  std::vector<OutputEntryFinder> outputs_;


  // The cache, consisting of a pool of cache entries that are individually
  // validated and invalidated, and upon which other calculations may depend.
  mutable std::vector<CacheEntry> cache_;


  // Entries here correspond directly to System subsystems, using the
  // same index. Any of these may be context diagrams recursively.
  std::vector<std::unique_ptr<AbstractContext3>> subcontexts_;

  // If this context is a subcontext in a context diagram, this is set to the
  // owning parent context, with the subcontext number as known to the parent.
  AbstractContext3* owner_{};
  int subcontext_num_{-1};
};

/// Contains information about the independent variable including time and
/// step number.
// TODO(sherm1) Add step information.
template <typename T>
struct StepInfo {
  /// The time, in seconds. For typical T implementations based on
  /// doubles, time resolution will gradually degrade as time increases.
  // TODO(sherm1): Consider whether this is sufficiently robust.
  T time_sec{};
};

/** The context is a container for all data necessary to uniquely determine the
results of computations performed by a System. Specifically, a Context3 contains
 - the current value of time
 - pointers to values for a set of InputPorts (and ParameterPorts)
 - values for the state variables
 - cache entries for computed OutputPort values
 - cache entries for time derivatives of continuous states
 - a cache entry for update values of discrete states

A Context is *compatible* with a particular System, but is otherwise independent
of any actual System object. That is, if you had two identical System objects
you could use the same Context object for both; there are no pointers from
Context to non-static System data or methods.

Context may be subclassed within the framework to support specialized kinds
of Systems, such as Diagrams, but should not be subclassed by users.

@tparam T The mathematical type of the context, which must be a valid Eigen
          scalar. **/
// TODO(david-german-tri): Manage cache invalidation.
template <typename T>
class Context3 : public AbstractContext3 {
 public:
  Context3() {}
  ~Context3() override {}

  /// Returns the current time in seconds.
  const T& get_time() const { return get_step_info().time_sec; }

  /// Set the current time in seconds.
  void set_time(const T& time_sec) {
    get_mutable_step_info()->time_sec = time_sec;
  }

  //const State<T>& get_state() const { return state_; }

  /// Returns writable access to the State. No cache invalidation occurs until
  /// mutable access is requested for particular blocks of state variables.
  //State<T>* get_mutable_state() { return &state_; }

  /** Returns a deep copy of this Context3. The clone's input ports will hold
  deep copies of the data that appears on this context's input ports at the time
  the clone is created. This is a non-virtual "covariant" replacement for the
  base class `Clone()` method with a more derived pointer type. **/
  std::unique_ptr<Context3<T>> Clone() const {
    return std::unique_ptr<Context3<T>>(DoClone());
  }

  /** Copy constructor makes a deep copy of the `source` Context3, and also
  copies the current values of time and any connected input and parameter ports
  into standalone internal values so that the entire computation context is
  preserved. **/
  Context3(const Context3& source) : AbstractContext3(source) {

  }

 protected:
  /// The Context implementation for Diagrams must override this method, since
  /// the state of a Diagram will not be a LeafStateVector. The caller owns the
  /// returned memory.
  Context3<T>* DoClone() const override {
    Context3<T>* context = new Context3<T>();

    // TODO(sherm1)

    //// Make a deep copy of the state using LeafStateVector::Clone().
    //const ContinuousState<T>& xc = *this->get_state().continuous_state;
    //const int num_q = xc.get_generalized_position().size();
    //const int num_v = xc.get_generalized_velocity().size();
    //const int num_z = xc.get_misc_continuous_state().size();
    //const LeafStateVector<T>& xc_vector =
    //    dynamic_cast<const LeafStateVector<T>&>(xc.get_state());
    //context->get_mutable_state()->continuous_state.reset(
    //    new ContinuousState<T>(xc_vector.Clone(), num_q, num_v, num_z));

    //// Make deep copies of the inputs into FreestandingInputPorts.
    //// TODO(david-german-tri): Preserve version numbers as well.
    //for (const auto& port : this->inputs_) {
    //  context->inputs_.emplace_back(new FreestandingInputPort<T>(
    //      port->get_vector_data()->Clone(), port->get_sample_time_sec()));
    //}

    //// Make deep copies of everything else using copy assignment.
    //*context->get_mutable_step_info() = this->get_step_info();
    //*context->get_mutable_cache() = this->get_cache();
    return context;
  }

 private:
  // Returns a const reference to current time and step information.
  const StepInfo<T>& get_step_info() const { return step_info_; }

  // Provides writable access to time and step information, with the side
  // effect of invaliding any computation that is dependent on them.
  // TODO(david-german-tri) Invalidate all cached time- and step-dependent
  // computations.
  StepInfo<T>* get_mutable_step_info() { return &step_info_; }

  // Context3 objects are neither assignable nor moveable, but they are
  // copy constructible.
  Context3& operator=(const Context3& other) = delete;
  Context3(Context3&& other) = delete;
  Context3& operator=(Context3&& other) = delete;

  // Typed independent quantities: time and state.

  // Current time and step information. Think of this as a "time port".
  // TODO(sherm1) Consider literally making this a port.
  StepInfo<T> step_info_;

  // The internal state of the System.
  //State<T> state_;

  // Typed dependent quantities: state derivatives, discrete state updates.
  // TODO(sherm1) These should be VectorCacheEntry<T>.

  // Continuous state time derivative values (numerical vectors). Together
  // these are xc_dot.
  mutable CacheEntry qdot_, udot_, zdot_;

  // Second time-derivative of the configuration states q (numerical vector).
  mutable CacheEntry qdotdot_;

  // Discrete state updates (numerical vector). This is xd_hat.
  mutable CacheEntry updates_;
};

}  // namespace systems
}  // namespace drake
