#pragma once

#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/input_port_value.h"
#include "drake/systems/framework/parameters.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

/** %LeafContext contains all prerequisite data necessary to uniquely determine
the results of computations performed by the associated LeafSystem. It also
contains mutable cache for holding the results of those computations, and
infrastructure for ensuring that those results are up to date with their
prerequisites. Here is an inventory:

##### Source data
- Time t, accuracy a (from base class)
- %State values x
- %Parameter values p
- %Value sources for input ports u
  - Stored values for locally-fixed input ports
  - References to value sources for externally-connected input ports

##### Cached computations
- Output port values
- %State derivatives and discrete update values
- Constraint errors
- User-specified cached computations

@tparam T The mathematical type of the context, which must be a valid Eigen
          scalar. */
template <typename T>
class LeafContext : public Context<T> {
 public:
  /// @name  Does not allow move or assignment; copy is protected.
  //@{
  LeafContext(LeafContext&&) = delete;
  LeafContext& operator=(const LeafContext&) = delete;
  LeafContext& operator=(LeafContext&&) = delete;
  //@}

  LeafContext()
      : state_(std::make_unique<State<T>>()) {}
  ~LeafContext() override {}

 protected:
  /// Protected copy constructor takes care of the local data members and
  /// all base class members, but doesn't update base class pointers so is
  /// not a complete copy.
  LeafContext(const LeafContext& source) : Context<T>(source) {
    // Make a deep copy of the state.
    state_ = source.CloneState();

    // Everything else was handled by the Context<T> copy constructor.
  }

  /// Derived classes should reimplement and replace this; don't invoke it.
  LeafContext<T>* DoCloneWithoutPointers() const override {
    return new LeafContext<T>(*this);
  }

  /// The caller owns the returned memory.
  State<T>* DoCloneState() const override {
    State<T>* clone = new State<T>();

    // Make a deep copy of the continuous state using BasicVector::Clone().
    const ContinuousState<T>& xc = this->get_continuous_state();
    const int num_q = xc.get_generalized_position().size();
    const int num_v = xc.get_generalized_velocity().size();
    const int num_z = xc.get_misc_continuous_state().size();
    const BasicVector<T>& xc_vector =
        dynamic_cast<const BasicVector<T>&>(xc.get_vector());
    clone->set_continuous_state(std::make_unique<ContinuousState<T>>(
        xc_vector.Clone(), num_q, num_v, num_z));

    // Make deep copies of the discrete and abstract states.
    clone->set_discrete_state(state_->get_discrete_state().Clone());
    clone->set_abstract_state(state_->get_abstract_state().Clone());

    return clone;
  }

 private:
  const State<T>& do_access_state() const final {
    DRAKE_ASSERT(state_ != nullptr);
    return *state_;
  }

  State<T>& do_access_mutable_state() final {
    DRAKE_ASSERT(state_ != nullptr);
    return *state_.get();
  }

  // The internal state x of this LeafSystem.
  std::unique_ptr<State<T>> state_;
};

}  // namespace systems
}  // namespace drake
