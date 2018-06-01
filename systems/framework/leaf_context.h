#pragma once

#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/parameters.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

/// %LeafContext contains all prerequisite data necessary to uniquely determine
/// the results of computations performed by the associated LeafSystem.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
template <typename T>
class LeafContext : public Context<T> {
 public:
  /// @name  Does not allow copy, move, or assignment.
  //@{
  // Copy constructor is protected for use in implementing Clone().
  LeafContext(LeafContext&&) = delete;
  LeafContext& operator=(const LeafContext&) = delete;
  LeafContext& operator=(LeafContext&&) = delete;
  //@}

  LeafContext()
      : state_(std::make_unique<State<T>>()),
        parameters_(std::make_unique<Parameters<T>>()) {}
  ~LeafContext() override {}

  const State<T>& get_state() const final {
    DRAKE_ASSERT(state_ != nullptr);
    return *state_;
  }

  State<T>& get_mutable_state() final {
    DRAKE_ASSERT(state_ != nullptr);
    return *state_.get();
  }

  // =========================================================================
  // Accessors and Mutators for Parameters.

  /// Sets the parameters to @p params, deleting whatever was there before.
  void set_parameters(std::unique_ptr<Parameters<T>> params) {
    parameters_ = std::move(params);
  }

  /// Returns the entire Parameters object.
  const Parameters<T>& get_parameters() const final {
    return *parameters_;
  }

  /// Returns the entire Parameters object.
  Parameters<T>& get_mutable_parameters() final {
    return *parameters_;
  }

 protected:
  /// Protected copy constructor takes care of the local data members and
  /// all base class members, but doesn't update base class pointers so is
  /// not a complete copy.
  LeafContext(const LeafContext& source) : Context<T>(source) {
    // Make a deep copy of the state.
    state_ = source.CloneState();

    // Make deep copies of the parameters.
    set_parameters(source.parameters_->Clone());

    // Everything else was handled by the Context<T> copy constructor.
  }

  /// Derived classes should reimplement and replace this; don't recursively
  /// invoke it.
  std::unique_ptr<ContextBase> DoCloneWithoutPointers() const override {
    return std::unique_ptr<ContextBase>(new LeafContext<T>(*this));
  }

  std::unique_ptr<State<T>> DoCloneState() const override {
    auto clone = std::make_unique<State<T>>();

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
    clone->set_discrete_state(get_state().get_discrete_state().Clone());
    clone->set_abstract_state(get_state().get_abstract_state().Clone());

    return clone;
  }

 private:
  // The internal state of the System.
  std::unique_ptr<State<T>> state_;

  // The parameters of the system.
  std::unique_ptr<Parameters<T>> parameters_;
};

}  // namespace systems
}  // namespace drake
