#pragma once

#include <memory>
#include <string>

#include "drake/common/default_scalars.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace systems {

/** %LeafContext contains all prerequisite data necessary to uniquely determine
the results of computations performed by the associated LeafSystem.
@see Context for more information.

@tparam_default_scalar
*/
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

  LeafContext();
  ~LeafContext() override;

#ifndef DRAKE_DOXYGEN_CXX
  // Temporarily promoting these to public so that LeafSystem and testing code
  // can construct a LeafContext with state & parameters. Users should never
  // call these because state & parameters should not be resized once allocated
  // (or at least should be done under Framework control so that dependency
  // tracking can be correctly revised).
  // TODO(sherm1) Make these inaccessible to users. See discussion in PR #9029.
  using Context<T>::init_continuous_state;
  using Context<T>::init_discrete_state;
  using Context<T>::init_abstract_state;
  using Context<T>::init_parameters;
#endif

 protected:
  /// Protected copy constructor takes care of the local data members and
  /// all base class members, but doesn't update base class pointers so is
  /// not a complete copy.
  LeafContext(const LeafContext& source);

  /// Derived classes should reimplement and replace this; don't recursively
  /// invoke it.
  std::unique_ptr<ContextBase> DoCloneWithoutPointers() const override;

  std::unique_ptr<State<T>> DoCloneState() const override;

 private:
  friend class LeafContextTest;
  using ContextBase::AddInputPort;    // For LeafContextTest.
  using ContextBase::AddOutputPort;
  using ContextBase::AddDiscreteStateTicket;
  using ContextBase::AddAbstractStateTicket;
  using ContextBase::AddNumericParameterTicket;
  using ContextBase::AddAbstractParameterTicket;

  const State<T>& do_access_state() const final {
    DRAKE_ASSERT(state_ != nullptr);
    return *state_;
  }

  State<T>& do_access_mutable_state() final {
    DRAKE_ASSERT(state_ != nullptr);
    return *state_;
  }

  void notify_set_system_id(internal::SystemId id) final;

  /// Returns a partial textual description of the Context, intended to be
  /// human-readable.  It is not guaranteed to be unambiguous nor complete.
  std::string do_to_string() const final;

  // The state values (x) for this LeafContext; this is never null.
  std::unique_ptr<State<T>> state_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::LeafContext)
