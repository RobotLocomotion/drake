#pragma once

#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/parameters.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/vector_base.h"

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

  LeafContext()
      : state_(std::make_unique<State<T>>()) {}
  ~LeafContext() override {}

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
  LeafContext(const LeafContext& source) : Context<T>(source) {
    // Make a deep copy of the state.
    state_ = source.CloneState();

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
    clone->set_discrete_state(state_->get_discrete_state().Clone());
    clone->set_abstract_state(state_->get_abstract_state().Clone());

    return clone;
  }

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

  /// Returns a partial textual description of the Context, intended to be
  /// human-readable.  It is not guaranteed to be unambiguous nor complete.
  std::string do_to_string() const final {
    std::ostringstream os;

    os << this->GetSystemPathname() << " Context\n";
    os << std::string(this->GetSystemPathname().size() + 9, '-') << "\n";
    os << "Time: " << this->get_time() << "\n";

    if (this->num_continuous_states() ||
        this->num_discrete_state_groups() ||
        this->num_abstract_states()) {
      os << "States:\n";
      if (this->num_continuous_states()) {
        os << "  " << this->num_continuous_states()
           << " continuous states\n";
        os << "    " << this->get_continuous_state_vector() << "\n";
      }
      if (this->num_discrete_state_groups()) {
        os << "  " << this->num_discrete_state_groups()
           << " discrete state groups with\n";
        for (int i = 0; i < this->num_discrete_state_groups(); i++) {
          os << "     " << this->get_discrete_state(i).size() << " states\n";
          os << "       " << this->get_discrete_state(i) << "\n";
        }
      }
      if (this->num_abstract_states()) {
        os << "  " << this->num_abstract_states() << " abstract states\n";
      }
      os << "\n";
    }

    if (this->num_numeric_parameter_groups() ||
        this->num_abstract_parameters()) {
      os << "Parameters:\n";
      if (this->num_numeric_parameter_groups()) {
        os << "  " << this->num_numeric_parameter_groups()
           << " numeric parameter groups";
        os << " with\n";
        for (int i = 0; i < this->num_numeric_parameter_groups(); i++) {
          os << "     " << this->get_numeric_parameter(i).size()
             << " parameters\n";
          os << "       " << this->get_numeric_parameter(i) << "\n";
        }
      }
      if (this->num_abstract_parameters()) {
        os << "  " << this->num_abstract_parameters()
           << " abstract parameters\n";
      }
    }
    return os.str();
  }


  // The state values (x) for this LeafContext; this is never null.
  std::unique_ptr<State<T>> state_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::LeafContext)
