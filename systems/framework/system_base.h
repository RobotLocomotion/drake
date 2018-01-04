#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_throw.h"
#include "drake/common/unused.h"
#include "drake/systems/framework/context_base.h"
#include "drake/systems/framework/framework_common.h"

namespace drake {
namespace systems {

/** Provides non-templatized functionality shared by the templatized System
classes. */
// TODO(sherm1) This is a stub for now with just enough to allow us to
// test DependencyTracker. However the code that's here should be reviewed.
class SystemBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SystemBase)

  virtual ~SystemBase() = default;

  /** Returns a Context suitable for use with this System. Context resources
  are allocated based on resource requests that were made during System
  construction. ContextBase resources are added directly; derived classes
  are asked to add theirs via DoAcquireContextResources(). Derived classes
  also provide the concrete Context object and may reject the final result
  if they impose restrictions on the kind of System they support. */
  std::unique_ptr<ContextBase> AllocateContext() const {
    std::unique_ptr<ContextBase> context = MakeContext();
    MakeContextConnections(context.get());
    AcquireContextResources(context.get());
    return context;
  }

  /** (Internal use only) */
  std::unique_ptr<ContextBase> MakeContext() const;

  /** (Internal use only) */
  void MakeContextConnections(ContextBase* context) const {
    DRAKE_DEMAND(context != nullptr);
    DoMakeContextConnections(&*context);
  }

  /** (Internal use only) */
  void AcquireContextResources(ContextBase* context) const;

  // TODO(sherm1) DeclareCacheEntry methods go here.

  //============================================================================
  /** @name                     Dependency Tickets
  Use these tickets to declare well-known sources as prerequisites of a
  downstream computation such as an output port, derivative, update, or cache
  entry. The ticket numbers for these sources are the same for all subsystems.
  For time and accuracy they refer to the same global resource; otherwise they
  refer to the specified sources within the referencing subsystem.

  A dependency ticket for a more specific resource (a particular input or
  output port, a discrete variable group, abstract state variable, a parameter,
  or a cache entry) is allocated and stored with the resource when it is
  declared. Usually the tickets are obtained directly from the resource but
  you can recover them with methods here knowning only the resource index. */
  //@{

  /** Returns a ticket indicating dependence on every possible independent
  source value, including time, state, input ports, parameters, and the accuracy
  setting (but not cache entries). This is the default dependency for
  computations that have not specified anything more refined. */
  static constexpr DependencyTicket all_sources_ticket() {
    return all_sources_ticket_;
  }

  /** Returns a ticket indicating that a computation does not depend on *any*
  source value; that is, it is a constant. If this appears in a prerequisite
  list, it must be the only entry. */
  static constexpr DependencyTicket nothing_ticket() {
    return nothing_ticket_;
  }

  /** Returns a ticket indicating dependence on time. This is the same ticket
  for all subsystems and refers to the same time value. */
  static constexpr DependencyTicket time_ticket() { return time_ticket_; }

  /** Returns a ticket indicating dependence on the accuracy setting in the
  Context. This is the same ticket for all subsystems and refers to the same
  accuracy value. */
  static constexpr DependencyTicket accuracy_ticket() {
    return accuracy_ticket_;
  }

  /** Returns a ticket indicating that a computation depends on configuration
  state variables q. */
  static constexpr DependencyTicket q_ticket() { return q_ticket_; }

  /** Returns a ticket indicating dependence on velocity state variables v. This
  does _not_ also indicate a dependence on configuration variables q -- you must
  list that explicitly or use kinematics_ticket() instead. */
  static constexpr DependencyTicket v_ticket() { return v_ticket_; }

  /** Returns a ticket indicating dependence on all of the miscellaneous
  continuous state variables z. */
  static constexpr DependencyTicket z_ticket() { return z_ticket_; }

  /** Returns a ticket indicating dependence on all of the continuous
  state variables q, v, or z. */
  static constexpr DependencyTicket xc_ticket() { return xc_ticket_; }

  /** Returns a ticket indicating dependence on all of the numerical
  discrete state variables, in any discrete variable group. */
  static constexpr DependencyTicket xd_ticket() { return xd_ticket_; }

  /** Returns a ticket indicating dependence on all of the abstract
  state variables in the current Context. */
  static constexpr DependencyTicket xa_ticket() { return xa_ticket_; }

  /** Returns a ticket indicating dependence on _all_ state variables x in this
  subsystem, including continuous variables xc, discrete (numeric) variables xd,
  and abstract state variables xa. This does not imply dependence on time,
  parameters, or inputs; those must be specified separately. If you mean to
  express dependence on all possible value sources, use all_sources_ticket()
  instead. */
  static constexpr DependencyTicket all_state_ticket() {
    return x_ticket_;
  }

  /** Returns a ticket for the cache entry that holds time derivatives of
  the continuous variables. */
  static constexpr DependencyTicket xcdot_ticket() { return xcdot_ticket_; }

  /** Returns a ticket for the cache entry that holds the discrete state
  update for the numerical discrete variables in the state. */
  static constexpr DependencyTicket xdhat_ticket() { return xdhat_ticket_; }

  /** Returns a ticket indicating dependence on all the configuration
  variables for this System. By default this is set to the continuous
  second-order state variables q, but configuration may be represented
  differently in some systems (discrete ones, for example), in which case this
  ticket should have been set to depend on that representation. */
  static constexpr DependencyTicket configuration_ticket() {
    return configuration_ticket_;
  }

  /** Returns a ticket indicating dependence on all of the velocity variables
  for this System. By default this is set to the continuous state variables v,
  but velocity may be represented differently in some systems (discrete ones,
  for example), in which case this ticket should have been set to depend on that
  representation. */
  static constexpr DependencyTicket velocity_ticket() {
    return velocity_ticket_;
  }

  /** Returns a ticket indicating dependence on all of the configuration
  and velocity state variables of this System. This ticket depends on the
  configuration_ticket and the velocity_ticket.
  @see configuration_ticket(), velocity_ticket() */
  static constexpr DependencyTicket kinematics_ticket() {
    return kinematics_ticket_;
  }

  /** Returns a ticket indicating dependence on _all_ parameters p in this
  subsystem, including numeric parameters pn, and abstract parameters pa. */
  static constexpr DependencyTicket all_parameters_ticket() {
    return all_parameters_ticket_;
  }

  /** Returns a ticket indicating dependence on _all_ input ports u of this
  subsystem. */
  static constexpr DependencyTicket all_input_ports_ticket() {
    return all_input_ports_ticket_;
  }

  // TODO(sherm1) Methods for port, cache, state, parameter tickets go here.
  //@}

  #ifndef DRAKE_DOXYGEN_CXX
  // (Internal use only) Assigns the next unused dependency ticket number,
  // unique only within a particular subsystem. Each call to this method
  // increments the ticket number.
  DependencyTicket assign_next_dependency_ticket() {
    return next_available_ticket_++;
  }
  #endif  // DRAKE_DOXYGEN_CXX

 protected:
  SystemBase() = default;

  /** Derived class implementations should allocate a suitable
  default-constructed Context, with default-constructed subcontexts for
  diagrams. The base class allocates trackers for known resources and
  intra-subcontext dependencies. No inter-subcontext dependencies should be
  made in this step. */
  virtual std::unique_ptr<ContextBase> DoMakeContext() const = 0;

  /** If the derived class is a diagram it should implement this method to
  set up the inter-subcontext dependencies. The given `context` already has
  the right structure and each subcontext has trackers available for each of
  its resources.  The supplied context is guaranteed to be
  non-null; you don't need to error-check that. */
  virtual void DoMakeContextConnections(ContextBase* context) const {
    unused(context);
  }

  /** Derived classes should override to complete resource allocation, and to
  validate that the Context resource collection is acceptable. The
  supplied Context is the one returned earlier from DoMakeContext(), with all
  base class resources allocated. The supplied context is guaranteed to be
  non-null; you don't need to error-check that. */
  virtual void DoAcquireContextResources(ContextBase* context) const = 0;

  /** Derived classes must implement this to verify that the supplied
  context is suitable, and throw an exception if not. */
  virtual void DoCheckValidContext(const ContextBase&) const = 0;

 private:
  void CreateSourceTrackers(ContextBase*) const;

  // Ports and cache entries hold their own DependencyTickets. Note that the
  // addresses of the elements are stable even if the std::vectors are resized.

  // TODO(sherm1) Ports and cache entries go here.

  // DependencyTickets are assigned during System construction, then used both
  // in System and Context so it is easy and very fast to find the
  // DependencyTracker that goes with a given ticket.

  // These built-in sources have the same ticket number in every System.
  static constexpr DependencyTicket
      nothing_ticket_        { 0, true},
      time_ticket_           { 1, true},  // t (second argument is a dummy)
      accuracy_ticket_       { 2, true},  // a
      q_ticket_              { 3, true},
      v_ticket_              { 4, true},
      z_ticket_              { 5, true},
      xc_ticket_             { 6, true},
      xd_ticket_             { 7, true},
      xa_ticket_             { 8, true},
      x_ticket_              { 9, true},  // x
      configuration_ticket_  {10, true},  // typically just q
      velocity_ticket_       {11, true},  // typically just v
      kinematics_ticket_     {12, true},  // configuration + velocity
      all_parameters_ticket_ {13, true},  // p
      all_input_ports_ticket_{14, true},  // u
      all_sources_ticket_    {15, true},  // t,a,x,p,u
      xcdot_ticket_          {16, true},  // xc time derivatives cache entry
      xdhat_ticket_          {17, true};  // xd update cache entry

  DependencyTicket next_available_ticket_{xdhat_ticket_ + 1};

  // TODO(sherm1) Tickets for state & parameters go here.
};

}  // namespace systems
}  // namespace drake
