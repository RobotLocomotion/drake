#include "drake/multibody/plant/sap_driver.h"

#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/contact_solvers/sap/sap_friction_cone_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_holonomic_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_limit_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_solver.h"
#include "drake/multibody/contact_solvers/sap/sap_solver_results.h"

using drake::multibody::contact_solvers::internal::SapConstraint;
using drake::multibody::contact_solvers::internal::SapContactProblem;
using drake::multibody::contact_solvers::internal::SapFrictionConeConstraint;
using drake::multibody::contact_solvers::internal::SapHolonomicConstraint;
using drake::multibody::contact_solvers::internal::SapLimitConstraint;
using drake::multibody::contact_solvers::internal::SapSolver;
using drake::multibody::contact_solvers::internal::SapSolverResults;
using drake::multibody::contact_solvers::internal::SapSolverStatus;

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
SapDriver<T>::SapDriver(CompliantContactManager<T>* manager)
    : manager_(manager) {
  (void)manager;
  // Declare cache entries that SAP might need.
  // Maybe even allocate data/info SAP might need (joint locking stuff?)
}

template <typename T>
void SapDriver<T>::DeclareCacheEntries(
    CompliantContactManager<T>* mutable_manager) {
  // TODO: consider a public accessor for cache entries instead of friendship?
  // Though I imagine I'll need to call methods now private.... Maybe those
  // belong to an even tighter class?
  const auto& contact_problem_cache_entry = mutable_manager->DeclareCacheEntry(
      "Contact Problem.",
      systems::ValueProducer(this, ContactProblemCache<T>(plant().time_step()),
                             &SapDriver<T>::CalcContactProblemCache),
      {plant().cache_entry_ticket(
          manager().cache_indexes_.discrete_contact_pairs)});
  contact_problem_ = contact_problem_cache_entry.cache_index();
}

template <typename T>
const ContactProblemCache<T>&
SapDriver<T>::EvalContactProblemCache(
    const systems::Context<T>& context) const {
  return plant()
      .get_cache_entry(contact_problem_)
      .template Eval<ContactProblemCache<T>>(context);
}

template <typename T>
void SapDriver<T>::CalcContactProblemCache(
    const systems::Context<T>& context, ContactProblemCache<T>* cache) const {
  SapContactProblem<T>& problem = *cache->sap_problem;
  (void)problem;
  (void)context;
#if 0  
  std::vector<MatrixX<T>> A;
  CalcLinearDynamicsMatrix(context, &A);
  VectorX<T> v_star;
  CalcFreeMotionVelocities(context, &v_star);
  problem.Reset(std::move(A), std::move(v_star));
  // N.B. All contact constraints must be added before any other constraint
  // types. This manager assumes this ordering of the constraints in order to
  // extract contact impulses for reporting contact results.
  // Do not change this order here!
  cache->R_WC = AddContactConstraints(context, &problem);
  AddLimitConstraints(context, problem.v_star(), &problem);
  AddCouplerConstraints(context, &problem);
#endif
}

template <typename T>
void SapDriver<T>::CalcContactSolverResults(
    const systems::Context<T>& context,
    contact_solvers::internal::ContactSolverResults<T>* results) const {
  (void)context;
  (void)results;
#if 0        
  const MultibodyPlant<T>& plant = manager().plant();

  const ContactProblemCache<T>& contact_problem_cache =
      EvalContactProblemCache(context);
  const SapContactProblem<T>& sap_problem = *contact_problem_cache.sap_problem;

  // We use the velocity stored in the current context as initial guess.
  const VectorX<T>& x0 =
      context.get_discrete_state(this->multibody_state_index()).value();
  const auto v0 = x0.bottomRows(this->plant().num_velocities());

  // Solve contact problem.
  SapSolver<T> sap;
  sap.set_parameters(sap_parameters_);
  SapSolverResults<T> sap_results;
  const SapSolverStatus status =
      sap.SolveWithGuess(sap_problem, v0, &sap_results);
  if (status != SapSolverStatus::kSuccess) {
    const std::string msg = fmt::format(
        "The SAP solver failed to converge at simulation time = {:7.3g}. "
        "Reasons for divergence and possible solutions include:\n"
        "  1. Externally applied actuation values diverged due to external "
        "     reasons to the solver. Revise your control logic.\n"
        "  2. External force elements such as spring or bushing elements can "
        "     lead to unstable temporal dynamics if too stiff. Revise your "
        "     model and consider whether these forces can be better modeled "
        "     using one of SAP's compliant constraints. E.g., use a distance "
        "     constraint instead of a spring element.\n"
        "  3. Numerical ill conditioning of the model caused by, for instance, "
        "     extremely large mass ratios. Revise your model and consider "
        "     whether very small objects can be removed or welded to larger "
        "     objects in the model.",
        context.get_time());
    throw std::runtime_error(msg);
  }

  const std::vector<DiscreteContactPair<T>>& discrete_pairs =
      EvalDiscreteContactPairs(context);
  const int num_contacts = discrete_pairs.size();

  PackContactSolverResults(sap_problem, num_contacts, sap_results,
                           contact_results);
#endif
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::internal::SapDriver);