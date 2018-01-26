#pragma once

#include <sstream>

#include "drake/examples/rod2d/rod2d.h"
#include "drake/examples/rod2d/rod_witness_function.h"

#include "drake/multibody/constraint/constraint_solver.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace examples {
namespace rod2d {

/// A witness function that is used to determine when the forces used to
/// enforce non-sliding contact move outside of the friction cone.
template <class T>
class StickingFrictionForcesSlackWitness : public RodWitnessFunction<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StickingFrictionForcesSlackWitness)

  StickingFrictionForcesSlackWitness(const Rod2D<T>* rod, int contact_index) :
      RodWitnessFunction<T>(
          rod, 
          systems::WitnessFunctionDirection::kPositiveThenNonPositive,
          contact_index) {
    std::ostringstream oss;
    oss << "StickingFrictionForcesSlack (" << contact_index << ")";
    this->set_name(oss.str());
    solver_ = &rod->solver_;
  }

  typename RodWitnessFunction<T>::WitnessType
      get_witness_function_type() const override {  
    return RodWitnessFunction<T>::WitnessType::kStickingFrictionForceSlack;
  }

 private:
  T DoEvaluate(const systems::Context<T>& context) const override {
    using std::sin;
    using std::abs;

    // Get the rod system.
    const Rod2D<T>& rod = this->get_rod();

    // Verify the system is simulated using piecewise DAE.
    DRAKE_DEMAND(rod.get_simulation_type() ==
        Rod2D<T>::SimulationType::kPiecewiseDAE);

    // Get the contact information.
    const int contact_index = this->get_contact_index();
    const auto& contact =
        rod.get_contacts_used_in_force_calculations(
        context.get_state())[contact_index];

    // Verify rod is not undergoing sliding contact at the specified index.
    DRAKE_DEMAND(contact.sliding_type ==
        multibody::constraint::SlidingModeType::kNotSliding);

    // TODO(edrumwri): Speed this up (presumably) using caching. 

    // Populate problem data and solve the contact problem.
    const int ngv = 3;  // Number of rod generalized velocities.
    VectorX<T> cf;
    multibody::constraint::ConstraintAccelProblemData<T> problem_data(ngv);
    rod.CalcConstraintProblemData(context, context.get_state(), &problem_data);
    solver_->SolveConstraintProblem(problem_data, &cf);

    // Get the index of this contact in the force calculation set.
    const int force_index = rod.GetContactArrayIndex(
        context.get_state(), contact_index);

    // Determine the index of this contact in the non-sliding constraint set.
    const std::vector<int>& sliding_contacts = problem_data.sliding_contacts;
    const std::vector<int>& non_sliding_contacts =
        problem_data.non_sliding_contacts;
    DRAKE_ASSERT(std::is_sorted(non_sliding_contacts.begin(),
                                non_sliding_contacts.end()));
    const int non_sliding_index = std::distance(
        non_sliding_contacts.begin(),
        std::lower_bound(non_sliding_contacts.begin(),
                         non_sliding_contacts.end(),
                         force_index));
    const int num_sliding = sliding_contacts.size();
    const int num_non_sliding = non_sliding_contacts.size();
    const int nc = num_sliding + num_non_sliding;
    const int k = rod.get_num_tangent_directions_per_contact();
    const int r = k / 2;

    // Get the normal force and the l1-norm of the frictional force.
    const auto fN = cf[force_index];
    const auto fF = cf.segment(nc + non_sliding_index * r, r).template
        lpNorm<1>();

    // Determine the slack.
    return problem_data.mu_non_sliding[non_sliding_index] * fN - fF;
  }

  /// Pointer to the rod's constraint solver.
  const multibody::constraint::ConstraintSolver<T>* solver_;
};

}  // namespace rod2d
}  // namespace examples
}  // namespace drake

