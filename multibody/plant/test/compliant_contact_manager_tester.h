#pragma once

#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/multibody/plant/compliant_contact_manager.h"

namespace drake {
namespace multibody {
namespace internal {

// Helper class for the testing of CompliantContactManager. It provides access
// to a selection of private functions in CompliantContactManager and a number
// of helper methods.
class CompliantContactManagerTester {
 public:
  template <typename T>
  static const internal::SpanningForest& get_forest(
      const CompliantContactManager<T>& manager) {
    return manager.get_forest();
  }

  template <typename T>
  static BodyIndex FindBodyByGeometryId(
      const CompliantContactManager<T>& manager, geometry::GeometryId id) {
    return manager.FindBodyByGeometryId(id);
  }

  template <typename T>
  static const GeometryContactData<T>& EvalGeometryContactData(
      const CompliantContactManager<T>& manager,
      const systems::Context<T>& context) {
    return manager.EvalGeometryContactData(context);
  }

  // N.B. Actuation input is always included, regardless of solver choice.
  template <typename T>
  static void CalcNonContactForces(const CompliantContactManager<T>& manager,
                                   const drake::systems::Context<T>& context,
                                   bool include_joint_limit_penalty_forces,
                                   MultibodyForces<T>* forces) {
    const bool include_pd_controlled_input = true;
    manager.CalcNonContactForces(context, include_joint_limit_penalty_forces,
                                 include_pd_controlled_input, forces);
  }

  template <typename T>
  static const SapDriver<T>& sap_driver(
      const CompliantContactManager<T>& manager) {
    DRAKE_DEMAND(manager.sap_driver_ != nullptr);
    return *manager.sap_driver_;
  }

  template <typename T>
  static const TamsiDriver<T>& tamsi_driver(
      const CompliantContactManager<T>& manager) {
    DRAKE_DEMAND(manager.tamsi_driver_ != nullptr);
    return *manager.tamsi_driver_;
  }

  // Returns the Jacobian J_AcBc_C. This method takes the Jacobian blocks
  // evaluated with EvalContactJacobianCache() and assembles them into a dense
  // Jacobian matrix.
  template <typename T>
  static Eigen::MatrixX<T> CalcDenseJacobianMatrixInContactFrame(
      const CompliantContactManager<T>& manager,
      const DiscreteContactData<DiscreteContactPair<T>>& contact_pairs) {
    const int nc = contact_pairs.size();
    Eigen::MatrixX<T> J_AcBc_C(3 * nc, manager.plant().num_velocities());
    J_AcBc_C.setZero();
    const SpanningForest& forest =
        CompliantContactManagerTester::get_forest(manager);
    for (int i = 0; i < nc; ++i) {
      const int row_offset = 3 * i;
      const DiscreteContactPair<T>& contact_pair = contact_pairs[i];
      for (const typename DiscreteContactPair<T>::JacobianTreeBlock&
               tree_jacobian : contact_pair.jacobian) {
        // If added to the Jacobian, it must have a valid index.
        EXPECT_TRUE(tree_jacobian.tree.is_valid());
        const SpanningForest::Tree& tree = forest.trees(tree_jacobian.tree);
        J_AcBc_C.block(row_offset, tree.v_start(), 3, tree.nv()) =
            tree_jacobian.J.MakeDenseMatrix();
      }
    }
    return J_AcBc_C;
  }
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
