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
  static const internal::MultibodyTreeTopology& topology(
      const CompliantContactManager<double>& manager) {
    return manager.tree_topology();
  }

  static BodyIndex FindBodyByGeometryId(
      const CompliantContactManager<double>& manager, geometry::GeometryId id) {
    return manager.FindBodyByGeometryId(id);
  }

  static const std::vector<geometry::ContactSurface<double>>&
  EvalContactSurfaces(const CompliantContactManager<double>& manager,
                      const drake::systems::Context<double>& context) {
    return manager.EvalContactSurfaces(context);
  }

  // N.B. Actuation input is always included, regardless of solver choice.
  static void CalcNonContactForces(
      const CompliantContactManager<double>& manager,
      const drake::systems::Context<double>& context,
      bool include_joint_limit_penalty_forces,
      MultibodyForces<double>* forces) {
    const bool include_pd_controlled_input = true;
    manager.CalcNonContactForces(context, include_joint_limit_penalty_forces,
                                 include_pd_controlled_input, forces);
  }

  static const SapDriver<double>& sap_driver(
      const CompliantContactManager<double>& manager) {
    DRAKE_DEMAND(manager.sap_driver_ != nullptr);
    return *manager.sap_driver_;
  }

  static const TamsiDriver<double>& tamsi_driver(
      const CompliantContactManager<double>& manager) {
    DRAKE_DEMAND(manager.tamsi_driver_ != nullptr);
    return *manager.tamsi_driver_;
  }

  // Returns the Jacobian J_AcBc_C. This method takes the Jacobian blocks
  // evaluated with EvalContactJacobianCache() and assembles them into a dense
  // Jacobian matrix.
  static Eigen::MatrixXd CalcDenseJacobianMatrixInContactFrame(
      const CompliantContactManager<double>& manager,
      const DiscreteContactData<DiscreteContactPair<double>>& contact_pairs) {
    const int nc = contact_pairs.size();
    Eigen::MatrixXd J_AcBc_C(3 * nc, manager.plant().num_velocities());
    J_AcBc_C.setZero();
    const auto& topology = CompliantContactManagerTester::topology(manager);
    for (int i = 0; i < nc; ++i) {
      const int row_offset = 3 * i;
      const DiscreteContactPair<double>& contact_pair = contact_pairs[i];
      for (const DiscreteContactPair<double>::JacobianTreeBlock& tree_jacobian :
           contact_pair.jacobian) {
        // If added to the Jacobian, it must have a valid index.
        EXPECT_TRUE(tree_jacobian.tree.is_valid());
        const int col_offset =
            topology.tree_velocities_start_in_v(tree_jacobian.tree);
        const int tree_nv = topology.num_tree_velocities(tree_jacobian.tree);
        J_AcBc_C.block(row_offset, col_offset, 3, tree_nv) =
            tree_jacobian.J.MakeDenseMatrix();
      }
    }
    return J_AcBc_C;
  }
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
