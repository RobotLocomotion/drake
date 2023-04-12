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
      const CompliantContactManager<double>& manager,
      geometry::GeometryId id) {
    return manager.FindBodyByGeometryId(id);
  }

  static const std::vector<geometry::ContactSurface<double>>&
  EvalContactSurfaces(const CompliantContactManager<double>& manager,
                      const drake::systems::Context<double>& context) {
    return manager.EvalContactSurfaces(context);
  }

  static const std::vector<DiscreteContactPair<double>>&
  EvalDiscreteContactPairs(const CompliantContactManager<double>& manager,
                           const drake::systems::Context<double>& context) {
    return manager.EvalDiscreteContactPairs(context);
  }

  static void CalcNonContactForces(
      const CompliantContactManager<double>& manager,
      const drake::systems::Context<double>& context,
      bool include_joint_limit_penalty_forces,
      MultibodyForces<double>* forces) {
    manager.CalcNonContactForces(context, include_joint_limit_penalty_forces,
                                 forces);
  }

  static std::vector<ContactPairKinematics<double>> CalcContactKinematics(
      const CompliantContactManager<double>& manager,
      const drake::systems::Context<double>& context) {
    return manager.CalcContactKinematics(context);
  }

  static void DoCalcContactResults(
      const CompliantContactManager<double>& manager,
      const drake::systems::Context<double>& context,
      ContactResults<double>* contact_results) {
    return manager.DoCalcContactResults(context, contact_results);
  }

  static const DeformableDriver<double>* deformable_driver(
      const CompliantContactManager<double>& manager) {
    return manager.deformable_driver_.get();
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
      const std::vector<ContactPairKinematics<double>>& contact_kinematics) {
    const int nc = contact_kinematics.size();
    Eigen::MatrixXd J_AcBc_C(3 * nc, manager.plant().num_velocities());
    J_AcBc_C.setZero();
    const auto& topology = CompliantContactManagerTester::topology(manager);
    for (int i = 0; i < nc; ++i) {
      const int row_offset = 3 * i;
      const ContactPairKinematics<double>& pair_kinematics =
          contact_kinematics[i];
      for (const ContactPairKinematics<double>::JacobianTreeBlock&
               tree_jacobian : pair_kinematics.jacobian) {
        // If added to the Jacobian, it must have a valid index.
        EXPECT_TRUE(tree_jacobian.tree.is_valid());
        const int col_offset =
            topology.tree_velocities_start(tree_jacobian.tree);
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
