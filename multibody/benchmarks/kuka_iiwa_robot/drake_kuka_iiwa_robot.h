#pragma once

#include <cmath>
#include <functional>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/make_kuka_iiwa_model.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/test_utilities/spatial_kinematics.h"
#include "drake/multibody/tree/fixed_offset_frame.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

class MultibodyPlantTester {
 public:
  MultibodyPlantTester() = delete;

  // Use private constructor to create an MBP from an MBT.
  template <typename T>
  static std::unique_ptr<MultibodyPlant<T>>
  CreateMultibodyPlantFromTree(
      std::unique_ptr<internal::MultibodyTree<T>> tree, double time_step = 0.) {
    // Do not use `make_unique` for the private constructor, as it would have
    // to be a friend of MultibodyPlant.
    return std::unique_ptr<MultibodyPlant<T>>(
        new MultibodyPlant<T>(std::move(tree), time_step));
  }
};

namespace benchmarks {
namespace kuka_iiwa_robot {

/// Utility struct to assist with returning joint torques/forces.
/// --------|----------------------------------------------------------
/// F_Ao_W  | Spatial force on Ao from W, expressed in frame W (world).
/// F_Bo_W  | Spatial force on Bo from A, expressed in frame W (world).
/// F_Co_W  | Spatial force on Co from B, expressed in frame W (world).
/// F_Do_W  | Spatial force on Do from C, expressed in frame W (world).
/// F_Eo_W  | Spatial force on Eo from D, expressed in frame W (world).
/// F_Fo_W  | Spatial force on Fo from E, expressed in frame W (world).
/// F_Go_W  | Spatial force on Go from F, expressed in frame W (world).
template <typename T>
struct KukaRobotJointReactionForces {
  SpatialForce<T> F_Ao_W;
  SpatialForce<T> F_Bo_W;
  SpatialForce<T> F_Co_W;
  SpatialForce<T> F_Do_W;
  SpatialForce<T> F_Eo_W;
  SpatialForce<T> F_Fo_W;
  SpatialForce<T> F_Go_W;
};

/// This class is a MultibodyTree model for a 7-DOF Kuka iiwa robot arm.
/// It is used to compare Drake results for the robot end-effector (rigid linkG)
/// relative to world (Newtonian frame linkN) versus MotionGenesis solution.
/// This class takes input values for each of the 7 joint angles (q) as well as
/// their 1st and 2nd time derivatives (q̇, q̈) and calculates the end-effector
/// rotation matrix (relative to world), position (from world origin) angular
/// velocity, velocity, angular acceleration, and acceleration.
/// Geometrical and connectivity data is in file kuka_iiwa_robot.urdf.
template <typename T>
class DrakeKukaIIwaRobot {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrakeKukaIIwaRobot)

  /// Construct a 7-DOF Kuka iiwa robot arm (from file kuka_iiwa_robot.urdf).
  /// The robot is constructed with 7 revolute joints.
  /// @param[in] gravity Earth's gravitational acceleration in m/s².  The world
  /// z-unit vector is vertically upward.  If a gravity value of 9.8 is passed
  /// to this constructor, it means the gravity vector is directed opposite the
  /// world upward z-unit vector (which is correct -- gravity is downward).
  explicit DrakeKukaIIwaRobot(double gravity) {
    plant_ =
        MultibodyPlantTester::CreateMultibodyPlantFromTree(
            MakeKukaIiwaModel<T>(
                false /* finalized model */,
                gravity /* acceleration of gravity */));

    linkN_ = &tree().world_body();

    // Get this robot's seven links.
    linkA_ = &tree().GetBodyByName("iiwa_link_1");
    linkB_ = &tree().GetBodyByName("iiwa_link_2");
    linkC_ = &tree().GetBodyByName("iiwa_link_3");
    linkD_ = &tree().GetBodyByName("iiwa_link_4");
    linkE_ = &tree().GetBodyByName("iiwa_link_5");
    linkF_ = &tree().GetBodyByName("iiwa_link_6");
    linkG_ = &tree().GetBodyByName("iiwa_link_7");

    // Get this robot's seven joints.
    NA_joint_ =
        &tree().template GetJointByName<RevoluteJoint>("iiwa_joint_1");
    AB_joint_ =
        &tree().template GetJointByName<RevoluteJoint>("iiwa_joint_2");
    BC_joint_ =
        &tree().template GetJointByName<RevoluteJoint>("iiwa_joint_3");
    CD_joint_ =
        &tree().template GetJointByName<RevoluteJoint>("iiwa_joint_4");
    DE_joint_ =
        &tree().template GetJointByName<RevoluteJoint>("iiwa_joint_5");
    EF_joint_ =
        &tree().template GetJointByName<RevoluteJoint>("iiwa_joint_6");
    FG_joint_ =
        &tree().template GetJointByName<RevoluteJoint>("iiwa_joint_7");

    // After Finalize() method has been called, Context can be created.
    context_ = plant_->CreateDefaultContext();
  }

  /// This method gets the number of rigid bodies in this robot.
  /// @returns the number of rigid bodies in this robot.
  int get_number_of_rigid_bodies() const  { return tree().num_bodies(); }

  /// This method calculates kinematic properties of the end-effector (herein
  /// denoted as rigid body G) of a 7-DOF KUKA LBR iiwa robot (14 kg payload).
  /// Right-handed orthogonal unit vectors Nx, Ny, Nz are fixed in N (Earth)
  /// with Nz vertically upward and right-handed orthogonal unit vectors
  /// Gx, Gy, Gz are fixed in G.  The origin of frame N (Earth) is denoted No.
  /// The origin Go of end-effector G is located at G's inboard revolute joint.
  ///
  /// @param[in] q robot's joint angles (generalized coordinates).
  /// @param[in] qDt 1st-time-derivative of q.
  /// @param[in] qDDt 2nd-time-derivative of q.
  ///
  /// @returns G's kinematics in N, expressed in N.
  test_utilities::SpatialKinematicsPVA<T>
  CalcEndEffectorKinematics(const Eigen::Ref<const VectorX<T>>& q,
                            const Eigen::Ref<const VectorX<T>>& qDt,
                            const Eigen::Ref<const VectorX<T>>& qDDt) {
    SetJointAnglesAnd1stDerivatives(q.data(), qDt.data());

    // For each body, set the pose and spatial velocity in the position,
    // velocity, and acceleration caches with specified values for testing.
    multibody::internal::PositionKinematicsCache<T> pc(tree().get_topology());
    multibody::internal::VelocityKinematicsCache<T> vc(tree().get_topology());

    // Retrieve end-effector pose from position kinematics cache.
    tree().CalcPositionKinematicsCache(*context_, &pc);
    const math::RigidTransform<T>& X_NG = pc.get_X_WB(linkG_->node_index());

    // Retrieve end-effector spatial velocity from velocity kinematics cache.
    tree().CalcVelocityKinematicsCache(*context_, pc, &vc);
    const SpatialVelocity<T>& V_NG_N = vc.get_V_WB(linkG_->node_index());

    // Retrieve end-effector spatial acceleration from acceleration cache.
    std::vector<SpatialAcceleration<T>> A_WB(tree().num_bodies());
    // TODO(eric.cousineau): For this model, the end effector's BodyIndex
    // matches its BodyNodeIndex, thus we're not really checking the difference
    // between MultibodyPlant and MultibodyTree's ordering.
    DRAKE_DEMAND(int{linkG_->index()} == int{linkG_->node_index()});
    plant().CalcSpatialAccelerationsFromVdot(*context_, qDDt, &A_WB);
    const SpatialAcceleration<T>& A_NG_N = A_WB[linkG_->index()];

    // Create a class to return the results.
    return test_utilities::SpatialKinematicsPVA<T>(X_NG, V_NG_N, A_NG_N);
  }

  /// This method calculates joint reaction torques/forces for a 7-DOF KUKA iiwa
  /// robot, from known joint angles and their 1st and 2nd time-derivatives.
  ///
  /// @param[in] q robot's joint angles (generalized coordinates).
  /// @param[in] qDt 1st-time-derivative of q.
  /// @param[in] qDDt 2nd-time-derivative of q.
  ///
  /// @returns a structure holding the quantities defined below.
  /// --------|----------------------------------------------------------
  /// F_Ao_W  | Spatial force on Ao from W, expressed in frame W (world).
  /// F_Bo_W  | Spatial force on Bo from A, expressed in frame W (world).
  /// F_Co_W  | Spatial force on Co from B, expressed in frame W (world).
  /// F_Do_W  | Spatial force on Do from C, expressed in frame W (world).
  /// F_Eo_W  | Spatial force on Eo from D, expressed in frame W (world).
  /// F_Fo_W  | Spatial force on Fo from E, expressed in frame W (world).
  /// F_Go_W  | Spatial force on Go from F, expressed in frame W (world).
  KukaRobotJointReactionForces<T> CalcJointReactionForces(
      const Eigen::Ref<const VectorX<T>>& q,
      const Eigen::Ref<const VectorX<T>>& qDt,
      const Eigen::Ref<const VectorX<T>>& qDDt) {
    SetJointAnglesAnd1stDerivatives(q.data(), qDt.data());

    // Get the position, velocity, and acceleration cache from the context.
    multibody::internal::PositionKinematicsCache<T> pc(tree().get_topology());
    multibody::internal::VelocityKinematicsCache<T> vc(tree().get_topology());
    multibody::internal::AccelerationKinematicsCache<T> ac(
        tree().get_topology());
    tree().CalcPositionKinematicsCache(*context_, &pc);
    tree().CalcVelocityKinematicsCache(*context_, pc, &vc);
    tree().CalcAccelerationKinematicsCache(*context_, pc, vc, qDDt, &ac);

    // Applied forces:
    MultibodyForces<T> forces(plant());

    // Adds the previously included effect of gravity into forces.
    tree().CalcForceElementsContribution(*context_, pc, vc, &forces);

    // Output vector of generalized forces for calculated motor torques
    // required to drive the Kuka robot at its specified rate.
    const int number_of_generalized_speeds = tree().num_velocities();
    VectorX<T> generalized_force_output(number_of_generalized_speeds);

    // Output vector of spatial forces for joint reaction force/torques for
    // each body B at their inboard frame Mo, expressed in the world W.
    const int number_of_bodies = get_number_of_rigid_bodies();
    std::vector<SpatialForce<T>> F_BMo_W_array(number_of_bodies);

    // Output vector of generalized forces associated with the motor torques
    // required to drive the Kuka robot at its specified rate.
    std::vector<SpatialAcceleration<T>> A_WB_array(number_of_bodies);

    // Aliases to the arrays of applied forces:
    std::vector<SpatialForce<T>>& Fapplied_Bo_W_array =
        forces.mutable_body_forces();
    VectorX<T>& generalized_force_applied =
        forces.mutable_generalized_forces();

    // Calculate inverse dynamics on this robot.
    tree().CalcInverseDynamics(*context_, qDDt,
                Fapplied_Bo_W_array, generalized_force_applied,
                &A_WB_array, &F_BMo_W_array, &generalized_force_output);

    // Put joint reaction forces into return struct.
    KukaRobotJointReactionForces<T> reaction_forces;
    reaction_forces.F_Ao_W = F_BMo_W_array[linkA_->node_index()];
    reaction_forces.F_Bo_W = F_BMo_W_array[linkB_->node_index()];
    reaction_forces.F_Co_W = F_BMo_W_array[linkC_->node_index()];
    reaction_forces.F_Do_W = F_BMo_W_array[linkD_->node_index()];
    reaction_forces.F_Eo_W = F_BMo_W_array[linkE_->node_index()];
    reaction_forces.F_Fo_W = F_BMo_W_array[linkF_->node_index()];
    reaction_forces.F_Go_W = F_BMo_W_array[linkG_->node_index()];
    return reaction_forces;
  }

  const multibody::internal::MultibodyTree<T>& tree() const {
    return multibody::internal::GetInternalTree(*plant_);
  }
  const MultibodyPlant<T>& plant() const { return *plant_; }

 private:
  // This method sets the Kuka joint angles and their 1st and 2nd derivatives.
  // @param[in] q robot's joint angles (generalized coordinates).
  // @param[in] qDt 1st-time-derivative of q (q̇).
  void SetJointAnglesAnd1stDerivatives(const T q[7],
                                       const T qDt[7]) {
    systems::Context<T>* context = context_.get();

    NA_joint_->set_angle(context, q[0]);
    AB_joint_->set_angle(context, q[1]);
    BC_joint_->set_angle(context, q[2]);
    CD_joint_->set_angle(context, q[3]);
    DE_joint_->set_angle(context, q[4]);
    EF_joint_->set_angle(context, q[5]);
    FG_joint_->set_angle(context, q[6]);

    NA_joint_->set_angular_rate(context, qDt[0]);
    AB_joint_->set_angular_rate(context, qDt[1]);
    BC_joint_->set_angular_rate(context, qDt[2]);
    CD_joint_->set_angular_rate(context, qDt[3]);
    DE_joint_->set_angular_rate(context, qDt[4]);
    EF_joint_->set_angular_rate(context, qDt[5]);
    FG_joint_->set_angular_rate(context, qDt[6]);
  }

  // This model's MultibodyTree always has a built-in "world" body.
  // Newtonian reference frame (linkN) is the world body.
  std::unique_ptr<MultibodyPlant<T>> plant_;
  const Body<T>* linkN_{nullptr};

  // Rigid bodies (robot links).
  const Body<T>* linkA_{nullptr};
  const Body<T>* linkB_{nullptr};
  const Body<T>* linkC_{nullptr};
  const Body<T>* linkD_{nullptr};
  const Body<T>* linkE_{nullptr};
  const Body<T>* linkF_{nullptr};
  const Body<T>* linkG_{nullptr};

  // Joints.
  const RevoluteJoint<T>* NA_joint_{nullptr};
  const RevoluteJoint<T>* AB_joint_{nullptr};
  const RevoluteJoint<T>* BC_joint_{nullptr};
  const RevoluteJoint<T>* CD_joint_{nullptr};
  const RevoluteJoint<T>* DE_joint_{nullptr};
  const RevoluteJoint<T>* EF_joint_{nullptr};
  const RevoluteJoint<T>* FG_joint_{nullptr};

  // After model is finalized, create default context.
  std::unique_ptr<systems::Context<T>> context_;
};

}  // namespace kuka_iiwa_robot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
