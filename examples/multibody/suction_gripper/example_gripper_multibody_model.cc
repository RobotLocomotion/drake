#include "drake/examples/multibody/suction_gripper/example_gripper_multibody_model.h"

#include <cmath>
#include <vector>

#include "drake/geometry/shape_specification.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/prismatic_spring.h"

namespace drake::examples::multibody::suction_gripper {

ExampleGripperMultibodyModel::ExampleGripperMultibodyModel(
    drake::multibody::MultibodyPlant<double>* plant_ptr,
    const drake::multibody::Body<double>& wrist_body)
    : SuctionGripperMultibodyModel(plant_ptr, wrist_body) {
  // add the base
  auto& base_body = plant_ptr_->AddRigidBody(
      /*name*/ "base_body",
      /*model_instance*/ gripper_model_instance_,
      /*inertia*/
      drake::multibody::SpatialInertia<double>::MakeFromCentralInertia(
          /*m*/ kBaseMass,
          /*p_PScm_E*/ Eigen::Vector3d(0., 0., -kBaseHeight / 2),
          /*I_SScm_E*/
          drake::multibody::RotationalInertia<double>(
              kBaseInertiaX, kBaseInertiaY, kBaseInertiaZ)));

  plant_ptr_->AddJoint<drake::multibody::WeldJoint>(
      "wrist_to_gripper_joint", wrist_body,
      drake::math::RigidTransform<double>(), base_body,
      drake::math::RigidTransform<double>(), kWristGripperTransform);

  drake::multibody::CoulombFriction base_friction(
      /*static_friction */ kBaseFriction, /* dynamic_friction*/ kBaseFriction);

  plant_ptr_->RegisterVisualGeometry(
      /*body*/ base_body,
      /*X_BG*/
      drake::math::RigidTransform<double>(
          Eigen::Vector3d(0., 0., -kBaseHeight / 2)),
      /*shape*/ drake::geometry::Box(kBaseWidth, kBaseWidth, kBaseHeight),
      /*name*/ "base_body_visual_geom",
      /*diffuse_color*/ Eigen::Vector4d(0.2, 0.2, 0.2, 1.0));

  plant_ptr_->RegisterVisualGeometry(
      /*body*/ base_body,
      /*X_BG*/
      drake::math::RigidTransform<double>(
          Eigen::Vector3d(0., 0., -kBaseHeight - kCupFittingHeight / 2)),
      /*shape*/
      drake::geometry::Cylinder(kCupFittingDiameter / 2, kCupFittingHeight),
      /*name*/ "base_body_cup_fitting_visual_geom",
      /*diffuse_color*/ Eigen::Vector4d(0.5, 0.5, 0.5, 1.0));

  plant_ptr_->RegisterVisualGeometry(
      /*body*/ base_body,
      /*X_BG*/
      drake::math::RigidTransform<double>(Eigen::Vector3d(
          0., 0., -kBaseHeight - kCupFittingHeight - kCupHeight / 4)),
      /*shape*/
      drake::geometry::Cylinder(kCupOuterDiameter / 2, kCupHeight / 2),
      /*name*/ "base_body_cup_upper_half_visual_geom",
      /*diffuse_color*/ Eigen::Vector4d(0.1, 0.1, 0.1, 1.0));

  plant_ptr_->RegisterCollisionGeometry(
      /*body*/ base_body,
      /*X_BG*/
      drake::math::RigidTransform<double>(
          Eigen::Vector3d(0., 0., -kBaseHeight - kCupFittingHeight / 2)),
      /*shape*/
      drake::geometry::Cylinder(kCupFittingDiameter / 2, kCupFittingHeight),
      /*name*/ "base_body_cup_fitting_collision_geom",
      /*coulomb_friction*/ base_friction);

  auto base_body_cup_act_pt_collision_geom =
      plant_ptr_->RegisterCollisionGeometry(
          /*body*/ base_body,
          /*X_BG*/
          drake::math::RigidTransform<double>(Eigen::Vector3d(
              0., 0., -kBaseHeight - kCupFittingHeight - kCupHeight / 4)),
          /*shape*/ drake::geometry::Sphere(kCupActPtDiameter / 2),
          /*name*/ "base_body_cup_act_pt_collision_geom",
          /*coulomb_friction*/ base_friction);

  suction_cup_act_pt_geom_id_vec_.push_back(
      base_body_cup_act_pt_collision_geom);
  suction_cup_act_pt_geom_id_to_body_idx_map_
      [base_body_cup_act_pt_collision_geom] = base_body.index();

  // add the cup
  auto& cup_body = plant_ptr_->AddRigidBody(
      /*name*/ "cup_body",
      /*model_instance*/ gripper_model_instance_,
      /*inertia*/
      drake::multibody::SpatialInertia<double>::MakeFromCentralInertia(
          /*m*/ kCupMass / 2,
          /*p_PScm_E*/ Eigen::Vector3d(0., 0., -kCupHeight / 2),
          /*I_SScm_E*/
          drake::multibody::RotationalInertia<double>(
              kCupInertiaX / 2, kCupInertiaY / 2, kCupInertiaZ / 2)));

  auto& base_to_cup_body_joint =
      plant_ptr_->AddJoint<drake::multibody::PrismaticJoint>(
          /*name*/ "base_to_cup_body_joint",
          /*parent*/ base_body,
          /*X_PF*/
          drake::math::RigidTransform<double>(Eigen::Vector3d(
              0., 0., -kBaseHeight - kCupFittingHeight - kCupHeight * 3 / 4)),
          /*child*/ cup_body,
          /*X_BM*/ drake::math::RigidTransform<double>(),
          /*axis*/ Eigen::Vector3d::UnitZ(),
          /*pos_lower_limit*/ 0.,
          /*pos_upper_limit*/ kCupHeight / 2,
          /*damping*/ kCupDamping);

  plant_ptr_->AddForceElement<drake::multibody::PrismaticSpring>(
      /*joint*/ base_to_cup_body_joint,
      /*nominal_position*/ 0.,
      /*stiffness*/ kCupStiffness);

  plant_ptr_->RegisterVisualGeometry(
      /*body*/ cup_body,
      /*X_BG*/ drake::math::RigidTransform<double>(),
      /*shape*/
      drake::geometry::Cylinder(kCupOuterDiameter / 2, kCupHeight / 2),
      /*name*/ "cup_body_lower_half_visual_geom",
      /*diffuse_color*/ Eigen::Vector4d(0.1, 0.1, 0.1, 1.0));

  suction_cup_edge_pt_geom_id_vec_ =
      std::vector<std::vector<drake::geometry::GeometryId>>(
          1, std::vector<drake::geometry::GeometryId>(
                 kNumEdgePtsPerCup));  // allocate memory

  for (int suction_cup_edge_pt_idx = 0;
       suction_cup_edge_pt_idx < kNumEdgePtsPerCup; suction_cup_edge_pt_idx++) {
    auto& cup_edge_pt_body = plant_ptr_->AddRigidBody(
        /*name*/ "cup_edge_pt_body" + std::to_string(suction_cup_edge_pt_idx),
        /*model_instance*/ gripper_model_instance_,
        /*inertia*/
        drake::multibody::SpatialInertia<double>::MakeFromCentralInertia(
            /*m*/ kCupMass / 2 / kNumEdgePtsPerCup,
            /*p_PScm_E*/ Eigen::Vector3d(0., 0., 0.),
            /*I_SScm_E*/
            drake::multibody::RotationalInertia<double>(
                kCupInertiaX / 2 / kNumEdgePtsPerCup,
                kCupInertiaY / 2 / kNumEdgePtsPerCup,
                kCupInertiaZ / 2 / kNumEdgePtsPerCup)));

    auto& cup_body_to_edge_pt_joint =
        plant_ptr_->AddJoint<drake::multibody::PrismaticJoint>(
            /*name*/ "cup_body_to_edge_pt_joint" +
                std::to_string(suction_cup_edge_pt_idx),
            /*parent*/ cup_body,
            /*X_PF*/ drake::math::RigidTransform<double>(),
            /*child*/ cup_edge_pt_body,
            /*X_BM*/ drake::math::RigidTransform<double>(),
            /*axis*/ Eigen::Vector3d::UnitZ(),
            /*pos_lower_limit*/ 0,
            /*pos_upper_limit*/ kCupEdgeMoveRange,
            /*damping*/ kCupEdgeDamping);

    plant_ptr_->AddForceElement<drake::multibody::PrismaticSpring>(
        /*joint*/ cup_body_to_edge_pt_joint,
        /*nominal_position*/ 0.,
        /*stiffness*/ kCupEdgeStiffness);

    drake::multibody::CoulombFriction cup_edge_friction(
        /*static_friction */ kCupEdgeFriction,
        /* dynamic_friction*/ kCupEdgeFriction);

    auto angle = 2 * M_PI / kNumEdgePtsPerCup * suction_cup_edge_pt_idx;
    auto cup_edge_pt_collision_geom = plant_ptr_->RegisterCollisionGeometry(
        /*body*/ cup_edge_pt_body,
        /*X_BG*/
        drake::math::RigidTransform<double>(Eigen::Vector3d(
            (kCupOuterDiameter / 2) * std::cos(angle),
            (kCupOuterDiameter / 2) * std::sin(angle), -kCupHeight / 4)),
        /*shape*/ drake::geometry::Sphere(kCupEdgePtDiameter / 2),
        /*name*/ "cup_edge_pt_collision_geom" +
            std::to_string(suction_cup_edge_pt_idx),
        /*coulomb_friction*/ cup_edge_friction);
    suction_cup_edge_pt_geom_id_vec_[0][suction_cup_edge_pt_idx] =
        cup_edge_pt_collision_geom;
  }
}

}  // namespace drake::examples::multibody::suction_gripper
