#pragma once
#include "drake/examples/multibody/suction_gripper/suction_gripper_multibody_model.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake::examples::multibody::suction_gripper {

/// @brief A suction gripper with a single 30mm Chloroprene cup as an example.
class ExampleGripperMultibodyModel : public SuctionGripperMultibodyModel {
 public:
  ExampleGripperMultibodyModel(
      drake::multibody::MultibodyPlant<double>* plant_ptr,
      const drake::multibody::Body<double>& wrist_body);
  double CalcCupArea() const override {
    return M_PI * kCupEffectiveDiameter * kCupEffectiveDiameter / 4.0;
  }

 private:
  // All values are in SI units
  const drake::math::RigidTransform<double> kWristGripperTransform =
      drake::math::RigidTransform<double>();
  const double kBaseHeight = 0.3;
  const double kBaseWidth = 0.1;
  const double kBaseMass = 1.0;
  const double kBaseInertiaX = 0.001;
  const double kBaseInertiaY = 0.001;
  const double kBaseInertiaZ = 0.0001;
  const double kBaseFriction = 0.2;

  const double kCupFittingHeight = 0.05;
  const double kCupFittingDiameter = 0.026;

  const double kCupHeight = 0.027;
  const double kCupOuterDiameter = 0.03;
  const double kCupInnerDiameter = 0.022;
  // Usually the effective diamter is between the inner and outer diameters of a
  // cup, here we use the average value for simplicity.
  const double kCupEffectiveDiameter =
      (kCupInnerDiameter + kCupOuterDiameter) / 2;
  const double kCupMass = 0.02;
  const double kCupInertiaX = 0.000008;
  const double kCupInertiaY = 0.000008;
  const double kCupInertiaZ = 0.000007;
  const double kCupStiffness = 700;
  const double kCupDamping = 1.0;

  const double kCupActPtDiameter = 0.01;
  const int kNumEdgePtsPerCup = 4;
  const double kCupEdgePtDiameter = 0.002;
  const double kCupEdgeFriction = 0.6;
  const double kCupEdgeMoveRange = 0.02;
  const double kCupEdgeStiffness = 700 / 4;
  const double kCupEdgeDamping = 1.0 / 4;
};

}  // namespace drake::examples::multibody::suction_gripper
