#pragma once
#include <unordered_map>
#include <vector>

#include "drake/geometry/geometry_ids.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake::examples::multibody::suction_gripper {

/// @brief Base class of the suction gripper multibody model. Any suction
/// gripper multibody model should inherit from this class and construct the
/// gripper geometries additionally.
class SuctionGripperMultibodyModel {
 public:
  SuctionGripperMultibodyModel(
      drake::multibody::MultibodyPlant<double>* plant_ptr,
      const drake::multibody::Body<double>& wrist_body)
      : plant_ptr_{plant_ptr}, wrist_body_{wrist_body} {
    gripper_model_instance_ =
        plant_ptr_->AddModelInstance("SuctionGripper" + std::to_string(count));
    SuctionGripperMultibodyModel::count++;
  }

  virtual ~SuctionGripperMultibodyModel() = default;

  const std::vector<drake::geometry::GeometryId>&
  get_suction_cup_act_pt_geom_id_vec() const;
  const std::unordered_map<drake::geometry::GeometryId,
                           drake::multibody::BodyIndex>&
  get_suction_cup_act_pt_geom_id_to_body_idx_map() const;
  const std::vector<std::vector<drake::geometry::GeometryId>>&
  get_suction_cup_edge_pt_geom_id_vec() const;

  drake::multibody::ModelInstanceIndex get_gripper_model_instance() const;
  virtual double CalcCupArea() const = 0;

  static int count;

 protected:
  drake::multibody::MultibodyPlant<double>* plant_ptr_{nullptr};
  const drake::multibody::Body<double>& wrist_body_;
  drake::multibody::ModelInstanceIndex gripper_model_instance_;

  std::vector<drake::geometry::GeometryId> suction_cup_act_pt_geom_id_vec_;
  std::unordered_map<drake::geometry::GeometryId, drake::multibody::BodyIndex>
      suction_cup_act_pt_geom_id_to_body_idx_map_;
  std::vector<std::vector<drake::geometry::GeometryId>>
      suction_cup_edge_pt_geom_id_vec_;
};

}  // namespace drake::examples::multibody::suction_gripper
