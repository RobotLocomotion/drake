#include "drake/examples/multibody/suction_gripper/suction_gripper_multibody_model.h"

namespace drake::examples::multibody::suction_gripper {

int SuctionGripperMultibodyModel::count = 0;

drake::multibody::ModelInstanceIndex
SuctionGripperMultibodyModel::get_gripper_model_instance() const {
  if (!gripper_model_instance_.is_valid()) {
    throw std::runtime_error("Suction gripper model instance not initialized.");
  } else {
    return gripper_model_instance_;
  }
}

const std::vector<drake::geometry::GeometryId>&
SuctionGripperMultibodyModel::get_suction_cup_act_pt_geom_id_vec() const {
  if (suction_cup_act_pt_geom_id_vec_.empty()) {
    throw std::runtime_error(
        "Suction cup action point geometries not defined.");
  } else {
    return suction_cup_act_pt_geom_id_vec_;
  }
}

const std::unordered_map<drake::geometry::GeometryId,
                         drake::multibody::BodyIndex>&
SuctionGripperMultibodyModel::get_suction_cup_act_pt_geom_id_to_body_idx_map()
    const {
  if (suction_cup_act_pt_geom_id_to_body_idx_map_.empty()) {
    throw std::runtime_error(
        "Suction cup action-point-to-body map not defined");
  } else {
    return suction_cup_act_pt_geom_id_to_body_idx_map_;
  }
}

const std::vector<std::vector<drake::geometry::GeometryId>>&
SuctionGripperMultibodyModel::get_suction_cup_edge_pt_geom_id_vec() const {
  if (suction_cup_edge_pt_geom_id_vec_.empty()) {
    throw std::runtime_error("Suction cup edge point geometries not defined.");
  } else {
    return suction_cup_edge_pt_geom_id_vec_;
  }
}

}  // namespace drake::examples::multibody::suction_gripper
