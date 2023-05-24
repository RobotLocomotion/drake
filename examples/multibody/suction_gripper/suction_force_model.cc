#include "drake/examples/multibody/suction_gripper/suction_force_model.h"

#include <algorithm>
#include <limits>

#include "drake/geometry/query_object.h"
#include "drake/geometry/query_results/signed_distance_pair.h"

namespace drake::examples::multibody::suction_gripper {

// ------------------- ExternallyAppliedSpatialForcePair -------------------

std::pair<drake::multibody::ExternallyAppliedSpatialForce<double>,
          drake::multibody::ExternallyAppliedSpatialForce<double>>
ExternallyAppliedSpatialForcePair::GetAsPair() {
  std::pair<drake::multibody::ExternallyAppliedSpatialForce<double>,
            drake::multibody::ExternallyAppliedSpatialForce<double>>
      pair;

  pair.first.body_index = body_index1_;
  pair.first.p_BoBq_B = p_BoBq_B1_;
  pair.first.F_Bq_W = drake::multibody::SpatialForce<double>(
      /*tau*/ trq_axis_ * tau_mag_, /*f*/ force_axis_ * f_mag_);

  pair.second.body_index = body_index2_;
  pair.second.p_BoBq_B = p_BoBq_B2_;
  pair.second.F_Bq_W = drake::multibody::SpatialForce<double>(
      /*tau*/ -trq_axis_ * tau_mag_, /*f*/ -force_axis_ * f_mag_);
  return pair;
}

// ------------------- CupPressureSource -------------------

CupPressureSource::CupPressureSource(double vacuum_source_pressure,
                                     double max_suction_dist,
                                     int num_suction_cups)
    : vacuum_source_pressure_(vacuum_source_pressure),
      max_suction_dist_(max_suction_dist),
      num_suction_cups_(num_suction_cups) {
  /// ----- Input Ports ----- ///
  suction_cup_obj_dist_input_port_idx_ =
      DeclareVectorInputPort(
          "cup_obj_dists",
          drake::systems::BasicVector<double>(num_suction_cups_))
          .get_index();
  suction_cmd_input_port_idx_ =
      DeclareVectorInputPort(
          "suction_cmds",
          drake::systems::BasicVector<double>(num_suction_cups_))
          .get_index();

  /// ----- Output Ports ----- ///
  suction_cup_pressure_output_port_idx_ =
      DeclareVectorOutputPort(
          "suction_cup_pressures",
          drake::systems::BasicVector<double>(num_suction_cups_),
          &CupPressureSource::CalcSuctionCupPressure,
          {all_input_ports_ticket()})
          .get_index();
}

void CupPressureSource::CalcSuctionCupPressure(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* suction_cup_pressure_ptr) const {
  const auto& cup_obj_dist_vec =
      GetCupObjDistInputPort().Eval<drake::systems::BasicVector<double>>(
          context);
  const auto& suction_cmd_vec =
      GetSuctionCmdInputPort().Eval<drake::systems::BasicVector<double>>(
          context);

  for (int suction_cup_idx = 0; suction_cup_idx < num_suction_cups_;
       suction_cup_idx++) {
    double suction_cmd = suction_cmd_vec[suction_cup_idx];
    DRAKE_DEMAND(suction_cmd >= 0. && suction_cmd <= 1.);
    double dist = cup_obj_dist_vec[suction_cup_idx];
    double pressure = 0.;
    // use a simple linear pressure-distance model
    if (dist <= 0.) {
      pressure = vacuum_source_pressure_;
    } else if (dist <= max_suction_dist_) {
      pressure = (max_suction_dist_ - dist) * vacuum_source_pressure_ /
                 max_suction_dist_;
    } else {
      pressure = 0.;
    }
    (*suction_cup_pressure_ptr)[suction_cup_idx] = suction_cmd * pressure;
  }
}

// ------------------- CupObjInterface -------------------
CupObjInterface::CupObjInterface(
    double time_step, double suction_cup_area,
    const std::vector<drake::geometry::GeometryId>&
        suction_cup_act_pt_geom_id_vec,  // ordered by cup idx
    const std::unordered_map<drake::geometry::GeometryId,
                             drake::multibody::BodyIndex>&
        suction_cup_act_pt_geom_id_to_body_idx_map,
    const std::vector<std::vector<drake::geometry::GeometryId>>&
        suction_cup_edge_pt_geom_id_vec,
    const std::unordered_map<drake::geometry::GeometryId,
                             drake::multibody::BodyIndex>&
        obj_geom_id_to_body_idx_map)
    : suction_cup_area_(suction_cup_area),
      suction_cup_act_pt_geom_id_vec_(suction_cup_act_pt_geom_id_vec),
      suction_cup_act_pt_geom_id_to_body_idx_map_(
          suction_cup_act_pt_geom_id_to_body_idx_map),
      suction_cup_edge_pt_geom_id_vec_(suction_cup_edge_pt_geom_id_vec),
      obj_geom_id_to_body_idx_map_(obj_geom_id_to_body_idx_map),
      num_suction_cups_(suction_cup_act_pt_geom_id_vec.size()) {
  /// ----- Input Ports ----- ///
  geom_query_input_port_idx_ =
      DeclareAbstractInputPort(
          "geom_query", drake::Value<drake::geometry::QueryObject<double>>())
          .get_index();

  suction_cup_pressure_input_port_idx_ =
      DeclareVectorInputPort(
          "suction_cup_pressures",
          drake::systems::BasicVector<double>(num_suction_cups_))
          .get_index();

  /// ----- Output Ports ----- ///
  suction_force_output_port_idx_ =
      DeclareAbstractOutputPort(
          /*name*/ "suction_forces",
          /*alloc_function*/
          std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>(
              2 * num_suction_cups_),
          /*calc_function*/ &CupObjInterface::CalcSuctionForce,
          /*dependency*/
          {xa_ticket(), xd_ticket(),
           input_port_ticket(drake::systems::InputPortIndex(
               suction_cup_pressure_input_port_idx_))})
          .get_index();

  suction_cup_obj_dist_output_port_idx_ =
      DeclareVectorOutputPort(
          /*name*/ "cup_obj_dists",
          /*alloc_function*/
          drake::systems::BasicVector<double>(num_suction_cups_),
          /*calc_function*/ &CupObjInterface::OutputCupObjDist,
          /*dependency*/ {xa_ticket(), xd_ticket()})
          .get_index();

  /// ----- States ----- ///
  // each entry is a signed dist pair between the center action point and the
  // closest object
  suction_cup_act_pt_closest_obj_signed_dist_pair_state_idx_ =
      DeclareAbstractState(
          drake::Value<
              std::vector<drake::geometry::SignedDistancePair<double>>>(
              std::vector<drake::geometry::SignedDistancePair<double>>(
                  num_suction_cups_)));

  // each entry is the mean distance between the suction cup edge points to the
  // closest object
  suction_cup_edge_pt_closest_obj_dist_state_idx_ =
      DeclareDiscreteState(num_suction_cups_);

  DeclareInitializationUnrestrictedUpdateEvent(&CupObjInterface::UpdateDists);
  DeclarePeriodicUnrestrictedUpdateEvent(time_step, 0,
                                         &CupObjInterface::UpdateDists);
}

drake::systems::EventStatus CupObjInterface::UpdateDists(
    const drake::systems::Context<double>& context,
    drake::systems::State<double>* state_ptr) const {
  const auto& geom_query =
      GetGeomQueryInputPort().Eval<drake::geometry::QueryObject<double>>(
          context);
  auto& suction_cup_act_pt_closest_obj_signed_dist_pair_state =
      state_ptr->get_mutable_abstract_state()
          .get_mutable_value(
              suction_cup_act_pt_closest_obj_signed_dist_pair_state_idx_)
          .get_mutable_value<
              std::vector<drake::geometry::SignedDistancePair<double>>>();

  auto& suction_cup_edge_pt_closest_obj_dist_state =
      state_ptr->get_mutable_discrete_state(
          suction_cup_edge_pt_closest_obj_dist_state_idx_);

  for (int suction_cup_idx = 0; suction_cup_idx < num_suction_cups_;
       suction_cup_idx++) {
    auto suction_cup_act_pt_geom_id =
        suction_cup_act_pt_geom_id_vec_[suction_cup_idx];
    drake::geometry::GeometryId closest_obj_geom_id;

    auto min_suction_cup_act_pt_obj_dist =
        std::numeric_limits<double>::infinity();
    for (const auto& obj_geom_id_to_body_idx_pair :
         obj_geom_id_to_body_idx_map_) {
      auto obj_geom_id = obj_geom_id_to_body_idx_pair.first;
      auto signed_dist_pair = geom_query.ComputeSignedDistancePairClosestPoints(
          suction_cup_act_pt_geom_id, obj_geom_id);
      if (signed_dist_pair.distance < min_suction_cup_act_pt_obj_dist) {
        min_suction_cup_act_pt_obj_dist = signed_dist_pair.distance;
        suction_cup_act_pt_closest_obj_signed_dist_pair_state.at(
            suction_cup_idx) = signed_dist_pair;
        closest_obj_geom_id = obj_geom_id;
      }
    }

    auto each_suction_cup_edge_pt_geom_id_vec =
        suction_cup_edge_pt_geom_id_vec_.at(suction_cup_idx);
    auto mean_suction_cup_edge_pt_obj_dist = 0.;
    for (const auto& suction_cup_edge_pt_geom_id :
         each_suction_cup_edge_pt_geom_id_vec) {
      auto signed_dist_pair = geom_query.ComputeSignedDistancePairClosestPoints(
          suction_cup_edge_pt_geom_id, closest_obj_geom_id);
      mean_suction_cup_edge_pt_obj_dist +=
          std::max(signed_dist_pair.distance, 0.);
    }
    mean_suction_cup_edge_pt_obj_dist /=
        each_suction_cup_edge_pt_geom_id_vec.size();
    suction_cup_edge_pt_closest_obj_dist_state[suction_cup_idx] =
        mean_suction_cup_edge_pt_obj_dist;
  }
  return drake::systems::EventStatus::Succeeded();
}

void CupObjInterface::CalcSuctionForce(
    const drake::systems::Context<double>& context,
    std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>*
        suction_force_vec_ptr) const {
  const auto& pressure_vec =
      GetSuctionCupPressureInputPort()
          .Eval<drake::systems::BasicVector<double>>(context);

  const auto& suction_cup_act_pt_closest_obj_signed_dist_pair_state =
      context.get_abstract_state<
          std::vector<drake::geometry::SignedDistancePair<double>>>(
          suction_cup_act_pt_closest_obj_signed_dist_pair_state_idx_);

  for (int suction_cup_idx = 0; suction_cup_idx < num_suction_cups_;
       suction_cup_idx++) {
    auto signed_dist_pair =
        suction_cup_act_pt_closest_obj_signed_dist_pair_state.at(
            suction_cup_idx);
    auto suction_cup_act_pt_geom_id =
        suction_cup_act_pt_geom_id_vec_.at(suction_cup_idx);

    drake::geometry::GeometryId closest_obj_geom_id;
    Eigen::Vector3d p_GC;  // geometry to closest point
    Eigen::Vector3d cup_act_pt_obj_vec;

    if (signed_dist_pair.id_A ==
        suction_cup_act_pt_geom_id) {  // if A is cup, then B is obj
      closest_obj_geom_id = signed_dist_pair.id_B;
      cup_act_pt_obj_vec = -signed_dist_pair.nhat_BA_W;
      p_GC = signed_dist_pair.p_BCb;
    } else if (signed_dist_pair.id_B ==
               suction_cup_act_pt_geom_id) {  // if B is cup, then A is obj
      closest_obj_geom_id = signed_dist_pair.id_A;
      cup_act_pt_obj_vec = signed_dist_pair.nhat_BA_W;
      p_GC = signed_dist_pair.p_ACa;
    } else {
      throw std::runtime_error(
          "Mismatch bwtween "
          "suction_cup_act_pt_closest_obj_signed_dist_pair_state "
          "and suction_cup_act_pt_geom_id_vec_, both should be ordered by "
          "cup index.");
    }

    double f_mag = -pressure_vec[suction_cup_idx] * suction_cup_area_;

    auto cup_body_idx = suction_cup_act_pt_geom_id_to_body_idx_map_.at(
        suction_cup_act_pt_geom_id);
    auto obj_body_idx = obj_geom_id_to_body_idx_map_.at(closest_obj_geom_id);

    const auto& scene_graph_inspector =
        GetGeomQueryInputPort()
            .Eval<drake::geometry::QueryObject<double>>(context)
            .inspector();
    const auto& X_BG = scene_graph_inspector.GetPoseInFrame(
        closest_obj_geom_id);  // body to geometry

    ExternallyAppliedSpatialForcePair suction_force_pair(
        /*body_index1*/ cup_body_idx,
        /*body_index2*/ obj_body_idx,
        /*p_BoBq_B1*/ Eigen::Vector3d::Zero(),
        /*p_BoBq_B2*/
        (X_BG * drake::math::RigidTransform<double>(p_GC)).translation(),
        /*force_axis*/ cup_act_pt_obj_vec,
        /*f_mag*/ f_mag,
        /*trq_axis*/ Eigen::Vector3d::UnitZ(),
        /*tau_mag*/ 0);

    auto suction_force_pair_vec = suction_force_pair.GetAsPair();

    suction_force_vec_ptr->at(2 * suction_cup_idx) =
        suction_force_pair_vec.first;
    suction_force_vec_ptr->at(2 * suction_cup_idx + 1) =
        suction_force_pair_vec.second;
  }
}

void CupObjInterface::OutputCupObjDist(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* cup_obj_dist_vec_ptr) const {
  const auto& suction_cup_edge_pt_closest_obj_dist_state =
      context.get_discrete_state(
          suction_cup_edge_pt_closest_obj_dist_state_idx_);
  cup_obj_dist_vec_ptr->SetFromVector(
      suction_cup_edge_pt_closest_obj_dist_state.get_value());
}

}  // namespace drake::examples::multibody::suction_gripper
