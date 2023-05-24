#pragma once

#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/geometry/geometry_ids.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake::examples::multibody::suction_gripper {

/// @brief The action-reaction pair of ExternallyAppliedSpatialForce.
class ExternallyAppliedSpatialForcePair {
 public:
  ExternallyAppliedSpatialForcePair(
      const drake::multibody::BodyIndex& body_index1,
      const drake::multibody::BodyIndex& body_index2,
      const Eigen::Vector3d& p_BoBq_B1, const Eigen::Vector3d& p_BoBq_B2,
      const Eigen::Vector3d& force_axis,  // direction of force applied on body1
      double f_mag,
      const Eigen::Vector3d& trq_axis,  // direction of trq applied on body1
      double tau_mag)
      : body_index1_(body_index1),
        body_index2_(body_index2),
        p_BoBq_B1_(p_BoBq_B1),
        p_BoBq_B2_(p_BoBq_B2),
        force_axis_(force_axis.normalized()),
        f_mag_(f_mag),
        trq_axis_(trq_axis.normalized()),
        tau_mag_(tau_mag) {}

  std::pair<drake::multibody::ExternallyAppliedSpatialForce<double>,
            drake::multibody::ExternallyAppliedSpatialForce<double>>
  GetAsPair();

 public:
  const drake::multibody::BodyIndex body_index1_;
  const drake::multibody::BodyIndex body_index2_;
  Eigen::Vector3d p_BoBq_B1_;
  Eigen::Vector3d p_BoBq_B2_;
  Eigen::Vector3d force_axis_;
  double f_mag_;
  Eigen::Vector3d trq_axis_;
  double tau_mag_;
};

/// @brief The (suction cup) pressure input to the CupObjInterface. Here we use
/// a linear model with saturation to represent the pressure-distance
/// relationship.
/// @note In this model we assume no interplay between different cups, i.e. the
/// maximum vaccum pressure is not affected by the sealing status of other cups.
/// In reality, if multiple cups share the same vaccum source, this may not hold
/// true, and more sophisticated fluid dynamics models are needed.
class CupPressureSource : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CupPressureSource)
  /// @brief Constructor of the CupPressureSource class.
  /// @param vacuum_source_pressure The maximum vacuum pressure or the vacuum
  /// source (pump or vaccum generator) relative to atmosphere pressure, in Pa.
  /// @param max_suction_dist The maximum suction action distance between the
  /// suction cup and the object, in m.
  /// @param num_suction_cups The number of suction cups.
  CupPressureSource(double vacuum_source_pressure, double max_suction_dist,
                    int num_suction_cups);

  const drake::systems::InputPort<double>& GetCupObjDistInputPort() const {
    return get_input_port(suction_cup_obj_dist_input_port_idx_);
  }
  const drake::systems::InputPort<double>& GetSuctionCmdInputPort() const {
    return get_input_port(suction_cmd_input_port_idx_);
  }
  const drake::systems::OutputPort<double>& GetSuctionCupPressureOutputPort()
      const {
    return get_output_port(suction_cup_pressure_output_port_idx_);
  }

 private:
  void CalcSuctionCupPressure(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* suction_cup_pressure_ptr) const;

  double vacuum_source_pressure_{0};
  double max_suction_dist_{0};
  int num_suction_cups_{-1};

  int suction_cmd_input_port_idx_{-1};
  int suction_cup_obj_dist_input_port_idx_{-1};
  int suction_cup_pressure_output_port_idx_{-1};
};

/// @brief The cup-object interface, which calculates (1) the closest distance
/// between the suction cup and the object (for the CupPressureSource to
/// determine cup pressure), and (2) the suction force (from cup pressure) which
/// will be applied to the MultibodyPlant.
/// @note "Action point" is the point on the cup which the
/// ExternallyAppliedSpatialForcePair is applied on. "Edge points" are the
/// points on the edge of the suction cups to detect cup-object distance (i.e.
/// sealing status).
class CupObjInterface : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CupObjInterface)
  /// @brief Constructor of the CupObjInterface class.
  /// @param time_step Time step of the CupObjInterface leaf system, in s.
  /// @param suction_cup_area Effective suction cup area for each cup, in m^2.
  /// @param suction_cup_act_pt_geom_id_vec A vector of GeometryId of the cup
  /// action points, ordered by cup index (if there are multiple cups).
  /// @param suction_cup_act_pt_geom_id_to_body_idx_map An unordered_map from
  /// GeometryId of the cup action points to the corresponding BodyIndex.
  /// @param suction_cup_edge_pt_geom_id_vec A vector of vector of GeometryId of
  /// the cup edge points, each inner vector is for different edge point
  /// geometries and the outer vector is for different cups.
  /// @param obj_geom_id_to_body_idx_map An unordered_map from GeometryId of the
  /// object to the corresponding BodyIndex.
  CupObjInterface(double time_step, double suction_cup_area,
                  const std::vector<drake::geometry::GeometryId>&
                      suction_cup_act_pt_geom_id_vec,
                  const std::unordered_map<drake::geometry::GeometryId,
                                           drake::multibody::BodyIndex>&
                      suction_cup_act_pt_geom_id_to_body_idx_map,
                  const std::vector<std::vector<drake::geometry::GeometryId>>&
                      suction_cup_edge_pt_geom_id_vec,
                  const std::unordered_map<drake::geometry::GeometryId,
                                           drake::multibody::BodyIndex>&
                      obj_geom_id_to_body_idx_map);

  const drake::systems::InputPort<double>& GetGeomQueryInputPort() const {
    return get_input_port(geom_query_input_port_idx_);
  }
  const drake::systems::InputPort<double>& GetSuctionCupPressureInputPort()
      const {
    return get_input_port(suction_cup_pressure_input_port_idx_);
  }
  const drake::systems::OutputPort<double>& GetSuctionForceOutputPort() const {
    return get_output_port(suction_force_output_port_idx_);
  }
  const drake::systems::OutputPort<double>& GetCupObjDistOutputPort() const {
    return get_output_port(suction_cup_obj_dist_output_port_idx_);
  }

 private:
  drake::systems::EventStatus UpdateDists(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state_ptr) const;
  void CalcSuctionForce(
      const drake::systems::Context<double>& context,
      std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>*
          suction_force_vec_ptr) const;
  void OutputCupObjDist(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* cup_obj_dist_vec_ptr) const;

  double suction_cup_area_;

  std::vector<drake::geometry::GeometryId> suction_cup_act_pt_geom_id_vec_;
  std::unordered_map<drake::geometry::GeometryId, drake::multibody::BodyIndex>
      suction_cup_act_pt_geom_id_to_body_idx_map_;
  std::vector<std::vector<drake::geometry::GeometryId>>
      suction_cup_edge_pt_geom_id_vec_;
  std::unordered_map<drake::geometry::GeometryId, drake::multibody::BodyIndex>
      obj_geom_id_to_body_idx_map_;

  int num_suction_cups_{-1};

  int geom_query_input_port_idx_{-1};
  int suction_cup_pressure_input_port_idx_{-1};
  int suction_force_output_port_idx_{-1};
  int suction_cup_obj_dist_output_port_idx_{-1};

  int suction_cup_act_pt_closest_obj_signed_dist_pair_state_idx_{-1};
  int suction_cup_edge_pt_closest_obj_dist_state_idx_{-1};
};

}  // namespace drake::examples::multibody::suction_gripper
