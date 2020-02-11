#include "drake/multibody/plant/calc_distance_and_time_derivative.h"

namespace drake {
namespace multibody {
void CalcDistanceAndTimeDerivative(
    const multibody::MultibodyPlant<double>& plant,
    const SortedPair<geometry::GeometryId>& geometry_pair,
    const systems::Context<double>& context, double* distance,
    double* distance_time_derivative) {
  DRAKE_DEMAND(distance != nullptr);
  DRAKE_DEMAND(distance_time_derivative != nullptr);
  if (!plant.geometry_source_is_registered()) {
    throw std::invalid_argument(
        "CalcDistanceAndTimeDerivative: MultibodyPlant has not registered with "
        "a SceneGraph yet.");
  }
  const auto& query_port = plant.get_geometry_query_input_port();
  const auto& query_object =
      query_port.Eval<geometry::QueryObject<double>>(context);
  const geometry::SignedDistancePair<double> signed_distance_pair =
      query_object.ComputeSignedDistancePairClosestPoints(
          geometry_pair.first(), geometry_pair.second());
  const geometry::SceneGraphInspector<double>& inspector =
      query_object.inspector();
  const geometry::FrameId frame_A_id =
      inspector.GetFrameId(signed_distance_pair.id_A);
  const geometry::FrameId frame_B_id =
      inspector.GetFrameId(signed_distance_pair.id_B);
  const multibody::Frame<double>& frameA =
      plant.GetBodyFromFrameId(frame_A_id)->body_frame();
  const multibody::Frame<double>& frameB =
      plant.GetBodyFromFrameId(frame_B_id)->body_frame();
  *distance = signed_distance_pair.distance;
  // d = n̂_BA_Wᵀ * p_CbCa_W
  // where n̂_BA_W is the unit length contact normal pointing from body B to
  // body A, expressed in the world frame W. Ca is the witness point in
  // body A, Cb is the witness point body B.
  // Hence ḋ = p_CbCa_Wᵀ * dn̂_BA_W/dt + n̂_BA_Wᵀ * dp_CbCa_W/dt
  // As p_CbCa_W = d * n̂_BA_W
  // Hence ḋ = d * n̂_BA_Wᵀ * dn̂_BA_W/dt + n̂_BA_Wᵀ * dp_CbCa_W/dt
  // Since n̂_BA_W is unit length, n̂_BA_Wᵀ * n̂_BA_W = 1, taking the time
  // derivative we get n̂_BA_Wᵀ * dn̂_BA_W/dt = 0
  // Hence ḋ = n̂_BA_Wᵀ * dp_CbCa_W/dt
  // When the geometries A and B move, the witness point Ca and Cb can also
  // move, but they move in a direction that is perpendicular to the contact
  // normal n̂_BA_W. Hence n̂_BA_Wᵀ * dp_CbCa_W/dt = n̂_BA_Wᵀ * v_BCa_W
  const Eigen::Vector3d p_ACa =
      inspector.GetPoseInFrame(signed_distance_pair.id_A) *
      signed_distance_pair.p_ACa;
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jv_v_BCa_W(3,
                                                      plant.num_velocities());
  plant.CalcJacobianTranslationalVelocity(
      context, multibody::JacobianWrtVariable::kV, frameA, p_ACa, frameB,
      plant.world_frame(), &Jv_v_BCa_W);
  *distance_time_derivative = signed_distance_pair.nhat_BA_W.dot(
      Jv_v_BCa_W * plant.GetVelocities(context));
}
}  // namespace multibody
}  // namespace drake
