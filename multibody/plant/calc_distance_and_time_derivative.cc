#include "drake/multibody/plant/calc_distance_and_time_derivative.h"

namespace drake {
namespace multibody {
using geometry::FrameId;
// TODO(Sean.Curtis@tri.global): when SceneGraph has frame velocities, this
// function should be rolled into SceneGraph query.
SignedDistanceWithTimeDerivative CalcDistanceAndTimeDerivative(
    const MultibodyPlant<double>& plant,
    const SortedPair<geometry::GeometryId>& geometry_pair,
    const systems::Context<double>& context) {
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
  const FrameId frame_A_id = inspector.GetFrameId(signed_distance_pair.id_A);
  const FrameId frame_B_id = inspector.GetFrameId(signed_distance_pair.id_B);
  const Frame<double>& frameA =
      plant.GetBodyFromFrameId(frame_A_id)->body_frame();
  const Frame<double>& frameB =
      plant.GetBodyFromFrameId(frame_B_id)->body_frame();
  SignedDistanceWithTimeDerivative ret;
  ret.distance = signed_distance_pair.distance;
  // d = n̂_BA_Wᵀ * p_CbCa_W
  // where n̂_BA_W is the unit length contact normal pointing from body B to
  // body A, expressed in the world frame W. Ca is the witness point in
  // body A, Cb is the witness point body B.
  // Hence ḋ = p_CbCa_Wᵀ * dn̂_BA_W/dt + n̂_BA_Wᵀ * dp_CbCa_W/dt
  // As p_CbCa_W = d * n̂_BA_W
  // Hence ḋ = d * n̂_BA_Wᵀ * dn̂_BA_W/dt + n̂_BA_Wᵀ * dp_CbCa_W/dt
  // Since n̂_BA_W is unit length, n̂_BA_Wᵀ * n̂_BA_W = 1, taking the time
  // derivative we get 2 * n̂_BA_Wᵀ * dn̂_BA_W/dt = 0
  // Hence ḋ = n̂_BA_Wᵀ * dp_CbCa_W/dt
  // We know that p_CbCa_W = p_BCa_W - p_BCb_W, so taking the time derivative,
  // dp_CbCa_W/dt = dp_BCa_W/dt - dp_BCb_W/dt.
  // When the geometries A and B move, the witness point Ca and Cb can also
  // move, but they move in a direction that is perpendicular to the contact
  // normal n̂_BA_W. Hence
  // n̂_BA_Wᵀ * dp_CbCa_W/dt = n̂_BA_Wᵀ * (dp_BCa_W/dt - dp_BCb_W/dt)
  //                        = n̂_BA_Wᵀ * v_BCa_W
  // where v_BCa_W is the velocity of a point that instantaneously coincides
  // with Ca, but fixed on the geometry A.

  // signed_distance_pair.p_ACa is the location of the witness point Ca in the
  // geometry frame Ga. We need to compute Ca's position in body frame A.
  // We assume that the body frame A and the geometry frame F (the parent frame
  // of geometry A) coincide, hence X_AGa = X_FGa =
  // inspector.GetPoseInFrame(...).
  const Eigen::Vector3d& p_GaCa = signed_distance_pair.p_ACa;
  const Eigen::Vector3d p_ACa =
      inspector.GetPoseInFrame(signed_distance_pair.id_A) * p_GaCa;
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jv_v_BCa_W(3,
                                                      plant.num_velocities());
  plant.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable::kV, frameA, p_ACa, frameB,
      plant.world_frame(), &Jv_v_BCa_W);
  ret.distance_time_derivative = signed_distance_pair.nhat_BA_W.dot(
      Jv_v_BCa_W * plant.GetVelocities(context));
  return ret;
}
}  // namespace multibody
}  // namespace drake
