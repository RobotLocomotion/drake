#pragma once

#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace examples {
namespace multibody {
namespace suction_gripper {

/* Parameters for a SuctionCupModel. */
struct SuctionCupModelParameters {
  // The normal vector, expressed in a frame B, specifies the direction in which
  // the cup opening is pointing. That is, points on the side of the normal are
  // affected by the suction of cup. Points behind the plane defined by this
  // normal are behind the cup and are not affected by suction forces.
  Vector3<double> cup_normal_B;
  // The effective area of the cup. Typically the area defined by the outer rim
  // of a suction cup. For non-circular shapes a model could decide to use the
  // hydraulic diameter or similar quantities to define the area. In meters.
  double area;
  // Force at distances farther than max_suction_distance are zero, in meters.
  double max_suction_distance;
  // Maximum pump pressure when the cup is fully blocked and the flow rate is
  // zero. In Pa.
  double zero_flow_pressure;
};

/** Model for a suction cup.

 Assumptions:
  - Point model: The cup is modeled as a single point C on a body B.
  - Massless cup: The mass of the cup itself is neglected. If the mass of the
    cup must be accounted for, the modeler can lump its mass to the body B on
    which it attaches.
  - Ad-hoc force law: The force model is ad-hoc, not based on physics nor
    available experimental data. However, the model does encode the main
    attributes that we would expect. That is, the force is a decaying function
    of distance, with symmetry of revolution around the cup's axis and it goes
    to zero as points move behind the plane defined by the cup's normal.
   - Geometry: All geometry of the cup is lumped into its area. The model does
     not include deformations nor it predicts how the cup might conform to
     external objects.

 To be more specific, consider an object B in the neighborhood of the cup. Let ϕ
 be the distance of the object to the cup point C, and let P be the point on B
 closest to C. The position vector p_CP defines an angle θ with the suction
 normal n̂. Notice that ϕ = ‖p_CP‖. We then model the pressure at P with an
 ad-hoc law that vanishes at distances larger than a model threshold dₘₐₓ and
 that goes to zero "behind" the cup smoothly with the angle to the normal:

   p(P) = (dₘₐₓ-ϕ)₊/dₘₐₓ ⋅ pₘₐₓ ⋅ cos(θ)₊

 where pₘₐₓ is the maximum pressure when the cup is blocked (the flow rate is
 zero) and (x)₊ = max(0, x).

 @tparam_default_scalar
*/
template <typename T>
class SuctionCupModel {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SuctionCupModel)

  SuctionCupModel(SuctionCupModelParameters parameters)
      : parameters_(std::move(parameters)) {
    // TODO: check parameters invariants. E.g. positive pressures, etc.
  }

  /* Computes the suction force at a point P specified by its relative position
    to the center of the cup C, expressed in the frame B in which the cup is
    defined. Refer to the class's documentation for further details. 
    param[in] p_CP_B position of an external point P, relative to the cup's center C, expressed in a frame B. 
    */
  Vector3<T> CalcSuctionForce(const Vector3<T>& p_CP_B) const {
    using std::max;
    const double d_max = parameters_.max_suction_distance;
    // Distance along the normal. Put it another way, distance from P to C,
    // multiplied by cos(θ), with θ the angle between the normal and the vector
    // p_BC. This gives the model a sense of "flow direction", though completely
    // ad-hoc. Better models based on published data could use a more realistic
    // function of θ.
    // Only the positive part is considered (that is, the force should at least
    // become zero "behind the cup").
    const T phi = p_CP_B.norm();
    constexpr double epsilon =
        1.0e-14;  // small tolerance to avoid division by zero. This effectively
                  // smooths out the distance function for values lower than
                  // epsilon.
    const T cost_theta = parameters_.cup_normal_B.dot(p_CP_B) / (phi + epsilon);

    // Ad-hoc functional form of the pressure with distance and angle to the
    // cup's normal (i.e. the force should be zero if "behind" the cup).
    // In reality, pressure will be a  function of the pump's characteristic
    // curve and the blockage parameter at the cup's exit (in turn function of
    // external geometry). These will determine the flow rate and pressure for a
    // given pump head. Here this ad-hoc model is simply lumping this
    // computation into a single algebraic relationship.
    const T pressure =
        p_max * max(0.0, (d_max - phi) / d_max) * max(0.0, cost_theta);

    // Force at pint P, expressed in frame B.
    return pressure * parameters_.area * parameters_.cup_normal_B;
  }

 private:
  SuctionCupModelParameters parameters_;
};

/* Convenience struct to store the triplet {body, relative postion, cup
 * parameters} for a suction cup model attached at point C on a given body. */
struct SuctionCupPerBodyParameters {
  // Index for the body on which cup C attaches.
  multibody::BodyIndex body_index;
  // Position of cup C on body B.
  Vector3<double> p_BC;
  // Parameters for a SuctionCupModel.
  SuctionCupModelParameters cup_parameters;
};

/** A System that connects to the MultibodyPlant in order to model the effects
 of one or more suction cups attached to specific bodies.

 Each suction cup is modeled according to SuctionCupModel, refer to
 SuctionCupModel's documentation for details.
 
 @system
 name: SuctionCupsSystem
 input_ports:
 - command
 - query_object
 output_ports:
 - spatial_forces
 @endsystem

 - The command input is a BasicVector<T> with one element per suction cup. It is a scalar multiplier on the total force computed with SuctionCupModel. Typically a number in (0, 1) to model actuation on the suction strength.
 - The query_object port must be the same query object used by the
   MultibodyPlant model. It can be requested with MultibodPlat::
   get_geometry_query_input_port().
 - The output is of type std::vector<ExternallyAppliedSpatialForce<T>>; it is
  expected that this output will be connected to the @ref
  MultibodyPlant::get_applied_spatial_force_input_port()
  "externally_applied_spatial_force input port" of the MultibodyPlant.
- This system does not have any state.
- "command" and "query_object" direct feedthrough to the output spatial_forces.

@tparam_default_scalar
**/
template <typename T>
class SuctionCupsSystem : public drake::systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SuctionCupModel)

  /* Constructor for a suction cup model on a single body B specified in
    `parameters` */
  SuctionCupModel(const MultibodyPlant<T>* plant, const Body<T>& body,
                  Vector3<double> p_BC, SuctionCupParameters parameters)
      : plant_(plant), parameters_{body.index(), std::move(p_BC), std::move(parameters)} {
    // N.B. This system has not state. It is a feedthrough with its outputs
    // being completely determined from the inputs.
    query_object_input_port_ =
        DeclareAbstractInputPort(
            "query_object",
            drake::Value<drake::geometry::QueryObject<double>>())
            .get_index();
    this->DeclareInputPort("command", systems::kVectorValued,
                           num_suction_cups());
    this->DeclareAbstractOutputPort(
        "spatial_forces", std::vector<ExternallyAppliedSpatialForce<T>>(),
        &SuctionCupsSystem<T>::CalcSpatialForces);
  }

  /* Constructor for a model with a set of suction cup models. */
  SuctionCupModel(std::vector<SuctionCupPerBodyParameters> parameters);

  int num_suction_cups() const { return parameters_.size(); }

  const MultibodyPlant<T>& plant() const { return plant_; }

 private:
  const Body<T>& GetBodyGivenGeometryId(
      const drake::geometry::QueryObject<T>& query_object,
      GeometryId id) const {
    // TODO: Fill in the blanks. Get inspector form the query objet.
    const FrameId f_id = inspector.GetFrameId(id);
    const Body<T>* body = plant.GetBodyFromFrameId(f_id);
    DRAKE_DEMAND(body != nullptr);
    return *body;
  }

  void CalcSpatialForces(
      const systems::Context<T>& context,
      std::vector<ExternallyAppliedSpatialForce<T>>* spatial_forces) const {
    // spatial_forces->resize(num_propellers());
    const auto& command = get_command_input_port().Eval(context);
    const auto& query_object =
        get_query_object_input_port()
            .Eval<drake::geometry::QueryObject<T>>(context);

    // N.B. We do not know the number of externally applied force a priory.
    // Therefore we clear and the accumulate the results as we perform distance
    // queries.
    spatial_forces->clear();
    int cup_index = 0;
    for (const auto& body_cup : parameters_) {
      const BodyIndex body_index = body_cup.body_index;
      const Body<T>& body = plant().get_body(body_index);
      const math::RigidTransform<T>& X_WB = body.EvalPoseInWorld(context);
      const Vector3<T> p_WCup = X_WB * body_cup.p_BC;
      const std::vector<SignedDistanceToPoint<T>> distances =
          query_object.ComputeSignedDistanceToPoint(
              p_WCup, body_cup.parameters.max_suction_distance);

      const SuctionCupModel<T> cup_model(body_cup.parameters);

      // Compute suction force applied by cup C on all bodies at distance below
      // max_suction_distance.
      for (const SignedDistanceToPoint<T>& point_distance : distances) {
        const Body<T>& body = GetBodyGivenGeometryId(point_distance.id_G);
        const Vector3<T>& p_GP = point_distance.p_GN;
        // TODO: use inspector to query X_BG and transform p_GP to p_BP, the
        // postion of point P expressed in the body frame B.
        const T& distance = point_distance.distance;

        // Force on body B at point P, expressed in body frame B.
        const Vector3<T> f_Bp_B =
            command[cup_index] * cup_model.CalcSuctionForce(p_BP);

        // TODO: Transform to world frame, compute relative postion p_BoP_W and
        // load the externally applied forces vector.

        // TODO: Force on suction cup body is minus the force on B.

        spatial_forces->push_back({body.index(), p_BP, F_Bp_B});

        // Spatial force on cup body (usually a gripper or end effector), at
        // point C, expressed in the cup body frame.
        spatial_forces->push_back(
            {body_cup.body_index, body_cup.p_BC, F_CupBodyAtC});
      } 
      ++cup_index; 
    }
  }

  const MultibodyPlant<T>* plant_{nullptr};
  std::vector<SuctionCupPerBodyParameters> parameters_;
  systems::InputPortIndex query_object_input_port_{};  
};

}  // namespace suction_gripper
}  // namespace multibody
}  // namespace examples
}  // namespace drake