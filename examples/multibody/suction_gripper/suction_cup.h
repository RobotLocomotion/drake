#pragma once

#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace examples {
namespace multibody {
namespace suction_gripper {

/* Parameters for a SuctionCup system. */
struct SuctionCupParameters {
  // The mass of the suction cup device, in kilograms.
  double mass;
  // The effective linear stiffness k, in N/m, of the suction cup device. That
  // is, if we press the suction cup a distance d, we'd need to apply a force f
  // = k⋅d.
  double stiffness;
  // The effective damping of the suction cup.
  double damping;
  // Radius of the circular rim of the suction cup, in meters.
  double suction_rim_radius;
  // Force at distances farther than max_suction_distance are zero, in meters.
  double max_suction_distance;
  // The rim geometry is discretized with a number of discrete points.
  int num_rim_nodes;
  // Height of the suction cup, from it's attachment point C to the rim.
  double height;
  // Coefficient of dynamic friction of the cup's rim.
  double rim_friction;
};

/** A System that connects to the MultibodyPlant in order to model a compliant
 suction cup.

 // TODO: consider moving jpg image from readme to here, so that you can explain
 with a figure how the cup's rim is discretized and where springs/masses are
 located to model the cup's compliance. The README could reuse the same figure
 or simply reference these docs.

 @system
 name: SuctionCup
 input_ports:
 - suction_pressure
 - body_poses
 - query_object
 output_ports:
 - spatial_forces
 - loss_coefficient
 @endsystem

 - The suction_pressure input is a scalar value for the pressure on the
   suction side of the cup. It is a gage pressure relative to atmospheric (i.e.
   negative for vacuum).
 - The query_object port must be the same query object used by the
   MultibodyPlant model. It can be requested with MultibodPlat::
   get_geometry_query_input_port().
 - It is expected that the body_poses input should be connected to the
  @ref MultibodyPlant::get_body_poses_output_port() "MultibodyPlant body_poses
  output port".
 - The output is of type std::vector<ExternallyAppliedSpatialForce<T>>; it is
  expected that this output will be connected to the @ref
  MultibodyPlant::get_applied_spatial_force_input_port()
   "externally_applied_spatial_force input port" of the MultibodyPlant. It is a
   direct feedthrough of all input ports.
- This system does not have any state.
- loss_coefficient reports the loss coefficient of this cup model as a function
  of the cup's blockage. This is defined as K = Δp/(2ρv²), where Δp is the
  pressure drop across the cup, v the fluid's velocity and ρ it's density. This
  allows to connect the cup model to a hydraulic circuit that can be used to
  determines the suction pressure. Refer Section 6.9 of White, F.M., 2011. Fluid
  mechanics, seventh edition). It is a direct feedthrough of query_object only.

@tparam_default_scalar
**/
template <typename T>
class SuctionCup : public drake::systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SuctionCup)

  /** This function adds the necessary mass and spring elements into  `plant` to
   model a compliant suction cup model, and returns a new SuctionCup system
   modeled with these elements. Refer to this class's documentation for further
   details on the model.

   @param[in,out] plant On ouput, `plant` will contain new mass and spring
   elements needed to model a suction cup. After this call, plant.num_bodies()
   and plant.num_velocities() will increase by cup_parameters.num_rim_nodes+1.
   @param[in] body_index Index of a body already present in the input `plant` to
   which this new suction cup model is attached.
   @param[in] X_BC The pose of the cup frame C in the body frame B. Per this
   class's documentation, the z-axis points towards the outside of the cup,
   perpendicular to its suction area.
   @param[in] cup_parameters Parameters of the model defining the cup's size,
   mass and compliance. **/
  static std::unique_ptr<SuctionCup<T>> MakeAndAddToPlant(
      MultibodyPlant<T>* plant, BodyIndex body_index,
      const RigidTransform<double>& X_BC, SuctionCupParameters cup_parameters);

  /** This function makes a new SuctionCup model with MakeAndAddToPlant(), adds
   it to `builder` and connects its ports accordingly. **/
  static SuctionCup<T>* AddToBuilder(systems::DiagramBuilder<T>* builder,
                                     MultibodyPlant<T>* plant,
                                     BodyIndex body_index,
                                     const RigidTransform<double>& X_BC,
                                     SuctionCupParameters cup_parameters);

  /** The plant model to which this cup is added. **/
  const MultibodyPlant<T>& plant() const { return plant_; }

 private:
  // Struct to store MultibodyPlant quantities used to model a suction cup model
  // of mass m and linear stiffness k, with its rim discretized in N nodes.
  // There is a total of N + 1 bodies, with a total
  // mass of m.
  struct MultibodyElements {
    // Main body of the cup model, of mass m / 2.
    BodyIndex cup_body;
    // Bodies conforming the discretized rim of the cup.
    // Each of mass m / (2 * N).
    std::vector<BodyIndex> rim_bodies;
    // rim_contact_points_[i] stores he id for the contact point (zero radius
    // sphere) for body rim_bodies_[i].
    std::vector<GeometryId> rim_contact_points;
  };

  /* Constructor for a suction cup model attached on a body B.
   N.B. Since making a cup model requires the coordination of constructing the
   system, adding new elements to a MultibodyPlant and appropriately wiring
   input ports, we make construction private and provide users with helper
   methods MakeAndAddToPlant() and AddToBuilder(). */
  SuctionCup(const MultibodyPlant<T>* plant, BodyIndex body_index,
             SuctionCupParameters parameters,
             MultibodyElements multibody_elements);

  // The suction cup is modeled as a network of spring and masses.
  // This method add the necessary multibody elements as described in the
  // class's documentation.
  MultibodyElements AddMultibodyPlantElements(
      BodyIndex body_index, const RigidTransform<double>& X_BC,
      const SuctionCupParameters& cup_parameters
          MultibodyPlant<T>* plant) const;

  // Helper method to get the body corresponding to a given geometry.
  const Body<T>& GetBodyGivenGeometryId(
      const drake::geometry::QueryObject<T>& query_object,
      GeometryId id) const {
    // TODO: Fill in the blanks. Get inspector form the query objet.
    const FrameId f_id = inspector.GetFrameId(id);
    const Body<T>* body = plant.GetBodyFromFrameId(f_id);
    DRAKE_DEMAND(body != nullptr);
    return *body;
  }

  // TODO: implement this method. Righ now only an (old) example.
  void CalcSpatialForces(
      const systems::Context<T>& context,
      std::vector<ExternallyAppliedSpatialForce<T>>* spatial_forces) const {
    // spatial_forces->resize(num_propellers());
    const auto& command = get_command_input_port().Eval(context);
    const auto& query_object =
        get_query_object_input_port().Eval<drake::geometry::QueryObject<T>>(
            context);

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
  SuctionCupParameters parameters_;
  systems::InputPortIndex suction_pressure_index_{};
  systems::InputPortIndex body_poses_index_{};
  systems::InputPortIndex query_object_input_port_{};
  // MultibodyPlant elements used in the modeling of the suction cup.
  MultibodyElements multibody_elements_;
};

}  // namespace suction_gripper
}  // namespace multibody
}  // namespace examples
}  // namespace drake