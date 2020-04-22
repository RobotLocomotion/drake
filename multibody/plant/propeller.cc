#include "drake/multibody/plant/propeller.h"

#include "drake/geometry/frame_kinematics_vector.h"

namespace drake {
namespace multibody {

template <typename T>
Propeller<T>::Propeller(const MultibodyPlant<T>& multibody_plant,
                        const Body<T>& body,
                        const math::RigidTransform<double>& X_BP,
                        double thrust_ratio, double moment_ratio)
    : Propeller({PropellerInfo(multibody_plant, body, X_BP, thrust_ratio,
                               moment_ratio)}) {}

template <typename T>
Propeller<T>::Propeller(const std::vector<PropellerInfo>& propeller_info)
    : systems::LeafSystem<T>(systems::SystemTypeTag<Propeller>{}),
      info_(propeller_info) {
  this->DeclareInputPort("command", systems::kVectorValued, num_propellers());

  this->DeclareAbstractInputPort("geometry_poses",
                                 Value<geometry::FramePoseVector<T>>());

  this->DeclareAbstractOutputPort("spatial_forces",
                                  &Propeller<T>::CalcSpatialForces);
}

template <typename T>
void Propeller<T>::CalcSpatialForces(
    const systems::Context<T>& context,
    std::vector<ExternallyAppliedSpatialForce<T>>* spatial_forces) const {
  spatial_forces->resize(num_propellers());

  const VectorX<T> command = get_command_input_port().Eval(context);
  const auto& poses = get_geometry_poses_input_port()
                          .template Eval<geometry::FramePoseVector<T>>(context);

  for (int i = 0; i < num_propellers(); i++) {
    const PropellerInfo& prop = info_[i];
    const math::RigidTransform<T>& X_WB = poses.value(prop.frame_id);
    const math::RigidTransform<T> X_WP = X_WB * prop.X_BP.cast<T>();

    // Map to the ExternalSpatialForce structure:
    //  - the origin of my frame P is at Bq, and
    //  - the origin of my frame B is at Bo.
    const SpatialForce<T> F_P_B(
        Vector3<T>(0, 0, command[i] * prop.moment_ratio),
        Vector3<T>(0, 0, command[i] * prop.thrust_ratio));
    spatial_forces->at(i).body_index = prop.body_index;
    spatial_forces->at(i).p_BoBq_B = prop.X_BP.translation();
    spatial_forces->at(i).F_Bq_W = X_WP.rotation() * F_P_B;
  }
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::Propeller)
