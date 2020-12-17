#include "drake/multibody/plant/propeller.h"

namespace drake {
namespace multibody {

template <typename T>
Propeller<T>::Propeller(const BodyIndex& body_index,
                        const math::RigidTransform<double>& X_BP,
                        double thrust_ratio, double moment_ratio)
    : Propeller({PropellerInfo(body_index, X_BP, thrust_ratio,
                               moment_ratio)}) {}

template <typename T>
Propeller<T>::Propeller(const std::vector<PropellerInfo>& propeller_info)
    : systems::LeafSystem<T>(systems::SystemTypeTag<Propeller>{}),
      info_(propeller_info) {
  this->DeclareInputPort("command", systems::kVectorValued, num_propellers());

  this->DeclareAbstractInputPort("body_poses",
                                 Value<std::vector<math::RigidTransform<T>>>());

  this->DeclareAbstractOutputPort(
      "spatial_forces",
      std::vector<ExternallyAppliedSpatialForce<T>>(num_propellers()),
      &Propeller<T>::CalcSpatialForces);
}

template <typename T>
void Propeller<T>::CalcSpatialForces(
    const systems::Context<T>& context,
    std::vector<ExternallyAppliedSpatialForce<T>>* spatial_forces) const {
  spatial_forces->resize(num_propellers());

  const auto& command = get_command_input_port().Eval(context);
  const auto& poses =
      get_body_poses_input_port()
          .template Eval<std::vector<math::RigidTransform<T>>>(context);

  for (int i = 0; i < num_propellers(); i++) {
    const PropellerInfo& prop = info_[i];

    // Map to the ExternalSpatialForce structure:
    //  - the origin of my frame P is Po == Bq, and
    //  - the origin of my frame B is Bo.
    const math::RigidTransform<T>& X_WB = poses[prop.body_index];
    const Eigen::Vector3d& p_BoPo_B = prop.X_BP.translation();
    const math::RigidTransform<T> X_WP = X_WB * prop.X_BP.cast<T>();
    const math::RotationMatrix<T>& R_WP = X_WP.rotation();

    const SpatialForce<T> F_BPo_P(
        Vector3<T>(0, 0, command[i] * prop.moment_ratio),
        Vector3<T>(0, 0, command[i] * prop.thrust_ratio));
    const SpatialForce<T> F_BPo_W = R_WP * F_BPo_P;
    spatial_forces->at(i).body_index = prop.body_index;
    spatial_forces->at(i).p_BoBq_B = p_BoPo_B;
    spatial_forces->at(i).F_Bq_W = F_BPo_W;
  }
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::Propeller)
