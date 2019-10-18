#include "drake/multibody/plant/point_pair_contact_info.h"

namespace drake {
namespace multibody {

template <typename T>
PointPairContactInfo<T>::PointPairContactInfo(
    BodyIndex bodyA_index, BodyIndex bodyB_index,
    const Vector3<T>& f_Bc_W, const Vector3<T>& p_WC,
    const T& separation_velocity, const T& slip_speed,
    const drake::geometry::PenetrationAsPointPair<T>& point_pair) :
    point_pair_(point_pair),
    bodyA_index_(bodyA_index),
    bodyB_index_(bodyB_index),
    f_Bc_W_(f_Bc_W),
    p_WC_(p_WC),
    separation_speed_(separation_velocity),
    slip_speed_(slip_speed) {}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::PointPairContactInfo)
