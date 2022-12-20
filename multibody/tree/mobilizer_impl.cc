#include "drake/multibody/tree/mobilizer_impl.h"

#include <memory>

#include "drake/multibody/tree/planar_mobilizer.h"
#include "drake/multibody/tree/prismatic_mobilizer.h"
#include "drake/multibody/tree/quaternion_floating_mobilizer.h"
#include "drake/multibody/tree/revolute_mobilizer.h"
#include "drake/multibody/tree/screw_mobilizer.h"
#include "drake/multibody/tree/space_xyz_floating_mobilizer.h"
#include "drake/multibody/tree/space_xyz_mobilizer.h"
#include "drake/multibody/tree/universal_mobilizer.h"
#include "drake/multibody/tree/weld_mobilizer.h"

namespace drake {
namespace multibody {
namespace internal {

// Helper classes to aid the explicit instantiation with macro
// DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(), which
// expects a template class as an argument.
template <typename T>
class WeldImpl : public MobilizerImpl<T, 0, 0, WeldMobilizer> {};
template <typename T>
class PrismaticImpl : public MobilizerImpl<T, 1, 1, PrismaticMobilizer> {};
template <typename T>
class RevoluteImpl : public MobilizerImpl<T, 1, 1, RevoluteMobilizer> {};
template <typename T>
class ScrewImpl : public MobilizerImpl<T, 1, 1, ScrewMobilizer> {};
template <typename T>
class UniversalImpl : public MobilizerImpl<T, 2, 2, UniversalMobilizer> {};
template <typename T>
class PlanarImpl : public MobilizerImpl<T, 3, 3, PlanarMobilizer> {};
template <typename T>
class SpaceXYZImpl : public MobilizerImpl<T, 3, 3, SpaceXYZMobilizer> {};
template <typename T>
class SpaceXYZFloatingImpl
    : public MobilizerImpl<T, 6, 6, SpaceXYZFloatingMobilizer> {};
template <typename T>
class QuaternionFloatingImpl
    : public MobilizerImpl<T, 7, 6, QuaternionFloatingMobilizer> {};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::WeldImpl)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::PrismaticImpl)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::RevoluteImpl)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::ScrewImpl)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::UniversalImpl)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::PlanarImpl)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::SpaceXYZImpl)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::SpaceXYZFloatingImpl)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::QuaternionFloatingImpl)
