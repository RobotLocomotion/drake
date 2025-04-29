#include "drake/multibody/tree/prismatic_joint.h"

#include <memory>
#include <stdexcept>

#include "drake/multibody/tree/multibody_tree-inl.h"

namespace drake {
namespace multibody {

template <typename T>
PrismaticJoint<T>::PrismaticJoint(const std::string& name,
                                  const Frame<T>& frame_on_parent,
                                  const Frame<T>& frame_on_child,
                                  const Vector3<double>& axis,
                                  double pos_lower_limit,
                                  double pos_upper_limit, double damping)
    : Joint<T>(
          name, frame_on_parent, frame_on_child,
          VectorX<double>::Constant(1, damping),
          VectorX<double>::Constant(1, pos_lower_limit),
          VectorX<double>::Constant(1, pos_upper_limit),
          VectorX<double>::Constant(1,
                                    -std::numeric_limits<double>::infinity()),
          VectorX<double>::Constant(1, std::numeric_limits<double>::infinity()),
          VectorX<double>::Constant(1,
                                    -std::numeric_limits<double>::infinity()),
          VectorX<double>::Constant(1,
                                    std::numeric_limits<double>::infinity())) {
  const double kEpsilon = std::sqrt(std::numeric_limits<double>::epsilon());
  if (axis.isZero(kEpsilon)) {
    throw std::logic_error(
        "Prismatic joint axis vector must have nonzero length.");
  }
  if (damping < 0) {
    throw std::logic_error("Prismatic joint damping must be nonnegative.");
  }
  axis_ = axis.normalized();
}

template <typename T>
PrismaticJoint<T>::~PrismaticJoint() = default;

template <typename T>
const std::string& PrismaticJoint<T>::type_name() const {
  static const never_destroyed<std::string> name{kTypeName};
  return name.access();
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Joint<ToScalar>> PrismaticJoint<T>::TemplatedDoCloneToScalar(
    const internal::MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& frame_on_parent_body_clone =
      tree_clone.get_variant(this->frame_on_parent());
  const Frame<ToScalar>& frame_on_child_body_clone =
      tree_clone.get_variant(this->frame_on_child());

  // Make the Joint<T> clone.
  auto joint_clone = std::make_unique<PrismaticJoint<ToScalar>>(
      this->name(), frame_on_parent_body_clone, frame_on_child_body_clone,
      this->translation_axis(), this->position_lower_limit(),
      this->position_upper_limit(), this->default_damping());
  joint_clone->set_velocity_limits(this->velocity_lower_limits(),
                                   this->velocity_upper_limits());
  joint_clone->set_acceleration_limits(this->acceleration_lower_limits(),
                                       this->acceleration_upper_limits());
  joint_clone->set_default_positions(this->default_positions());

  return joint_clone;
}

template <typename T>
std::unique_ptr<Joint<double>> PrismaticJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Joint<AutoDiffXd>> PrismaticJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Joint<symbolic::Expression>> PrismaticJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Joint<T>> PrismaticJoint<T>::DoShallowClone() const {
  return std::make_unique<PrismaticJoint<T>>(
      this->name(), this->frame_on_parent(), this->frame_on_child(),
      this->translation_axis(), this->position_lower_limit(),
      this->position_upper_limit(), this->default_damping());
}

/* For a prismatic joint, we are given Jp on parent P, Jc on child C, and a
translation unit vector â whose components are identical in Jp and Jc. At q=0,
Jp and Jc are coincident. At all times, the coordinate axes of Jp and Jc remain
aligned, and vector â has constant and equal components when expressed in these
frames. Jc translates with respect to Jp by a translation distance q meters
along the translation vector â. The origins Jpo and Jco are thus separated by
a vector q⋅â so that Jco = Jpo + q⋅â.

We need to implement this joint with one of three available prismatic
mobilizers. Every mobilizer has an inboard frame F, and an outboard frame M. The
available prismatic mobilizers translate along one of the coordinate axes x, y,
or z. If â happens already to be a coordinate axis of Jp (and Jc), we are golden
and can use Jp and Jc as F and M. Otherwise we are going to have to create new
frames F and M such that one of their coordinate axes is aligned with â.

To create new frames, we use the MakeFromOneVector() function to create a
RotationMatrix R_JpF (R_JcM) such that F(M)'s z axis is aligned with â (or -â if
the mobilizer is reversed from the joint). That rotation matrix is what we need
to create FixedOffsetFrame F from Jp (or Jc if reversed) and M from Jc (or Jp
if reversed). Then we use the z-axial prismatic mobilizer to implement the
joint. */
template <typename T>
std::unique_ptr<internal::Mobilizer<T>>
PrismaticJoint<T>::MakeMobilizerForJoint(
    const internal::SpanningForest::Mobod& mobod,
    internal::MultibodyTree<T>* tree) const {
  DRAKE_DEMAND(tree != nullptr);
  const bool reverse = mobod.is_reversed();
  // These are the joint's parent and child frames, but adjusted for
  // reversal to locate them on the inboard and outboard bodies. We may also
  // need to reverse the axis so that q will retain its expected sign.
  const Frame<T>& Jin =
      reverse ? this->frame_on_child() : this->frame_on_parent();
  const Frame<T>& Jout =
      reverse ? this->frame_on_parent() : this->frame_on_child();
  const Eigen::Vector3d axis = reverse ? -axis_ : axis_;  // a unit vector

  // Determine whether the axis is one of +x, +y, +z, or something else.
  // In the latter case we'll change this to +z below.
  std::optional<int> which_axis = [&axis]() -> std::optional<int> {
    for (int i = 0; i < 3; ++i) {
      if (axis[i] == 1.0) return i;  // exact match only
    }
    return {};
  }();

  const Frame<T>* F{};
  const Frame<T>* M{};
  if (!which_axis) {
    which_axis = 2;  // Arbitrarily using the z axis mobilizer.
    const math::RotationMatrixd R_JinF =  // Also R_JoutM, since Jp=Jc at q=0.
        math::RotationMatrixd::MakeFromOneUnitVector(axis, *which_axis);
    F = &tree->AddEphemeralFrame(std::make_unique<FixedOffsetFrame<T>>(
        this->MakeUniqueOffsetFrameName(Jin, "F"), Jin,
        math::RigidTransformd(R_JinF), this->model_instance()));
    M = &tree->AddEphemeralFrame(std::make_unique<FixedOffsetFrame<T>>(
        this->MakeUniqueOffsetFrameName(Jout, "M"), Jout,
        math::RigidTransformd(R_JinF), this->model_instance()));
  } else {
    F = &Jin;
    M = &Jout;
  }

  std::unique_ptr<internal::PrismaticMobilizer<T>> prismatic_mobilizer;

  switch (*which_axis) {
    case 0:
      prismatic_mobilizer =
          std::make_unique<internal::PrismaticMobilizerAxial<T, 0>>(mobod, *F,
                                                                    *M);
      break;
    case 1:
      prismatic_mobilizer =
          std::make_unique<internal::PrismaticMobilizerAxial<T, 1>>(mobod, *F,
                                                                    *M);
      break;
    case 2:
      prismatic_mobilizer =
          std::make_unique<internal::PrismaticMobilizerAxial<T, 2>>(mobod, *F,
                                                                    *M);
      break;
  }

  DRAKE_DEMAND(prismatic_mobilizer != nullptr);
  prismatic_mobilizer->set_default_position(this->default_positions());
  return prismatic_mobilizer;
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::PrismaticJoint);
