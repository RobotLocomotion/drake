#include "drake/multibody/tree/fixed_offset_frame.h"

#include <exception>
#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/rigid_body.h"

namespace drake {
namespace multibody {

template <typename T>
FixedOffsetFrame<T>::FixedOffsetFrame(
    const std::string& name, const Frame<T>& P,
    const math::RigidTransform<double>& X_PF,
    std::optional<ModelInstanceIndex> model_instance)
    : Frame<T>(name, P.body(), model_instance.value_or(P.model_instance())),
      parent_frame_(P),
      X_PF_(X_PF) {}

template <typename T>
FixedOffsetFrame<T>::FixedOffsetFrame(const std::string& name,
                                      const RigidBody<T>& B,
                                      const math::RigidTransform<double>& X_BF)
    : Frame<T>(name, B), parent_frame_(B.body_frame()), X_PF_(X_BF) {}

template <typename T>
FixedOffsetFrame<T>::~FixedOffsetFrame() = default;

template <typename T>
void FixedOffsetFrame<T>::SetPoseInParentFrame(
    systems::Context<T>* context, const math::RigidTransform<T>& X_PF) const {
  systems::BasicVector<T>& X_PF_parameter =
      context->get_mutable_numeric_parameter(X_PF_parameter_index_);
  X_PF_parameter.set_value(
      Eigen::Map<const VectorX<T>>(X_PF.GetAsMatrix34().data(), 12, 1));
}

template <typename T>
math::RigidTransform<T> FixedOffsetFrame<T>::GetPoseInParentFrame(
    const systems::Context<T>& context) const {
  const systems::BasicVector<T>& X_PF_parameter =
      context.get_numeric_parameter(X_PF_parameter_index_);
  return math::RigidTransform<T>(Eigen::Map<const Eigen::Matrix<T, 3, 4>>(
      X_PF_parameter.get_value().data()));
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Frame<ToScalar>> FixedOffsetFrame<T>::TemplatedDoCloneToScalar(
    const internal::MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& parent_frame_clone =
      tree_clone.get_variant(parent_frame_);
  auto new_frame = std::make_unique<FixedOffsetFrame<ToScalar>>(
      this->name(), parent_frame_clone, X_PF_, this->model_instance());
  new_frame->set_is_ephemeral(this->is_ephemeral());
  return new_frame;
}

template <typename T>
std::unique_ptr<Frame<double>> FixedOffsetFrame<T>::DoCloneToScalar(
    const internal::MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Frame<AutoDiffXd>> FixedOffsetFrame<T>::DoCloneToScalar(
    const internal::MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Frame<symbolic::Expression>>
FixedOffsetFrame<T>::DoCloneToScalar(
    const internal::MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Frame<T>> FixedOffsetFrame<T>::DoShallowClone() const {
  auto new_frame = std::make_unique<FixedOffsetFrame<T>>(
      this->name(), parent_frame_, X_PF_, this->model_instance());
  new_frame->set_is_ephemeral(this->is_ephemeral());
  return new_frame;
}

template <typename T>
math::RigidTransform<T> FixedOffsetFrame<T>::DoCalcPoseInBodyFrame(
    const systems::Parameters<T>& parameters) const {
  // X_BF = X_BP * X_PF
  const systems::BasicVector<T>& X_PF_parameter =
      parameters.get_numeric_parameter(X_PF_parameter_index_);
  const math::RigidTransform<T> X_PF =
      math::RigidTransform<T>(Eigen::Map<const Eigen::Matrix<T, 3, 4>>(
          X_PF_parameter.get_value().data()));
  return parent_frame_.CalcOffsetPoseInBody(parameters, X_PF);
}

template <typename T>
math::RotationMatrix<T> FixedOffsetFrame<T>::DoCalcRotationMatrixInBodyFrame(
    const systems::Parameters<T>& parameters) const {
  // R_BF = R_BP * R_PF
  const systems::BasicVector<T>& X_PF_parameter =
      parameters.get_numeric_parameter(X_PF_parameter_index_);
  return parent_frame_.CalcOffsetRotationMatrixInBody(
      parameters,
      math::RotationMatrix<T>(Eigen::Map<const Eigen::Matrix<T, 3, 4>>(
                                  X_PF_parameter.get_value().data())
                                  .template block<3, 3>(0, 0)));
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::FixedOffsetFrame);
