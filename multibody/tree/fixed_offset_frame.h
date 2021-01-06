#pragma once

#include <memory>
#include <optional>
#include <string>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/frame.h"
#include "drake/multibody/tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {

// Forward declarations.
template <class T> class BodyFrame;
template <class T> class RigidBody;

/// %FixedOffsetFrame represents a material frame F whose pose is fixed with
/// respect to a _parent_ material frame P. The pose offset is given by a
/// spatial transform `X_PF`, which is constant after construction. For
/// instance, we could rigidly attach a frame F to move with a rigid body B at a
/// fixed pose `X_BF`, where B is the BodyFrame associated with body B.
/// Thus, the World frame pose `X_WF` of a %FixedOffsetFrame F depends only on
/// the World frame pose `X_WP` of its parent P, and the constant pose `X_PF`,
/// with `X_WF=X_WP*X_PF`.
///
/// For more information about spatial transforms, see
/// @ref multibody_spatial_pose. <!-- https://drake.mit.edu/doxygen_cxx/
///                                   group__multibody__spatial__pose.html -->
///
/// @tparam_default_scalar
template <typename T>
class FixedOffsetFrame final : public Frame<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FixedOffsetFrame)

  /// Creates a material Frame F whose pose is fixed with respect to its
  /// parent material Frame P. The pose is given by a spatial transform `X_PF`;
  /// see class documentation for more information.
  ///
  /// @param[in] name
  ///   The name of this frame.
  /// @param[in] P
  ///   The frame to which this frame is attached with a fixed pose.
  /// @param[in] X_PF
  ///   The _default_ transform giving the pose of F in P, therefore only the
  ///   value (as a RigidTransform<double>) is provided.
  /// @param[in] model_instance
  ///   The model instance to which this frame belongs to. If unspecified, will
  ///   use P.body().model_instance().
  FixedOffsetFrame(
      const std::string& name, const Frame<T>& P,
      const math::RigidTransform<double>& X_PF,
      std::optional<ModelInstanceIndex> model_instance = {});

  /// Creates an unnamed material Frame F. See overload with name for more
  /// information.
  FixedOffsetFrame(
      const Frame<T>& P, const math::RigidTransform<double>& X_PF)
      : FixedOffsetFrame("", P, X_PF) {}

  /// Creates a material Frame F whose pose is fixed with respect to the
  /// BodyFrame B of the given Body, which serves as F's parent frame.
  /// The pose is given by a spatial transform `X_BF`; see class documentation
  /// for more information.
  ///
  /// @param[in] name  The name of this frame.
  /// @param[in] bodyB The body whose BodyFrame B is to be F's parent frame.
  /// @param[in] X_BF  The transform giving the pose of F in B.
  FixedOffsetFrame(
      const std::string& name, const Body<T>& bodyB,
      const math::RigidTransform<double>& X_BF);

  /// Creates an unnamed material Frame F. See overload with name for more
  /// information.
  FixedOffsetFrame(
      const Body<T>& bodyB, const math::RigidTransform<double>& X_BF)
      : FixedOffsetFrame("", bodyB, X_BF) {}

  math::RigidTransform<T> CalcPoseInBodyFrame(
      const systems::Context<T>& context) const override {
    // X_BF = X_BP * X_PF
    const systems::BasicVector<T>& X_PF_parameter =
        context.get_numeric_parameter(X_PF_parameter_index_);
    return parent_frame_.CalcOffsetPoseInBody(
        context,
        math::RigidTransform<T>(Eigen::Map<const Eigen::Matrix<T, 3, 4>>(
            X_PF_parameter.get_value().data())));
  }

  math::RotationMatrix<T> CalcRotationMatrixInBodyFrame(
      const systems::Context<T>& context) const override {
    // R_BF = R_BP * R_PF
    const systems::BasicVector<T>& X_PF_parameter =
        context.get_numeric_parameter(X_PF_parameter_index_);
    return parent_frame_.CalcOffsetRotationMatrixInBody(
        context,
        math::RotationMatrix<T>(Eigen::Map<const Eigen::Matrix<T, 3, 4>>(
                                    X_PF_parameter.get_value().data())
                                    .template block<3, 3>(0, 0)));
  }

  void SetPoseInBodyFrame(systems::Context<T>* context,
                          const math::RigidTransform<T>& X_PF) const {
    systems::BasicVector<T>& X_PF_parameter =
        context->get_mutable_numeric_parameter(X_PF_parameter_index_);
    X_PF_parameter.set_value(
        Eigen::Map<const VectorX<T>>(X_PF.GetAsMatrix34().data(), 12, 1));
  }

  /// @returns The default fixed pose in the body frame.
  math::RigidTransform<T> GetFixedPoseInBodyFrame() const override {
    // X_BF = X_BP * X_PF
    return parent_frame_.GetFixedOffsetPoseInBody(X_PF_.cast<T>());
  }

  /// @returns The default rotation matrix of this fixed pose in the body frame.
  math::RotationMatrix<T> GetFixedRotationMatrixInBodyFrame() const override {
    // R_BF = R_BP * R_PF
    const math::RotationMatrix<double>& R_PF = X_PF_.rotation();
    return parent_frame_.GetFixedRotationMatrixInBody(R_PF.cast<T>());
  }

 protected:
  /// @name Methods to make a clone templated on different scalar types.
  ///
  /// These methods provide implementations to the different overrides of
  /// Frame::DoCloneToScalar().
  /// The only const argument to these methods is the new MultibodyTree clone
  /// under construction, which is required to already own the clone to the
  /// parent frame of the frame being cloned.
  /// @{

  /// @pre The parent frame to this frame already has a clone in `tree_clone`.
  std::unique_ptr<Frame<double>> DoCloneToScalar(
      const internal::MultibodyTree<double>& tree_clone) const override;

  /// @pre The parent frame to this frame already has a clone in `tree_clone`.
  std::unique_ptr<Frame<AutoDiffXd>> DoCloneToScalar(
      const internal::MultibodyTree<AutoDiffXd>& tree_clone) const override;

  std::unique_ptr<Frame<symbolic::Expression>> DoCloneToScalar(
      const internal::MultibodyTree<symbolic::Expression>&) const override;
  /// @}

 private:
  // Implementation for MultibodyElement::DoDeclareParameters().
  // FixedOffsetFrame declares a single parameter for its RigidTransform.
  void DoDeclareParameters(
      internal::MultibodyTreeSystem<T>* tree_system) final {
    // Declare parent classes' parameters
    Frame<T>::DoDeclareParameters(tree_system);
    X_PF_parameter_index_ = this->DeclareNumericParameter(
        tree_system,
        systems::BasicVector<T>(Eigen::Map<const VectorX<T>>(
            X_PF_.template cast<T>().GetAsMatrix34().data(), 12, 1)));
  }

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Frame<ToScalar>> TemplatedDoCloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const;

  // The frame to which this frame is attached.
  const Frame<T>& parent_frame_;

  // Spatial transform giving the fixed pose of this frame F measured in the
  // parent frame P.
  const math::RigidTransform<double> X_PF_;

  // System parameter indices for `this` frame's RigidTransform stored in a
  // context.
  systems::NumericParameterIndex X_PF_parameter_index_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::FixedOffsetFrame)
