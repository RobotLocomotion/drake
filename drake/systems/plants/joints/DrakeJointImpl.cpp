#include "DrakeJointImpl.h"

using namespace Eigen;

template<typename Derived, typename Scalar>
Transform<Scalar, 3, Isometry> DrakeJointImpl<Derived>::jointTransform(const Ref< const Matrix<Scalar, Dynamic, 1> > & q) const
{
    return derived.jointTransform(q);
}

template<typename Derived, typename Scalar>
GradientVar<Scalar, 6, Dynamic> DrakeJointImpl<Derived>::motionSubspace(const Ref< const Matrix<Scalar, Dynamic, 1> > & q) const
{
    return derived.motionSubspace(q);
}

template<typename Derived, typename Scalar>
GradientVar<Scalar, 6, 1> DrakeJointImpl<Derived>::motionSubspaceDotTimesV(const Ref< const Matrix<Scalar, Dynamic, 1> > & q, const Ref< const Matrix<Scalar, Dynamic, 1> > & v, int gradient_order) const
{
    return derived.motionSubspaceDotTimesV(q, v, gradient_order);
}

template<typename Derived, typename Scalar>
GradientVar<Scalar, Dynamic, Dynamic> DrakeJointImpl<Derived>::qdot2v(const Ref< const Matrix<Scalar, Dynamic, 1> > & q, int gradient_order) const
{
    return derived.qdot2v(q, gradient_order);
}

template<typename Derived, typename Scalar>
GradientVar<Scalar, Dynamic, Dynamic> DrakeJointImpl<Derived>::v2qdot(const Ref< const Matrix<Scalar, Dynamic, 1> > & q, int gradient_order) const
{
    return derived.v2qdot(q, gradient_order);
}

template<typename Derived, typename Scalar>
GradientVar<Scalar, Dynamic, 1> DrakeJointImpl<Derived>::frictionTorque(const Ref< const Matrix<Scalar, Dynamic, 1> > & v, int gradient_order) const
{
    return derived.frictionTorque(v, gradient_order);
}

template DLLEXPORT_DRAKEJOINT Transform<double, 3, Isometry> DrakeJointImpl::jointTransform(const Ref< const Matrix<double, Dynamic, 1> > & q) const;
template DLLEXPORT_DRAKEJOINT Transform< AutoDiffScalar<VectorXd> , 3, Isometry> DrakeJointImpl::jointTransform(const Ref< const Matrix< AutoDiffScalar<VectorXd>, Dynamic, 1> > & q) const;

template DLLEXPORT_DRAKEJOINT GradientVar<double, 6, Dynamic> DrakeJointImpl::motionSubspace(const Ref< const Matrix<double, Dynamic, 1> > & q) const;
template DLLEXPORT_DRAKEJOINT GradientVar< AutoDiffScalar<VectorXd>, 6, Dynamic> DrakeJointImpl::motionSubspace(const Ref< const Matrix< AutoDiffScalar<VectorXd>, Dynamic, 1> > & q) const;

template DLLEXPORT_DRAKEJOINT GradientVar<double, 6, 1> DrakeJointImpl::motionSubspaceDotTimesV(const Ref< const Matrix<double, Dynamic, 1> > & q, const Ref< const Matrix<double, Dynamic, 1> > & v, int gradient_order) const;
template DLLEXPORT_DRAKEJOINT GradientVar< AutoDiffScalar<VectorXd>, 6, 1> DrakeJointImpl::motionSubspaceDotTimesV(const Ref< const Matrix< AutoDiffScalar<VectorXd>, Dynamic, 1> > & q, const Ref< const Matrix< AutoDiffScalar<VectorXd>, Dynamic, 1> > & v, int gradient_order) const;

template DLLEXPORT_DRAKEJOINT GradientVar<double, Dynamic, Dynamic> DrakeJointImpl::qdot2v(const Ref< const Matrix<double, Dynamic, 1> > & q, int gradient_order) const;
template DLLEXPORT_DRAKEJOINT GradientVar< AutoDiffScalar<VectorXd>, Dynamic, Dynamic> DrakeJointImpl::qdot2v(const Ref< const Matrix< AutoDiffScalar<VectorXd>, Dynamic, 1> > & q, int gradient_order) const;

template DLLEXPORT_DRAKEJOINT GradientVar<double, Dynamic, Dynamic> DrakeJointImpl::v2qdot(const Ref< const Matrix<double, Dynamic, 1> > & q, int gradient_order) const;
template DLLEXPORT_DRAKEJOINT GradientVar< AutoDiffScalar<VectorXd>, Dynamic, Dynamic> DrakeJointImpl::v2qdot(const Ref< const Matrix< AutoDiffScalar<VectorXd>, Dynamic, 1> > & q, int gradient_order) const;

template DLLEXPORT_DRAKEJOINT GradientVar<double, Dynamic, 1> DrakeJointImpl::frictionTorque(const Ref< const Matrix<double, Dynamic, 1> > & v, int gradient_order) const;
template DLLEXPORT_DRAKEJOINT GradientVar< AutoDiffScalar<VectorXd>, Dynamic, 1> DrakeJointImpl::frictionTorque(const Ref< const Matrix< AutoDiffScalar<VectorXd>, Dynamic, 1> > & v, int gradient_order) const;

