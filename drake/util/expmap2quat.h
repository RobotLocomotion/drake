#include <Eigen/Dense>
#include "drake/drakeGeometryUtil_export.h"

DRAKEGEOMETRYUTIL_EXPORT Eigen::Vector4d expmap2quatNonDegenerate(const Eigen::Ref<const Eigen::Vector3d>& v, double theta);
DRAKEGEOMETRYUTIL_EXPORT Eigen::Vector4d expmap2quatDegenerate(const Eigen::Ref<const Eigen::Vector3d>& v, double theta);
DRAKEGEOMETRYUTIL_EXPORT Eigen::Matrix<double, 4, 3> dexpmap2quatNonDegenerate(const Eigen::Ref<const Eigen::Vector3d>& v, double theta);
DRAKEGEOMETRYUTIL_EXPORT Eigen::Matrix<double, 4, 3> dexpmap2quatDegenerate(const Eigen::Ref<const Eigen::Vector3d>& v, double theta);
DRAKEGEOMETRYUTIL_EXPORT Eigen::Matrix<double, 12, 3> ddexpmap2quatNonDegenerate(const Eigen::Ref<const Eigen::Vector3d>& v, double theta);
DRAKEGEOMETRYUTIL_EXPORT Eigen::Matrix<double, 12, 3> ddexpmap2quatDegenerate(const Eigen::Ref<const Eigen::Vector3d>& v, double theta);
