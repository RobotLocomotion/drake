#include <Eigen/Dense>

Eigen::Vector4d expmap2quatNonDegenerate(const Eigen::Ref<const Eigen::Vector3d>& v, double theta);
Eigen::Vector4d expmap2quatDegenerate(const Eigen::Ref<const Eigen::Vector3d>& v, double theta);
Eigen::Matrix<double, 4, 3> dexpmap2quatNonDegenerate(const Eigen::Ref<const Eigen::Vector3d>& v, double theta);
Eigen::Matrix<double, 4, 3> dexpmap2quatDegenerate(const Eigen::Ref<const Eigen::Vector3d>& v, double theta);
Eigen::Matrix<double, 12, 3> ddexpmap2quatNonDegenerate(const Eigen::Ref<const Eigen::Vector3d>& v, double theta);
Eigen::Matrix<double, 12, 3> ddexpmap2quatDegenerate(const Eigen::Ref<const Eigen::Vector3d>& v, double theta);
