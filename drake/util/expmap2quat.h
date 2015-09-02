#include <Eigen/Dense>

#undef DLLEXPORT
#if defined(WIN32) || defined(WIN64)
#if defined(drakeGeometryUtil_EXPORTS)
#define DLLEXPORT __declspec( dllexport )
#else
#define DLLEXPORT __declspec( dllimport )
#endif
#else
#define DLLEXPORT
#endif

DLLEXPORT Eigen::Vector4d expmap2quatNonDegenerate(const Eigen::Ref<const Eigen::Vector3d>& v, double theta);
DLLEXPORT Eigen::Vector4d expmap2quatDegenerate(const Eigen::Ref<const Eigen::Vector3d>& v, double theta);
DLLEXPORT Eigen::Matrix<double, 4, 3> dexpmap2quatNonDegenerate(const Eigen::Ref<const Eigen::Vector3d>& v, double theta);
DLLEXPORT Eigen::Matrix<double, 4, 3> dexpmap2quatDegenerate(const Eigen::Ref<const Eigen::Vector3d>& v, double theta);
DLLEXPORT Eigen::Matrix<double, 12, 3> ddexpmap2quatNonDegenerate(const Eigen::Ref<const Eigen::Vector3d>& v, double theta);
DLLEXPORT Eigen::Matrix<double, 12, 3> ddexpmap2quatDegenerate(const Eigen::Ref<const Eigen::Vector3d>& v, double theta);
