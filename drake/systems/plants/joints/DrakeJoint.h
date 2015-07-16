#ifndef DRAKEJOINT_H_
#define DRAKEJOINT_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
//#include "RigidBody.h"
#include <random>
#include "drakeGeometryUtil.h"
#include "GradientVar.h"

#undef DLLEXPORT_DRAKEJOINT
#if defined(WIN32) || defined(WIN64)
  #if defined(drakeJoints_EXPORTS)
    #define DLLEXPORT_DRAKEJOINT __declspec( dllexport )
  #else
    #define DLLEXPORT_DRAKEJOINT __declspec( dllimport )
  #endif
#else
  #define DLLEXPORT_DRAKEJOINT
#endif

#define INF -2147483648    // this number is only used for checking the pitch to see if it's a revolute joint or a helical joint, and is set to match the value handed to us for inf from matlab.

class RigidBody;
class RigidBodyManipulator;

class DLLEXPORT_DRAKEJOINT DrakeJoint
{
  // disable copy construction and assignment
  // not available in MSVC2010...
  // DrakeJoint(const DrakeJoint&) = delete;
  // DrakeJoint& operator=(const DrakeJoint&) = delete;
public:
  enum FloatingBaseType {
    FIXED        = 0,
    ROLLPITCHYAW = 1,
    QUATERNION   = 2
  };

private:
  const Eigen::Isometry3d transform_to_parent_body;
  const int num_positions;
  const int num_velocities;

protected:
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  const std::string name;

public:
  DrakeJoint(const std::string& name, const Eigen::Isometry3d& transform_to_parent_body, int num_positions, int num_velocities);
  typedef Eigen::Matrix<double, TWIST_SIZE, Eigen::Dynamic> MotionSubspaceType;

  virtual ~DrakeJoint();

  const Eigen::Isometry3d& getTransformToParentBody() const;

  const int getNumPositions() const;

  const int getNumVelocities() const;

  const std::string& getName() const;
  virtual std::string getPositionName(int index) const = 0;
  virtual std::string getVelocityName(int index) const { return getPositionName(index)+"dot"; }

  virtual Eigen::Isometry3d jointTransform(const Eigen::Ref<const Eigen::VectorXd>& q) const = 0;

  virtual bool isFloating() const { return false; }

  virtual void motionSubspace(const Eigen::Ref<const Eigen::VectorXd>& q, MotionSubspaceType& motion_subspace, Eigen::MatrixXd* dmotion_subspace = nullptr) const = 0;

  virtual void motionSubspaceDotTimesV(const Eigen::Ref<const Eigen::VectorXd>& q, const Eigen::Ref<const Eigen::VectorXd>& v, Vector6d& motion_subspace_dot_times_v,
      Gradient<Vector6d, Eigen::Dynamic>::type* dmotion_subspace_dot_times_vdq = nullptr,
      Gradient<Vector6d, Eigen::Dynamic>::type* dmotion_subspace_dot_times_vdv = nullptr) const = 0;

  virtual Eigen::VectorXd randomConfiguration(std::default_random_engine& generator) const = 0;

  virtual void qdot2v(const Eigen::Ref<const Eigen::VectorXd>& q, Eigen::MatrixXd& qdot_to_v, Eigen::MatrixXd* dqdot_to_v) const = 0;

  virtual void v2qdot(const Eigen::Ref<const Eigen::VectorXd>& q, Eigen::MatrixXd& v_to_qdot, Eigen::MatrixXd* dv_to_qdot) const = 0;

  virtual GradientVar<double, Eigen::Dynamic, 1> frictionTorque(const Eigen::Ref<const Eigen::VectorXd>& v, int gradient_order) const;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

#endif /* DRAKEJOINT_H_ */
