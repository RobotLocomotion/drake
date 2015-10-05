#ifndef DRAKEJOINT_H_
#define DRAKEJOINT_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/AutoDiff>
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


#define POSITION_AND_VELOCITY_DEPENDENT_METHODS(Scalar) \
  virtual Eigen::Transform<Scalar, 3, Eigen::Isometry> jointTransform(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> > &q) const = 0; \
  virtual void motionSubspace(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> > &q, Eigen::Matrix<Scalar, TWIST_SIZE, Eigen::Dynamic, 0, TWIST_SIZE, MAX_NUM_VELOCITIES> &motion_subspace, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> *dmotion_subspace = nullptr) const = 0; \
  virtual void motionSubspaceDotTimesV(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>> &q, const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>> &v, Eigen::Matrix<Scalar, 6, 1> &motion_subspace_dot_times_v, Gradient<Eigen::Matrix<Scalar, 6, 1>, Eigen::Dynamic>::type *dmotion_subspace_dot_times_vdq = nullptr, Gradient<Eigen::Matrix<Scalar, 6, 1>, Eigen::Dynamic>::type *dmotion_subspace_dot_times_vdv = nullptr) const = 0; \
  virtual void qdot2v(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> > &q, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, MAX_NUM_VELOCITIES, MAX_NUM_POSITIONS> &qdot_to_v, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> *dqdot_to_v) const = 0; \
  virtual void v2qdot(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>> &q, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, MAX_NUM_POSITIONS, MAX_NUM_VELOCITIES> &v_to_qdot, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> *dv_to_qdot) const = 0; \
  virtual GradientVar<Scalar, Eigen::Dynamic, 1> frictionTorque(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& v, int gradient_order) const = 0;

class RigidBody;

class DLLEXPORT_DRAKEJOINT DrakeJoint
{
  // disable copy construction and assignment
  //DrakeJoint(const DrakeJoint&) = delete;
  //DrakeJoint& operator=(const DrakeJoint&) = delete;

public:
  enum FloatingBaseType {
    FIXED        = 0,
    ROLLPITCHYAW = 1,
    QUATERNION   = 2
  };
  static const int MAX_NUM_POSITIONS = 7;
  static const int MAX_NUM_VELOCITIES = 6;

private:
  const Eigen::Isometry3d transform_to_parent_body;
  const int num_positions;
  const int num_velocities;

protected:
  const std::string name;
  Eigen::VectorXd joint_limit_min;
  Eigen::VectorXd joint_limit_max;

public:
  DrakeJoint(const std::string& name, const Eigen::Isometry3d& transform_to_parent_body, int num_positions, int num_velocities);

  virtual ~DrakeJoint();

  const Eigen::Isometry3d& getTransformToParentBody() const;

  const int getNumPositions() const;

  const int getNumVelocities() const;

  const std::string& getName() const;

  virtual std::string getPositionName(int index) const = 0;

  virtual std::string getVelocityName(int index) const { return getPositionName(index)+"dot"; }

  virtual bool isFloating() const { return false; }

  virtual Eigen::VectorXd randomConfiguration(std::default_random_engine& generator) const = 0;

  virtual const Eigen::VectorXd& getJointLimitMin() const;

  virtual const Eigen::VectorXd& getJointLimitMax() const;

  POSITION_AND_VELOCITY_DEPENDENT_METHODS(double)

  POSITION_AND_VELOCITY_DEPENDENT_METHODS(Eigen::AutoDiffScalar<Eigen::VectorXd>)

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif /* DRAKEJOINT_H_ */
