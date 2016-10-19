%module(package="pydrake") rbtree

%include "exception_helper.i"
%include <std_string.i>
%include <windows.i>
#define DRAKE_EXPORT

%{
#ifdef SWIGPYTHON
  #define SWIG_FILE_WITH_INIT
  #include <Python.h>
#endif
#include "drake/systems/plants/RigidBodyTree.h"
%}

%include <typemaps.i>
%include <std_vector.i>
%include <std_map.i>

#define SWIG_SHARED_PTR_NAMESPACE std
// SWIG has built-in support for shared pointers, and can use either
// std::shared_ptr or boost::shared_ptr, since they provide similar enough
// interfaces. Even though the interface file is called 'boost_shared_ptr.i',
// the above #define tells SWIG to use the std:: implementation instead. Note
// that this does NOT result in any boost headers being included.
%include <boost_shared_ptr.i>

%include <eigen.i>
%include <autodiff.i>
%import <autodiffutils.i>

%template(vectorVectorXd) std::vector<Eigen::VectorXd>;
%template(vectorMatrixXd) std::vector<Eigen::MatrixXd>;
%template(vectorString) std::vector<std::string>;
%template(vectorInt) std::vector<int>;
%template(vectorFloat) std::vector<float>;
%template(vectorDouble) std::vector<double>;
%template(mapStringString) std::map<std::string,std::string>;
%shared_ptr(RigidBody)
%template(vectorRigidBody) std::vector<std::shared_ptr<RigidBody> >;
%shared_ptr(RigidBodyFrame)

%eigen_typemaps(Eigen::VectorXd)
%eigen_typemaps(Eigen::Vector2d)
%eigen_typemaps(Eigen::Vector3d)
%eigen_typemaps(Eigen::Vector4d)
%eigen_typemaps(Eigen::Matrix4d)
%eigen_typemaps(Eigen::Matrix<double, drake::kSpaceDimension, 1>)
%eigen_typemaps(Eigen::Matrix3Xd)
%eigen_typemaps(Eigen::Matrix<double, drake::kSpaceDimension, Eigen::Dynamic>)
%eigen_typemaps(Eigen::MatrixXd)
%eigen_typemaps(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>)
%eigen_typemaps(Eigen::VectorXi)

%include "drake/systems/plants/KinematicsCache.h"
%template(KinematicsCache_d) KinematicsCache<double>;
%template(KinematicsCache_adVectorDynamic) KinematicsCache<Eigen::AutoDiffScalar<Eigen::VectorXd> >;
%template(KinematicsCache_adVectorMax73) KinematicsCache<Eigen::AutoDiffScalar<Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 73> > >;
%template(AutoDiff3XDynamic) AutoDiffWrapper<Eigen::VectorXd, drake::kSpaceDimension, Eigen::Dynamic>;
%template(AutoDiff3XMax73) AutoDiffWrapper<Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 73>, drake::kSpaceDimension, Eigen::Dynamic>;

// unique_ptr confuses SWIG, so we'll ignore it for now
%ignore RigidBody::setJoint(std::unique_ptr<DrakeJoint> joint);
%include "drake/systems/plants/RigidBody.h"

%include "drake/systems/plants/RigidBodyFrame.h"

%immutable RigidBodyTree::actuators;
%immutable RigidBodyTree::loops;

// unique_ptr confuses SWIG, so we'll ignore it for now
%ignore RigidBodyTree::add_rigid_body(std::unique_ptr<RigidBody> body);

// Ignore this member so that it doesn't generate setters/getters.
// These cause problems since bodies is a vector of unique_ptr's and
// SWIG doesn't support them.
%ignore RigidBodyTree::bodies;
%include "drake/systems/plants/RigidBodyTree.h"
%include "drake/systems/plants/joints/floating_base_types.h"
%extend RigidBodyTree {
  RigidBodyTree(const std::string& urdf_filename, const std::string& joint_type) {
    // FIXED = 0, ROLLPITCHYAW = 1, QUATERNION = 2
    drake::systems::plants::joints::FloatingBaseType floating_base_type;

    if (joint_type == "FIXED")
      floating_base_type = drake::systems::plants::joints::kFixed;
    else if (joint_type == "ROLLPITCHYAW")
      floating_base_type = drake::systems::plants::joints::kRollPitchYaw;
    else if (joint_type == "QUATERNION")
      floating_base_type = drake::systems::plants::joints::kQuaternion;
    else {
      std::cerr << "Joint Type not supported" << std::endl;
      return nullptr;
    }

    return new RigidBodyTree(urdf_filename, floating_base_type);
  }

  KinematicsCache<double> doKinematics(
    const Eigen::MatrixBase<Eigen::VectorXd>& q,
    const Eigen::MatrixBase<Eigen::VectorXd>& v) {
    return $self->doKinematics(q, v);
  }

  KinematicsCache<Eigen::AutoDiffScalar<Eigen::VectorXd> > doKinematics(const AutoDiffWrapper<Eigen::VectorXd, Eigen::Dynamic, 1>& q, const AutoDiffWrapper<Eigen::VectorXd, Eigen::Dynamic, 1>& v) {
    return $self->doKinematics(q, v);
  }

  KinematicsCache<Eigen::AutoDiffScalar<Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 73> > > doKinematics(const AutoDiffWrapper<Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 73>, Eigen::Dynamic, 1>& q, const AutoDiffWrapper<Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 73>, Eigen::Dynamic, 1>& v) {
    return $self->doKinematics(q, v);
  }

  Eigen::Matrix4d relativeTransform(
      const KinematicsCache<double>& cache, int base_or_frame_ind, int body_or_frame_ind) const
  {
    return $self->relativeTransform(cache, base_or_frame_ind, body_or_frame_ind).matrix();
  }

  Eigen::Matrix3Xd getTerrainContactPoints(
      const RigidBody& body,
      const std::string& group_name = "") const {
    Eigen::Matrix3Xd pts;
    $self->getTerrainContactPoints(body, &pts, group_name);
    return pts;
  }

  Eigen::Matrix<double, drake::kSpaceDimension, Eigen::Dynamic> transformPoints(
      const KinematicsCache<double> &cache, const Eigen::Matrix<double, drake::kSpaceDimension, Eigen::Dynamic> &points, int current_body_or_frame_ind, int new_body_or_frame_ind) const
  {
    return $self->transformPoints(cache, points, current_body_or_frame_ind, new_body_or_frame_ind);
  }

  AutoDiffWrapper<Eigen::VectorXd, drake::kSpaceDimension, Eigen::Dynamic> transformPoints(
      const KinematicsCache<Eigen::AutoDiffScalar<Eigen::VectorXd> > &cache, const Eigen::Matrix<double, drake::kSpaceDimension, Eigen::Dynamic> &points, int current_body_or_frame_ind, int new_body_or_frame_ind) const
  {
    return $self->transformPoints(cache, points, current_body_or_frame_ind, new_body_or_frame_ind);
  }

  AutoDiffWrapper<Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 73>, drake::kSpaceDimension, Eigen::Dynamic> transformPoints(
      const KinematicsCache<Eigen::AutoDiffScalar<Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 73> > > &cache, const Eigen::Matrix<double, drake::kSpaceDimension, Eigen::Dynamic> &points, int current_body_or_frame_ind, int new_body_or_frame_ind) const
  {
    return $self->transformPoints(cache, points, current_body_or_frame_ind, new_body_or_frame_ind);
  }

  Eigen::Matrix<double, drake::kSpaceDimension, 1> centerOfMass(KinematicsCache<double> &cache, const std::set<int> &model_instance_id = default_model_instance_id_set) const {
    return $self->centerOfMass(cache, model_instance_id);
  }

  Eigen::Matrix<double, drake::kSpaceDimension, Eigen::Dynamic> centerOfMassJacobian(KinematicsCache<double>& cache, const std::set<int>& model_instance_ids = default_model_instance_id_set, bool in_terms_of_qdot = false) const {
    return $self->centerOfMassJacobian(cache, model_instance_ids, in_terms_of_qdot);
  }

  Eigen::VectorXd getRandomConfiguration() const {
    std::default_random_engine generator(std::random_device{}());
    return $self->getRandomConfiguration(generator);
  }
}
