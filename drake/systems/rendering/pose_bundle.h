#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {
namespace rendering {

// TODO(david-german-tri, SeanCurtis-TRI): Subsume this functionality into
// GeometryWorld/GeometrySystem when they become available. In particular,
// once GeometryWorld provides an id<->semantics mapping, PoseBundle will
// no longer need to maintain names.

// TODO(david-german-tri): In the meantime, provide better lookup facilities
// here.  "Names" as currently construed are unlikely to be useful to
// consumers of aggregated poses.

/// PoseBundle is a container for a set of poses, represented by an Isometry3.
/// The poses are expressed in the world frame: X_WFi. Each pose has a name,
/// and a model instance ID.  If two poses in the bundle have the same model
/// instance ID, they must not have the same name.
///
/// This class is explicitly instantiated for the following scalar types. No
/// other scalar types are supported.
/// - double
/// - AutoDiffXd
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///           Only double and AutoDiffXd are supported.
template<typename T>
class PoseBundle {
 public:
  explicit PoseBundle(int num_poses);
  ~PoseBundle();

  int get_num_poses() const;
  const Isometry3<T>& get_pose(int index) const;
  void set_pose(int index, const Isometry3<T>& pose);

  const std::string& get_name(int index) const;
  void set_name(int index, const std::string& name);

  int get_model_instance_id(int index) const;
  void set_model_instance_id(int index, int id);

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PoseBundle)

 private:
  std::vector<Isometry3<T>> poses_;
  std::vector<std::string> names_;
  std::vector<int> ids_;
};

}  // namespace rendering
}  // namespace systems
}  // namespace drake
