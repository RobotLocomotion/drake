#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/planning/robot_collision_type.h"

namespace drake {
namespace planning {

/** A summary of the clearance -- a collection of distance measurements --
 between the robot and everything in the world. This data can be used to define
 collision avoidance strategies.

 Conceptually, this class represents a table:

     | body index R | body index O | type | ϕᴼ(R) | Jq_ϕᴼ(R) |
     | :----------: | :----------: | :--: | :---: | :------: |
     |      ...     |      ...     |  ..  |  ...  |    ...   |

 Member functions return each column of the table as an ordered collection. The
 iᵗʰ entry in each collection belongs to the iᵗʰ row.

 Each row characterizes the relationship between a particular *robot* body
 (annotated as body R) and some "other" body (annotated as body O) in the model.
 That other body O may be part of the robot or the environment.

   - `body index R` is the BodyIndex of body R.
   - `body index O` is the BodyIndex of body O.
   - `type` implies the type of body O. Given that we know body R is a robot
     body, `type` indicates that body O is also a robot body with the value
     RobotCollisionType::kSelfCollision or an environment body with the value
     RobotCollisionType::kEnvironmentCollision. For a correct implementation of
     CollisionChecker, it will never be
     RobotCollisionType::kEnvironmentAndSelfCollision.
   - `ϕᴼ(R)` is the signed distance function of the other body O evaluated on
     body R. The reported distance is offset by the padding value for the body
     pair recorded in the CollisionChecker. It is the minimum padded distance
     between bodies O and R. A point on the padded surface of body O would
     report a distance of zero. Points inside that boundary report negative
     distance and points outside have positive distance.
   - `Jqᵣ_ϕᴼ(R)` is the Jacobian of ϕᴼ(R) with respect to the robot
     configuration vector `qᵣ`. The Jacobian is the derivative as observed in
     the world frame.
     - The vector `qᵣ` will be a subset of the plant's full configuration `q`
       when there are floating bodies or joints in the plant other than the
       robot. The Jacobian is only taken with respect to the robot.
     - The `jacobians()` matrix has `plant.num_positions()` columns and the
       column order matches up with the full `plant.GetPositions()` order.
       The columns associated with non-robot joints will be zero.

  Several important notes:

    - A single robot body index can appear many times as there may be many
      measured distances between that robot body and other bodies in the model.
    - A row *may* contain a zero-valued Jacobian Jqᵣ_ϕᴼ(R). Appropriately
      filtering collisions will cull most of these Jacobians. But depending on
      the structure of the robot and its representative collision geometry, it
      is still possible to evaluate the Jacobian at a configuration that
      represents a local optimum (zero-valued Jacobian).

 @ingroup planning_collision_checker */
class RobotClearance {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RobotClearance);

  /** Creates an empty clearance with size() == 0 and num_positions as given. */
  explicit RobotClearance(int num_positions) : nq_(num_positions) {
    DRAKE_THROW_UNLESS(num_positions >= 0);
  }

  ~RobotClearance();

  /** @returns the number of distance measurements (rows in the table). */
  int size() const { return robot_indices_.size(); }

  /** @returns the number of positions (i.e., columns) in jacobians(). */
  int num_positions() const { return nq_; }

  // TODO(sean.curtis) Provide a guaranteed order on the rows, based on body
  // index.
  /** @returns the vector of *robot* body indices. */
  const std::vector<multibody::BodyIndex>& robot_indices() const {
    return robot_indices_;
  }

  /** @returns the vector of *other* body indices. */
  const std::vector<multibody::BodyIndex>& other_indices() const {
    return other_indices_;
  }

  /** @returns the vector of body collision types. */
  const std::vector<RobotCollisionType>& collision_types() const {
    return collision_types_;
  }

  /** @returns the vector of distances (`ϕᴼ(R)`). */
  Eigen::Map<const Eigen::VectorXd> distances() const {
    return Eigen::Map<const Eigen::VectorXd>(distances_.data(), size());
  }

  /** @returns the vector of distance Jacobians (`Jqᵣ_ϕᴼ(R)`); the return type
  is a readonly Eigen::Map with size() rows and num_positions() columns. */
  auto jacobians() const {
    using RowMatrixXd =
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    return Eigen::Map<const RowMatrixXd>(jacobians_.data(), size(), nq_);
  }

  /** (Advanced) The mutable flavor of jacobians(). */
  auto mutable_jacobians() {
    using RowMatrixXd =
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    return Eigen::Map<RowMatrixXd>(jacobians_.data(), size(), nq_);
  }

  /** Ensures this object has storage for at least `size` rows. */
  void Reserve(int size);

  /** Appends one measurement to this table. */
  void Append(multibody::BodyIndex robot_index,
              multibody::BodyIndex other_index,
              RobotCollisionType collision_type, double distance,
              const Eigen::Ref<const Eigen::RowVectorXd>& jacobian);

 private:
  std::vector<multibody::BodyIndex> robot_indices_;
  std::vector<multibody::BodyIndex> other_indices_;
  std::vector<RobotCollisionType> collision_types_;
  std::vector<double> distances_;
  std::vector<double> jacobians_;  // Stored in row-major order.
  int nq_{};
};

}  // namespace planning
}  // namespace drake
