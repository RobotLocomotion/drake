#pragma once

#include "drake/common/drake_copyable.h"

namespace drake {
namespace planning {

/** The measure of the distance of the edge from q1 to q2 and the portion of
 that is collision free.

 Distance is that produced by CollisionChecker::ComputeConfigurationDistance()
 for the entire edge between q1 and q2.

 The portion of the edge between q1 and q2 that is collision free is encoded as
 the value α with the following semantics:

 - α = 1:
     No collisions were detected. The full edge can be considered collision
     free. This is the *only* time completely_free() reports `true`.
 - 0 ≤ α < 1:
     A collision was detected between q1 and q2. α is the *largest*
     interpolation value such that an edge from q1 to qα can be considered
     collision free (where qα = interpolate(q1, q2, α)). partially_free()
     reports `true`.
 - α is undefined:
     q1 was found to be in collision. That means there exists no α for which the
     edge (q1, qα) can be collision free.

 @note The length of the collision-free edge can be computed via distance * α.
 To simplify comparisons between a number of edges, some of which may not have a
 defined α, the function alpha_or(default_value) is provided. This is equivalent
 to `edge.partially_free() ? edge.alpha() : default_value`.

 @note For α to be meaningful, the caller is obliged to make sure that they
 use the same interpolating function as the CollisionChecker did when generating
 the measure. Calling CollisionChecker::InterpolateBetweenConfigurations() on
 the same checker instance would satisfy that requirement.

 @ingroup planning_collision_checker */
class EdgeMeasure {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(EdgeMeasure);

  /** @pre `0 ≤ distance`
   @pre `0 ≤ alpha ≤ 1` to indicate defined `alpha`, negative otherwise. */
  EdgeMeasure(double distance, double alpha)
      : distance_(distance), alpha_(alpha < 0 ? -1 : alpha) {
    DRAKE_THROW_UNLESS(distance >= 0.0);
    DRAKE_THROW_UNLESS(alpha <= 1.0);
  }

  /** Reports `true` if all samples were collision free. */
  bool completely_free() const { return alpha_ == 1.0; }

  /** Reports `true` if there's *any* portion of the edge (starting from q1)
   that is collision free. By implication, if completely_free() reports
   `true`, so will this. */
  bool partially_free() const { return alpha_ >= 0.0; }

  /** Returns the edge distance. */
  double distance() const { return distance_; }

  /** Returns the value of alpha, if defined.

   Note: Due to the sampling nature of the edge check, the edge (q1, qα) may not
   actually be collision free (due to a missed collision). There's a further
   subtlety. Subsequently calling CheckEdgeCollisionFree(q1, qα) may return
   `false`. This apparent contradiction is due to the fact that the samples
   on the edge (q1, qα) will not necessarily be the same as the samples
   originally tested on the edge (q1, q2). It is possible for those new samples
   to detect a previously missed collision. This is not a bug, merely a property
   of sampling-based testing.

   @pre partially_free() returns `true`. */
  double alpha() const {
    DRAKE_THROW_UNLESS(partially_free());
    return alpha_;
  }

  /** Returns the value of alpha, if defined, or the provided default value. */
  double alpha_or(double default_value) const {
    if (partially_free()) {
      return alpha_;
    } else {
      return default_value;
    }
  }

 private:
  double distance_{};
  double alpha_{};  // -1 implies invalid; caller is not obliged to know this.
};

}  // namespace planning
}  // namespace drake
