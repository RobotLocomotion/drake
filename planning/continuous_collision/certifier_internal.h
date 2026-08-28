#pragma once

// Internal driver of the adaptive interval certifier and of the independent
// certificate replay. Nothing here is part of the public API; it exists so
// that continuous_collision_checker.cc, certificate.cc and
// certifier_internal.cc can share one set of per-call data structures.

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/query_object.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/planning/collision_checker_context.h"
#include "drake/planning/continuous_collision/certificate.h"
#include "drake/planning/continuous_collision/distance_oracle.h"
#include "drake/planning/continuous_collision/motion_bound_table.h"
#include "drake/planning/continuous_collision/options.h"
#include "drake/planning/continuous_collision/piecewise_bezier_path.h"
#include "drake/planning/robot_diagram.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace internal {

/* One thread's view of the model: a CollisionCheckerContext (which owns the
root diagram context and the plant and scene-graph sub-contexts pulled out of
it once), plus the two model queries the node loop makes of it. */
class ThreadContext {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ThreadContext);

  /* Allocates a root context of `model`. `model` is aliased and must outlive
  this object. */
  explicit ThreadContext(const RobotDiagram<double>& model)
      : model_(&model), context_(&model) {}

  /* The one FK trigger per node: sets the plant's generalized positions.
  Drake caches forward kinematics per context afterwards, so body poses and
  the query object are pulled lazily and only for the bodies/pairs that are
  still active. */
  void SetPositions(const Eigen::VectorXd& q);

  /* The scene graph's query object at the configuration last set. */
  const geometry::QueryObject<double>& query_object() const {
    return context_.GetQueryObject();
  }

  /* World pose of `body` at the configuration last set (Drake's cache
  computes it on first use and reuses it afterwards). */
  const math::RigidTransform<double>& EvalBodyPose(
      multibody::BodyIndex body) const;

 private:
  const RobotDiagram<double>* model_{};
  CollisionCheckerContext context_;
};

/* A checkout pool of ThreadContexts; construction allocates
`parallelism.num_threads()` RobotDiagram contexts.

The pool is a *checkout* pool rather than a thread-indexed array so that the
public Check* methods stay safe to call concurrently from several threads:
each call leases the contexts it needs for its duration and no two workers can
ever share one. A lease larger than the pre-warmed pool grows it (a cold-path
allocation); nothing shrinks it. */
class ContextPool {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContextPool);

  /* Pre-warms `initial_size` contexts of `model`, which is aliased and must
  outlive this pool. */
  ContextPool(const RobotDiagram<double>& model, int initial_size);

  /* RAII handle for a set of leased contexts. */
  class Lease {
   public:
    Lease(const Lease&) = delete;
    Lease& operator=(const Lease&) = delete;
    /* Hand-written rather than defaulted: the moved-from lease must stop
     owning its slots, and move-assignment must return the slots it already
     holds, or those pool entries stay marked in-use forever. */
    Lease(Lease&& other) noexcept { *this = std::move(other); }
    Lease& operator=(Lease&& other) noexcept;
    ~Lease();

    ThreadContext& operator[](int i) const { return *contexts_[i]; }

   private:
    friend class ContextPool;
    Lease(const ContextPool* pool, std::vector<ThreadContext*> contexts,
          std::vector<int> slots)
        : pool_(pool),
          contexts_(std::move(contexts)),
          slots_(std::move(slots)) {}

    const ContextPool* pool_{};
    std::vector<ThreadContext*> contexts_;
    std::vector<int> slots_;
  };

  /* Leases exactly `count` contexts, growing the pool if it is exhausted. */
  Lease Acquire(int count) const;

 private:
  void Release(const std::vector<int>& slots) const;

  const RobotDiagram<double>* model_{};
  mutable std::mutex mutex_;
  /* A deque so that growing never invalidates the ThreadContext addresses
   already handed out. */
  mutable std::deque<std::unique_ptr<ThreadContext>> slots_;
  mutable std::vector<bool> in_use_;
};

/* Per-pair broadphase data for the free-sphere prefilter: geometry bounding
spheres in their body frames, indexed by dense slots so the node loop can cache
one world-frame center per geometry per node. */
struct PrefilterTable {
  struct Geometry {
    multibody::BodyIndex body;
    Eigen::Vector3d center_L{Eigen::Vector3d::Zero()};
    double radius{0.0};
  };
  /* Dense geometry slots; only geometries that *have* a bounding sphere
  appear (HalfSpace has none). */
  std::vector<Geometry> geometries;
  /* Per pair: slot of geometry a / b, or -1 when that geometry has no sphere
  (a HalfSpace), in which case the pair skips the prefilter and goes straight
  to the (cheap, analytic) oracle route. */
  std::vector<int> slot_a;
  std::vector<int> slot_b;
};

/* Everything one certification run needs; assembled by the facade. All
pointers are aliased and must outlive the call. */
struct CertifierInput {
  const RobotDiagram<double>* model{};
  const DistanceOracle* oracle{};
  const MotionBoundTable* table{};
  const PiecewiseBezierPath* path{};
  /* Pair records with `threshold` = margin + padding resolved for this call.
  Indexed consistently with `table`, `tau` and `prefilter`. */
  const std::vector<PairRecord>* pairs{};
  /* Per-pair oracle tolerance τ_p; see the accuracy table in
  continuous_collision_checker.cc. */
  const std::vector<double>* tau{};
  const PrefilterTable* prefilter{};
  Options options;
};

/* Result of one run, converted to a CertificationResult by the facade. */
struct CertifierOutput {
  Verdict verdict{Verdict::kCertifiedFree};
  /* Earliest-first, capped at Options::max_reported_findings. */
  std::vector<Finding> findings;
  Statistics stats;
  /* Filled iff Options::emit_certificate; records are sorted by
  (segment, s_start, pair_index) so a run is comparable across thread counts
  and across time reparametrizations.

  Only a run that ends Verdict::kCertifiedFree produces a *complete* audit
  trail, i.e. one whose certified intervals cover the whole domain for every
  pair, which is what ReplayCertificate() demands. A run that found a
  violation, hit the resolution floor, exhausted its budget, or pruned the
  search (kFindFirstViolation) leaves the uncertified parts uncovered by
  construction; its records are still individually valid, but they do not
  amount to a proof and ReplayCertificate() will say so. */
  Certificate certificate;
};

/* Runs the breakpoint pre-pass, the static-pair resolution and the adaptive
node recursion over every segment of `input.path`, serially or in parallel
according to `input.options.parallelism`. `pool` supplies the per-thread
contexts; helper threads, if any are hired, are created and joined within this
call.

Parallel driver. Static seeding, i.e. cutting a fixed set of node roots up
front, does not work here: the trees are unbalanced, because a grazing
trajectory concentrates its subdivision in a band a few 10⁻³ wide in segment
parameter, so whatever fixed set of seeds is cut, one of them holds nearly the
whole tree. Three policies replace it. The only shared state is the per-thread
contexts, one atomic earliest-violation bound, and a findings sink under a
mutex.

Sharing is occupancy-driven, not depth-driven. There is one shared LIFO work
source; a worker that has just split a node pushes its *right* child there when
the queue is shorter than the number of live workers, and otherwise keeps both
children. A saturated queue therefore costs nothing, and sharing does not stop
at any depth: a worker on the last deep subtree with every other worker idle
hands out a node per level until the tail is spread. Giving away the right
child keeps each worker's own descent left-first, which is what makes
kFindFirstViolation's bound tighten early.

Recruitment is lazy. The call starts as a serial descent on the calling thread
with sharing disabled and hires helpers only after visiting
`kNodesBeforeHiringHelpers` nodes, so a check whose whole workload is smaller
than that runs at exactly serial speed whatever `Options::parallelism` says.
That matters because `Parallelism::Max()` is the default. Helpers are
call-scoped threads; nothing owns a background thread between calls.

Determinism survives sharing, because moving nodes between workers does not
change which nodes exist. Every node's decisions depend only on its own control
points and its inherited active set, so the tree, the statistics summed over
workers, and the findings are identical serially and at any thread count in
kCertifyAll. In kFindFirstViolation the *reported witness* is identical too,
because the bound only ever prunes nodes that start at or after a witness
already found; the statistics are not. Two exceptions: a run that exhausts
`max_nodes` truncates at a thread-count dependent place, and on a degenerate
segment with t_start == t_end every node maps to the same time, so the bound
prunes on a tie and the reported configuration (not its time) may differ.

@throws std::exception if the oracle throws for any pair; a parallel run waits
for every worker first and rethrows the first failure. */
CertifierOutput RunCertifier(const CertifierInput& input, ContextPool* pool);

// ---------------------------------------------------------------------------
// Certificate assembly + independent replay (implemented in certificate.cc).
// ---------------------------------------------------------------------------

/* Restricts the Bézier control points `cps` (n × (m+1)) of a segment to the
sub-interval [a, b] ⊆ [0, 1] by two de Casteljau subdivisions, writing the
n × (m+1) control points of the restricted curve into `out`.

This is a local, cold-path implementation used only by the certificate replay:
the replay must not reuse the certifier's own subdivision code path if it is to
be an independent check. */
void RestrictBezier(const Eigen::MatrixXd& cps, double a, double b,
                    Eigen::MatrixXd* out);

/* Evaluates the Bézier curve with control points `cps` at u ∈ [0, 1] by de
Casteljau (the apex of the triangle). Cold path. */
Eigen::VectorXd EvaluateBezier(const Eigen::MatrixXd& cps, double u);

/* Inputs of the independent certificate replay. All pointers are aliased. */
struct ReplayInput {
  const RobotDiagram<double>* model{};
  const DistanceOracle* oracle{};
  const MotionBoundTable* table{};
  const PiecewiseBezierPath* path{};
  const std::vector<PairRecord>* pairs{};
  const std::vector<double>* tau{};
  double slack{1e-9};
};

/* Independently re-evaluates every record of `certificate` and checks that
the certified intervals cover the whole domain for every pair. Returns true iff
the certificate is a complete, self-consistent proof that every pair stays
above its recorded threshold everywhere on the path. When it returns false and
`message` is non-null, `*message` explains why. */
bool ReplayCertificate(const ReplayInput& input, const Certificate& certificate,
                       std::string* message);

}  // namespace internal
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
