#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/parallelism.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/planning/continuous_collision/certifier_internal.h"
#include "drake/planning/continuous_collision/numerics.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace internal {
namespace {

using drake::geometry::QueryObject;
using drake::math::RigidTransformd;
using drake::multibody::BodyIndex;

constexpr double kInfinity = std::numeric_limits<double>::infinity();

/* Global (trajectory) time of parameter s in `seg`. Segment times are pure
 bookkeeping: the recursion runs in the segment parameter s ∈ [0, 1] and only
 the *reported* times go through this map, which is why the certificate is
 invariant under time reparametrization (the soundness argument, T6). */
double TimeOf(const BezierSegment& seg, double s) {
  return seg.t_start + s * (seg.t_end - seg.t_start);
}

// ---------------------------------------------------------------------------
// Per-node world-frame geometry sphere centers.
// ---------------------------------------------------------------------------

/* Caches one world-frame bounding-sphere center per geometry per node
 (requirement P2: the poses behind them are pulled lazily from Drake's FK cache
 and only for geometries of still-active pairs). Invalidation is a stamp bump,
 so switching to a new configuration is O(1). */
class GeometryCache {
 public:
  GeometryCache(const PrefilterTable& table, const ThreadContext& context)
      : table_(&table),
        context_(&context),
        center_W_(table.geometries.size()),
        stamp_of_(table.geometries.size(), 0) {}

  /* Invalidates every cached center; call once per configuration. */
  void NewConfiguration() { ++stamp_; }

  const Eigen::Vector3d& Center(int slot) {
    if (stamp_of_[slot] != stamp_) {
      const PrefilterTable::Geometry& g = table_->geometries[slot];
      center_W_[slot] = context_->EvalBodyPose(g.body) * g.center_L;
      stamp_of_[slot] = stamp_;
    }
    return center_W_[slot];
  }

  double radius(int slot) const { return table_->geometries[slot].radius; }

 private:
  const PrefilterTable* table_{};
  const ThreadContext* context_{};
  std::vector<Eigen::Vector3d> center_W_;
  std::vector<std::uint64_t> stamp_of_;
  std::uint64_t stamp_{1};
};

// ---------------------------------------------------------------------------
// Findings sink.
// ---------------------------------------------------------------------------

/* Collects findings from every worker. Cold path: guarded by one mutex.
 Each list keeps only the `cap` earliest entries, so memory stays bounded no
 matter how many violating nodes a pathological trajectory produces, while the
 "earliest-first" contract of CertificationResult::findings is preserved
 exactly (dropping the *latest* entry can never remove an earlier one). */
class FindingSink {
 public:
  explicit FindingSink(int cap) : cap_(std::max(1, cap)) {}

  void AddDefinite(Finding finding) {
    const double time = finding.time;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      Insert(&definite_, std::move(finding));
    }
    // Branch-and-bound bound for kFindFirstViolation (the search algorithm;
    // parallelism and determinism): workers skip nodes whose interval starts at
    // or after the earliest witness known so far. The bound decreases
    // monotonically, so a node that could hold an earlier witness is never
    // pruned and the answer does not depend on timing.
    double previous = best_violation_time_.load(std::memory_order_relaxed);
    while (time < previous && !best_violation_time_.compare_exchange_weak(
                                  previous, time, std::memory_order_relaxed)) {
    }
  }

  void AddInconclusive(Finding finding) {
    std::lock_guard<std::mutex> guard(mutex_);
    Insert(&inconclusive_, std::move(finding));
  }

  double best_violation_time() const {
    return best_violation_time_.load(std::memory_order_relaxed);
  }

  /* Reports a node that was left unexplored when the node budget ran out; the
   earliest such node over all workers is what the run reports (the search
   algorithm: the budget "truncates in parameter order and reports the
   remainder"). */
  void ReportPending(double time, const Eigen::VectorXd& q, int pair_index) {
    std::lock_guard<std::mutex> guard(mutex_);
    if (!pending_valid_ || time < pending_time_) {
      pending_valid_ = true;
      pending_time_ = time;
      pending_q_ = q;
      pending_pair_ = pair_index;
    }
  }

  bool pending_valid() const { return pending_valid_; }
  double pending_time() const { return pending_time_; }
  const Eigen::VectorXd& pending_q() const { return pending_q_; }
  int pending_pair() const { return pending_pair_; }

  const std::vector<Finding>& definite() const { return definite_; }
  const std::vector<Finding>& inconclusive() const { return inconclusive_; }

 private:
  void Insert(std::vector<Finding>* list, Finding&& finding) {
    if (static_cast<int>(list->size()) >= cap_ &&
        finding.time >= list->back().time) {
      return;
    }
    const auto position =
        std::upper_bound(list->begin(), list->end(), finding.time,
                         [](double time, const Finding& other) {
                           return time < other.time;
                         });
    list->insert(position, std::move(finding));
    if (static_cast<int>(list->size()) > cap_) list->pop_back();
  }

  const int cap_;
  std::mutex mutex_;
  std::vector<Finding> definite_;
  std::vector<Finding> inconclusive_;
  std::atomic<double> best_violation_time_{kInfinity};
  bool pending_valid_{false};
  double pending_time_{kInfinity};
  Eigen::VectorXd pending_q_;
  int pending_pair_{0};
};

// ---------------------------------------------------------------------------
// Shared work source for the parallel driver (parallelism and determinism).
// ---------------------------------------------------------------------------

/* One unit of shared work: a node, self-contained so a worker can pick it up
 without touching any other worker's arenas.

 Work items carry copies (control points and the active-pair span) rather than
 pointing into the producing worker's arenas, because the producer walks on
 immediately. Requirement P1 (no per-node heap allocation) survives that
 because the queue recycles item *shells*: a popped shell goes back on a free
 list and is handed to the next producer, whose `resize`/`assign` then reuse
 the buffers already attached to it. Allocation happens while the free list is
 filling up and never again. */
struct WorkItem {
  int segment{};
  double s_lo{0.0};
  double s_hi{1.0};
  int depth{0};
  Eigen::MatrixXd control_points;
  std::vector<int> active;
};

/* Mutex-guarded LIFO work source with quiescence detection, shell recycling
 and the occupancy counter that drives the sharing policy. The *only* shared
 mutable state of the parallel driver is this queue, the FindingSink, and the
 atomic node counter / violation bound (parallelism and determinism), which is
 what makes the driver TSan-clean by construction. */
class WorkQueue {
 public:
  /* Moves `*item` into the queue and hands back a recycled shell (or an empty
   one) so the producer can fill it again without allocating. */
  void Push(WorkItem* item) {
    {
      std::lock_guard<std::mutex> guard(mutex_);
      items_.push_back(std::move(*item));
      if (free_.empty()) {
        *item = WorkItem{};
      } else {
        *item = std::move(free_.back());
        free_.pop_back();
      }
      size_.store(static_cast<int>(items_.size()), std::memory_order_relaxed);
    }
    condition_.notify_one();
  }

  /* Blocks until an item is available, or until every worker is idle and the
   queue is empty (returns false), or until Abort() (returns false). `*item`'s
   previous contents are recycled into the free list. */
  bool Pop(WorkItem* item) {
    std::unique_lock<std::mutex> lock(mutex_);
    while (true) {
      if (aborted_ || done_) return false;
      if (!items_.empty()) {
        free_.push_back(std::move(*item));
        *item = std::move(items_.back());
        items_.pop_back();
        size_.store(static_cast<int>(items_.size()), std::memory_order_relaxed);
        ++busy_;
        return true;
      }
      if (busy_ == 0) {
        done_ = true;
        condition_.notify_all();
        return false;
      }
      condition_.wait(lock);
    }
  }

  void FinishItem() {
    {
      std::lock_guard<std::mutex> guard(mutex_);
      --busy_;
      if (busy_ > 0 && items_.empty()) return;
    }
    condition_.notify_all();
  }

  void Abort() {
    {
      std::lock_guard<std::mutex> guard(mutex_);
      aborted_ = true;
    }
    condition_.notify_all();
  }

  /* The sharing policy (see certifier_internal.h): a worker gives one child
   away whenever the queue holds fewer items than there are live workers.
   Reading the length through a relaxed atomic keeps the *test* off the queue's
   mutex, so only an actual share pays for the lock; a stale answer costs at
   most one redundant or one skipped share. `num_workers` is 0 until helpers are
   hired, which is exactly how lazy recruitment disables sharing. */
  bool ShouldShare() const {
    return size_.load(std::memory_order_relaxed) <
           num_workers_.load(std::memory_order_relaxed);
  }

  void set_num_workers(int count) {
    num_workers_.store(count, std::memory_order_relaxed);
  }

  /* Items never picked up; used to report what the node budget left
   uncovered. Call only after every worker has finished. */
  std::vector<WorkItem>& remaining() { return items_; }

 private:
  std::mutex mutex_;
  std::condition_variable condition_;
  std::vector<WorkItem> items_;
  std::vector<WorkItem> free_;
  std::atomic<int> size_{0};
  std::atomic<int> num_workers_{0};
  int busy_{0};
  bool done_{false};
  bool aborted_{false};
};

/* Lazy-recruitment state. Only the lead worker ever touches it, so it needs no
 synchronization of its own: `hire` is called from inside the lead's node loop
 the first time the run has visited enough nodes to be worth spreading. */
struct Recruitment {
  std::uint64_t nodes_before_hire{0};
  std::uint64_t nodes{0};
  std::function<void()> hire;
};

/* How many nodes a run must have visited before it hires helpers.

 This is a measured break-even, not a taste knob. Hiring costs one WorkerPool
 reservation, one ContextPool lease, the construction of the helper Worker
 objects, one notification per helper and — at the end of the run — one wakeup
 per helper before the lead can collect their statistics: about 6-7 us per
 helper, ~65 us for a full fifteen, on the machine the benchmark suite
 was measured on. A node on that machine costs ~7-13 us. Sixteen nodes of work
 already done is therefore roughly a 3x margin over the price of the helpers,
 and it bounds the damage in the one case lazy recruitment cannot avoid — a
 check that ends immediately after hiring — to that same ~65 us (~15% of such a
 check).

 Everything smaller than this runs at exactly serial speed at any
 Options::parallelism, which is the property that matters most in practice
 because Parallelism::Max() is the default value of that field. */
constexpr std::uint64_t kNodesBeforeHiringHelpers = 16;

// ---------------------------------------------------------------------------
// The node loop.
// ---------------------------------------------------------------------------

/* One frame of the explicit LIFO node stack. The frame at stack index k owns
 control-point slab k of the worker's pool, and the pair indices it is still
 active for live in arena[active_offset, active_offset + active_length). */
struct NodeFrame {
  double s_lo{0.0};
  double s_hi{1.0};
  int depth{0};
  int active_offset{0};
  int active_length{0};
};

/* A worker owns all per-thread scratch of the node recursion; nothing in the
 steady-state loop allocates (requirement P1):

  - `slabs_` is the node pool: slab k is the n × (m+1) control-point matrix of
    the frame at stack index k. Splitting a node writes its left child into
    slab k+1 and swaps its right child into slab k (an O(1) Eigen buffer
    swap), so a split costs zero copies and zero allocations. Slabs are
    (re)sized only when the worker moves to a segment of a different Bézier
    order.
  - `arena_` is the survivor arena: each frame's active pair list is an index
    span into it. Both children of a node share one span (their active sets
    are identical), and a popped frame writes its survivors immediately above
    its own span, which is free because every frame still on the stack owns a
    span at or below that point.
  - `stack_` grows by one per level, so all three arrays are bounded by the
    depth the resolution floor allows. */
class Worker {
 public:
  Worker(const CertifierInput& input, ThreadContext* context, FindingSink* sink,
         std::atomic<std::uint64_t>* node_counter, WorkQueue* queue,
         Recruitment* recruit)
      : input_(input),
        context_(context),
        sink_(sink),
        node_counter_(node_counter),
        queue_(queue),
        recruit_(recruit),
        geometry_(*input.prefilter, *context),
        find_first_(input.options.mode == SearchMode::kFindFirstViolation),
        emit_certificate_(input.options.emit_certificate) {
    const int n = input_.path->num_positions();
    q_mid_.resize(n);
    w_.resize(n);
  }

  /* Parallel entry point: pulls items until the work source is quiescent. The
   item buffer is reused across iterations and recycled through the queue's
   free list, so the loop allocates nothing after the first few rounds. */
  void Run() {
    while (queue_->Pop(&item_)) {
      RunItem(&item_);
      queue_->FinishItem();
    }
  }

  /* Serial entry point (and the body of the parallel one). */
  void RunItem(WorkItem* item);

  const Statistics& stats() const { return stats_; }
  std::vector<CertificateRecord>& records() { return records_; }

 private:
  /* Ensures the pool holds `count` slabs of the given shape. Cold path: hit
   once per worker and again whenever the Bézier order changes. */
  void EnsureSlabs(int count, int rows, int cols) {
    if (rows != slab_rows_ || cols != slab_cols_) {
      for (Eigen::MatrixXd& slab : slabs_) slab.resize(rows, cols);
      split_scratch_.resize(rows, cols);
      slab_rows_ = rows;
      slab_cols_ = cols;
    }
    while (static_cast<int>(slabs_.size()) < count) {
      slabs_.emplace_back(rows, cols);
    }
  }

  void EnsureArena(int size) {
    if (static_cast<int>(arena_.size()) < size) {
      arena_.resize(std::max(size, 2 * static_cast<int>(arena_.size()) + 64));
    }
  }

  /* Appends one certification event to the audit trail (the search algorithm).
   */
  void RecordCertification(int segment, double s_lo, double s_hi, int pair,
                           double phi_hat, double motion_bound,
                           double threshold) {
    records_.push_back(CertificateRecord{segment, s_lo, s_hi, pair, q_mid_,
                                         phi_hat, motion_bound, threshold});
  }

  const CertifierInput& input_;
  ThreadContext* context_{};
  FindingSink* sink_{};
  std::atomic<std::uint64_t>* node_counter_{};
  WorkQueue* queue_{};
  /* Non-null only for the lead worker, and only until it has hired. */
  Recruitment* recruit_{};
  GeometryCache geometry_;
  const bool find_first_{false};
  const bool emit_certificate_{false};

  /* Reused buffers for the queue's two directions (see WorkItem). */
  WorkItem item_;
  WorkItem share_;

  std::vector<Eigen::MatrixXd> slabs_;
  Eigen::MatrixXd split_scratch_;
  int slab_rows_{-1};
  int slab_cols_{-1};
  std::vector<NodeFrame> stack_;
  std::vector<int> arena_;
  Eigen::VectorXd q_mid_;
  Eigen::VectorXd w_;
  Eigen::Vector3d nearest_a_;
  Eigen::Vector3d nearest_b_;

  Statistics stats_;
  std::vector<CertificateRecord> records_;
};

void Worker::RunItem(WorkItem* item) {
  const PiecewiseBezierPath& path = *input_.path;
  const BezierSegment& segment = path.segments()[item->segment];
  const MotionBoundTable& table = *input_.table;
  const DistanceOracle& oracle = *input_.oracle;
  const std::vector<PairRecord>& pairs = *input_.pairs;
  const std::vector<double>& tau = *input_.tau;
  const PrefilterTable& prefilter = *input_.prefilter;
  const Options& options = input_.options;
  const double slack = options.certificate_slack;
  const int rows = static_cast<int>(item->control_points.rows());
  const int cols = static_cast<int>(item->control_points.cols());
  const std::uint64_t max_nodes =
      options.max_nodes.value_or(std::numeric_limits<std::uint64_t>::max());

  // Seed the local stack with this work item.
  EnsureSlabs(2, rows, cols);
  slabs_[0] = item->control_points;
  EnsureArena(static_cast<int>(item->active.size()) + 1);
  std::copy(item->active.begin(), item->active.end(), arena_.begin());
  stack_.clear();
  stack_.push_back(NodeFrame{item->s_lo, item->s_hi, item->depth, 0,
                             static_cast<int>(item->active.size())});

  while (!stack_.empty()) {
    const NodeFrame frame = stack_.back();
    stack_.pop_back();
    const int k = static_cast<int>(stack_.size());

    // Branch-and-bound on time (the search algorithm; parallelism and
    // determinism): a node starting at or after the earliest witness known so
    // far cannot contain an earlier one.
    if (find_first_ &&
        TimeOf(segment, frame.s_lo) >= sink_->best_violation_time()) {
      continue;
    }
    if (node_counter_->fetch_add(1, std::memory_order_relaxed) >= max_nodes) {
      // Budget exhausted: stop here and report the earliest node this worker
      // leaves uncovered — which is exactly this one, because a left-first DFS
      // pops in increasing parameter order and every frame still on the stack
      // starts at or after this node's end.
      sink_->ReportPending(
          TimeOf(segment, frame.s_lo), slabs_[k].col(0),
          frame.active_length > 0 ? arena_[frame.active_offset] : 0);
      if (queue_ != nullptr) queue_->Abort();
      stack_.clear();
      return;
    }
    ++stats_.nodes;
    stats_.max_depth = std::max(stats_.max_depth, frame.depth);

    // Lazy recruitment (see certifier_internal.h): the lead worker runs alone
    // until the run has visited enough nodes to pay for helpers, then hires
    // them once and drops the hook. Every other worker carries a null
    // `recruit_`.
    if (recruit_ != nullptr &&
        ++recruit_->nodes >= recruit_->nodes_before_hire) {
      Recruitment* const recruitment = recruit_;
      recruit_ = nullptr;
      recruitment->hire();
    }

    EnsureSlabs(k + 2, rows, cols);
    const Eigen::MatrixXd& control_points = slabs_[k];

    // The split *is* the evaluation: the apex of the de Casteljau triangle at
    // the midpoint is exactly q(s_mid), so the node's representative
    // configuration comes for free (trajectory normalization; the search
    // algorithm).
    DeCasteljauSplitAtHalf(control_points, &slabs_[k + 1], &split_scratch_,
                           &q_mid_);

    // w_i = max_j |P_{j,i} − qc_i|. By the convex-hull property of the
    // Bernstein basis, |q_i(s) − qc_i| ≤ w_i for every s in this node
    // (the interval certificate).
    w_.setZero();
    for (int j = 0; j < cols; ++j) {
      for (int i = 0; i < rows; ++i) {
        w_[i] = std::max(w_[i], std::abs(control_points(i, j) - q_mid_[i]));
      }
    }

    // One FK per node (requirement P2); body poses and the query object are
    // pulled lazily below, and only for pairs that survive that far.
    context_->SetPositions(q_mid_);
    geometry_.NewConfiguration();
    const QueryObject<double>& query_object = context_->query_object();

    const double s_mid = 0.5 * (frame.s_lo + frame.s_hi);
    const double t_mid = TimeOf(segment, s_mid);
    // The resolution floor, plus a hard floating-point backstop: once the
    // midpoint no longer separates the endpoints in double arithmetic the node
    // cannot be split any further, whatever min_interval says. Without it a
    // pathologically small min_interval would spin forever.
    const bool at_floor = (frame.s_hi - frame.s_lo) <= options.min_interval ||
                          !(s_mid > frame.s_lo && s_mid < frame.s_hi);

    const int survivor_offset = frame.active_offset + frame.active_length;
    int survivor_count = 0;
    EnsureArena(survivor_offset + frame.active_length + 1);

    for (int e = frame.active_offset;
         e < frame.active_offset + frame.active_length; ++e) {
      const int p = arena_[e];
      const PairRecord& pair = pairs[p];
      const double threshold = pair.threshold;
      const double tau_p = tau[p];
      // Δ_p(ν) = Σ_{j ∈ J(p)} λ(j,p)·w_j — a sparse dot product over this
      // pair's CSR row (requirement P3).
      const double motion_bound = table.MotionBound(p, w_);

      // --- Early-out 1: the free-sphere prefilter (requirements P4, P5). ---
      // φ_p ≥ ‖c_A − c_B‖ − ρ_A − ρ_B with the bounding spheres posed at qc,
      // so the lower bound stands in for φ̂ in the certificate test below and
      // is sound by the same displacement-lemma argument. It needs no
      // narrowphase and no allocation, only the lazily pulled body poses. It
      // is charged the same τ_p as the oracle even though it is exact given
      // the poses: that costs nothing (τ ~ 1e-6 m against centimetre-scale
      // sphere gaps) and keeps the certificate replay's arithmetic uniform.
      const int slot_a = prefilter.slot_a[p];
      const int slot_b = prefilter.slot_b[p];
      if (slot_a >= 0 && slot_b >= 0) {
        const double lower_bound =
            (geometry_.Center(slot_a) - geometry_.Center(slot_b)).norm() -
            geometry_.radius(slot_a) - geometry_.radius(slot_b);
        if (IsCertified(lower_bound, tau_p, motion_bound, threshold, slack)) {
          ++stats_.sphere_certifications;
          if (emit_certificate_) {
            RecordCertification(item->segment, frame.s_lo, frame.s_hi, p,
                                lower_bound, motion_bound, threshold);
          }
          continue;
        }
      }

      // --- Narrowphase. ----------------------------------------------------
      ++stats_.narrowphase_queries;
      const double phi_hat =
          oracle.SignedDistance(query_object, pair, &nearest_a_, &nearest_b_);

      if (IsDefiniteViolation(phi_hat, tau_p, threshold)) {
        // qc is exactly on the trajectory (it is the de Casteljau apex), so
        // φ_true(qc) ≤ φ̂ + τ_p < m_p is a definite violation of the
        // continuum statement, not a sampling artifact (the problem statement;
        // the interval certificate).
        Finding finding;
        finding.time = t_mid;
        finding.q = q_mid_;
        finding.pair = pair.id;
        finding.distance = phi_hat;
        finding.motion_bound = motion_bound;
        finding.definite = true;
        finding.nearest_a_W = nearest_a_;
        finding.nearest_b_W = nearest_b_;
        sink_->AddDefinite(std::move(finding));
        if (!find_first_ || at_floor) {
          // kCertifyAll (or a floor node, which has no children to refine
          // into): drop p from this subtree. Without this a single
          // violating pair would report one finding per node all the way down
          // to the resolution floor; the earliest-first ordering and the
          // max_reported_findings cap still apply, and every *disjoint*
          // violating region of p is still reported because sibling subtrees
          // carry their own copy of the active set.
          continue;
        }
        // kFindFirstViolation: keep p active so the branch-and-bound recursion
        // can refine the witness toward the earliest violating time.
      } else if (IsCertified(phi_hat, tau_p, motion_bound, threshold, slack)) {
        // Displacement lemma: for every s in this node,
        //   φ_p(q(s)) ≥ φ_true(qc) − Σ_{j∈J(p)} λ(j,p)·|q_j(s) − qc_j|
        //            ≥ (φ̂ − τ_p) − Δ_p(ν) > m_p + ε,
        // using |q_j(s) − qc_j| ≤ w_j from the convex-hull property. The whole
        // closed parameter interval of the node is therefore certified and the
        // pair drops out of the entire subtree — the dominant work saver.
        if (emit_certificate_) {
          RecordCertification(item->segment, frame.s_lo, frame.s_hi, p, phi_hat,
                              motion_bound, threshold);
        }
        continue;
      }

      // --- Gray: subdivide, unless we are already at the resolution floor. -
      if (at_floor) {
        Finding finding;
        finding.time = t_mid;
        finding.q = q_mid_;
        finding.pair = pair.id;
        finding.distance = phi_hat;
        finding.motion_bound = motion_bound;
        finding.definite = false;
        finding.nearest_a_W = nearest_a_;
        finding.nearest_b_W = nearest_b_;
        sink_->AddInconclusive(std::move(finding));
      } else {
        arena_[survivor_offset + survivor_count] = p;
        ++survivor_count;
      }
    }

    if (survivor_count == 0) continue;

    // At this point the split has left the *left* child in slab k+1 and the
    // *right* child in split_scratch_.
    const NodeFrame right{s_mid, frame.s_hi, frame.depth + 1, survivor_offset,
                          survivor_count};
    const NodeFrame left{frame.s_lo, s_mid, frame.depth + 1, survivor_offset,
                         survivor_count};
    if (queue_ != nullptr && queue_->ShouldShare()) {
      // Occupancy-driven sharing (see certifier_internal.h): the shared queue
      // is running dry, so hand the right child over and carry on down the left
      // one. This is the only mechanism that spreads a deep tree, and because
      // it is driven by how hungry the other workers are rather than by depth,
      // it keeps spreading right down to the last subtree — which is exactly
      // what a fixed seeding depth cannot do.
      share_.segment = item->segment;
      share_.s_lo = right.s_lo;
      share_.s_hi = right.s_hi;
      share_.depth = right.depth;
      share_.control_points = split_scratch_;
      share_.active.assign(arena_.begin() + survivor_offset,
                           arena_.begin() + survivor_offset + survivor_count);
      queue_->Push(&share_);
      // The frame at stack index k must own slab k, so move the left child
      // down into it (an O(1) Eigen buffer swap, like the split itself).
      slabs_[k].swap(slabs_[k + 1]);
      stack_.push_back(left);  // slab k holds the left child.
    } else {
      slabs_[k].swap(split_scratch_);  // O(1): slab k = right child.
      // LIFO with the left child on top ⇒ a left-to-right sweep in time, so
      // the serial driver walks the trajectory in order (the search algorithm).
      stack_.push_back(right);  // slab k holds the right child.
      stack_.push_back(left);   // slab k+1 holds the left child.
    }
  }
}

// ---------------------------------------------------------------------------
// Breakpoint pre-pass and static-pair resolution (the search algorithm, steps 1
// and 2).
// ---------------------------------------------------------------------------

/* Evaluates one breakpoint configuration against every pair. Breakpoints are
 the finitely many configurations the midpoint recursion only approaches in the
 limit (t0, every junction, tf), so checking them discretely is what gives
 violations *at* interval endpoints clean semantics.

 When `resolve_static` is true (the t0 breakpoint) the pairs with J(p) = ∅ are
 also resolved here, once and for all: no motion of the trajectory can change
 their relative pose, so their status at q(t0) is their status everywhere. */
void RunBreakpointPass(const CertifierInput& input, ThreadContext* context,
                       GeometryCache* geometry, const Eigen::VectorXd& q,
                       double time, bool resolve_static, FindingSink* sink,
                       Statistics* stats,
                       std::vector<CertificateRecord>* records) {
  const std::vector<PairRecord>& pairs = *input.pairs;
  const std::vector<double>& tau = *input.tau;
  const PrefilterTable& prefilter = *input.prefilter;
  const MotionBoundTable& table = *input.table;
  const double slack = input.options.certificate_slack;
  const int num_segments = static_cast<int>(input.path->segments().size());

  context->SetPositions(q);
  geometry->NewConfiguration();
  const QueryObject<double>& query_object = context->query_object();

  Eigen::Vector3d nearest_a;
  Eigen::Vector3d nearest_b;
  for (int p = 0; p < static_cast<int>(pairs.size()); ++p) {
    const PairRecord& pair = pairs[p];
    const double threshold = pair.threshold;
    const double tau_p = tau[p];
    const bool is_static = table.pair_is_static(p);
    // Δ_p for a static pair: J(p) is empty, so the sparse dot product is empty
    // and MotionBound() collapses to the pair's carve-out slack whatever w is.
    // That slack is normally exactly 0 — "static" then means genuinely
    // immobile — but a pair whose whole J_topo(p) was carved out on a
    // *tolerance* can still drift by that much, and the discrete test below
    // has to charge it or the carved coordinates' residual would go
    // unaccounted for on exactly the pairs made entirely of them.
    const double static_bound = table.carveout_slack(p);
    // A static pair's clearance is the same at every configuration of the
    // trajectory, so the t0 pass settles it for good: re-testing it at every
    // junction would only duplicate its finding (crowding out genuine ones
    // under max_reported_findings) and pay a narrowphase query per junction.
    if (is_static && !resolve_static) continue;

    double lower_bound = -kInfinity;
    const int slot_a = prefilter.slot_a[p];
    const int slot_b = prefilter.slot_b[p];
    if (slot_a >= 0 && slot_b >= 0) {
      lower_bound =
          (geometry->Center(slot_a) - geometry->Center(slot_b)).norm() -
          geometry->radius(slot_a) - geometry->radius(slot_b);
    }

    if (is_static) {
      // Δ_p is the constant `static_bound` for a static pair, so the node
      // certificate degenerates to a single discrete test that holds for the
      // whole domain.
      if (IsCertified(lower_bound, tau_p, static_bound, threshold, slack)) {
        ++stats->sphere_certifications;
        if (records != nullptr) {
          for (int k = 0; k < num_segments; ++k) {
            records->push_back(CertificateRecord{k, 0.0, 1.0, p, q, lower_bound,
                                                 static_bound, threshold});
          }
        }
        continue;
      }
    } else if (lower_bound >= threshold) {
      // A definite violation needs φ̂ + τ_p < m_p, and φ̂ ≥ φ_true − τ_p ≥
      // lower_bound − τ_p, so lower_bound ≥ m_p rules one out with no query.
      continue;
    }

    ++stats->narrowphase_queries;
    const double phi_hat = input.oracle->SignedDistance(query_object, pair,
                                                        &nearest_a, &nearest_b);

    if (IsDefiniteViolation(phi_hat, tau_p, threshold)) {
      Finding finding;
      finding.time = time;
      finding.q = q;
      finding.pair = pair.id;
      finding.distance = phi_hat;
      finding.motion_bound = 0.0;
      finding.definite = true;
      finding.nearest_a_W = nearest_a;
      finding.nearest_b_W = nearest_b;
      sink->AddDefinite(std::move(finding));
      continue;
    }
    if (!is_static) continue;

    if (IsCertified(phi_hat, tau_p, static_bound, threshold, slack)) {
      if (records != nullptr) {
        for (int k = 0; k < num_segments; ++k) {
          records->push_back(CertificateRecord{k, 0.0, 1.0, p, q, phi_hat,
                                               static_bound, threshold});
        }
      }
      continue;
    }
    // Neither certified nor violating, and no subdivision can help: this
    // pair's clearance is constant along the trajectory (up to the carve-out
    // residual) and sits within oracle tolerance of the threshold.
    Finding finding;
    finding.time = time;
    finding.q = q;
    finding.pair = pair.id;
    finding.distance = phi_hat;
    finding.motion_bound = static_bound;
    finding.definite = false;
    finding.nearest_a_W = nearest_a;
    finding.nearest_b_W = nearest_b;
    sink->AddInconclusive(std::move(finding));
  }
}

/* Orders the audit trail so that a run is comparable across thread counts and
 across time reparametrizations (T6). */
void SortRecords(std::vector<CertificateRecord>* records) {
  std::sort(records->begin(), records->end(),
            [](const CertificateRecord& a, const CertificateRecord& b) {
              if (a.segment != b.segment) return a.segment < b.segment;
              if (a.s_start != b.s_start) return a.s_start < b.s_start;
              if (a.s_end != b.s_end) return a.s_end < b.s_end;
              return a.pair_index < b.pair_index;
            });
}

}  // namespace

// ---------------------------------------------------------------------------
// ThreadContext / ContextPool.
// ---------------------------------------------------------------------------

ThreadContext::ThreadContext(const drake::planning::RobotDiagram<double>& model)
    : model_(&model), root_(model.CreateDefaultContext()) {
  plant_context_ = &model.plant().GetMyMutableContextFromRoot(root_.get());
  scene_graph_context_ = &model.scene_graph().GetMyContextFromRoot(*root_);
}

void ThreadContext::SetPositions(const Eigen::VectorXd& q) {
  model_->plant().SetPositions(plant_context_, q);
}

const QueryObject<double>& ThreadContext::query_object() const {
  return model_->scene_graph()
      .get_query_output_port()
      .Eval<QueryObject<double>>(*scene_graph_context_);
}

const RigidTransformd& ThreadContext::EvalBodyPose(BodyIndex body) const {
  return model_->plant().EvalBodyPoseInWorld(*plant_context_,
                                             model_->plant().get_body(body));
}

ContextPool::ContextPool(const drake::planning::RobotDiagram<double>& model,
                         int initial_size)
    : model_(&model) {
  for (int i = 0; i < std::max(1, initial_size); ++i) {
    slots_.push_back(std::make_unique<ThreadContext>(model));
    in_use_.push_back(false);
  }
}

ContextPool::Lease ContextPool::Acquire(int count) const {
  DRAKE_DEMAND(count >= 1);
  std::vector<ThreadContext*> contexts;
  std::vector<int> slots;
  contexts.reserve(count);
  slots.reserve(count);
  std::lock_guard<std::mutex> guard(mutex_);
  for (int i = 0; i < static_cast<int>(slots_.size()) &&
                  static_cast<int>(slots.size()) < count;
       ++i) {
    if (!in_use_[i]) {
      in_use_[i] = true;
      slots.push_back(i);
      contexts.push_back(slots_[i].get());
    }
  }
  while (static_cast<int>(slots.size()) < count) {
    slots_.push_back(std::make_unique<ThreadContext>(*model_));
    in_use_.push_back(true);
    slots.push_back(static_cast<int>(slots_.size()) - 1);
    contexts.push_back(slots_.back().get());
  }
  return Lease(this, std::move(contexts), std::move(slots));
}

int ContextPool::size() const {
  std::lock_guard<std::mutex> guard(mutex_);
  return static_cast<int>(slots_.size());
}

void ContextPool::Release(const std::vector<int>& slots) const {
  std::lock_guard<std::mutex> guard(mutex_);
  for (const int slot : slots) in_use_[slot] = false;
}

ContextPool::Lease& ContextPool::Lease::operator=(Lease&& other) noexcept {
  if (this == &other) return *this;
  if (pool_ != nullptr && !slots_.empty()) pool_->Release(slots_);
  pool_ = other.pool_;
  contexts_ = std::move(other.contexts_);
  slots_ = std::move(other.slots_);
  other.pool_ = nullptr;
  other.contexts_.clear();
  other.slots_.clear();
  return *this;
}

ContextPool::Lease::~Lease() {
  if (pool_ != nullptr && !slots_.empty()) pool_->Release(slots_);
}

// ---------------------------------------------------------------------------
// WorkerPool.
// ---------------------------------------------------------------------------

/* One parked thread. Each slot has its own mutex/condition variable so that
 dispatching to n slots is n independent handoffs rather than a broadcast every
 waiter has to filter. */
struct WorkerPool::Slot {
  std::mutex mutex;
  std::condition_variable condition;
  bool shutdown{false};
  bool has_task{false};
  const std::function<void(int)>* task{nullptr};
  int index{0};
  std::shared_ptr<BatchState> state;
  std::thread thread;
};

/* Completion counter of one dispatch. Held by shared_ptr — the batch and every
 slot that ran one of its tasks own a reference — because the slot signals
 completion *through* this object and the waiter would otherwise be free to
 destroy it while the signalling thread is still inside notify_all(). */
struct WorkerPool::BatchState {
  std::mutex mutex;
  std::condition_variable condition;
  int remaining{0};
};

WorkerPool::WorkerPool() = default;

WorkerPool::~WorkerPool() {
  std::deque<std::unique_ptr<Slot>> slots;
  {
    std::lock_guard<std::mutex> guard(mutex_);
    shutdown_ = true;
    slots.swap(slots_);
    idle_.clear();
  }
  // Outside the pool mutex: a slot thread never takes it, but keeping the
  // teardown lock-free makes that independent of future edits.
  for (const std::unique_ptr<Slot>& slot : slots) {
    {
      std::lock_guard<std::mutex> guard(slot->mutex);
      slot->shutdown = true;
    }
    slot->condition.notify_one();
  }
  for (const std::unique_ptr<Slot>& slot : slots) {
    if (slot->thread.joinable()) slot->thread.join();
  }
}

WorkerPool::Batch WorkerPool::Reserve(int count) {
  Batch batch;
  batch.pool_ = this;
  if (count <= 0) return batch;
  // Bounding the pool by the machine's width keeps a program that runs many
  // concurrent parallel checks from multiplying threads without limit; a call
  // that finds nothing free simply runs with fewer workers, which is only a
  // performance difference. The width comes from Parallelism::Max() rather
  // than hardware_concurrency() directly, so the cap honours DRAKE_NUM_THREADS
  // like the rest of Drake.
  const int cap = Parallelism::Max().num_threads();
  std::lock_guard<std::mutex> guard(mutex_);
  if (shutdown_) return batch;
  while (static_cast<int>(batch.slots_.size()) < count && !idle_.empty()) {
    const int index = idle_.back();
    idle_.pop_back();
    batch.slots_.push_back(index);
    batch.handles_.push_back(slots_[index].get());
  }
  while (static_cast<int>(batch.slots_.size()) < count &&
         static_cast<int>(slots_.size()) < cap) {
    slots_.push_back(std::make_unique<Slot>());
    Slot* const slot = slots_.back().get();
    const int index = static_cast<int>(slots_.size()) - 1;
    try {
      slot->thread = std::thread([slot]() {
        while (true) {
          const std::function<void(int)>* task = nullptr;
          int task_index = 0;
          std::shared_ptr<BatchState> state;
          {
            std::unique_lock<std::mutex> lock(slot->mutex);
            slot->condition.wait(lock, [slot]() {
              return slot->shutdown || slot->has_task;
            });
            if (!slot->has_task) return;  // shutdown
            task = slot->task;
            task_index = slot->index;
            state = std::move(slot->state);
            slot->task = nullptr;
            slot->has_task = false;
          }
          // Tasks are documented not to throw (the certifier's worker lambda
          // catches everything); swallowing here is the last line of defence
          // against terminating the process and stranding the waiter.
          try {
            (*task)(task_index);
          } catch (...) {  // NOLINT(bugprone-empty-catch)
          }
          {
            std::lock_guard<std::mutex> state_guard(state->mutex);
            --state->remaining;
            state->condition.notify_all();
          }
        }
      });
    } catch (const std::system_error&) {
      // Reserve() promises never to fail: running with fewer workers is only a
      // performance difference. So when the system refuses a thread, drop the
      // half-built slot and hand back what was already reserved.
      slots_.pop_back();
      break;
    }
    batch.slots_.push_back(index);
    batch.handles_.push_back(slot);
  }
  return batch;
}

int WorkerPool::size() const {
  std::lock_guard<std::mutex> guard(mutex_);
  return static_cast<int>(slots_.size());
}

void WorkerPool::Release(const std::vector<int>& slots) {
  std::lock_guard<std::mutex> guard(mutex_);
  if (shutdown_) return;
  for (const int slot : slots) idle_.push_back(slot);
}

void WorkerPool::Batch::Dispatch(const std::function<void(int)>& task) {
  if (handles_.empty()) return;
  DRAKE_DEMAND(state_ == nullptr);
  state_ = std::make_shared<BatchState>();
  state_->remaining = static_cast<int>(handles_.size());
  for (int i = 0; i < static_cast<int>(handles_.size()); ++i) {
    Slot* const slot = handles_[i];
    {
      std::lock_guard<std::mutex> guard(slot->mutex);
      slot->task = &task;
      slot->index = i;
      slot->state = state_;
      slot->has_task = true;
    }
    slot->condition.notify_one();
  }
}

void WorkerPool::Batch::Wait() {
  if (state_ == nullptr) return;
  {
    std::unique_lock<std::mutex> lock(state_->mutex);
    state_->condition.wait(lock, [this]() {
      return state_->remaining == 0;
    });
  }
  state_.reset();
}

WorkerPool::Batch& WorkerPool::Batch::operator=(Batch&& other) noexcept {
  if (this == &other) return *this;
  Wait();
  if (pool_ != nullptr && !slots_.empty()) pool_->Release(slots_);
  pool_ = other.pool_;
  slots_ = std::move(other.slots_);
  handles_ = std::move(other.handles_);
  state_ = std::move(other.state_);
  other.pool_ = nullptr;
  other.slots_.clear();
  other.handles_.clear();
  other.state_.reset();
  return *this;
}

WorkerPool::Batch::~Batch() {
  Wait();
  if (pool_ != nullptr && !slots_.empty()) pool_->Release(slots_);
}

// ---------------------------------------------------------------------------
// RunCertifier.
// ---------------------------------------------------------------------------

CertifierOutput RunCertifier(const CertifierInput& input, ContextPool* pool,
                             WorkerPool* workers) {
  DRAKE_DEMAND(input.model != nullptr);
  DRAKE_DEMAND(input.oracle != nullptr);
  DRAKE_DEMAND(input.table != nullptr);
  DRAKE_DEMAND(input.path != nullptr);
  DRAKE_DEMAND(input.pairs != nullptr);
  DRAKE_DEMAND(input.tau != nullptr);
  DRAKE_DEMAND(input.prefilter != nullptr);
  DRAKE_DEMAND(pool != nullptr);

  const Options& options = input.options;
  const PiecewiseBezierPath& path = *input.path;
  const std::vector<PairRecord>& pairs = *input.pairs;
  const int num_pairs = static_cast<int>(pairs.size());
  const int num_segments = static_cast<int>(path.segments().size());
  const bool emit = options.emit_certificate;
  const std::uint64_t max_nodes =
      options.max_nodes.value_or(std::numeric_limits<std::uint64_t>::max());

  CertifierOutput output;
  if (emit) {
    output.certificate.pairs.reserve(num_pairs);
    for (const PairRecord& pair : pairs) {
      output.certificate.pairs.push_back(pair.id);
    }
  }

  FindingSink sink(options.max_reported_findings);
  std::atomic<std::uint64_t> node_counter{0};
  Statistics stats;
  std::vector<CertificateRecord> records;

  const int requested_threads = std::max(1, options.parallelism.num_threads());
  // Only the lead worker's context is leased up front. Helpers lease theirs
  // when (if) they are hired, so a small check under the default
  // Parallelism::Max() never pays for sixteen leases it will not use.
  ContextPool::Lease lease = pool->Acquire(1);

  // --- Steps 1 and 2: breakpoints and static pairs (serial, O(#segments)). --
  if (num_segments > 0 && num_pairs > 0) {
    GeometryCache geometry(*input.prefilter, lease[0]);
    for (int k = 0; k <= num_segments; ++k) {
      // Segment k's start, plus the last segment's end. At a junction the two
      // sides are the same physical configuration (they may differ by 2πk in a
      // continuous-revolute coordinate, which forward kinematics ignores), so
      // one evaluation per breakpoint suffices. Endpoints are Bézier control
      // points, so they are exact — no curve evaluation needed.
      const Eigen::VectorXd q =
          (k < num_segments)
              ? Eigen::VectorXd(path.segments()[k].control_points.col(0))
              : Eigen::VectorXd(
                    path.segments()[k - 1].control_points.rightCols(1));
      const double time = (k < num_segments) ? path.segments()[k].t_start
                                             : path.segments()[k - 1].t_end;
      RunBreakpointPass(input, &lease[0], &geometry, q, time,
                        /* resolve_static = */ k == 0, &sink, &stats,
                        emit ? &records : nullptr);
    }
  }

  // --- Step 3: the adaptive recursion over every segment. ------------------
  std::vector<int> moving_pairs;
  moving_pairs.reserve(num_pairs);
  for (int p = 0; p < num_pairs; ++p) {
    if (!input.table->pair_is_static(p)) moving_pairs.push_back(p);
  }

  const bool have_work = !moving_pairs.empty() && num_segments > 0;
  const int num_threads =
      (workers == nullptr) ? 1 : std::max(1, requested_threads);
  const auto accumulate = [&](Worker* worker) {
    stats.nodes += worker->stats().nodes;
    stats.narrowphase_queries += worker->stats().narrowphase_queries;
    stats.sphere_certifications += worker->stats().sphere_certifications;
    stats.max_depth = std::max(stats.max_depth, worker->stats().max_depth);
    if (emit) {
      records.insert(records.end(),
                     std::make_move_iterator(worker->records().begin()),
                     std::make_move_iterator(worker->records().end()));
    }
  };

  if (have_work && num_threads <= 1) {
    // Serial: one worker, one local stack, no shared queue and no thread
    // interleaving ⇒ bit-deterministic results and stats (requirement P7).
    Worker worker(input, &lease[0], &sink, &node_counter, nullptr, nullptr);
    for (int k = 0; k < num_segments; ++k) {
      WorkItem item;
      item.segment = k;
      item.control_points = path.segments()[k].control_points;
      item.active = moving_pairs;
      worker.RunItem(&item);
      if (node_counter.load(std::memory_order_relaxed) > max_nodes) break;
    }
    accumulate(&worker);
  } else if (have_work) {
    // Parallel driver: lazy recruitment + occupancy-driven sharing. The full
    // rationale — and the deviation from parallelism and determinism's static
    // seeding — is documented on RunCertifier() in certifier_internal.h.
    WorkQueue queue;
    {
      // Seeded in reverse so the LIFO hands segment 0 out first. Before any
      // helper exists that reproduces the serial left-to-right sweep exactly,
      // and once helpers arrive it still lets kFindFirstViolation's bound
      // tighten from the front of the trajectory.
      WorkItem seed;
      for (int k = num_segments - 1; k >= 0; --k) {
        seed.segment = k;
        seed.s_lo = 0.0;
        seed.s_hi = 1.0;
        seed.depth = 0;
        seed.control_points = path.segments()[k].control_points;
        seed.active = moving_pairs;
        queue.Push(&seed);
      }
    }

    // The oracle is documented to throw, and any allocation can. A worker that
    // let an exception escape would terminate the process, and — because it
    // would skip WorkQueue::FinishItem() — would also strand every other
    // worker in Pop(). So every worker catches, aborts the work source, and
    // the first exception is rethrown once all of them have finished.
    std::exception_ptr first_error;
    std::mutex error_mutex;
    const auto record_error = [&]() {
      std::lock_guard<std::mutex> guard(error_mutex);
      if (first_error == nullptr) first_error = std::current_exception();
    };

    // Declared before the batch so that the batch — whose destructor waits for
    // the helpers — is torn down first on every path, including the throwing
    // one.
    std::optional<ContextPool::Lease> helper_lease;
    std::vector<std::unique_ptr<Worker>> helpers;
    std::function<void(int)> helper_task;
    WorkerPool::Batch batch;

    Recruitment recruitment;
    recruitment.nodes_before_hire = kNodesBeforeHiringHelpers;
    recruitment.hire = [&]() {
      batch = workers->Reserve(num_threads - 1);
      const int hired = batch.size();
      if (hired == 0) return;  // Pool exhausted: stay serial, still correct.
      helper_lease.emplace(pool->Acquire(hired));
      helpers.reserve(hired);
      for (int i = 0; i < hired; ++i) {
        helpers.push_back(std::make_unique<Worker>(
            input, &(*helper_lease)[i], &sink, &node_counter, &queue, nullptr));
      }
      helper_task = [&](int index) {
        try {
          helpers[index]->Run();
        } catch (...) {
          record_error();
          queue.Abort();
        }
      };
      // Every consumer of the queue, the lead included: this count is the
      // occupancy target of the sharing policy, and setting it from zero is
      // what switches sharing on.
      queue.set_num_workers(hired + 1);
      batch.Dispatch(helper_task);
    };

    Worker lead(input, &lease[0], &sink, &node_counter, &queue, &recruitment);
    try {
      lead.Run();
    } catch (...) {
      record_error();
      queue.Abort();
    }
    batch.Wait();
    if (first_error != nullptr) std::rethrow_exception(first_error);
    accumulate(&lead);
    for (const std::unique_ptr<Worker>& helper : helpers) {
      accumulate(helper.get());
    }
    // Anything the budget left in the queue is uncovered too.
    for (const WorkItem& item : queue.remaining()) {
      sink.ReportPending(TimeOf(path.segments()[item.segment], item.s_lo),
                         item.control_points.col(0),
                         item.active.empty() ? 0 : item.active.front());
    }
  }

  // --- Step 4: reduce per the search mode. ---------------------------------
  const bool budget_exhausted =
      options.max_nodes.has_value() &&
      node_counter.load(std::memory_order_relaxed) > *options.max_nodes;

  std::vector<Finding> findings;
  if (!sink.definite().empty() &&
      options.mode == SearchMode::kFindFirstViolation) {
    // The branch-and-bound recursion refines toward the earliest witness and
    // the sink keeps entries earliest-first, so this *is* the earliest witness
    // the run found — identical serially and in parallel.
    findings.push_back(sink.definite().front());
  } else {
    findings = sink.definite();
    findings.insert(findings.end(), sink.inconclusive().begin(),
                    sink.inconclusive().end());
  }

  if (budget_exhausted && sink.pending_valid()) {
    // Report what the budget left uncovered as a non-definite finding at the
    // earliest uncovered time (the search algorithm: truncate in parameter
    // order, report the remainder).
    Finding finding;
    finding.time = sink.pending_time();
    finding.q = sink.pending_q();
    finding.pair = pairs[sink.pending_pair()].id;
    finding.motion_bound = 0.0;
    finding.definite = false;
    Eigen::Vector3d nearest_a;
    Eigen::Vector3d nearest_b;
    lease[0].SetPositions(finding.q);
    finding.distance = input.oracle->SignedDistance(lease[0].query_object(),
                                                    pairs[sink.pending_pair()],
                                                    &nearest_a, &nearest_b);
    finding.nearest_a_W = nearest_a;
    finding.nearest_b_W = nearest_b;
    ++stats.narrowphase_queries;
    findings.push_back(std::move(finding));
  }

  std::stable_sort(findings.begin(), findings.end(),
                   [](const Finding& a, const Finding& b) {
                     return a.time < b.time;
                   });
  const int cap = std::max(1, options.max_reported_findings);
  if (static_cast<int>(findings.size()) > cap) findings.resize(cap);

  if (!sink.definite().empty()) {
    output.verdict = Verdict::kViolationFound;
  } else if (budget_exhausted) {
    output.verdict = Verdict::kBudgetExhausted;
  } else if (!sink.inconclusive().empty()) {
    output.verdict = Verdict::kInconclusive;
  } else {
    output.verdict = Verdict::kCertifiedFree;
  }

  output.findings = std::move(findings);
  output.stats = stats;
  if (emit) {
    SortRecords(&records);
    output.certificate.records = std::move(records);
  }
  return output;
}

}  // namespace internal
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
