#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/trajectories/bezier_curve.h"
#include "drake/common/trajectories/composite_trajectory.h"
#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/graph_of_convex_sets.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/binding.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/cost.h"

namespace drake {
namespace planning {
namespace trajectory_optimization {

/**
GcsTrajectoryOptimization implements a simplified motion planning optimization
problem introduced in the paper
["Motion Planning around Obstacles with Convex Optimization"](https://arxiv.org/abs/2205.04422)
by Tobia Marcucci, Mark Petersen, David von Wrangel, Russ Tedrake.

@experimental

Instead of using the full time-scaling curve, this problem uses a single
time-scaling variable for each region. This formulation yields continuous
trajectories, which are not differentiable at the transition times between the
regions since non-convex continuity constraints are not supported yet. However,
it supports continuity on the path r(s) for arbitrary degree. The path
r(s) can be reconstructed from the gcs solution q(t) with
`NormalizeSegmentTimes()` and post-processed with e.g. Toppra to enforce
acceleration bounds.

The ith piece of the composite trajectory is defined as q(t) = r((t - tᵢ) /
hᵢ). r : [0, 1] → ℝⁿ is a the path, parametrized as a Bézier curve with order
n. tᵢ and hᵢ are the initial time and duration of the ith sub-trajectory.

This class supports the notion of a Subgraph of regions. This has proven useful
to facilitate multi-modal motion planning such as: Subgraph A: find a
collision-free trajectory for the robot to a grasping posture, Subgraph B: find
a collision-free trajectory for the robot with the object in its hand to a
placing posture, etc.

@anchor continuous_revolute_joints
### Continuous Revolute Joints

This class also supports robots with continuous revolute joints (revolute joints
that don't have any joint limits) and mobile bases. Adding or subtracting 2π to
such a joint's angle leaves it unchanged; this logic is implemented behind the
scenes. To use it, one should specify the joint indices that don't have limits,
and ensure all sets satisfy the "convexity radius" property -- their width along
a dimension corresponding to a continuous revolute joint must be less than π.
This can be enforced when constructing the convex sets, or after the fact with
`geometry::optimization::PartitionConvexSet`. The `GcsTrajectoryOptimization`
methods `AddRegions` and `AddEdges` will handle all of the intersection checks
behind the scenes, including applying the appropriate logic to connect sets that
"wrap around" 2π.
*/
class GcsTrajectoryOptimization final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GcsTrajectoryOptimization);

  /** Constructs the motion planning problem.
  @param num_positions is the dimension of the configuration space.
  @param continuous_revolute_joints is a list of indices corresponding to
  continuous revolute joints, i.e., revolute joints which don't have any joint
  limits, and hence "wrap around" at 2π. Each entry in
  continuous_revolute_joints must be non-negative, less than num_positions, and
  unique. This feature is currently only supported within a single subgraph:
  continuous revolute joints won't be taken into account when constructing edges
  between subgraphs or checking if sets intersect through a subspace.
  */
  explicit GcsTrajectoryOptimization(
      int num_positions,
      std::vector<int> continuous_revolute_joints = std::vector<int>());

  ~GcsTrajectoryOptimization();

  /** A Subgraph is a subset of the larger graph. It is defined by a set of
  regions and edges between them based on intersection. From an API standpoint,
  a Subgraph is useful to define a multi-modal motion planning problem. Further,
  it allows different constraints and objects to be added to different
  subgraphs. Note that the the GraphOfConvexSets does not differentiate between
  subgraphs and can't be mixed with other instances of
  GcsTrajectoryOptimization.
  */
  class Subgraph final {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Subgraph);

    ~Subgraph();

    /** Returns the name of the subgraph. */
    const std::string& name() const { return name_; }

    /** Returns the order of the Bézier trajectory within the region. */
    int order() const { return order_; }

    /** Returns the number of vertices in the subgraph. */
    int size() const { return vertices_.size(); }

    /** Returns constant reference to a vector of mutable pointers to the
    vertices stored in the subgraph. The order of the vertices is the same as
    the order the regions were added.*/
    const std::vector<geometry::optimization::GraphOfConvexSets::Vertex*>&
    Vertices() {
      return vertices_;
    }

    /** Returns pointers to the vertices stored in the subgraph.
    The order of the vertices is the same as the order the regions were added.
    @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.} */
    std::vector<const geometry::optimization::GraphOfConvexSets::Vertex*>
    Vertices() const;

    /** Returns constant reference to a vector of mutable pointers to the
    edges. */
    const std::vector<geometry::optimization::GraphOfConvexSets::Edge*>&
    Edges() {
      return edges_;
    }

    /** Returns pointers to the edges stored in the subgraph.
    @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.} */
    std::vector<const geometry::optimization::GraphOfConvexSets::Edge*> Edges()
        const;

    /** Returns the regions associated with this subgraph before the
    CartesianProduct. */
    const geometry::optimization::ConvexSets& regions() const {
      return regions_;
    }

    /** Adds a minimum time cost to all regions in the subgraph. The cost is the
    sum of the time scaling variables.
    @param weight is the relative weight of the cost.
    */
    void AddTimeCost(double weight = 1.0);

    /** Adds multiple L2Norm Costs on the upper bound of the path length.
    Since we cannot directly compute the path length of a Bézier curve, we
    minimize the upper bound of the path integral by minimizing the sum of
    distances between control points. For Bézier curves, this is equivalent to
    the sum of the L2Norm of the derivative control points of the curve divided
    by the order.

    @param weight_matrix is the relative weight of each component for the cost.
    The diagonal of the matrix is the weight for each dimension. The
    off-diagonal elements are the weight for the cross terms, which can be used
    to penalize diagonal movement.
    @pre weight_matrix must be of size num_positions() x num_positions().
    */
    void AddPathLengthCost(const Eigen::MatrixXd& weight_matrix);

    /** Similar to AddPathLengthCost in usage, but minimizes ∑ |weight_matrix *
    (rᵢ₊₁ − rᵢ)|₂². In comparison to AddPathLength cost, this cost encourages
    control points to be evenly spaced but may result in greater number of
    regions and larger path length on the solution. It is recommended to use
    this cost only with SolveConvexRestriction when it becomes a quadratic cost
    for which some solvers show a better performance.

    @param weight_matrix is the relative weight of each component for the cost.
    The diagonal of the matrix is the weight for each dimension. The
    off-diagonal elements are the weight for the cross terms, which can be used
    to penalize diagonal movement.
    @pre weight_matrix must be of size num_positions() x num_positions().
    */
    void AddPathEnergyCost(const Eigen::MatrixXd& weight_matrix);

    /** Adds multiple L2Norm Costs on the upper bound of the path length.
    We upper bound the trajectory length by the sum of the distances between
    control points. For Bézier curves, this is equivalent to the sum
    of the L2Norm of the derivative control points of the curve divided by the
    order.

    @param weight is the relative weight of the cost.
    */
    void AddPathLengthCost(double weight = 1.0);

    /** Similar to AddPathLengthCost in usage, but minimizes ∑ |(rᵢ₊₁ − rᵢ)|₂²
    with weight being applied uniformly to all dimensions. In comparison to
    AddPathLength cost, this cost encourages control points to be evenly spaced
    but may result in greater number of regions and larger path length on the
    solution. It is recommended to use this cost only with
    SolveConvexRestriction when it becomes a quadratic cost for which some
    solvers show a better performance.

    @param weight is the relative weight of the cost.
    */
    void AddPathEnergyCost(double weight = 1.0);

    /** Adds a linear velocity constraint to the subgraph `lb` ≤ q̇(t) ≤
    `ub`.
    @param lb is the lower bound of the velocity.
    @param ub is the upper bound of the velocity.

    @throws std::exception if subgraph order is zero, since the velocity is
    defined as the derivative of the Bézier curve.
    @throws std::exception if lb or ub are not of size num_positions().
    */
    void AddVelocityBounds(const Eigen::Ref<const Eigen::VectorXd>& lb,
                           const Eigen::Ref<const Eigen::VectorXd>& ub);

    /** Adds a nonlinear derivative constraints to the subgraph `lb` ≤
    dᴺq(t) / dtᴺ ≤ `ub`.

    This adds a nonlinear constraint to the restriction and MIP
    GraphOfConvexSets::Transcription, while adding a convex surrogate to the
    relaxation. For more details, see @ref nonconvex_graph_of_convex_sets
    "Guiding Non-convex Optimization with the GraphOfConvexSets".

    The nonlinear constraint involves the derivative dᴺq(t) / dtᴺ
    which is decomposed as dᴺr(s) / dsᴺ / hᴺ. The convex surrogate replaces the
    nonlinear component hᴺ with h₀ᴺ⁻¹h, where h₀ is the characteristic time of
    the set. For now, h₀ is set to 1.0 for all sets.

    @param lb is the lower bound of the derivative.
    @param ub is the upper bound of the derivative.
    @param derivative_order is the order of the derivative to be constrained.

    @throws std::exception if subgraph order is less than the derivative order.
    @throws std::exception if the derivative order <= 1, since the linear
      velocity bounds are preferred.
    @throws std::exception if lb or ub are not of size num_positions().
    */
    void AddNonlinearDerivativeBounds(
        const Eigen::Ref<const Eigen::VectorXd>& lb,
        const Eigen::Ref<const Eigen::VectorXd>& ub, int derivative_order);

    /** Enforces that for any two subsequent path segments within the subgraph,
    the `continuity_order`th path derivative at the end of the first segment
    equals that of the start of the second segment.

    Note that the constraints are on the control points of the
    derivatives of r(s) and not q(t). This may result in discontinuities of the
    trajectory return by `SolvePath()` since the r(s) will get rescaled by the
    duration h to yield q(t). `NormalizeSegmentTimes()` will return r(s) with
    valid continuity.

    @param continuity_order is the order of the continuity constraint.

    @throws std::exception if the continuity order is not equal or less than
        the order the subgraphs.
    @throws std::exception if the continuity order is less than one since path
    continuity is enforced by default.

    @note To enforce that the trajectory is of class C^k, you must call
    AddPathContinuityConstraint for each continuity_order 1 through k.
    */
    void AddPathContinuityConstraints(int continuity_order);

    /** Enforces that for any two subsequent path segments within the subgraph,
    the `continuity_order`th time derivative at the end of the first segment
    equals that of the start of the second segment.

    This adds a nonlinear constraint to the restriction and MIP
    GraphOfConvexSets::Transcription, while adding a convex surrogate to the
    relaxation. For more details, see @ref nonconvex_graph_of_convex_sets
    "Guiding Non-convex Optimization with the GraphOfConvexSets".

    The continuity is enforced on the control points of q(t), which appear as
    nonlinear constraints.
    <pre> (dᴺrᵤ(s=1) / dsᴺ) / hᵤᴺ == (dᴺrᵥ(s=0) / dsᴺ) / hᵥᴺ </pre>
    The convex surrogate is simply the path continuity, where hᵤᴺ and hᵥᴺ are
    replaced by the characteristic times of the respective sets:
    <pre> (dᴺrᵤ(s=1) / dsᴺ) / hᵤ₀ᴺ == (dᴺrᵥ(s=0) / dsᴺ) / hᵥ₀ᴺ </pre>.
    For now, these are set to one, but future work may involve scaling them by
    the size of the sets.

    @param continuity_order is the order of the continuity constraint.

    @throws std::exception if the continuity order is not equal or less than
      the order the subgraphs.
    @throws std::exception if the continuity order is less than one since path
      continuity is enforced by default.

    @note To enforce that the trajectory is of class C^k, you must call
    AddContinuityConstraint for each continuity_order 1 through k.
    */
    void AddContinuityConstraints(int continuity_order);

    /** Returns a placeholder decision variable (not actually declared as a
    decision variable in the MathematicalProgram) associated with the time
    scaling of the trajectory in a set within this subgraph. This variable will
    be substituted for real decision variables in methods like AddVertexCost and
    AddVertexConstraint. Passing this variable directly into
    objectives/constraints will result in an error. */
    const symbolic::Variable& vertex_duration() const {
      return placeholder_vertex_duration_var_;
    }

    /** Returns a placeholder decision variable (not actually declared as a
    decision variable in the MathematicalProgram) associated with the control
    points of the trajectory in a set within this subgraph. The variable will be
    of shape (num_positions(), order+1), where the ith column is the ith control
    point. This variable will be substituted for real decision variables in
    methods like AddVertexCost and AddVertexConstraint. Passing this variable
    directly into objectives/constraints will result in an error. */
    const solvers::MatrixXDecisionVariable& vertex_control_points() const {
      return placeholder_vertex_control_points_var_;
    }

    /** Returns a pair of placeholder decision variables (not actually declared
    as decision variables in the MathematicalProgram) associated with the time
    scaling of the trajectory in two sets within this subgraph that are
    connected by an internal edge. These variables will be substituted for real
    decision variables in methods like AddEdgeCost and AddEdgeConstraint.
    Passing this variable directly into objectives/constraints will result in an
    error. */
    const std::pair<symbolic::Variable, symbolic::Variable>&
    edge_constituent_vertex_durations() const {
      return placeholder_edge_durations_var_;
    }

    /** Returns a pair of placeholder decision variables (not actually declared
    as decision variables in the MathematicalProgram) associated with the
    control points of the trajectory in two sets within this subgraph that are
    connected by an internal edge. Each variable will be of shape
    (num_positions(), order+1), where the ith column is the ith control point.
    These variables will be substituted for real decision variables in methods
    like AddEdgeCost and AddEdgeConstraint. Passing this variable directly into
    objectives/constraints will result in an error. */
    const std::pair<solvers::MatrixXDecisionVariable,
                    solvers::MatrixXDecisionVariable>&
    edge_constituent_vertex_control_points() const {
      return placeholder_edge_control_points_var_;
    }

    /** Adds an arbitrary user-defined cost to every vertex in the subgraph. The
    cost should be defined using the placeholder control point variables
    (obtained from vertex_control_points()) and the placeholder time scaling
    variable (obtained from vertex_duration()). This enables greater modeling
    freedom, but we cannot guarantee a feasible solution for all possible costs.

    @throws std::exception if any variables besides those from @ref
    vertex_duration and @ref vertex_control_points are used.

    Costs which do not support the perspective operation cannot be used with
    Transcription::kMIP or Transcription::kRelaxation. Consider providing an
    appropriate "convex surrogate" that is supported within GraphOfConvexSets,
    or exclusively using the SolveConvexRestriction method. */
    void AddVertexCost(
        const symbolic::Expression& e,
        const std::unordered_set<
            geometry::optimization::GraphOfConvexSets::Transcription>&
            use_in_transcription = {
                geometry::optimization::GraphOfConvexSets::Transcription::kMIP,
                geometry::optimization::GraphOfConvexSets::Transcription::
                    kRelaxation,
                geometry::optimization::GraphOfConvexSets::Transcription::
                    kRestriction});

    /** Convenience overload of AddVertexCost to take in a Binding<Cost>.

    @throws std::exception if any variables besides those from @ref
    vertex_duration and @ref vertex_control_points are used. */
    void AddVertexCost(
        const solvers::Binding<solvers::Cost>& binding,
        const std::unordered_set<
            geometry::optimization::GraphOfConvexSets::Transcription>&
            use_in_transcription = {
                geometry::optimization::GraphOfConvexSets::Transcription::kMIP,
                geometry::optimization::GraphOfConvexSets::Transcription::
                    kRelaxation,
                geometry::optimization::GraphOfConvexSets::Transcription::
                    kRestriction});

    /** Adds an arbitrary user-defined constraint to every vertex in the
    subgraph. The constraint should be defined using the placeholder control
    point variables (obtained from vertex_control_points()) and the placeholder
    time scaling variable (obtained from vertex_duration()). This enables
    greater modeling freedom, but we cannot guarantee a feasible solution for
    all possible constraints.

    @throws std::exception if any variables besides those from @ref
    vertex_duration and @ref vertex_control_points are used.

    Constraints which do not support the perspective operation cannot be used
    with Transcription::kMIP or Transcription::kRelaxation. Consider providing
    an appropriate "convex surrogate" that is supported within
    GraphOfConvexSets, or exclusively using the SolveConvexRestriction method.
    */
    void AddVertexConstraint(
        const symbolic::Formula& e,
        const std::unordered_set<
            geometry::optimization::GraphOfConvexSets::Transcription>&
            use_in_transcription = {
                geometry::optimization::GraphOfConvexSets::Transcription::kMIP,
                geometry::optimization::GraphOfConvexSets::Transcription::
                    kRelaxation,
                geometry::optimization::GraphOfConvexSets::Transcription::
                    kRestriction});

    /** Convenience overload of AddVertexConstraint to take in a
    Binding<Constraint>.

    @throws std::exception if any variables besides those from @ref
    vertex_duration and @ref vertex_control_points are used. */
    void AddVertexConstraint(
        const solvers::Binding<solvers::Constraint>& binding,
        const std::unordered_set<
            geometry::optimization::GraphOfConvexSets::Transcription>&
            use_in_transcription = {
                geometry::optimization::GraphOfConvexSets::Transcription::kMIP,
                geometry::optimization::GraphOfConvexSets::Transcription::
                    kRelaxation,
                geometry::optimization::GraphOfConvexSets::Transcription::
                    kRestriction});

    /** Adds an arbitrary user-defined cost to every internal edge within the
    subgraph. The cost should be defined using the placeholder control point
    variables (obtained from edge_constituent_vertex_control_points()) and the
    placeholder time scaling variables (obtained from
    edge_constituent_vertex_durations()). This enables greater modeling freedom,
    but we cannot guarantee a feasible solution for all possible costs.

    @throws std::exception if any variables besides those from @ref
    edge_constituent_vertex_durations and @ref
    edge_constituent_vertex_control_points are used.

    Costs which do not support the perspective operation cannot be used with
    Transcription::kMIP or Transcription::kRelaxation. Consider providing an
    appropriate "convex surrogate" that is supported within GraphOfConvexSets,
    or exclusively using the SolveConvexRestriction method. */
    void AddEdgeCost(
        const symbolic::Expression& e,
        const std::unordered_set<
            geometry::optimization::GraphOfConvexSets::Transcription>&
            use_in_transcription = {
                geometry::optimization::GraphOfConvexSets::Transcription::kMIP,
                geometry::optimization::GraphOfConvexSets::Transcription::
                    kRelaxation,
                geometry::optimization::GraphOfConvexSets::Transcription::
                    kRestriction});

    /** Convenience overload of AddEdgeCost to take in a Binding<Cost>.

    @throws std::exception if any variables besides those from @ref
    edge_constituent_vertex_durations and @ref
    edge_constituent_vertex_control_points are used. */
    void AddEdgeCost(
        const solvers::Binding<solvers::Cost>& binding,
        const std::unordered_set<
            geometry::optimization::GraphOfConvexSets::Transcription>&
            use_in_transcription = {
                geometry::optimization::GraphOfConvexSets::Transcription::kMIP,
                geometry::optimization::GraphOfConvexSets::Transcription::
                    kRelaxation,
                geometry::optimization::GraphOfConvexSets::Transcription::
                    kRestriction});

    /** Adds an arbitrary user-defined constraint (in the form of a Formula or
    Binding<Constraint>) to every internal edge within the subgraph. The
    constraint should be defined using the placeholder control point variables
    (obtained from edge_constituent_vertex_control_points()) and the placeholder
    time scaling variables (obtained from edge_constituent_vertex_durations()).
    This enables greater modeling freedom, but we cannot guarantee a feasible
    solution for all possible constraints.

    @throws std::exception if any variables besides those from @ref
    edge_constituent_vertex_durations and @ref
    edge_constituent_vertex_control_points are used.

    Constraints which do not support the perspective operation cannot be used
    with Transcription::kMIP or Transcription::kRelaxation. Consider providing
    an appropriate "convex surrogate" that is supported within
    GraphOfConvexSets, or exclusively using the SolveConvexRestriction method.
    */
    void AddEdgeConstraint(
        const symbolic::Formula& e,
        const std::unordered_set<
            geometry::optimization::GraphOfConvexSets::Transcription>&
            use_in_transcription = {
                geometry::optimization::GraphOfConvexSets::Transcription::kMIP,
                geometry::optimization::GraphOfConvexSets::Transcription::
                    kRelaxation,
                geometry::optimization::GraphOfConvexSets::Transcription::
                    kRestriction});

    /** Convenience overload of AddEdgeConstraint to take in a
    Binding<Constraint>.

    @throws std::exception if any variables besides those from @ref
    edge_constituent_vertex_durations and @ref
    edge_constituent_vertex_control_points are used. */
    void AddEdgeConstraint(
        const solvers::Binding<solvers::Constraint>& binding,
        const std::unordered_set<
            geometry::optimization::GraphOfConvexSets::Transcription>&
            use_in_transcription = {
                geometry::optimization::GraphOfConvexSets::Transcription::kMIP,
                geometry::optimization::GraphOfConvexSets::Transcription::
                    kRelaxation,
                geometry::optimization::GraphOfConvexSets::Transcription::
                    kRestriction});

   private:
    /* Constructs a new subgraph and copies the regions. */
    Subgraph(const geometry::optimization::ConvexSets& regions,
             const std::vector<std::pair<int, int>>& regions_to_connect,
             int order, double h_min, double h_max, std::string name,
             const std::vector<Eigen::VectorXd>* edge_offsets,
             GcsTrajectoryOptimization* traj_opt);

    /* Convenience accessor, for brevity. */
    int num_positions() const { return traj_opt_.num_positions(); }

    /* Convenience accessor, for brevity. */
    const std::vector<int>& continuous_revolute_joints() const {
      return traj_opt_.continuous_revolute_joints();
    }

    /* Throw an error if any convex set in regions violates the convexity
    radius. */
    void ThrowsForInvalidConvexityRadius() const;

    /* Extracts the control points variables from a vertex. */
    Eigen::Map<const MatrixX<symbolic::Variable>> GetControlPoints(
        const geometry::optimization::GraphOfConvexSets::Vertex& v) const;

    /* Extracts the time scaling variable from a vertex. */
    symbolic::Variable GetTimeScaling(
        const geometry::optimization::GraphOfConvexSets::Vertex& v) const;

    /* Substitute any placeholder variables with the versions corresponding to
    a specific vertex. The return type will match the argument type. */
    template <typename T>
    T SubstituteVertexPlaceholderVariables(
        T e,
        const geometry::optimization::GraphOfConvexSets::Vertex& vertex) const;

    /* Substitute any placeholder variables with the versions corresponding to
    a specific internal edge. The return type will match the argument type. */
    template <typename T>
    T SubstituteEdgePlaceholderVariables(
        T e, const geometry::optimization::GraphOfConvexSets::Edge& edge) const;

    // Compatible with Expression and Binding<Cost>.
    template <typename T>
    void DoAddVertexCost(
        const T& e,
        const std::unordered_set<
            geometry::optimization::GraphOfConvexSets::Transcription>&
            use_in_transcription);

    // Compatible with Formula and Binding<Constraint>.
    template <typename T>
    void DoAddVertexConstraint(
        const T& e,
        const std::unordered_set<
            geometry::optimization::GraphOfConvexSets::Transcription>&
            use_in_transcription);

    // Compatible with Expression and Binding<Cost>.
    template <typename T>
    void DoAddEdgeCost(
        const T& e,
        const std::unordered_set<
            geometry::optimization::GraphOfConvexSets::Transcription>&
            use_in_transcription);

    // Compatible with Formula and Binding<Constraint>.
    template <typename T>
    void DoAddEdgeConstraint(
        const T& e,
        const std::unordered_set<
            geometry::optimization::GraphOfConvexSets::Transcription>&
            use_in_transcription);

    const geometry::optimization::ConvexSets regions_;
    const int order_;
    const double h_min_;
    const std::string name_;
    GcsTrajectoryOptimization& traj_opt_;

    std::vector<geometry::optimization::GraphOfConvexSets::Vertex*> vertices_;
    std::vector<geometry::optimization::GraphOfConvexSets::Edge*> edges_;

    // r(s) is a BezierCurve of the right shape and order, which can be used to
    // design costs and constraints for the underlying vertices and edges.
    trajectories::BezierCurve<double> r_trajectory_;

    symbolic::Variable placeholder_vertex_duration_var_;
    solvers::MatrixXDecisionVariable placeholder_vertex_control_points_var_;

    std::pair<symbolic::Variable, symbolic::Variable>
        placeholder_edge_durations_var_;
    std::pair<solvers::MatrixXDecisionVariable,
              solvers::MatrixXDecisionVariable>
        placeholder_edge_control_points_var_;

    friend class GcsTrajectoryOptimization;
  };

  /** EdgesBetweenSubgraphs are defined as the connecting edges between two
  given subgraphs. These edges are a subset of the many other edges in the
  larger graph. From an API standpoint, EdgesBetweenSubgraphs enable transitions
  between Subgraphs, which can enable transitions between modes. Further, it
  allows different constraints to be added in the transition between subgraphs.
  Note that the EdgesBetweenSubgraphs can't be separated from the actual edges
  in the GraphOfConvexSets framework, thus mixing it with other instances of
  GCSTrajetoryOptimization is not supported.
  */
  class EdgesBetweenSubgraphs final {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EdgesBetweenSubgraphs);

    ~EdgesBetweenSubgraphs();

    /** Adds a linear velocity constraint to the control point connecting the
    subgraphs `lb` ≤ q̇(t) ≤ `ub`.
    @param lb is the lower bound of the velocity.
    @param ub is the upper bound of the velocity.

    @throws std::exception if both subgraphs order is zero, since the velocity
    is defined as the derivative of the Bézier curve. At least one of the
    subgraphs must have an order of at least 1.
    @throws std::exception if lb or ub are not of size num_positions().
    */
    void AddVelocityBounds(const Eigen::Ref<const Eigen::VectorXd>& lb,
                           const Eigen::Ref<const Eigen::VectorXd>& ub);

    /** Adds a nonlinear derivative constraints to the control point connecting
    the subgraphs `lb` ≤ dᴺq(t) / dtᴺ ≤ `ub`.

    This adds a nonlinear constraint to the restriction and MIP
    GraphOfConvexSets::Transcription, while adding a convex surrogate to the
    relaxation. For more details, see @ref nonconvex_graph_of_convex_sets
    "Guiding Non-convex Optimization with the GraphOfConvexSets".

    The nonlinear constraint involves the derivative dᴺq(t) / dtᴺ
    which is decomposed as dᴺr(s) / dsᴺ / hᴺ. The convex surrogate replaces the
    nonlinear component hᴺ with h₀ᴺ⁻¹h, where h₀ is the characteristic time of
    the set. For now, h₀ is set to 1.0 for all sets.

    @param lb is the lower bound of the derivative.
    @param ub is the upper bound of the derivative.
    @param derivative_order is the order of the derivative to be constrained.

    @throws std::exception if both subgraphs order is less than the desired
    derivative order.
    @throws std::exception if the derivative order <= 1, since the linear
      velocity bounds are preferred.
    @throws std::exception if lb or ub are not of size num_positions().
    */
    void AddNonlinearDerivativeBounds(
        const Eigen::Ref<const Eigen::VectorXd>& lb,
        const Eigen::Ref<const Eigen::VectorXd>& ub, int derivative_order);

    /** Enforces zero derivatives on the control point connecting the subgraphs.

    For velocity, acceleration, jerk, etc. enforcing zero-derivative on the
    trajectory q(t) is equivalent to enforcing zero-derivative on the trajectory
    r(s). Hence this constraint is convex.
    @param derivative_order is the order of the derivative to be constrained.

    @throws std::exception if the derivative order < 1.
    @throws std::exception if both subgraphs order is less than the desired
    derivative order.
    */
    void AddZeroDerivativeConstraints(int derivative_order);

    /** Enforces that for any two subsequent path segments that are joined by an
    edge in this EdgesBetweenSubgraphs, the `continuity_order`th path derivative
    at the end of the first segment equals that of the start of the second
    segment.

    Note that the constraints are on the control points of the
    derivatives of r(s) and not q(t). This may result in discontinuities of the
    trajectory return by `SolvePath()` since the r(s) will get rescaled by the
    duration h to yield q(t). `NormalizeSegmentTimes()` will return r(s) with
    valid continuity.

    @param continuity_order is the order of the continuity constraint.

    @throws std::exception if the continuity order is not equal or less than
        the order of both subgraphs.
    @throws std::exception if the continuity order is less than one since path
    continuity is enforced by default.

    @note To enforce that the trajectory is of class C^k, you must call
    AddPathContinuityConstraint for each continuity_order 1 through k.
    */
    void AddPathContinuityConstraints(int continuity_order);

    /** Enforces that for any two subsequent path segments that are joined by an
    edge in this EdgesBetweenSubgraphs, the `continuity_order`th time derivative
    at the end of the first segment equals that of the start of the second
    segment.

    This adds a nonlinear constraint to the restriction and MIP
    GraphOfConvexSets::Transcription, while adding a convex surrogate to the
    relaxation. For more details, see @ref nonconvex_graph_of_convex_sets
    "Guiding Non-convex Optimization with the GraphOfConvexSets".

    The continuity is enforced on the control points of q(t), which appear as
    nonlinear constraints.
    <pre> (dᴺrᵤ(s=1) / dsᴺ) / hᵤᴺ == (dᴺrᵥ(s=0) / dsᴺ) / hᵥᴺ </pre>
    The convex surrogate is simply the path continuity, where hᵤᴺ and hᵥᴺ are
    replaced by the characteristic times of the respective sets:
    <pre> (dᴺrᵤ(s=1) / dsᴺ) / hᵤ₀ᴺ == (dᴺrᵥ(s=0) / dsᴺ) / hᵥ₀ᴺ </pre>.
    For now, these are set to one, but future work may involve scaling them by
    the size of the sets.

    @param continuity_order is the order of the continuity constraint.

    @throws std::exception if the continuity order is not equal or less than
      the order of both subgraphs.
    @throws std::exception if the continuity order is less than one since path
      continuity is enforced by default.

    @note To enforce that the trajectory is of class C^k, you must call
    AddContinuityConstraint for each continuity_order 1 through k.
    */
    void AddContinuityConstraints(int continuity_order);

    /** Returns constant reference to a vector of mutable pointers to the
    edges. */
    const std::vector<geometry::optimization::GraphOfConvexSets::Edge*>&
    Edges() {
      return edges_;
    }

    /** Returns pointers to the edges.
    @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.} */
    std::vector<const geometry::optimization::GraphOfConvexSets::Edge*> Edges()
        const;

    /** Returns a pair of placeholder decision variables (not actually declared
    as decision variables in the MathematicalProgram) associated with the time
    scaling of the trajectory in two sets that are
    connected by an edge from this EdgesBetweenSubgraphs. These variables will
    be substituted for real decision variables in methods like AddEdgeCost and
    AddEdgeConstraint. Passing this variable directly into
    objectives/constraints will result in an error. */
    const std::pair<symbolic::Variable, symbolic::Variable>&
    edge_constituent_vertex_durations() const {
      return placeholder_edge_durations_var_;
    }

    /** Returns a pair of placeholder decision variables (not actually declared
    as decision variables in the MathematicalProgram) associated with the
    control points of the trajectory in two sets that are
    connected by an edge from this EdgesBetweenSubgraphs. Each variable will be
    of shape (num_positions(), order+1), where the ith column is the ith control
    point. (Note that the first and second variable will have different shapes
    if the order of the two subgraphs is different.) These variables will be
    substituted for real decision variables in methods like AddEdgeCost and
    AddEdgeConstraint. Passing this variable directly into
    objectives/constraints will result in an error. */
    const std::pair<solvers::MatrixXDecisionVariable,
                    solvers::MatrixXDecisionVariable>&
    edge_constituent_vertex_control_points() const {
      return placeholder_edge_control_points_var_;
    }

    /** Adds an arbitrary user-defined cost to every edge within the
    EdgesBetweenSubgraphs. The cost should be defined using the placeholder
    control point variables (obtained from
    edge_constituent_vertex_control_points()) and the placeholder time scaling
    variables (obtained from edge_constituent_vertex_durations()). This enables
    greater modeling freedom, but we cannot guarantee a feasible solution for
    all possible costs.

    @throws std::exception if any variables besides those from @ref
    edge_constituent_vertex_durations and @ref
    edge_constituent_vertex_control_points are used.

    Costs which do not support the perspective operation cannot be used with
    Transcription::kMIP or Transcription::kRelaxation. Consider providing an
    appropriate "convex surrogate" that is supported within GraphOfConvexSets,
    or exclusively using the SolveConvexRestriction method. */
    void AddEdgeCost(
        const symbolic::Expression& e,
        const std::unordered_set<
            geometry::optimization::GraphOfConvexSets::Transcription>&
            use_in_transcription = {
                geometry::optimization::GraphOfConvexSets::Transcription::kMIP,
                geometry::optimization::GraphOfConvexSets::Transcription::
                    kRelaxation,
                geometry::optimization::GraphOfConvexSets::Transcription::
                    kRestriction});

    /** Convenience overload of AddEdgeCost to take in a Binding<Cost>.

    @throws std::exception if any variables besides those from @ref
    edge_constituent_vertex_durations and @ref
    edge_constituent_vertex_control_points are used. */
    void AddEdgeCost(
        const solvers::Binding<solvers::Cost>& binding,
        const std::unordered_set<
            geometry::optimization::GraphOfConvexSets::Transcription>&
            use_in_transcription = {
                geometry::optimization::GraphOfConvexSets::Transcription::kMIP,
                geometry::optimization::GraphOfConvexSets::Transcription::
                    kRelaxation,
                geometry::optimization::GraphOfConvexSets::Transcription::
                    kRestriction});

    /** Adds an arbitrary user-defined constraint to every edge within the
    EdgesBetweenSubgraphs. The constraint should be defined using the
    placeholder control point variables (obtained from
    edge_constituent_vertex_control_points()) and the placeholder time scaling
    variables (obtained from edge_constituent_vertex_durations()). This enables
    greater modeling freedom, but we cannot guarantee a feasible solution for
    all possible constraints.

    @throws std::exception if any variables besides those from @ref
    edge_constituent_vertex_durations and @ref
    edge_constituent_vertex_control_points are used.

    Constraints which do not support the perspective operation cannot be used
    with Transcription::kMIP or Transcription::kRelaxation. Consider providing
    an appropriate "convex surrogate" that is supported within
    GraphOfConvexSets, or exclusively using the SolveConvexRestriction method.
    */
    void AddEdgeConstraint(
        const symbolic::Formula& e,
        const std::unordered_set<
            geometry::optimization::GraphOfConvexSets::Transcription>&
            use_in_transcription = {
                geometry::optimization::GraphOfConvexSets::Transcription::kMIP,
                geometry::optimization::GraphOfConvexSets::Transcription::
                    kRelaxation,
                geometry::optimization::GraphOfConvexSets::Transcription::
                    kRestriction});

    /** Convenience overload of AddEdgeConstraint to take in a
    Binding<Constraint>.

    @throws std::exception if any variables besides those from @ref
    edge_constituent_vertex_durations and @ref
    edge_constituent_vertex_control_points are used. */
    void AddEdgeConstraint(
        const solvers::Binding<solvers::Constraint>& binding,
        const std::unordered_set<
            geometry::optimization::GraphOfConvexSets::Transcription>&
            use_in_transcription = {
                geometry::optimization::GraphOfConvexSets::Transcription::kMIP,
                geometry::optimization::GraphOfConvexSets::Transcription::
                    kRelaxation,
                geometry::optimization::GraphOfConvexSets::Transcription::
                    kRestriction});

   private:
    EdgesBetweenSubgraphs(
        const Subgraph& from_subgraph, const Subgraph& to_subgraph,
        const geometry::optimization::ConvexSet* subspace,
        GcsTrajectoryOptimization* traj_opt,
        const std::vector<std::pair<int, int>>* edges_between_regions = nullptr,
        const std::vector<Eigen::VectorXd>* edge_offsets = nullptr);

    /* Convenience accessor, for brevity. */
    int num_positions() const { return traj_opt_.num_positions(); }

    /* Convenience accessor, for brevity. */
    const std::vector<int>& continuous_revolute_joints() const {
      return traj_opt_.continuous_revolute_joints();
    }

    bool RegionsConnectThroughSubspace(
        const geometry::optimization::ConvexSet& A,
        const geometry::optimization::ConvexSet& B,
        const geometry::optimization::ConvexSet& subspace,
        const Eigen::VectorXd* maybe_set_B_offset = nullptr,
        const Eigen::VectorXd* maybe_subspace_offset = nullptr);

    /* Extracts the control points variables from an edge. */
    Eigen::Map<const MatrixX<symbolic::Variable>> GetControlPointsU(
        const geometry::optimization::GraphOfConvexSets::Edge& e) const;

    /* Extracts the control points variables from an edge. */
    Eigen::Map<const MatrixX<symbolic::Variable>> GetControlPointsV(
        const geometry::optimization::GraphOfConvexSets::Edge& e) const;

    /* Extracts the time scaling variable from a edge. */
    symbolic::Variable GetTimeScalingU(
        const geometry::optimization::GraphOfConvexSets::Edge& e) const;

    /* Extracts the time scaling variable from a edge. */
    symbolic::Variable GetTimeScalingV(
        const geometry::optimization::GraphOfConvexSets::Edge& e) const;

    template <typename T>
    T SubstituteEdgePlaceholderVariables(
        T e, const geometry::optimization::GraphOfConvexSets::Edge& edge) const;

    template <typename T>
    void DoAddEdgeCost(
        const T& e,
        const std::unordered_set<
            geometry::optimization::GraphOfConvexSets::Transcription>&
            use_in_transcription);

    template <typename T>
    void DoAddEdgeConstraint(
        const T& e,
        const std::unordered_set<
            geometry::optimization::GraphOfConvexSets::Transcription>&
            use_in_transcription);

    GcsTrajectoryOptimization& traj_opt_;
    const Subgraph& from_subgraph_;
    const Subgraph& to_subgraph_;

    trajectories::BezierCurve<double> ur_trajectory_;
    trajectories::BezierCurve<double> vr_trajectory_;

    std::vector<geometry::optimization::GraphOfConvexSets::Edge*> edges_;

    std::pair<symbolic::Variable, symbolic::Variable>
        placeholder_edge_durations_var_;
    std::pair<solvers::MatrixXDecisionVariable,
              solvers::MatrixXDecisionVariable>
        placeholder_edge_control_points_var_;

    friend class GcsTrajectoryOptimization;
  };

  /** Returns the number of position variables. */
  int num_positions() const { return num_positions_; }

  /** Returns a list of indices corresponding to continuous revolute joints. */
  const std::vector<int>& continuous_revolute_joints() {
    return continuous_revolute_joints_;
  }

  /** Returns a Graphviz string describing the graph vertices and edges.  If
  `result` is supplied, then the graph will be annotated with the solution
  values.
  */
  std::string GetGraphvizString(
      const solvers::MathematicalProgramResult* result = nullptr,
      const geometry::optimization::GcsGraphvizOptions& options = {}) const {
    return gcs_.GetGraphvizString(result, options);
  }

  /** Creates a Subgraph with the given regions and indices.
  @param regions represent the valid set a control point can be in. We retain a
  copy of the regions since other functions may access them. If any of the
  positions represent revolute joints without limits, each region has a maximum
  width of strictly less than π along dimensions corresponding to those joints.
  @param edges_between_regions is a list of pairs of indices into the regions
  vector. For each pair representing an edge between two regions, an edge is
  added within the subgraph. Note that the edges are directed so (i,j) will only
  add an edge from region i to region j.
  @param order is the order of the Bézier curve.
  @param h_max is the maximum duration to spend in a region (seconds). Some
  solvers struggle numerically with large values.
  @param h_min is the minimum duration to spend in a region (seconds) if that
  region is visited on the optimal path. Some cost and constraints are only
  convex for h > 0. For example the perspective quadratic cost of the path
  energy ||ṙ(s)||² / h becomes non-convex for h = 0. Otherwise h_min can be set
  to 0.
  @param name is the name of the subgraph. If the passed name is an empty
  string, a default name will be provided.
  @param edge_offsets is an optional list of vectors. If defined, the list must
  contain the same number of entries as `edges_between_regions`. For each pair
  of sets listed in `edges_between_regions`, the first set is translated (in
  configuration space) by the corresponding vector in edge_offsets before
  computing the constraints associated to that edge. This is used to add edges
  between sets that "wrap around" 2π along some dimension, due to, e.g., a
  continuous revolute joint. This edge offset corresponds to the translation
  component of the affine map τ_uv in equation (11) of "Non-Euclidean Motion
  Planning with Graphs of Geodesically-Convex Sets", and per the discussion in
  Subsection VI A, τ_uv has no rotation component. If edge_offsets is nullptr,
  it will instead be computed automatically.
  @throws std::exception if any index referenced in `edges_between_regions` is
  outside the range [0, ssize(regions)).
  */
  Subgraph& AddRegions(
      const geometry::optimization::ConvexSets& regions,
      const std::vector<std::pair<int, int>>& edges_between_regions, int order,
      double h_min = 0, double h_max = 20, std::string name = "",
      const std::vector<Eigen::VectorXd>* edge_offsets = nullptr);

  /** Creates a Subgraph with the given regions.
  This function will compute the edges between the regions based on the set
  intersections.
  @param regions represent the valid set a control point can be in. We retain a
  copy of the regions since other functions may access them. If any of the
  positions represent continuous revolute joints, each region must have a
  maximum width of strictly less than π along dimensions corresponding to those
  joints.
  @param order is the order of the Bézier curve.
  @param h_min is the minimum duration to spend in a region (seconds) if that
  region is visited on the optimal path. Some cost and constraints are only
  convex for h > 0. For example the perspective quadratic cost of the path
  energy ||ṙ(s)||² / h becomes non-convex for h = 0. Otherwise h_min can be set
  to 0.
  @param h_max is the maximum duration to spend in a region (seconds). Some
  solvers struggle numerically with large values.
  @param name is the name of the subgraph. A default name will be provided.
  @throws std::exception if any of the regions has a width of π or greater along
  dimensions corresponding to continuous revolute joints.
  */
  Subgraph& AddRegions(const geometry::optimization::ConvexSets& regions,
                       int order, double h_min = 1e-6, double h_max = 20,
                       std::string name = "");

  /** Remove a subgraph and all associated edges found in the subgraph and
  to and from other subgraphs.
  @pre The subgraph must have been created from a call to AddRegions() on this
    object.
  */
  void RemoveSubgraph(const Subgraph& subgraph);

  /** Connects two subgraphs with directed edges.
  @param from_subgraph is the subgraph to connect from. Must have been created
  from a call to AddRegions() on this object, not some other optimization
  program.
  @param to_subgraph is the subgraph to connect to. Must have been created from
  a call to AddRegions() on this object, not some other optimization program.
  @param subspace is the subspace that the connecting control points must be in.
  Subspace is optional. Only edges that connect through the subspace will be
  added, and the subspace is added as a constraint on the connecting control
  points. Subspaces of type point or HPolyhedron are supported since other sets
  require constraints that are not yet supported by the GraphOfConvexSets::Edge
  constraint, e.g., set containment of a Hyperellipsoid is formulated via
  LorentzCone constraints. Workaround: Create a subgraph of zero order with the
  subspace as the region and connect it between the two subgraphs. This works
  because GraphOfConvexSet::Vertex supports arbitrary instances of ConvexSets.
  @param edges_between_regions can be used to manually specify which edges
  should be added, avoiding the intersection checks. It should be a list of
  tuples `(i,j)`, where an edge will be added from the `i`th index region in
  `from_subgraph` to the `j`th index region in `to_subgraph`.
  @param edge_offsets is an optional list of vectors. If defined, the list must
  contain the same number of entries as `edges_between_regions`, and the order
  must match. In other words, if defined, there must be one edge offset for each
  specified edge, and they must be at the same index. For each pair of sets
  listed in `edges_between_regions`, the first set is translated (in
  configuration space) by the corresponding vector in edge_offsets before
  computing the constraints associated to that edge. This is used to add edges
  between sets that "wrap around" 2π along some dimension, due to, e.g., a
  continuous revolute joint. This edge offset corresponds to the translation
  component of the affine map τ_uv in equation (11) of "Non-Euclidean Motion
  Planning with Graphs of Geodesically-Convex Sets", and per the discussion in
  Subsection VI A, τ_uv has no rotation component. If edge_offsets is nullptr,
  it will instead be computed automatically.
  @throws std::exception if `edge_offsets` is provided, but `edge_offsets.size()
  != edges_between_regions.size()`.
  */
  EdgesBetweenSubgraphs& AddEdges(
      const Subgraph& from_subgraph, const Subgraph& to_subgraph,
      const geometry::optimization::ConvexSet* subspace = nullptr,
      const std::vector<std::pair<int, int>>* edges_between_regions = nullptr,
      const std::vector<Eigen::VectorXd>* edge_offsets = nullptr);

  /** Adds a minimum time cost to all regions in the whole graph. The cost is
  the sum of the time scaling variables.

  This cost will be added to the entire graph. Note that this cost
  will be applied even to subgraphs added in the future.

  @param weight is the relative weight of the cost.
  */
  void AddTimeCost(double weight = 1.0);

  /** Adds multiple L2Norm Costs on the upper bound of the path length.
  Since we cannot directly compute the path length of a Bézier curve, we
  minimize the upper bound of the path integral by minimizing the sum of
  (weighted) distances between control points: ∑ |weight_matrix * (rᵢ₊₁ − rᵢ)|₂.

  This cost will be added to the entire graph. Since the path length is only
  defined for Bézier curves that have two or more control points, this cost will
  only added to all subgraphs with order greater than zero. Note that this cost
  will be applied even to subgraphs added in the future.

  @param weight_matrix is the relative weight of each component for the cost.
  The diagonal of the matrix is the weight for each dimension. The
  off-diagonal elements are the weight for the cross terms, which can be used
  to penalize diagonal movement.
  @pre weight_matrix must be of size num_positions() x num_positions().
  */
  void AddPathLengthCost(const Eigen::MatrixXd& weight_matrix);

  /** Similar to AddPathLengthCost in usage, but minimizes ∑ |weight_matrix *
  (rᵢ₊₁ − rᵢ)|₂². In comparison to AddPathLength cost, this cost encourages
  control points to be evenly spaced but may result in greater number of regions
  and larger path length on the solution. It is recommended to use this cost
  only with SolveConvexRestriction when it becomes a quadratic cost for which
  some solvers show a better performance.

  This cost will be added to the entire graph. Since the path length is only
  defined for Bézier curves that have two or more control points, this cost will
  only added to all subgraphs with order greater than zero. Note that this cost
  will be applied even to subgraphs added in the future.

  @param weight_matrix is the relative weight of each component for the cost.
  The diagonal of the matrix is the weight for each dimension. The
  off-diagonal elements are the weight for the cross terms, which can be used
  to penalize diagonal movement.
  @pre weight_matrix must be of size num_positions() x num_positions().
  */
  void AddPathEnergyCost(const Eigen::MatrixXd& weight_matrix);

  /** Adds multiple L2Norm Costs on the upper bound of the path length.
  Since we cannot directly compute the path length of a Bézier curve, we
  minimize the upper bound of the path integral by minimizing the sum of
  distances between control points. For Bézier curves, this is equivalent to the
  sum of the L2Norm of the derivative control points of the curve divided by the
  order.

  This cost will be added to the entire graph. Since the path length is only
  defined for Bézier curves that have two or more control points, this cost will
  only added to all subgraphs with order greater than zero. Note that this cost
  will be applied even to subgraphs added in the future.

  @param weight is the relative weight of the cost.
  */
  void AddPathLengthCost(double weight = 1.0);

  /** Similar to AddPathLengthCost in usage, but minimizes ∑ |(rᵢ₊₁ − rᵢ)|₂²
  with weight being applied uniformly to all dimensions. In comparison to
  AddPathLength cost, this cost encourages control points to be evenly spaced
  but may result in greater number of regions and larger path length on the
  solution. It is recommended to use this cost only with SolveConvexRestriction
  when it becomes a quadratic cost for which some solvers show a better
  performance.

  This cost will be added to the entire graph. Since the path length is only
  defined for Bézier curves that have two or more control points, this cost will
  only added to all subgraphs with order greater than zero. Note that this cost
  will be applied even to subgraphs added in the future.

  @param weight is the relative weight of the cost.
  */
  void AddPathEnergyCost(double weight = 1.0);

  /** Adds a linear velocity constraint to the entire graph `lb` ≤ q̇(t) ≤
  `ub`.
  @param lb is the lower bound of the velocity.
  @param ub is the upper bound of the velocity.

  This constraint will be added to the entire graph. Since the velocity requires
  forming the derivative of the Bézier curve, this constraint will only added to
  all subgraphs with order greater than zero. Note that this constraint will be
  applied even to subgraphs added in the future.

  @throws std::exception if lb or ub are not of size num_positions().
  */
  void AddVelocityBounds(const Eigen::Ref<const Eigen::VectorXd>& lb,
                         const Eigen::Ref<const Eigen::VectorXd>& ub);

  /** Adds a nonlinear derivative constraints to the entire graph `lb` ≤
  dᴺq(t) / dtᴺ ≤ `ub`.

  This adds a nonlinear constraint to the restriction and MIP
  GraphOfConvexSets::Transcription, while adding a convex surrogate to the
  relaxation. For more details, see @ref nonconvex_graph_of_convex_sets
  "Guiding Non-convex Optimization with the GraphOfConvexSets".

  The nonlinear constraint involves the derivative dᴺq(t) / dtᴺ
  which is decomposed as dᴺr(s) / dsᴺ / hᴺ. The convex surrogate replaces the
  nonlinear component hᴺ with h₀ᴺ⁻¹h, where h₀ is the characteristic time of
  the set. For now, h₀ is set to 1.0 for all sets.


  @param lb is the lower bound of the derivative.
  @param ub is the upper bound of the derivative.
  @param derivative_order is the order of the derivative to be constrained.

  @throws std::exception if lb or ub are not of size num_positions().
  @throws std::exception if the derivative order <= 1, since the linear
    velocity bounds are preferred.
  */
  void AddNonlinearDerivativeBounds(const Eigen::Ref<const Eigen::VectorXd>& lb,
                                    const Eigen::Ref<const Eigen::VectorXd>& ub,
                                    int derivative_order);

  /** Enforces that for any two subsequent path segments in the entire graph,
  the `continuity_order`th path derivative at the end of the first segment
  equals that of the start of the second segment.

  Note that the constraints are on the control points of the
  derivatives of r(s) and not q(t). This may result in discontinuities of the
  trajectory return by `SolvePath()` since the r(s) will get rescaled by the
  duration h to yield q(t). `NormalizeSegmentTimes()` will return r(s) with
  valid continuity.

  @param continuity_order is the order of the continuity constraint.

  @throws std::exception if the continuity order is less than one since path
  continuity is enforced by default.

  @note To enforce that the trajectory is of class C^k, you must call
  AddPathContinuityConstraint for each continuity_order 1 through k.
  */
  void AddPathContinuityConstraints(int continuity_order);

  /** Enforces that for any two subsequent path segments in the entire graph,
  the `continuity_order`th time derivative at the end of the first segment
  equals that of the start of the second segment.

  This adds a nonlinear constraint to the restriction and MIP
  GraphOfConvexSets::Transcription, while adding a convex surrogate to the
  relaxation. For more details, see @ref nonconvex_graph_of_convex_sets
  "Guiding Non-convex Optimization with the GraphOfConvexSets".

  The continuity is enforced on the control points of q(t), which appear as
  nonlinear constraints.
  <pre> (dᴺrᵤ(s=1) / dsᴺ) / hᵤᴺ == (dᴺrᵥ(s=0) / dsᴺ) / hᵥᴺ </pre>
  The convex surrogate is simply the path continuity, where hᵤᴺ and hᵥᴺ are
  replaced by the characteristic times of the respective sets:
  <pre> (dᴺrᵤ(s=1) / dsᴺ) / hᵤ₀ᴺ == (dᴺrᵥ(s=0) / dsᴺ) / hᵥ₀ᴺ </pre>.
  For now, these are set to one, but future work may involve scaling them by
  the size of the sets.

  @param continuity_order is the order of the continuity constraint.

  @throws std::exception if the continuity order is less than one since path
    continuity is enforced by default.

  @note To enforce that the trajectory is of class C^k, you must call
  AddContinuityConstraint for each continuity_order 1 through k.
  */
  void AddContinuityConstraints(int continuity_order);

  /** Formulates and solves the mixed-integer convex formulation of the
  shortest path problem on the whole graph. @see
  `geometry::optimization::GraphOfConvexSets::SolveShortestPath()` for further
  details.

  @param source specifies the source subgraph. Must have been created from a
  call to AddRegions() on this object, not some other optimization program. If
  the source is a subgraph with more than one region, an empty set will be
  added and optimizer will choose the best region to start in. To start in a
  particular point, consider adding a subgraph of order zero with a single
  region of type Point.
  @param target specifies the target subgraph. Must have been created from a
  call to AddRegions() on this object, not some other optimization program. If
  the target is a subgraph with more than one region, an empty set will be
  added and optimizer will choose the best region to end in. To end in a
  particular point, consider adding a subgraph of order zero with a single
  region of type Point.
  @param options include all settings for solving the shortest path problem.
  The following default options will be used if they are not provided in
  `options`:
  - `options.convex_relaxation = true`,
  - `options.max_rounded_paths = 5`,
  - `options.preprocessing = true`.

  @see `geometry::optimization::GraphOfConvexSetsOptions` for further details.
  */
  std::pair<trajectories::CompositeTrajectory<double>,
            solvers::MathematicalProgramResult>
  SolvePath(
      const Subgraph& source, const Subgraph& target,
      const geometry::optimization::GraphOfConvexSetsOptions& options = {});

  /** Solves a trajectory optimization problem through specific vertices.

  This method allows for targeted optimization by considering only selected
  active vertices, reducing the problem's complexity.
  See geometry::optimization::GraphOfConvexSets::SolveConvexRestriction().
  This API prefers a sequence of vertices over edges, as a user may know which
  regions the solution should pass through.
  GcsTrajectoryOptimization::AddRegions() automatically manages edge creation
  and intersection checks, which makes passing a sequence of edges less
  convenient.

  @param active_vertices A sequence of ordered vertices of subgraphs to be
    included in the problem.
  @param options include all settings for solving the shortest path problem.

  @pre There must be at least two vertices in active_vertices.
  @throws std::exception if the vertices are not connected.
  @throws std::exception if two vertices are connected by multiple edges. This
    may happen if one connects two graphs through multiple subspaces, which is
    currently not supported with this method.
  @throws std::exception if the program cannot be written as a convex
  optimization consumable by one of the standard solvers.*/
  std::pair<trajectories::CompositeTrajectory<double>,
            solvers::MathematicalProgramResult>
  SolveConvexRestriction(
      const std::vector<
          const geometry::optimization::GraphOfConvexSets::Vertex*>&
          active_vertices,
      const geometry::optimization::GraphOfConvexSetsOptions& options = {});

  /** Provide a heuristic estimate of the complexity of the underlying
  GCS mathematical program, for regression testing purposes.
  Here we sum the total number of variable appearances in our costs and
  constraints as a rough approximation of the complexity of the subproblems. */
  double EstimateComplexity() const;

  /** Returns a vector of all subgraphs. */
  std::vector<Subgraph*> GetSubgraphs() const;

  /** Returns a vector of all edges between subgraphs. */
  std::vector<EdgesBetweenSubgraphs*> GetEdgesBetweenSubgraphs() const;

  /** Getter for the underlying GraphOfConvexSets. This is intended primarily
  for inspecting the resulting programs. */
  const geometry::optimization::GraphOfConvexSets& graph_of_convex_sets()
      const {
    return gcs_;
  }

  /** Normalizes each trajectory segment to one second in duration.
  Reconstructs the path r(s) from the solution trajectory q(t) of
  `SolvePath()` s.t. each segment of the resulting trajectory
  will be one second long. The start time will match the original start time.
  @param trajectory The solution trajectory returned by `SolvePath()`.

  @throws std::exception if not all trajectory segments of the
  CompositeTrajectory are of type BezierCurve<double>
  */
  static trajectories::CompositeTrajectory<double> NormalizeSegmentTimes(
      const trajectories::CompositeTrajectory<double>& trajectory);

  /** Unwraps a trajectory with continuous revolute joints into a continuous
   trajectory in the Euclidean space. Trajectories produced by
   GcsTrajectoryOptimization for robotic systems with continuous revolute joints
   may include apparent discontinuities, where a multiple of 2π is
   instantaneously added to a joint value at the boundary between two adjacent
   segments of the trajectory. This function removes such discontinuities by
   adding or subtracting the appropriate multiple of 2π, "unwrapping" the
   trajectory into a continuous representation suitable for downstream tasks
   that do not take the joint wraparound into account.
   @param gcs_trajectory The trajectory to unwrap.
   @param continuous_revolute_joints The indices of the continuous revolute
   joints.
   @param tol The numerical tolerance used to determine if two subsequent
   segments start and end at the same value modulo 2π for continuous revolute
   joints.
   @param starting_rounds A vector of integers that sets the starting rounds for
   each continuous revolute joint. Given integer k for the starting_round of a
   joint, its initial position will be wrapped into [2πk , 2π(k+1)). If the
   starting rounds are not provided, the initial position of @p trajectory will
   be unchanged.

   @returns an unwrapped (continous in Euclidean space) CompositeTrajectory.

   @throws std::exception if
   starting_rounds.size()!=continuous_revolute_joints.size().
   @throws std::exception if continuous_revolute_joints contain repeated indices
   and/or indices outside the range [0, gcs_trajectory.rows()).
   @throws std::exception if the gcs_trajectory is not continuous on the
   manifold defined by the continuous_revolute_joints, i.e., the shift between
   two consecutive segments is not an integer multiple of 2π (within a tolerance
   of `tol` radians).
   @throws std::exception if all the segments are not of type BezierCurve.
   Other types are not supported yet. Note that currently the output of
   GcsTrajectoryOptimization::SolvePath() is a CompositeTrajectory of
   BezierCurves.
    */
  static trajectories::CompositeTrajectory<double> UnwrapToContinuousTrajectory(
      const trajectories::CompositeTrajectory<double>& gcs_trajectory,
      std::vector<int> continuous_revolute_joints,
      std::optional<std::vector<int>> starting_rounds = std::nullopt,
      double tol = 1e-8);

 private:
  const int num_positions_;
  const std::vector<int> continuous_revolute_joints_;

  trajectories::CompositeTrajectory<double>
  ReconstructTrajectoryFromSolutionPath(
      std::vector<const geometry::optimization::GraphOfConvexSets::Edge*> edges,
      const solvers::MathematicalProgramResult& result);

  // Adds a Edge to gcs_ with the name "{u.name} -> {v.name}".
  geometry::optimization::GraphOfConvexSets::Edge* AddEdge(
      geometry::optimization::GraphOfConvexSets::Vertex* u,
      geometry::optimization::GraphOfConvexSets::Vertex* v);

  geometry::optimization::GraphOfConvexSets gcs_;

  // Store the subgraphs by reference.
  std::vector<std::unique_ptr<Subgraph>> subgraphs_;
  std::vector<std::unique_ptr<EdgesBetweenSubgraphs>> subgraph_edges_;
  std::map<const geometry::optimization::GraphOfConvexSets::Vertex*, Subgraph*>
      vertex_to_subgraph_;
  std::vector<double> global_time_costs_;
  std::vector<Eigen::MatrixXd> global_path_length_costs_;
  std::vector<Eigen::MatrixXd> global_path_energy_costs_;
  std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>
      global_velocity_bounds_{};
  std::vector<std::tuple<Eigen::VectorXd, Eigen::VectorXd, int>>
      global_nonlinear_derivative_bounds_{};
  std::vector<int> global_path_continuity_constraints_{};
  std::vector<int> global_continuity_constraints_{};
};

/** Returns a list of indices in the plant's generalized positions which
correspond to a continuous revolute joint (a revolute joint with no joint
limits). This includes UniversalJoint, and the revolute component of PlanarJoint
and RpyFloatingJoint. */
std::vector<int> GetContinuousRevoluteJointIndices(
    const multibody::MultibodyPlant<double>& plant);

}  // namespace trajectory_optimization
}  // namespace planning
}  // namespace drake
