#pragma once

#include <map>
#include <tuple>
#include <vector>

#include "drake/common/trajectories/bezier_curve.h"
#include "drake/common/trajectories/composite_trajectory.h"
#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/graph_of_convex_sets.h"

namespace drake {
namespace planning {
namespace trajectory_optimization {

/**
GCSTrajectoryOptimization implements a simplified motion planning optimization
problem introduced in the paper "Motion Planning around Obstacles with Convex
Optimization."

"Motion Planning around Obstacles with Convex Optimization" by Tobia Marcucci,
Mark Petersen, David von Wrangel, Russ Tedrake. https://arxiv.org/abs/2205.04422

Instead of using the full time scaling curve, this problem uses a single time
scaling variable for each region. This makes enforcing the continuity
constraints more difficult, but significantly simplifies higher-order derivative
constraints.
*/
class GCSTrajectoryOptimization {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GCSTrajectoryOptimization);

  ~GCSTrajectoryOptimization() = default;

  /**
  Constructs the motion planning problem.
  @param positions is the dimension of the configuration space.
  */
  GCSTrajectoryOptimization(int positions);

  class Subgraph final {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Subgraph);

    ~Subgraph() = default;

    /** Returns the name of the subgraph.*/
    const std::string& name() const { return name_; }

    /** Returns the order of the Bézier trajectory within the region.*/
    int order() const { return order_; }

    /** Returns the number of vertices in the subgraph.*/
    int size() const { return vertices_.size(); }

    /** Returns the regions associated with this subgraph before the
     * CartesianProduct.*/
    const geometry::optimization::ConvexSets& regions() const {
      return regions_;
    }

    /** Adds multiple L2Norm Costs on the upper bound of the path length.
    We upper bound the path integral by the sum of the distances between
    control points. For Bezier curves, this is equivalent to the sum
    of the L2Norm of the derivative control points of the curve divided by the
    order.

    @param weight_matrix is the relative weight of each component for the cost.
      The diagonal of the matrix is the weight for each dimension.
      The matrix must be square and of size num_positions() x num_positions().

    */
    void AddPathLengthCost(const Eigen::MatrixXd& weight_matrix);

    /** Adds multiple L2Norm Costs on the upper bound of the path length.
    We upper bound the path integral by the sum of the distances between
    control points. For Bezier curves, this is equivalent to the sum
    of the L2Norm of the derivative control points of the curve divided by the
    order.

    @param weight is the relative weight of the cost.
    */
    void AddPathLengthCost(double weight = 1.0);

   private:
    // construct a new subgraph
    Subgraph(const geometry::optimization::ConvexSets& regions,
             std::vector<std::pair<int, int>>& regions_to_connect, int order,
             double d_min, double d_max, const std::string& name,
             GCSTrajectoryOptimization* gcs);

    /** Returns all vertices associated with this subgraph.*/
    const std::vector<geometry::optimization::GraphOfConvexSets::Vertex*>&
    vertices() const {
      return vertices_;
    }

    /** Returns all Edges within this subgraph.*/
    const std::vector<geometry::optimization::GraphOfConvexSets::Edge*>& edges()
        const {
      return edges_;
    }

    const geometry::optimization::ConvexSets regions_;
    int order_;
    const std::string name_;
    GCSTrajectoryOptimization* gcs_;

    std::vector<geometry::optimization::GraphOfConvexSets::Vertex*> vertices_;
    std::vector<geometry::optimization::GraphOfConvexSets::Edge*> edges_;

    VectorX<symbolic::Variable> u_duration_;
    VectorX<symbolic::Variable> u_vars_;

    // r(s)
    trajectories::BezierCurve<symbolic::Expression> u_r_trajectory_;

    friend class GCSTrajectoryOptimization;
  };

  class SubgraphEdges final {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SubgraphEdges);

    ~SubgraphEdges() = default;

    /** Adds a linear velocity constraints to the control point connecting the
    subgraphs `lb` ≤ q̈(t) ≤ `ub`. At least one of the subgraphs must have an
    order of at least 1.
    @param lb is the lower bound of the velocity.
    @param ub is the upper bound of the velocity.
    */
    void AddVelocityBounds(const Eigen::Ref<const Eigen::VectorXd>& lb,
                           const Eigen::Ref<const Eigen::VectorXd>& ub);

   private:
    SubgraphEdges(const Subgraph* from, const Subgraph* to,
                  const geometry::optimization::ConvexSet* subspace,
                  GCSTrajectoryOptimization* gcs);

    bool RegionsConnectThroughSubspace(
        const geometry::optimization::ConvexSet& A,
        const geometry::optimization::ConvexSet& B,
        const geometry::optimization::ConvexSet& subspace);

    const std::vector<geometry::optimization::GraphOfConvexSets::Edge*>& edges()
        const {
      return edges_;
    }

    const Subgraph* from_;
    const Subgraph* to_;
    GCSTrajectoryOptimization* gcs_;

    std::vector<geometry::optimization::GraphOfConvexSets::Edge*> edges_;

    VectorX<symbolic::Variable> u_duration_;
    VectorX<symbolic::Variable> u_vars_;
    trajectories::BezierCurve<symbolic::Expression> u_r_trajectory_;

    VectorX<symbolic::Variable> v_duration_;
    VectorX<symbolic::Variable> v_vars_;
    trajectories::BezierCurve<symbolic::Expression> v_r_trajectory_;

    friend class GCSTrajectoryOptimization;
  };

  /** Returns the number of position variables. */
  int num_positions() const { return positions_; };

  /**
  @param show_slacks determines whether the values of the intermediate
  (slack) variables are also displayed in the graph.
  @param precision sets the floating point precision (how many digits are
  generated) of the annotations.
  @param scientific sets the floating point formatting to scientific (if true)
  or fixed (if false).
  */
  std::string GetGraphvizString(bool show_slack = true, int precision = 3,
                                bool scientific = false) const {
    return gcs_.GetGraphvizString(std::nullopt, show_slack, precision,
                                  scientific);
  }

  /** Creates a Subgraph with the given regions and indices.
  @param regions represent the valid set a control point can be in.
  @param order is the order of the Bézier curve.
  @param d_max is the maximum duration spend in a region (seconds).
    Some solvers struggle numerically with large values.
  @param d_min is the minimum duration spend in a region (seconds).
    Some cost and constraints are only convex for d > 0. For example the
  perspective quadratic cost of the path energy ||ṙ(s)||² / d becomes non-convex
  for d = 0. Otherwise d_min can be set to 0.
  @name is the name of the subgraph. A default name will be provided.
  */
  Subgraph& AddRegions(const geometry::optimization::ConvexSets& regions,
                       std::vector<std::pair<int, int>>& edges_between_regions,
                       int order, double d_min = 1e-6, double d_max = 20,
                       std::string name = "");

  /** Creates a Subgraph with the given regions.
  @param regions represent the valid set a control point can be in.
  @param order is the order of the Bézier curve.
  @param d_max is the maximum duration spend in a region (seconds).
    Some solvers struggle numerically with large values.
  @param d_min is the minimum duration spend in a region (seconds).
    Some cost and constraints are only convex for d > 0. For example the
  perspective quadratic cost of the path energy ||ṙ(s)||² / d becomes non-convex
  for d = 0. Otherwise d_min can be set to 0.
  @name is the name of the subgraph. A default name will be provided.
  */
  Subgraph& AddRegions(const geometry::optimization::ConvexSets& regions,
                       int order, double d_min = 1e-6, double d_max = 20,
                       std::string name = "");

  /** Connects two subgraphs with directed edges.
  @param from is the subgraph to connect from.
  @param to is the subgraph to connect to.
  @param subspace is the subspace that the connecting control points must be in.
    Subspace is optional. Only edges that connect through the subspace will be
  added. Only subspaces of type point or HPolyhedron are supported. Otherwise
  create a subgraph of zero order with the subspace as the region and connect it
  between the two subgraphs.
  */
  SubgraphEdges& AddEdges(
      const Subgraph* from, const Subgraph* to,
      const geometry::optimization::ConvexSet* subspace = nullptr);

  /** Adds multiple L2Norm Costs on the upper bound of the path length.
  We upper bound the path integral by the sum of the distances between
  control points. For Bezier curves, this is equivalent to the sum
  of the L2Norm of the derivative control points of the curve divided by the
  order.

  @param weight_matrix is the relative weight of each component for the cost.
    The diagonal of the matrix is the weight for each dimension.
    The matrix must be square and of size num_positions() x num_positions().

  */
  void AddPathLengthCost(const Eigen::MatrixXd& weight_matrix);

  /** Adds multiple L2Norm Costs on the upper bound of the path length.
  We upper bound the path integral by the sum of the distances between
  control points. For Bezier curves, this is equivalent to the sum
  of the L2Norm of the derivative control points of the curve divided by the
  order.

  @param weight is the relative weight of the cost.
  */
  void AddPathLengthCost(double weight = 1.0);

  std::pair<trajectories::CompositeTrajectory<double>,
            solvers::MathematicalProgramResult>
  SolvePath(Subgraph& source, Subgraph& target,
            const geometry::optimization::GraphOfConvexSetsOptions& options =
                geometry::optimization::GraphOfConvexSetsOptions());

 private:
  // store the subgraphs by reference
  std::vector<std::unique_ptr<Subgraph>> subgraphs_{};
  std::vector<std::unique_ptr<SubgraphEdges>> subgraph_edges_{};

  std::vector<Eigen::MatrixXd> global_path_length_costs_{};

  int positions_;

  geometry::optimization::GraphOfConvexSets gcs_{
      geometry::optimization::GraphOfConvexSets()};
};

}  // namespace trajectory_optimization
}  // namespace planning
}  // namespace drake