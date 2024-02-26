#include "drake/planning/trajectory_optimization/gcs_trajectory_optimization.h"

#include <limits>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

#include "drake/common/pointer_cast.h"
#include "drake/common/scope_exit.h"
#include "drake/common/symbolic/decompose.h"
#include "drake/geometry/optimization/cartesian_product.h"
#include "drake/geometry/optimization/geodesic_convexity.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/intersection.h"
#include "drake/geometry/optimization/point.h"
#include "drake/math/matrix_util.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/planar_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace planning {
namespace trajectory_optimization {

using Subgraph = GcsTrajectoryOptimization::Subgraph;
using EdgesBetweenSubgraphs = GcsTrajectoryOptimization::EdgesBetweenSubgraphs;

using drake::solvers::MathematicalProgram;
using drake::solvers::Solve;
using drake::solvers::VectorXDecisionVariable;
using Eigen::MatrixXd;
using Eigen::SparseMatrix;
using Eigen::VectorXd;
using geometry::optimization::CalcPairwiseIntersections;
using geometry::optimization::CartesianProduct;
using geometry::optimization::CheckIfSatisfiesConvexityRadius;
using geometry::optimization::ConvexSet;
using geometry::optimization::ConvexSets;
using geometry::optimization::GraphOfConvexSets;
using geometry::optimization::GraphOfConvexSetsOptions;
using geometry::optimization::HPolyhedron;
using geometry::optimization::Intersection;
using geometry::optimization::PartitionConvexSet;
using geometry::optimization::Point;
using geometry::optimization::internal::ComputeOffsetContinuousRevoluteJoints;
using geometry::optimization::internal::GetMinimumAndMaximumValueAlongDimension;
using geometry::optimization::internal::ThrowsForInvalidContinuousJointsList;
using multibody::Joint;
using multibody::JointIndex;
using multibody::MultibodyPlant;
using multibody::PlanarJoint;
using multibody::RevoluteJoint;
using solvers::Binding;
using solvers::ConcatenateVariableRefList;
using solvers::Constraint;
using solvers::Cost;
using solvers::L2NormCost;
using solvers::LinearConstraint;
using solvers::LinearCost;
using solvers::LinearEqualityConstraint;
using symbolic::DecomposeLinearExpressions;
using symbolic::Expression;
using symbolic::MakeMatrixContinuousVariable;
using symbolic::MakeVectorContinuousVariable;
using trajectories::BezierCurve;
using trajectories::CompositeTrajectory;
using trajectories::Trajectory;
using Vertex = GraphOfConvexSets::Vertex;
using Edge = GraphOfConvexSets::Edge;
using VertexId = GraphOfConvexSets::VertexId;
using EdgeId = GraphOfConvexSets::EdgeId;

const double kInf = std::numeric_limits<double>::infinity();

Subgraph::Subgraph(
    const ConvexSets& regions,
    const std::vector<std::pair<int, int>>& edges_between_regions, int order,
    double h_min, double h_max, std::string name,
    GcsTrajectoryOptimization* traj_opt,
    std::optional<const std::vector<VectorXd>> edge_offsets)
    : regions_(regions),
      order_(order),
      h_min_(h_min),
      name_(std::move(name)),
      traj_opt_(*traj_opt) {
  DRAKE_THROW_UNLESS(order >= 0);
  DRAKE_THROW_UNLESS(!regions_.empty());

  if (edge_offsets.has_value()) {
    DRAKE_THROW_UNLESS(edge_offsets->size() == edges_between_regions.size());
  }

  // Make sure all regions have the same ambient dimension.
  for (const std::unique_ptr<ConvexSet>& region : regions_) {
    DRAKE_THROW_UNLESS(region != nullptr);
    DRAKE_THROW_UNLESS(region->ambient_dimension() == num_positions());
  }

  // If there are any continuous revolute joints, make sure the convexity radius
  // is respected.
  if (continuous_revolute_joints().size() > 0) {
    ThrowsForInvalidConvexityRadius();
  }

  // Make time scaling set once to avoid many allocations when adding the
  // vertices to GCS.
  const HPolyhedron time_scaling_set =
      HPolyhedron::MakeBox(Vector1d(h_min), Vector1d(h_max));

  // Add Regions with time scaling set.
  Eigen::VectorXd this_edge_offset = Eigen::VectorXd::Zero(num_positions());
  for (int i = 0; i < ssize(regions_); ++i) {
    ConvexSets vertex_set;
    // Assign each control point to a separate set.
    const int num_points = order + 1;
    vertex_set.reserve(num_points + 1);
    vertex_set.insert(vertex_set.begin(), num_points,
                      ConvexSets::value_type{*regions_[i]});
    // Add time scaling set.
    vertex_set.emplace_back(time_scaling_set);

    vertices_.emplace_back(traj_opt_.gcs_.AddVertex(
        CartesianProduct(vertex_set), fmt::format("{}: Region{}", name_, i)));
    traj_opt->vertex_to_subgraph_[vertices_.back()] = this;
  }

  r_trajectory_ =
      BezierCurve<double>(0, 1, MatrixXd::Zero(num_positions(), order + 1));

  // GetControlPoints(u).col(order) - GetControlPoints(v).col(0) = 0, via Ax =
  // 0, A = [I, -I], x = [u_controls.col(order); v_controls.col(0)].
  Eigen::SparseMatrix<double> A(num_positions(), 2 * num_positions());
  std::vector<Eigen::Triplet<double>> tripletList;
  tripletList.reserve(2 * num_positions());
  for (int i = 0; i < num_positions(); ++i) {
    tripletList.push_back(Eigen::Triplet<double>(i, i, 1.0));
    tripletList.push_back(Eigen::Triplet<double>(i, num_positions() + i, -1.0));
  }
  A.setFromTriplets(tripletList.begin(), tripletList.end());
  for (int idx = 0; idx < ssize(edges_between_regions); ++idx) {
    // Add edge.
    Vertex* u = vertices_[edges_between_regions[idx].first];
    Vertex* v = vertices_[edges_between_regions[idx].second];
    Edge* uv_edge = traj_opt_.AddEdge(u, v);

    edges_.emplace_back(uv_edge);

    // Add path continuity constraints.
    if (edge_offsets.has_value()) {
      // In this case, we instead enforce the constraint
      // GetControlPoints(u).col(order) - GetControlPoints(v).col(0) =
      // -tau_uv.value(), via Ax = -edge_offsets.value()[idx], A = [I, -I],
      // x = [u_controls.col(order); v_controls.col(0)].
      this_edge_offset = -edge_offsets->at(idx);
    }
    const auto path_continuity_constraint =
        std::make_shared<LinearEqualityConstraint>(A, this_edge_offset);
    uv_edge->AddConstraint(Binding<Constraint>(
        path_continuity_constraint,
        {GetControlPoints(*u).col(order), GetControlPoints(*v).col(0)}));
  }
}

Subgraph::~Subgraph() = default;

void Subgraph::ThrowsForInvalidConvexityRadius() const {
  for (int i = 0; i < ssize(regions_); ++i) {
    for (const int& j : continuous_revolute_joints()) {
      auto [min_value, max_value] =
          GetMinimumAndMaximumValueAlongDimension(*regions_[i], j);
      if (max_value - min_value >= M_PI) {
        throw std::runtime_error(fmt::format(
            "GcsTrajectoryOptimization: Region at index {} is wider than π "
            "along dimension {}, so it doesn't respect the convexity radius! "
            "To add this set, separate it into smaller pieces so that along "
            "dimensions corresponding to continuous revolute joints, its width "
            "is strictly smaller than π. This can be done manually, or with "
            "the helper function PartitionConvexSet.",
            i, j));
      }
    }
  }
}

void Subgraph::AddTimeCost(double weight) {
  // The time cost is the sum of duration variables ∑ hᵢ
  auto time_cost =
      std::make_shared<LinearCost>(weight * Eigen::VectorXd::Ones(1), 0.0);

  for (Vertex* v : vertices_) {
    // The duration variable is the last element of the vertex.
    v->AddCost(Binding<LinearCost>(time_cost, v->x().tail(1)));
  }
}

void Subgraph::AddPathLengthCost(const MatrixXd& weight_matrix) {
  /*
    We will upper bound the trajectory length by the sum of the distances
    between the control points. ∑ |weight_matrix * (rᵢ₊₁ − rᵢ)|₂
  */
  DRAKE_THROW_UNLESS(weight_matrix.rows() == num_positions());
  DRAKE_THROW_UNLESS(weight_matrix.cols() == num_positions());

  if (order() == 0) {
    throw std::runtime_error(
        "Path length cost is not defined for a set of order 0.");
  }

  MatrixXd A(num_positions(), 2 * num_positions());
  A << weight_matrix, -weight_matrix;
  const auto path_length_cost =
      std::make_shared<L2NormCost>(A, VectorXd::Zero(num_positions()));

  for (Vertex* v : vertices_) {
    auto control_points = GetControlPoints(*v);
    for (int i = 0; i < control_points.cols() - 1; ++i) {
      v->AddCost(Binding<L2NormCost>(
          path_length_cost,
          {control_points.col(i + 1), control_points.col(i)}));
    }
  }
}

void Subgraph::AddPathLengthCost(double weight) {
  const MatrixXd weight_matrix =
      weight * MatrixXd::Identity(num_positions(), num_positions());
  return Subgraph::AddPathLengthCost(weight_matrix);
}

void Subgraph::AddVelocityBounds(const Eigen::Ref<const VectorXd>& lb,
                                 const Eigen::Ref<const VectorXd>& ub) {
  DRAKE_THROW_UNLESS(lb.size() == num_positions());
  DRAKE_THROW_UNLESS(ub.size() == num_positions());
  if (order() == 0) {
    throw std::runtime_error(
        "Velocity Bounds are not defined for a set of order 0.");
  }

  // We have q̇(t) = drds * dsdt = ṙ(s) / h, and h >= 0, so we
  // use h * lb <= ṙ(s) <= h * ub, formulated as:
  //     0 <= ṙ(s) - h * lb <= inf,
  // - inf <= ṙ(s) - h * ub <= 0.

  // This leverages the convex hull property of the B-splines: if all of the
  // control points satisfy these convex constraints and the curve is inside
  // the convex hull of these constraints, then the curve satisfies the
  // constraints for all t.

  // The relevant derivatives of the Bezier curve come in the form:
  // rdot_control.row(i).T = M.T * r_control.row(i).T, so we loop over the
  // positions, rather than over the control points.
  const VectorXd kVecInf = VectorXd::Constant(order_, kInf);
  const VectorXd kVecZero = VectorXd::Zero(order_);
  solvers::VectorXDecisionVariable vars(order_ + 2);
  SparseMatrix<double> H_lb(
      order_ /* number of control points for one row of ṙ(s)*/,
      order_ + 2 /* number of control points for one row of r(s) + 1*/);
  H_lb.leftCols(order_ + 1) =
      r_trajectory_.AsLinearInControlPoints(1).transpose();
  SparseMatrix<double> H_ub = H_lb;
  for (int i = 0; i < num_positions(); ++i) {
    // Lower bound.  0 <= ṙ(s).row(i) - h * lb <= inf.
    H_lb.rightCols<1>() = VectorXd::Constant(order_, -lb[i]).sparseView();
    const auto lb_constraint =
        std::make_shared<LinearConstraint>(H_lb, kVecZero, kVecInf);
    // Upper bound. -inf <= ṙ(s).row(i) - h * ub <= 0.
    H_ub.rightCols<1>() = VectorXd::Constant(order_, -ub[i]).sparseView();
    const auto ub_constraint =
        std::make_shared<LinearConstraint>(H_ub, -kVecInf, kVecZero);
    for (Vertex* v : vertices_) {
      vars << GetControlPoints(*v).row(i).transpose(), GetTimeScaling(*v);
      v->AddConstraint(Binding<LinearConstraint>(lb_constraint, vars));
      v->AddConstraint(Binding<LinearConstraint>(ub_constraint, vars));
    }
  }
}

void Subgraph::AddPathContinuityConstraints(int continuity_order) {
  if (continuity_order == 0) {
    throw std::runtime_error(
        "Path continuity is enforced by default. Choose a higher order.");
  }
  if (continuity_order < 1) {
    throw std::runtime_error("Order must be greater than or equal to 1.");
  }

  if (order_ < continuity_order) {
    throw std::runtime_error(
        "Cannot add continuity constraint of order greater than the set "
        "order.");
  }

  // The continuity on derivatives of r(s) will be enforced between the last
  // control point of the u set and the first control point of the v set in an
  // edge. urdot_control.col(order-continuity_order) - vrdot_control.col(0) = 0,
  // The latter can be achieved by getting the control point matrix M.
  // A = [M.col(order - continuity_order).T, -M.col(0).T],
  // x = [u_controls.row(i); v_controls.row(i)].
  // Ax = 0,

  SparseMatrix<double> Mu_transpose =
      r_trajectory_.AsLinearInControlPoints(continuity_order)
          .col(order_ - continuity_order)
          .transpose();

  SparseMatrix<double> Mv_transpose =
      r_trajectory_.AsLinearInControlPoints(continuity_order)
          .col(0)
          .transpose();

  // Concatenate Mu_transpose and Mv_transpose.
  // A = [Mu.T, - Mv.T]

  // The A matrix will have one row since sparsity allows us to enforce the
  // continuity in each dimension. The number of columns matches the number of
  // control points for one row of r_u(s) and r_v(s).
  SparseMatrix<double> A(1, 2 * (order_ + 1));
  A.leftCols(order_ + 1) = Mu_transpose;
  A.rightCols(order_ + 1) = -Mv_transpose;

  const auto continuity_constraint =
      std::make_shared<LinearEqualityConstraint>(A, VectorXd::Zero(1));

  for (int i = 0; i < num_positions(); ++i) {
    for (Edge* edge : edges_) {
      // Add continuity constraints.
      edge->AddConstraint(Binding<LinearEqualityConstraint>(
          continuity_constraint, {GetControlPoints(edge->u()).row(i),
                                  GetControlPoints(edge->v()).row(i)}));
    }
  }
}

Eigen::Map<const MatrixX<symbolic::Variable>> Subgraph::GetControlPoints(
    const geometry::optimization::GraphOfConvexSets::Vertex& v) const {
  DRAKE_DEMAND(v.x().size() == num_positions() * (order_ + 1) + 1);
  return Eigen::Map<const MatrixX<symbolic::Variable>>(
      v.x().data(), num_positions(), order_ + 1);
}

symbolic::Variable Subgraph::GetTimeScaling(
    const geometry::optimization::GraphOfConvexSets::Vertex& v) const {
  DRAKE_DEMAND(v.x().size() == num_positions() * (order_ + 1) + 1);
  return v.x()(v.x().size() - 1);
}

EdgesBetweenSubgraphs::EdgesBetweenSubgraphs(
    const Subgraph& from_subgraph, const Subgraph& to_subgraph,
    const ConvexSet* subspace, GcsTrajectoryOptimization* traj_opt)
    : traj_opt_(*traj_opt),
      from_subgraph_(from_subgraph),
      to_subgraph_(to_subgraph) {
  // Formulate edge costs and constraints.
  if (subspace != nullptr) {
    if (subspace->ambient_dimension() != num_positions()) {
      throw std::runtime_error(
          "Subspace dimension must match the number of positions.");
    }
    if (typeid(*subspace) != typeid(Point) &&
        typeid(*subspace) != typeid(HPolyhedron)) {
      throw std::runtime_error("Subspace must be a Point or HPolyhedron.");
    }
  }

  ur_trajectory_ = BezierCurve<double>(
      0, 1, MatrixXd::Zero(num_positions(), from_subgraph_.order() + 1));
  vr_trajectory_ = BezierCurve<double>(
      0, 1, MatrixXd::Zero(num_positions(), to_subgraph_.order() + 1));

  // Zeroth order continuity constraints.
  //  ur_control.col(-1) == vr_control.col(0).
  SparseMatrix<double> A(num_positions(), 2 * num_positions());  // A = [I, -I].
  std::vector<Eigen::Triplet<double>> tripletList;
  tripletList.reserve(2 * num_positions());
  for (int i = 0; i < num_positions(); ++i) {
    tripletList.push_back(Eigen::Triplet<double>(i, i, 1.0));
    tripletList.push_back(Eigen::Triplet<double>(i, num_positions() + i, -1.0));
  }
  A.setFromTriplets(tripletList.begin(), tripletList.end());

  std::vector<VectorXd> sets_A_subspace_offset;
  if (subspace != nullptr) {
    std::vector<std::pair<double, double>> subspace_revolute_joints_bbox =
        GetMinimumAndMaximumValueAlongDimension(*subspace,
                                                continuous_revolute_joints());
    for (const auto& region : from_subgraph.regions()) {
      std::vector<std::pair<double, double>> set_revolute_joints_bbox =
          GetMinimumAndMaximumValueAlongDimension(*region,
                                                  continuous_revolute_joints());
      sets_A_subspace_offset.push_back(ComputeOffsetContinuousRevoluteJoints(
          num_positions(), continuous_revolute_joints(),
          set_revolute_joints_bbox, subspace_revolute_joints_bbox));
    }
  }

  const std::vector<std::tuple<int, int, Eigen::VectorXd>> edge_data =
      CalcPairwiseIntersections(from_subgraph.regions(), to_subgraph.regions(),
                                continuous_revolute_joints());
  for (const auto& edge : edge_data) {
    int i = std::get<0>(edge);
    int j = std::get<1>(edge);
    Eigen::VectorXd edge_offset = std::get<2>(edge);

    // Check if the overlap between the sets is contained in the subspace.
    if (subspace != nullptr) {
      Eigen::VectorXd subspace_offset = sets_A_subspace_offset[i];
      if (!RegionsConnectThroughSubspace(*from_subgraph.regions()[i],
                                         *to_subgraph.regions()[j], *subspace,
                                         edge_offset, subspace_offset)) {
        continue;
      }
    }

    // Add edge.
    Vertex* u = from_subgraph.vertices_[i];
    Vertex* v = to_subgraph.vertices_[j];
    Edge* uv_edge = traj_opt_.AddEdge(u, v);
    edges_.emplace_back(uv_edge);

    // Add path continuity constraints. We instead enforce the constraint
    // u - v = -tau_uv, via Ax = -edge_offset, A = [I, -I],
    // x = [u_controls.col(order); v_controls.col(0)].
    const auto path_continuity_constraint =
        std::make_shared<LinearEqualityConstraint>(A, -edge_offset);
    uv_edge->AddConstraint(Binding<Constraint>(
        path_continuity_constraint,
        ConcatenateVariableRefList(
            {GetControlPointsU(*uv_edge).col(from_subgraph_.order()),
             GetControlPointsV(*uv_edge).col(0)})));

    if (subspace != nullptr) {
      // Add subspace constraints to the last control point of the u vertex.
      const auto vars = GetControlPointsU(*uv_edge).col(from_subgraph_.order());

      // subspace will either be a Point or HPolyhedron. We check if it's a
      // Point via the method ConvexSet::MaybeGetPoint(), which will return a
      // value for Point, and won't return a value for HPolyhedron.
      std::optional<VectorXd> subspace_point = subspace->MaybeGetPoint();
      if (subspace_point.has_value()) {
        // Encode u = subspace_point + subspace_offset via the linear equality
        // constraint [I][u] = [subspace_point + subspace_offset].
        const auto subgraph_constraint =
            std::make_shared<LinearEqualityConstraint>(
                Eigen::MatrixXd::Identity(num_positions(), num_positions()),
                subspace_point.value() + sets_A_subspace_offset[i]);
        uv_edge->AddConstraint(Binding<Constraint>(subgraph_constraint, vars));
      } else if (typeid(*subspace) == typeid(HPolyhedron)) {
        const HPolyhedron* hpoly = dynamic_cast<const HPolyhedron*>(subspace);
        const Eigen::MatrixXd& hpoly_A = hpoly->A();
        const Eigen::VectorXd& hpoly_b = hpoly->b();
        solvers::MathematicalProgram prog;
        const VectorX<symbolic::Variable> x =
            prog.NewContinuousVariables(num_positions(), "x");
        // To translate the HPolyhedron, rewrite A*(x+offset)<=b as
        // Ax<=b-A*offset.
        prog.AddLinearConstraint(
            hpoly_A,
            Eigen::VectorXd::Constant(hpoly_b.size(),
                                      std::numeric_limits<double>::infinity()),
            hpoly_b - (hpoly_A * sets_A_subspace_offset[i]), x);
        for (const auto& binding : prog.GetAllConstraints()) {
          const std::shared_ptr<Constraint>& constraint = binding.evaluator();
          uv_edge->AddConstraint(Binding<Constraint>(constraint, vars));
        }
      }
    }
  }
}

EdgesBetweenSubgraphs::~EdgesBetweenSubgraphs() = default;

bool EdgesBetweenSubgraphs::RegionsConnectThroughSubspace(
    const ConvexSet& A, const ConvexSet& B, const ConvexSet& subspace,
    std::optional<const VectorXd> maybe_set_B_offset,
    std::optional<const VectorXd> maybe_subspace_offset) {
  DRAKE_THROW_UNLESS(A.ambient_dimension() > 0);
  DRAKE_THROW_UNLESS(A.ambient_dimension() == B.ambient_dimension());
  DRAKE_THROW_UNLESS(A.ambient_dimension() == subspace.ambient_dimension());
  DRAKE_THROW_UNLESS(maybe_set_B_offset.has_value() ==
                     maybe_subspace_offset.has_value());
  if (maybe_set_B_offset.has_value()) {
    DRAKE_THROW_UNLESS(maybe_set_B_offset.value().size() ==
                       A.ambient_dimension());
    DRAKE_THROW_UNLESS(maybe_subspace_offset.value().size() ==
                       A.ambient_dimension());
  }
  if (std::optional<VectorXd> subspace_point = subspace.MaybeGetPoint()) {
    // If the subspace is a point, then the point must be in both A and B.
    if (maybe_set_B_offset.has_value()) {
      // Compute the value of the point such that it can be directly checked for
      // containment in A. We take its given value, and subtract the offset from
      // the set to the subspace.
      const VectorXd point_in_A_coords =
          *subspace_point - maybe_subspace_offset.value();
      // Compute the value of the point such that it can be directly checked for
      // containment in B. We take the value of the point that can be checked
      // for A, and add the offset form A to B.
      const VectorXd point_in_B_coords =
          point_in_A_coords + maybe_set_B_offset.value();
      return A.PointInSet(point_in_A_coords) && B.PointInSet(point_in_B_coords);
    } else {
      return A.PointInSet(*subspace_point) && B.PointInSet(*subspace_point);
    }
  } else if (!maybe_set_B_offset.has_value()) {
    // Otherwise, we can formulate a problem to check if a point is contained in
    // A, B and the subspace.
    Intersection intersection(MakeConvexSets(A, B, subspace));
    return !intersection.IsEmpty();
  } else {
    // The program has to be different if there's an offset.
    const int dimension = A.ambient_dimension();
    MathematicalProgram prog;
    VectorXDecisionVariable x = prog.NewContinuousVariables(dimension);
    VectorXDecisionVariable y = prog.NewContinuousVariables(dimension);
    VectorXDecisionVariable z = prog.NewContinuousVariables(dimension);
    A.AddPointInSetConstraints(&prog, x);
    B.AddPointInSetConstraints(&prog, y);
    subspace.AddPointInSetConstraints(&prog, z);

    MatrixXd equality_constraint_matrix(dimension, 2 * dimension);
    equality_constraint_matrix.leftCols(dimension) =
        MatrixXd::Identity(dimension, dimension);
    equality_constraint_matrix.rightCols(dimension) =
        -MatrixXd::Identity(dimension, dimension);
    // y = x + B_offset as [I, -I][y; x] = [B_offset].
    prog.AddLinearEqualityConstraint(equality_constraint_matrix,
                                     maybe_set_B_offset.value(), {x, y});
    // z = x + subspace_offset as [I, -I][z; x] = [subspace_offset]
    prog.AddLinearEqualityConstraint(equality_constraint_matrix,
                                     maybe_subspace_offset.value(), {x, z});

    const auto result = Solve(prog);
    return result.is_success();
  }
}

void EdgesBetweenSubgraphs::AddVelocityBounds(
    const Eigen::Ref<const VectorXd>& lb,
    const Eigen::Ref<const VectorXd>& ub) {
  DRAKE_THROW_UNLESS(lb.size() == num_positions());
  DRAKE_THROW_UNLESS(ub.size() == num_positions());

  // We have q̇(t) = drds * dsdt = ṙ(s) / h, and h >= 0, so we
  // use h * lb <= ṙ(s) <= h * ub, formulated as:
  //     0 <= ṙ(s) - h * lb <= inf,
  // - inf <= ṙ(s) - h * ub <= 0.

  // We will enforce the velocity bounds on the last control point of the u set
  // and on the first control point of the v set unless one of the sets are of
  // order zero. In the zero order case, velocity doesn't matter since its a
  // point.

  const Vector1d kVecInf = Vector1d::Constant(kInf);
  const Vector1d kVecZero = Vector1d::Zero();

  if (from_subgraph_.order() == 0 && to_subgraph_.order() == 0) {
    throw std::runtime_error(
        "Cannot add velocity bounds to a subgraph edges where both subgraphs "
        "have zero order.");
  }

  if (from_subgraph_.order() > 0) {
    // Add velocity bounds to the last control point of the u set.
    // See BezierCurve::AsLinearInControlPoints().
    solvers::VectorXDecisionVariable vars(from_subgraph_.order() + 2);
    SparseMatrix<double> m = ur_trajectory_.AsLinearInControlPoints(1)
                                 .col(from_subgraph_.order() - 1)
                                 .transpose();
    SparseMatrix<double> H_lb(
        1 /* we are only constraining the last point in the u set */,
        from_subgraph_.order() +
            2 /* number of control points for one row of r(s) + 1*/);
    H_lb.leftCols(from_subgraph_.order() + 1) = m;
    SparseMatrix<double> H_ub = H_lb;
    for (int i = 0; i < num_positions(); ++i) {
      // Lower bound.  0 <= ṙ(s).row(i) - h * lb <= inf.
      H_lb.coeffRef(0, from_subgraph_.order() + 1) = -lb[i];
      const auto lb_constraint =
          std::make_shared<LinearConstraint>(H_lb, kVecZero, kVecInf);
      // Upper bound. -inf <= ṙ(s).row(i) - h * ub <= 0.
      H_ub.coeffRef(0, from_subgraph_.order() + 1) = -ub[i];
      const auto ub_constraint =
          std::make_shared<LinearConstraint>(H_ub, -kVecInf, kVecZero);
      for (Edge* edge : edges_) {
        // vars = [control_points.row(i).T; time_scaling]
        vars << GetControlPointsU(*edge).row(i).transpose(),
            GetTimeScalingU(*edge);
        edge->AddConstraint(Binding<LinearConstraint>(lb_constraint, vars));
        edge->AddConstraint(Binding<LinearConstraint>(ub_constraint, vars));
      }
    }
  }

  if (to_subgraph_.order() > 0) {
    // Add velocity bounds to the first control point of the v set.
    // See Subgraph::AddVelocityBounds().
    solvers::VectorXDecisionVariable vars(to_subgraph_.order() + 2);
    SparseMatrix<double> m =
        vr_trajectory_.AsLinearInControlPoints(1).col(0).transpose();
    SparseMatrix<double> H_lb(
        1 /* we are only constraining the last point in the u set */,
        to_subgraph_.order() +
            2 /* number of control points for one row of r(s) + 1*/);
    H_lb.leftCols(to_subgraph_.order() + 1) = m;
    SparseMatrix<double> H_ub = H_lb;
    for (int i = 0; i < num_positions(); ++i) {
      // Lower bound.  0 <= ṙ(s).row(i) - h * lb <= inf.
      H_lb.coeffRef(0, to_subgraph_.order() + 1) = -lb[i];
      const auto lb_constraint =
          std::make_shared<LinearConstraint>(H_lb, kVecZero, kVecInf);
      // Upper bound. -inf <= ṙ(s).row(i) - h * ub <= 0.
      H_ub.coeffRef(0, to_subgraph_.order() + 1) = -ub[i];
      const auto ub_constraint =
          std::make_shared<LinearConstraint>(H_ub, -kVecInf, kVecZero);
      for (Edge* edge : edges_) {
        // vars = [control_points.row(i).T; time_scaling]
        vars << GetControlPointsV(*edge).row(i).transpose(),
            GetTimeScalingV(*edge);
        edge->AddConstraint(Binding<LinearConstraint>(lb_constraint, vars));
        edge->AddConstraint(Binding<LinearConstraint>(ub_constraint, vars));
      }
    }
  }
}

void EdgesBetweenSubgraphs::AddPathContinuityConstraints(int continuity_order) {
  if (continuity_order == 0) {
    throw std::runtime_error(
        "Path continuity is enforced by default. Choose a higher order.");
  }
  if (continuity_order < 1) {
    throw std::runtime_error("Order must be greater than or equal to 1.");
  }

  if (from_subgraph_.order() < continuity_order ||
      to_subgraph_.order() < continuity_order) {
    throw std::runtime_error(
        "Cannot add continuity constraint to a subgraph edge where both "
        "subgraphs order are not greater than or equal to the requested "
        "continuity order.");
  }

  // The continuity on derivatives of r(s) will be enforced between the last
  // control point of the u set and the first control point of the v set in an
  // edge. urdot_control.col(order-continuity_order) - vrdot_control.col(0) = 0,
  // The latter can be achieved by getting the control point matrix M.
  // A = [M.col(from_subgraph.order() - continuity_order).T, -M.col(0).T],
  // x = [u_controls.row(i); v_controls.row(i)].
  // Ax = 0,

  SparseMatrix<double> Mu_transpose =
      ur_trajectory_.AsLinearInControlPoints(continuity_order)
          .col(from_subgraph_.order() - continuity_order)
          .transpose();

  SparseMatrix<double> Mv_transpose =
      vr_trajectory_.AsLinearInControlPoints(continuity_order)
          .col(0)
          .transpose();

  // Concatenate Mu_transpose and Mv_transpose.
  // A = [Mu.T, - Mv.T]

  // The A matrix will have one row since sparsity allows us to enforce the
  // continuity in each dimension. The number of columns matches the number of
  // control points for one row of r_u(s) and r_v(s).
  SparseMatrix<double> A(
      1, (from_subgraph_.order() + 1) + (to_subgraph_.order() + 1));
  A.leftCols(from_subgraph_.order() + 1) = Mu_transpose;
  A.rightCols(to_subgraph_.order() + 1) = -Mv_transpose;

  const auto continuity_constraint =
      std::make_shared<LinearEqualityConstraint>(A, VectorXd::Zero(1));

  for (int i = 0; i < num_positions(); ++i) {
    for (Edge* edge : edges_) {
      // Add continuity constraints.
      edge->AddConstraint(Binding<LinearEqualityConstraint>(
          continuity_constraint,
          {GetControlPointsU(*edge).row(i), GetControlPointsV(*edge).row(i)}));
    }
  }
}

Eigen::Map<const MatrixX<symbolic::Variable>>
EdgesBetweenSubgraphs::GetControlPointsU(
    const geometry::optimization::GraphOfConvexSets::Edge& e) const {
  DRAKE_DEMAND(e.xu().size() ==
               num_positions() * (from_subgraph_.order() + 1) + 1);
  return Eigen::Map<const MatrixX<symbolic::Variable>>(
      e.xu().data(), num_positions(), from_subgraph_.order() + 1);
}

Eigen::Map<const MatrixX<symbolic::Variable>>
EdgesBetweenSubgraphs::GetControlPointsV(
    const geometry::optimization::GraphOfConvexSets::Edge& e) const {
  DRAKE_DEMAND(e.xv().size() ==
               num_positions() * (to_subgraph_.order() + 1) + 1);
  return Eigen::Map<const MatrixX<symbolic::Variable>>(
      e.xv().data(), num_positions(), to_subgraph_.order() + 1);
}

symbolic::Variable EdgesBetweenSubgraphs::GetTimeScalingU(
    const geometry::optimization::GraphOfConvexSets::Edge& e) const {
  DRAKE_DEMAND(e.xu().size() ==
               num_positions() * (from_subgraph_.order() + 1) + 1);
  return e.xu()(e.xu().size() - 1);
}

symbolic::Variable EdgesBetweenSubgraphs::GetTimeScalingV(
    const geometry::optimization::GraphOfConvexSets::Edge& e) const {
  DRAKE_DEMAND(e.xv().size() ==
               num_positions() * (to_subgraph_.order() + 1) + 1);
  return e.xv()(e.xv().size() - 1);
}

GcsTrajectoryOptimization::GcsTrajectoryOptimization(
    int num_positions, std::vector<int> continuous_revolute_joints)
    : num_positions_(num_positions),
      continuous_revolute_joints_(std::move(continuous_revolute_joints)) {
  DRAKE_THROW_UNLESS(num_positions >= 1);
  ThrowsForInvalidContinuousJointsList(num_positions,
                                       continuous_revolute_joints_);
}

GcsTrajectoryOptimization::~GcsTrajectoryOptimization() = default;

Subgraph& GcsTrajectoryOptimization::AddRegions(
    const ConvexSets& regions,
    const std::vector<std::pair<int, int>>& edges_between_regions, int order,
    double h_min, double h_max, std::string name,
    std::optional<const std::vector<VectorXd>> edge_offsets) {
  if (edge_offsets.has_value()) {
    DRAKE_THROW_UNLESS(edge_offsets->size() == edges_between_regions.size());
  }
  if (name.empty()) {
    name = fmt::format("Subgraph{}", subgraphs_.size());
  }
  Subgraph* subgraph =
      new Subgraph(regions, edges_between_regions, order, h_min, h_max,
                   std::move(name), this, edge_offsets);

  // Add global costs to the subgraph.
  for (double weight : global_time_costs_) {
    subgraph->AddTimeCost(weight);
  }

  if (order > 0) {
    // These costs and constraints rely on the derivative of the trajectory.
    for (const MatrixXd& weight_matrix : global_path_length_costs_) {
      subgraph->AddPathLengthCost(weight_matrix);
    }
    for (const auto& [lb, ub] : global_velocity_bounds_) {
      subgraph->AddVelocityBounds(lb, ub);
    }
  }

  // Add global continuity constraints to the subgraph.
  for (int continuity_order : global_continuity_constraints_) {
    if (order >= continuity_order) {
      subgraph->AddPathContinuityConstraints(continuity_order);
    }
  }

  return *subgraphs_.emplace_back(subgraph);
}

Subgraph& GcsTrajectoryOptimization::AddRegions(const ConvexSets& regions,
                                                int order, double h_min,
                                                double h_max,
                                                std::string name) {
  // TODO(wrangelvid): This is O(n^2) and can be improved.
  DRAKE_DEMAND(regions.size() > 0);

  const std::vector<std::tuple<int, int, Eigen::VectorXd>> edge_data =
      CalcPairwiseIntersections(regions, continuous_revolute_joints());

  std::vector<std::pair<int, int>> edges_between_regions;
  std::vector<Eigen::VectorXd> edge_offsets;
  edges_between_regions.reserve(edge_data.size());
  edge_offsets.reserve(edge_data.size());
  for (int i = 0; i < ssize(edge_data); ++i) {
    edges_between_regions.emplace_back(std::get<0>(edge_data[i]),
                                       std::get<1>(edge_data[i]));
    edge_offsets.emplace_back(std::get<2>(edge_data[i]));
  }

  return GcsTrajectoryOptimization::AddRegions(regions, edges_between_regions,
                                               order, h_min, h_max,
                                               std::move(name), edge_offsets);
}

void GcsTrajectoryOptimization::RemoveSubgraph(const Subgraph& subgraph) {
  // Check if the subgraph is in the list of subgraphs.
  if (!std::any_of(subgraphs_.begin(), subgraphs_.end(),
                   [&](const std::unique_ptr<Subgraph>& s) {
                     return s.get() == &subgraph;
                   })) {
    throw std::runtime_error(fmt::format(
        "Subgraph {} is not registered with `this`", subgraph.name()));
  }

  // Remove the underlying edges between subgraphs from the gcs problem.
  for (const std::unique_ptr<EdgesBetweenSubgraphs>& subgraph_edge :
       subgraph_edges_) {
    if (&subgraph_edge->from_subgraph_ == &subgraph ||
        &subgraph_edge->to_subgraph_ == &subgraph) {
      for (Edge* edge : subgraph_edge->edges_) {
        gcs_.RemoveEdge(edge);
      }
    }
  }

  // Remove the edges between subgraphs objects.
  subgraph_edges_.erase(
      std::remove_if(subgraph_edges_.begin(), subgraph_edges_.end(),
                     [&](const std::unique_ptr<EdgesBetweenSubgraphs>& e) {
                       return &e->from_subgraph_ == &subgraph ||
                              &e->to_subgraph_ == &subgraph;
                     }),
      subgraph_edges_.end());

  // Remove all vertices in the subgraph.
  for (Vertex* v : subgraph.vertices_) {
    // This will also remove all edges connected to the vertex.
    gcs_.RemoveVertex(v);
  }

  // Remove the subgraph from the list of subgraphs.
  subgraphs_.erase(std::remove_if(subgraphs_.begin(), subgraphs_.end(),
                                  [&](const std::unique_ptr<Subgraph>& s) {
                                    return s.get() == &subgraph;
                                  }),
                   subgraphs_.end());
}

EdgesBetweenSubgraphs& GcsTrajectoryOptimization::AddEdges(
    const Subgraph& from_subgraph, const Subgraph& to_subgraph,
    const ConvexSet* subspace) {
  EdgesBetweenSubgraphs* subgraph_edge =
      new EdgesBetweenSubgraphs(from_subgraph, to_subgraph, subspace, this);

  // Add global continuity constraints to the edges between subgraphs.
  for (int continuity_order : global_continuity_constraints_) {
    if (subgraph_edge->from_subgraph_.order() >= continuity_order &&
        subgraph_edge->to_subgraph_.order() >= continuity_order) {
      subgraph_edge->AddPathContinuityConstraints(continuity_order);
    }
  }

  return *subgraph_edges_.emplace_back(subgraph_edge);
}

void GcsTrajectoryOptimization::AddTimeCost(double weight) {
  // Add time cost to each subgraph.
  for (std::unique_ptr<Subgraph>& subgraph : subgraphs_) {
    subgraph->AddTimeCost(weight);
  }
  global_time_costs_.push_back(weight);
}

void GcsTrajectoryOptimization::AddPathLengthCost(
    const MatrixXd& weight_matrix) {
  // Add path length cost to each subgraph.
  for (std::unique_ptr<Subgraph>& subgraph : subgraphs_) {
    if (subgraph->order() > 0) {
      subgraph->AddPathLengthCost(weight_matrix);
    }
  }
  global_path_length_costs_.push_back(weight_matrix);
}

void GcsTrajectoryOptimization::AddPathLengthCost(double weight) {
  const MatrixXd weight_matrix =
      weight * MatrixXd::Identity(num_positions(), num_positions());
  return GcsTrajectoryOptimization::AddPathLengthCost(weight_matrix);
}

void GcsTrajectoryOptimization::AddVelocityBounds(
    const Eigen::Ref<const VectorXd>& lb,
    const Eigen::Ref<const VectorXd>& ub) {
  DRAKE_THROW_UNLESS(lb.size() == num_positions());
  DRAKE_THROW_UNLESS(ub.size() == num_positions());
  // Add path velocity bounds to each subgraph.
  for (std::unique_ptr<Subgraph>& subgraph : subgraphs_) {
    if (subgraph->order() > 0) {
      subgraph->AddVelocityBounds(lb, ub);
    }
  }
  global_velocity_bounds_.push_back({lb, ub});
}

void GcsTrajectoryOptimization::AddPathContinuityConstraints(
    int continuity_order) {
  if (continuity_order == 0) {
    throw std::runtime_error(
        "Path continuity is enforced by default. Choose a higher order.");
  }
  if (continuity_order < 1) {
    throw std::runtime_error("Order must be greater than or equal to 1.");
  }
  // Add continuity constraints to each subgraph.
  for (std::unique_ptr<Subgraph>& subgraph : subgraphs_) {
    if (subgraph->order() >= continuity_order) {
      subgraph->AddPathContinuityConstraints(continuity_order);
    }
  }
  // Add continuity constraints to the edges between subgraphs.
  for (std::unique_ptr<EdgesBetweenSubgraphs>& subgraph_edge :
       subgraph_edges_) {
    if (subgraph_edge->from_subgraph_.order() >= continuity_order &&
        subgraph_edge->to_subgraph_.order() >= continuity_order) {
      subgraph_edge->AddPathContinuityConstraints(continuity_order);
    }
  }

  global_continuity_constraints_.push_back(continuity_order);
}

std::pair<CompositeTrajectory<double>, solvers::MathematicalProgramResult>
GcsTrajectoryOptimization::SolvePath(
    const Subgraph& source, const Subgraph& target,
    const GraphOfConvexSetsOptions& specified_options) {
  // Fill in default options. Note: if these options change, they must also be
  // updated in the method documentation.
  GraphOfConvexSetsOptions options = specified_options;
  if (!options.convex_relaxation) {
    options.convex_relaxation = true;
  }
  if (!options.preprocessing) {
    options.preprocessing = true;
  }
  if (!options.max_rounded_paths) {
    options.max_rounded_paths = 5;
  }

  const VectorXd empty_vector;

  Vertex* source_vertex = source.vertices_[0];
  Vertex* dummy_source = nullptr;

  Vertex* target_vertex = target.vertices_[0];
  Vertex* dummy_target = nullptr;

  if (source.size() != 1) {
    // Source subgraph has more than one region. Add a dummy source vertex.
    dummy_source = gcs_.AddVertex(Point(empty_vector), "Dummy source");
    source_vertex = dummy_source;
    for (Vertex* v : source.vertices_) {
      AddEdge(dummy_source, v);
    }
  }
  const ScopeExit cleanup_dummy_source_before_returning([&]() {
    if (dummy_source != nullptr) {
      gcs_.RemoveVertex(dummy_source);
    }
  });

  if (target.size() != 1) {
    // Target subgraph has more than one region. Add a dummy target vertex.
    dummy_target = gcs_.AddVertex(Point(empty_vector), "Dummy target");
    target_vertex = dummy_target;
    for (Vertex* v : target.vertices_) {
      AddEdge(v, dummy_target);
    }
  }
  const ScopeExit cleanup_dummy_target_before_returning([&]() {
    if (dummy_target != nullptr) {
      gcs_.RemoveVertex(dummy_target);
    }
  });

  solvers::MathematicalProgramResult result =
      gcs_.SolveShortestPath(*source_vertex, *target_vertex, options);
  if (!result.is_success()) {
    return {CompositeTrajectory<double>({}), result};
  }

  const double kTolerance = 1.0;  // take any path we can get.
  std::vector<const Edge*> path_edges =
      gcs_.GetSolutionPath(*source_vertex, *target_vertex, result, kTolerance);

  // Remove the dummy edges from the path.
  if (dummy_source != nullptr) {
    DRAKE_DEMAND(!path_edges.empty());
    path_edges.erase(path_edges.begin());
  }
  if (dummy_target != nullptr) {
    DRAKE_DEMAND(!path_edges.empty());
    path_edges.erase(path_edges.end() - 1);
  }

  return {ReconstructTrajectoryFromSolutionPath(path_edges, result), result};
}

std::pair<trajectories::CompositeTrajectory<double>,
          solvers::MathematicalProgramResult>
GcsTrajectoryOptimization::SolveConvexRestriction(
    const std::vector<const Vertex*>& active_vertices,
    const GraphOfConvexSetsOptions& options) {
  DRAKE_DEMAND(active_vertices.size() > 1);

  std::vector<const Edge*> active_edges;
  for (size_t i = 0; i < active_vertices.size() - 1; ++i) {
    bool vertices_connected = false;
    for (const Edge* e : active_vertices[i]->outgoing_edges()) {
      if (e->v().id() == active_vertices[i + 1]->id()) {
        if (vertices_connected) {
          throw std::runtime_error(fmt::format(
              "Vertex: {} is connected to vertex: {} through multiple edges.",
              active_vertices[i]->name(), active_vertices[i + 1]->name()));
        }
        active_edges.push_back(e);
        vertices_connected = true;
      }
    }

    if (!vertices_connected) {
      throw std::runtime_error(fmt::format(
          "Vertex: {} is not connected to vertex: {}.",
          active_vertices[i]->name(), active_vertices[i + 1]->name()));
    }
  }

  solvers::MathematicalProgramResult result =
      gcs_.SolveConvexRestriction(active_edges, options);
  if (!result.is_success()) {
    return {CompositeTrajectory<double>({}), result};
  }

  return {ReconstructTrajectoryFromSolutionPath(active_edges, result), result};
}

CompositeTrajectory<double>
GcsTrajectoryOptimization::ReconstructTrajectoryFromSolutionPath(
    std::vector<const Edge*> edges,
    const solvers::MathematicalProgramResult& result) {
  // Extract the path from the edges.
  std::vector<copyable_unique_ptr<Trajectory<double>>> bezier_curves;
  for (const Edge* e : edges) {
    // Extract phi from the solution to rescale the control points and duration
    // in case we get the relaxed solution.
    const double phi_inv = 1 / result.GetSolution(e->phi());
    // Extract the control points from the solution.
    const int num_control_points = vertex_to_subgraph_[&e->u()]->order() + 1;
    const MatrixX<double> edge_path_points =
        phi_inv *
        Eigen::Map<MatrixX<double>>(result.GetSolution(e->xu()).data(),
                                    num_positions(), num_control_points);

    // Extract the duration from the solution.
    double h = phi_inv * result.GetSolution(e->xu()).tail<1>().value();
    const double start_time =
        bezier_curves.empty() ? 0 : bezier_curves.back()->end_time();

    // Skip edges with a single control point that spend near zero time in the
    // region, since zero order continuity constraint is sufficient. These edges
    // would result in a discontinuous trajectory for velocities and higher
    // derivatives.
    if (!(num_control_points == 1 &&
          vertex_to_subgraph_[&e->u()]->h_min_ == 0)) {
      bezier_curves.emplace_back(std::make_unique<BezierCurve<double>>(
          start_time, start_time + h, edge_path_points));
    }
  }

  // Get the final control points from the solution.
  const Edge& last_edge = *edges.back();
  const double phi_inv = 1 / result.GetSolution(last_edge.phi());
  const int num_control_points =
      vertex_to_subgraph_[&last_edge.v()]->order() + 1;
  const MatrixX<double> edge_path_points =
      phi_inv *
      Eigen::Map<MatrixX<double>>(result.GetSolution(last_edge.xv()).data(),
                                  num_positions(), num_control_points);

  double h = phi_inv * result.GetSolution(last_edge.xv()).tail<1>().value();
  const double start_time =
      bezier_curves.empty() ? 0 : bezier_curves.back()->end_time();

  // Skip edges with a single control point that spend near zero time in the
  // region, since zero order continuity constraint is sufficient.
  if (!(num_control_points == 1 &&
        vertex_to_subgraph_[&last_edge.v()]->h_min_ == 0)) {
    bezier_curves.emplace_back(std::make_unique<BezierCurve<double>>(
        start_time, start_time + h, edge_path_points));
  }
  return CompositeTrajectory<double>(bezier_curves);
}

Edge* GcsTrajectoryOptimization::AddEdge(Vertex* u, Vertex* v) {
  return gcs_.AddEdge(u, v, fmt::format("{} -> {}", u->name(), v->name()));
}

double GcsTrajectoryOptimization::EstimateComplexity() const {
  double result = 0;
  // TODO(ggould) A more correct computation estimate would be:
  // If each vertex and edge problem were solved only once, the cost would be
  // | constraint_var_sum = variables per constraint summed over constraints
  // | cost_vars = total unique variables over all costs
  // | constraint_vars = total unique variables over all constraints
  // : constraint_var_sum * cost_vars * constraint_vars^2
  // In fact each vertex must be solved at least as many times as it has
  // edges, so multiply the vertex cost by the vertex's arity.
  for (const auto* v : gcs_.Vertices()) {
    for (const auto& c : v->GetCosts()) {
      result += c.GetNumElements();
    }
    for (const auto& c : v->GetConstraints()) {
      result += c.GetNumElements();
    }
  }
  for (const auto* e : gcs_.Edges()) {
    for (const auto& c : e->GetCosts()) {
      result += c.GetNumElements();
    }
    for (const auto& c : e->GetConstraints()) {
      result += c.GetNumElements();
    }
  }
  return result;
}

std::vector<Subgraph*> GcsTrajectoryOptimization::GetSubgraphs() const {
  std::vector<Subgraph*> subgraphs;
  subgraphs.reserve(subgraphs_.size());
  for (const auto& subgraph : subgraphs_) {
    subgraphs.push_back(subgraph.get());
  }
  return subgraphs;
}

std::vector<EdgesBetweenSubgraphs*>
GcsTrajectoryOptimization::GetEdgesBetweenSubgraphs() const {
  std::vector<EdgesBetweenSubgraphs*> subgraph_edges;
  subgraph_edges.reserve(subgraph_edges_.size());
  for (const auto& edge : subgraph_edges_) {
    subgraph_edges.push_back(edge.get());
  }
  return subgraph_edges;
}

trajectories::CompositeTrajectory<double>
GcsTrajectoryOptimization::NormalizeSegmentTimes(
    const trajectories::CompositeTrajectory<double>& trajectory) {
  std::vector<copyable_unique_ptr<Trajectory<double>>> normalized_bezier_curves;

  double start_time = trajectory.start_time();
  for (int i = 0; i < trajectory.get_number_of_segments(); ++i) {
    // Create a new BezierCurve with the same control points, but with a
    // duration of one second.
    if (const BezierCurve<double>* gcs_segment =
            dynamic_cast<const BezierCurve<double>*>(&trajectory.segment(i))) {
      normalized_bezier_curves.emplace_back(
          std::make_unique<BezierCurve<double>>(start_time, start_time + 1.0,
                                                gcs_segment->control_points()));
      start_time += 1.0;
    } else {
      throw std::runtime_error(
          "All segments in the gcs trajectory must be of type "
          "BezierCurve<double>.");
    }
  }
  return CompositeTrajectory<double>(normalized_bezier_curves);
}

std::vector<int> GetContinuousRevoluteJointIndices(
    const multibody::MultibodyPlant<double>& plant) {
  std::vector<int> indices;
  for (int i = 0; i < plant.num_joints(); ++i) {
    const Joint<double>& joint = plant.get_joint(JointIndex(i));
    // The first possibility we check for is a revolute joint with no joint
    // limits.
    if (joint.type_name() == "revolute") {
      if (joint.position_lower_limits()[0] ==
              -std::numeric_limits<float>::infinity() &&
          joint.position_upper_limits()[0] ==
              std::numeric_limits<float>::infinity()) {
        indices.push_back(joint.position_start());
      }
      continue;
    }
    // The second possibility we check for is a planar joint. If it is (and the
    // angle component has no joint limits), we only add the third entry of the
    // position vector, corresponding to theta.
    if (joint.type_name() == "planar") {
      if (joint.position_lower_limits()[2] ==
              -std::numeric_limits<float>::infinity() &&
          joint.position_upper_limits()[2] ==
              std::numeric_limits<float>::infinity()) {
        indices.push_back(joint.position_start() + 2);
      }
      continue;
    }
    // TODO(cohnt): Determine if other joint types (e.g. UniversalJoint) can be
    // handled appropriately with wraparound edges, and if so, return their
    // indices as well.
  }
  return indices;
}

}  // namespace trajectory_optimization
}  // namespace planning
}  // namespace drake
