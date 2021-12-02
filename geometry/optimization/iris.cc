#include "drake/geometry/optimization/iris.h"

#include <algorithm>
#include <limits>
#include <optional>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/geometry/optimization/cartesian_product.h"
#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/minkowski_sum.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/ibex_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/snopt_solver.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::MatrixXd;
using Eigen::Ref;
using Eigen::Vector3d;
using Eigen::VectorXd;
using math::RigidTransform;
using multibody::Body;
using multibody::Frame;
using multibody::JacobianWrtVariable;
using multibody::MultibodyPlant;
using symbolic::Expression;
using systems::Context;

HPolyhedron Iris(const ConvexSets& obstacles, const Ref<const VectorXd>& sample,
                 const HPolyhedron& domain, const IrisOptions& options) {
  const int dim = sample.size();
  const int N = obstacles.size();
  DRAKE_DEMAND(domain.ambient_dimension() == dim);
  for (int i = 0; i < N; ++i) {
    DRAKE_DEMAND(obstacles[i]->ambient_dimension() == dim);
  }
  DRAKE_DEMAND(domain.IsBounded());
  const double kEpsilonEllipsoid = 1e-2;
  Hyperellipsoid E = Hyperellipsoid::MakeHypersphere(kEpsilonEllipsoid, sample);
  HPolyhedron P = domain;

  // On each iteration, we will build the collision-free polytope represented as
  // {x | A * x <= b}.  Here we pre-allocate matrices of the maximum size.
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(
      domain.A().rows() + N, dim);
  VectorXd b(domain.A().rows() + N);
  A.topRows(domain.A().rows()) = domain.A();
  b.head(domain.A().rows()) = domain.b();
  // Use pairs {scale, index}, so that I can back out the indices after a sort.
  std::vector<std::pair<double, int>> scaling(N);
  MatrixXd closest_points(dim, N);

  double best_volume = E.Volume();
  int iteration = 0;
  MatrixXd tangent_matrix;

  while (true) {
    DRAKE_ASSERT(best_volume > 0);
    // Find separating hyperplanes
    for (int i = 0; i < N; ++i) {
      const auto touch = E.MinimumUniformScalingToTouch(*obstacles[i]);
      scaling[i].first = touch.first;
      scaling[i].second = i;
      closest_points.col(i) = touch.second;
    }
    std::sort(scaling.begin(), scaling.end());

    int num_constraints = domain.A().rows();
    tangent_matrix = 2.0 * E.A().transpose() * E.A();
    for (int i = 0; i < N; ++i) {
      const VectorXd point = closest_points.col(scaling[i].second);
      // Only add a constraint if this point is still within the set that has
      // been constructed so far on this iteration.
      if (((A.topRows(num_constraints) * point).array() <=
           b.head(num_constraints).array())
              .all()) {
        // Add the tangent to the (scaled) ellipsoid at this point as a
        // constraint.
        A.row(num_constraints) =
            (tangent_matrix * (point - E.center())).normalized();
        b[num_constraints] = A.row(num_constraints) * point;
        num_constraints++;
      }
    }

    if (options.require_sample_point_is_contained &&
        ((A.topRows(num_constraints) * sample).array() >=
         b.head(num_constraints).array())
            .any()) {
      break;
    }
    P = HPolyhedron(A.topRows(num_constraints), b.head(num_constraints));

    iteration++;
    if (iteration >= options.iteration_limit) {
      break;
    }

    E = P.MaximumVolumeInscribedEllipsoid();
    const double volume = E.Volume();
    const double delta_volume = volume - best_volume;
    if (delta_volume <= options.termination_threshold) {
      break;
    }
    if (delta_volume / best_volume <= options.relative_termination_threshold) {
      break;
    }
    best_volume = volume;
  }

  return P;
}

namespace {
// Constructs a ConvexSet for each supported Shape and adds it to the set.
class IrisConvexSetMaker final : public ShapeReifier {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IrisConvexSetMaker)

  IrisConvexSetMaker(const QueryObject<double>& query,
                     std::optional<FrameId> reference_frame)
      : query_{query}, reference_frame_{reference_frame} {};

  void set_reference_frame(const FrameId& reference_frame) {
    DRAKE_DEMAND(reference_frame.is_valid());
    *reference_frame_ = reference_frame;
  }

  void set_geometry_id(const GeometryId& geom_id) { geom_id_ = geom_id; }

  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Sphere&, void* data) {
    DRAKE_DEMAND(geom_id_.is_valid());
    auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
    set = std::make_unique<Hyperellipsoid>(query_, geom_id_, reference_frame_);
  }

  void ImplementGeometry(const Cylinder&, void* data) {
    DRAKE_DEMAND(geom_id_.is_valid());
    auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
    set =
        std::make_unique<CartesianProduct>(query_, geom_id_, reference_frame_);
  }

  void ImplementGeometry(const HalfSpace&, void* data) {
    DRAKE_DEMAND(geom_id_.is_valid());
    auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
    set = std::make_unique<HPolyhedron>(query_, geom_id_, reference_frame_);
  }

  void ImplementGeometry(const Box&, void* data) {
    DRAKE_DEMAND(geom_id_.is_valid());
    auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
    // Note: We choose HPolyhedron over VPolytope here, but the IRIS paper
    // discusses a significant performance improvement using a "least-distance
    // programming" instance from CVXGEN that exploited the VPolytope
    // representation.  So we may wish to revisit this.
    set = std::make_unique<HPolyhedron>(query_, geom_id_, reference_frame_);
  }

  void ImplementGeometry(const Capsule&, void* data) {
    DRAKE_DEMAND(geom_id_.is_valid());
    auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
    set = std::make_unique<MinkowskiSum>(query_, geom_id_, reference_frame_);
  }

  void ImplementGeometry(const Ellipsoid&, void* data) {
    DRAKE_DEMAND(geom_id_.is_valid());
    auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
    set = std::make_unique<Hyperellipsoid>(query_, geom_id_, reference_frame_);
  }

 private:
  const QueryObject<double>& query_{};
  std::optional<FrameId> reference_frame_{};
  GeometryId geom_id_{};
};

}  // namespace

ConvexSets MakeIrisObstacles(const QueryObject<double>& query_object,
                             std::optional<FrameId> reference_frame) {
  const SceneGraphInspector<double>& inspector = query_object.inspector();
  const GeometrySet all_ids(inspector.GetAllGeometryIds());
  const std::unordered_set<GeometryId> geom_ids =
      inspector.GetGeometryIds(all_ids, Role::kProximity);
  ConvexSets sets(geom_ids.size());

  IrisConvexSetMaker maker(query_object, reference_frame);
  int count = 0;
  for (GeometryId geom_id : geom_ids) {
    maker.set_geometry_id(geom_id);
    inspector.GetShape(geom_id).Reify(&maker, &sets[count++]);
  }
  return sets;
}

namespace {

// Takes q, p_AA, and p_BB and enforces that p_WA == p_WB.
class SamePointConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SamePointConstraint)

  SamePointConstraint(const MultibodyPlant<double>* plant,
                      const Context<double>& context)
      : solvers::Constraint(3, plant ? plant->num_positions() + 6 : 0,
                            Vector3d::Zero(), Vector3d::Zero()),
        plant_(plant),
        context_(plant->CreateDefaultContext()) {
    DRAKE_DEMAND(plant_ != nullptr);
    context_->SetTimeStateAndParametersFrom(context);
  }

  ~SamePointConstraint() override {}

  void set_frameA(const multibody::Frame<double>* frame) { frameA_ = frame; }

  void set_frameB(const multibody::Frame<double>* frame) { frameB_ = frame; }

  void EnableSymbolic() {
    if (symbolic_plant_ != nullptr) {
      return;
    }
    symbolic_plant_ = systems::System<double>::ToSymbolic(*plant_);
    symbolic_context_ = symbolic_plant_->CreateDefaultContext();
    symbolic_context_->SetTimeStateAndParametersFrom(*context_);
  }

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    DRAKE_DEMAND(frameA_ != nullptr);
    DRAKE_DEMAND(frameB_ != nullptr);
    VectorXd q = x.head(plant_->num_positions());
    Vector3d p_AA = x.template segment<3>(plant_->num_positions()),
             p_BB = x.template tail<3>();
    Vector3d p_WA, p_WB;
    plant_->SetPositions(context_.get(), q);
    plant_->CalcPointsPositions(*context_, *frameA_, p_AA,
                                plant_->world_frame(), &p_WA);
    plant_->CalcPointsPositions(*context_, *frameB_, p_BB,
                                plant_->world_frame(), &p_WB);
    *y = p_WA - p_WB;
  }

  // p_WA = X_WA(q)*p_AA
  // dp_WA = Jq_v_WA*dq + X_WA(q)*dp_AA
  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    DRAKE_DEMAND(frameA_ != nullptr);
    DRAKE_DEMAND(frameB_ != nullptr);
    VectorX<AutoDiffXd> q = x.head(plant_->num_positions());
    Vector3<AutoDiffXd> p_AA = x.template segment<3>(plant_->num_positions()),
                        p_BB = x.template tail<3>();
    plant_->SetPositions(context_.get(), ExtractDoubleOrThrow(q));
    const RigidTransform<double>& X_WA =
        plant_->EvalBodyPoseInWorld(*context_, frameA_->body());
    const RigidTransform<double>& X_WB =
        plant_->EvalBodyPoseInWorld(*context_, frameB_->body());
    Eigen::Matrix3Xd Jq_v_WA(3, plant_->num_positions()),
        Jq_v_WB(3, plant_->num_positions());
    plant_->CalcJacobianTranslationalVelocity(
        *context_, JacobianWrtVariable::kQDot, *frameA_,
        ExtractDoubleOrThrow(p_AA), plant_->world_frame(),
        plant_->world_frame(), &Jq_v_WA);
    plant_->CalcJacobianTranslationalVelocity(
        *context_, JacobianWrtVariable::kQDot, *frameB_,
        ExtractDoubleOrThrow(p_BB), plant_->world_frame(),
        plant_->world_frame(), &Jq_v_WB);
    *y = X_WA.cast<AutoDiffXd>() * p_AA - X_WB.cast<AutoDiffXd>() * p_BB;
    // Now add it the dydq terms.  We don't use the standard autodiff tools
    // because these only impact a subset of the autodiff derivatives.
    for (int i = 0; i < 3; i++) {
      (*y)[i].derivatives().head(plant_->num_positions()) +=
          (Jq_v_WA.row(i) - Jq_v_WB.row(i)).transpose();
    }
  }

  void DoEval(const Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override {
    DRAKE_DEMAND(symbolic_plant_ != nullptr);
    DRAKE_DEMAND(frameA_ != nullptr);
    DRAKE_DEMAND(frameB_ != nullptr);
    const Frame<Expression>& frameA =
        symbolic_plant_->get_frame(frameA_->index());
    const Frame<Expression>& frameB =
        symbolic_plant_->get_frame(frameB_->index());
    VectorX<Expression> q = x.head(plant_->num_positions());
    Vector3<Expression> p_AA = x.template segment<3>(plant_->num_positions()),
                        p_BB = x.template tail<3>();
    Vector3<Expression> p_WA, p_WB;
    symbolic_plant_->SetPositions(symbolic_context_.get(), q);
    symbolic_plant_->CalcPointsPositions(*symbolic_context_, frameA, p_AA,
                                         symbolic_plant_->world_frame(), &p_WA);
    symbolic_plant_->CalcPointsPositions(*symbolic_context_, frameB, p_BB,
                                         symbolic_plant_->world_frame(), &p_WB);
    *y = p_WA - p_WB;
  }

  const MultibodyPlant<double>* const plant_;
  const multibody::Frame<double>* frameA_{nullptr};
  const multibody::Frame<double>* frameB_{nullptr};
  std::unique_ptr<Context<double>> context_;

  std::unique_ptr<MultibodyPlant<Expression>> symbolic_plant_{nullptr};
  std::unique_ptr<Context<Expression>> symbolic_context_{nullptr};
};

// Solves the optimization
// min_q (q-d)*CᵀC(q-d)
// s.t. setA in frameA and setB in frameB are in collision in q.
//      Aq ≤ b.
// where C, d are the matrix and center from the hyperellipsoid E.
// Returns true iff a collision is found.
// Sets `closest` to an optimizing solution q*, if a solution is found.
bool FindClosestCollision(
    std::shared_ptr<SamePointConstraint> same_point_constraint,
    const multibody::Frame<double>& frameA,
    const multibody::Frame<double>& frameB, const ConvexSet& setA,
    const ConvexSet& setB, const Hyperellipsoid& E,
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b,
    const solvers::SolverInterface& solver,
    const Eigen::Ref<const Eigen::VectorXd>& q_guess, VectorXd* closest) {
  solvers::MathematicalProgram prog;
  auto q = prog.NewContinuousVariables(A.cols(), "q");

  prog.AddLinearConstraint(
      A, VectorXd::Constant(b.size(), -std::numeric_limits<double>::infinity()),
      b, q);
  // Scale the objective so the eigenvalues are close to 1, using
  // scale*lambda_min = 1/scale*lambda_max.
  const MatrixXd Asq = E.A().transpose() * E.A();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(Asq);
  const double scale = 1.0 / std::sqrt(es.eigenvalues().maxCoeff() *
                                       es.eigenvalues().minCoeff());
  prog.AddQuadraticErrorCost(scale * Asq, E.center(), q);

  auto p_AA = prog.NewContinuousVariables<3>("p_AA");
  auto p_BB = prog.NewContinuousVariables<3>("p_BB");
  setA.AddPointInSetConstraints(&prog, p_AA);
  setB.AddPointInSetConstraints(&prog, p_BB);

  same_point_constraint->set_frameA(&frameA);
  same_point_constraint->set_frameB(&frameB);
  prog.AddConstraint(same_point_constraint, {q, p_AA, p_BB});

  // Help nonlinear optimizers (e.g. SNOPT) avoid trivial local minima at the
  // origin.
  prog.SetInitialGuess(q, q_guess);
  prog.SetInitialGuess(p_AA, Vector3d::Constant(.01));
  prog.SetInitialGuess(p_BB, Vector3d::Constant(.01));

  if (solver.solver_id() == solvers::IbexSolver::id()) {
    prog.SetSolverOption(solvers::IbexSolver::id(), "rigor", true);
    // Use kNonconvex instead of the default kConvexSmooth.
    std::vector<solvers::Binding<solvers::LorentzConeConstraint>> to_replace =
        prog.lorentz_cone_constraints();
    for (const auto& binding : to_replace) {
      const auto c = binding.evaluator();
      prog.AddConstraint(
          std::make_shared<solvers::LorentzConeConstraint>(
              c->A_dense(), c->b(),
              solvers::LorentzConeConstraint::EvalType::kNonconvex),
          binding.variables());
    }
    for (const auto& binding : to_replace) {
      prog.RemoveConstraint(binding);
    }
  }

  solvers::MathematicalProgramResult result;
  solver.Solve(prog, std::nullopt, std::nullopt, &result);
  if (result.is_success()) {
    *closest = result.GetSolution(q);
    return true;
  }
  return false;
}

// Add the tangent to the (scaled) ellipsoid at @p point as a
// constraint.
void AddTangentToPolytope(
    const Hyperellipsoid& E, const Eigen::Ref<const Eigen::VectorXd>& point,
    const IrisOptions& options,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>* A,
    Eigen::VectorXd* b, int* num_constraints) {
  if (*num_constraints >= A->rows()) {
    // Increase pre-allocated polytope size.
    A->conservativeResize(A->rows() * 2, A->cols());
    b->conservativeResize(b->rows() * 2);
  }

  A->row(*num_constraints) =
      (E.A().transpose() * E.A() * (point - E.center())).normalized();
  (*b)[*num_constraints] =
      A->row(*num_constraints) * point - options.configuration_space_margin;
  *num_constraints += 1;
}

struct GeometryPairWithDistance {
  GeometryId geomA;
  GeometryId geomB;
  double distance;

  GeometryPairWithDistance(GeometryId gA, GeometryId gB, double dist)
      : geomA(gA), geomB(gB), distance(dist) {}

  bool operator<(const GeometryPairWithDistance& other) const {
    return distance < other.distance;
  }
};

}  // namespace

HPolyhedron IrisInConfigurationSpace(
    const MultibodyPlant<double>& plant, const Context<double>& context,
    const Eigen::Ref<const Eigen::VectorXd>& sample,
    const IrisOptions& options) {
  const int nq = plant.num_positions();
  DRAKE_DEMAND(sample.size() == nq);

  auto sample_context = plant.CreateDefaultContext();
  plant.FixInputPortsFrom(plant, context, sample_context.get());
  sample_context->SetTimeStateAndParametersFrom(context);
  plant.SetPositions(sample_context.get(), sample);
  return IrisInConfigurationSpace(plant, *sample_context, options);
}

HPolyhedron IrisInConfigurationSpace(const MultibodyPlant<double>& plant,
                                     const Context<double>& context,
                                     const IrisOptions& options) {
  // Check the inputs.
  plant.ValidateContext(context);
  const int nq = plant.num_positions();
  const Eigen::VectorXd sample = plant.GetPositions(context);
  // Note: We require finite joint limits to define the bounding box for the
  // IRIS algorithm.
  DRAKE_DEMAND(plant.GetPositionLowerLimits().array().isFinite().all());
  DRAKE_DEMAND(plant.GetPositionUpperLimits().array().isFinite().all());

  // Make the polytope and ellipsoid.
  HPolyhedron P = HPolyhedron::MakeBox(plant.GetPositionLowerLimits(),
                                       plant.GetPositionUpperLimits());
  DRAKE_DEMAND(P.A().rows() == 2 * nq);
  const double kEpsilonEllipsoid = 1e-2;
  Hyperellipsoid E = Hyperellipsoid::MakeHypersphere(kEpsilonEllipsoid, sample);

  // Make all of the convex sets and supporting quantities.
  auto query_object =
      plant.get_geometry_query_input_port().Eval<QueryObject<double>>(context);
  const SceneGraphInspector<double>& inspector = query_object.inspector();
  IrisConvexSetMaker maker(query_object, inspector.world_frame_id());
  std::unordered_map<GeometryId, copyable_unique_ptr<ConvexSet>> sets{};
  std::unordered_map<GeometryId, const multibody::Frame<double>*> frames{};
  const std::unordered_set<GeometryId> geom_ids = inspector.GetGeometryIds(
      GeometrySet(inspector.GetAllGeometryIds()), Role::kProximity);
  copyable_unique_ptr<ConvexSet> temp_set;
  for (GeometryId geom_id : geom_ids) {
    // Make all sets in the local geometry frame.
    FrameId frame_id = inspector.GetFrameId(geom_id);
    maker.set_reference_frame(frame_id);
    maker.set_geometry_id(geom_id);
    inspector.GetShape(geom_id).Reify(&maker, &temp_set);
    sets.emplace(geom_id, std::move(temp_set));
    frames.emplace(geom_id, &plant.GetBodyFromFrameId(frame_id)->body_frame());
  }

  auto pairs = inspector.GetCollisionCandidates();
  const int N = static_cast<int>(pairs.size());
  auto same_point_constraint =
      std::make_shared<SamePointConstraint>(&plant, context);

  // As a surrogate for the true objective, the pairs are sorted by the distance
  // between each collision pair from the sample point configuration. This could
  // improve computation times in Ibex here and produce regions with fewer
  // faces.
  std::vector<GeometryPairWithDistance> sorted_pairs;
  for (const auto& [geomA, geomB] : pairs) {
    sorted_pairs.emplace_back(
        geomA, geomB,
        query_object.ComputeSignedDistancePairClosestPoints(geomA, geomB)
            .distance);
  }
  std::sort(sorted_pairs.begin(), sorted_pairs.end());

  // On each iteration, we will build the collision-free polytope represented as
  // {x | A * x <= b}.  Here we pre-allocate matrices with a generous maximum
  // size.
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(
      P.A().rows() + 2 * N, nq);
  VectorXd b(P.A().rows() + 2 * N);
  A.topRows(P.A().rows()) = P.A();
  b.head(P.A().rows()) = P.b();

  double best_volume = E.Volume();
  int iteration = 0;
  VectorXd closest(nq);

  auto solver = solvers::MakeFirstAvailableSolver(
      {solvers::SnoptSolver::id(), solvers::IpoptSolver::id()});
  std::unique_ptr<solvers::IbexSolver> ibex;
  if (options.enable_ibex) {
    ibex = std::make_unique<solvers::IbexSolver>();
    DRAKE_DEMAND(ibex->is_available() && ibex->is_enabled());
    same_point_constraint->EnableSymbolic();
  }

  while (true) {
    int num_constraints = 2 * nq;  // Start with just the joint limits.
    bool sample_point_requirement = true;
    DRAKE_ASSERT(best_volume > 0);
    // Find separating hyperplanes

    // First use a fast nonlinear optimizer to add as many constraint as it
    // can find.
    for (const auto& pair : sorted_pairs) {
      while (sample_point_requirement &&
             FindClosestCollision(
                 same_point_constraint, *frames.at(pair.geomA),
                 *frames.at(pair.geomB), *sets.at(pair.geomA),
                 *sets.at(pair.geomB), E, A.topRows(num_constraints),
                 b.head(num_constraints), *solver, sample, &closest)) {
        AddTangentToPolytope(E, closest, options, &A, &b, &num_constraints);
        if (options.require_sample_point_is_contained) {
          sample_point_requirement =
              A.row(num_constraints - 1) * sample <= b(num_constraints - 1);
        }
      }
    }

    if (options.enable_ibex) {
      // Now loop back through and use Ibex for rigorous certification.
      // TODO(russt): Consider (re-)implementing a "feasibility only" version of
      // the IRIS check + nonlinear optimization to improve.
      for (const auto& pair : sorted_pairs) {
        while (sample_point_requirement &&
               FindClosestCollision(
                   same_point_constraint, *frames.at(pair.geomA),
                   *frames.at(pair.geomB), *sets.at(pair.geomA),
                   *sets.at(pair.geomB), E, A.topRows(num_constraints),
                   b.head(num_constraints), *ibex, sample, &closest)) {
          AddTangentToPolytope(E, closest, options, &A, &b, &num_constraints);
          if (options.require_sample_point_is_contained) {
            sample_point_requirement =
                A.row(num_constraints - 1) * sample <= b(num_constraints - 1);
          }
        }
      }
    }

    if (!sample_point_requirement) {
      break;
    }
    P = HPolyhedron(A.topRows(num_constraints), b.head(num_constraints));

    iteration++;
    if (iteration >= options.iteration_limit) {
      break;
    }

    E = P.MaximumVolumeInscribedEllipsoid();
    const double volume = E.Volume();
    const double delta_volume = volume - best_volume;
    if (delta_volume <= options.termination_threshold) {
      break;
    }
    if (delta_volume / best_volume <= options.relative_termination_threshold) {
      break;
    }
    best_volume = volume;
  }
  return P;
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
