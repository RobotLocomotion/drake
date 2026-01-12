#include "drake/multibody/inverse_kinematics/point_to_line_distance_constraint.h"

#include <limits>

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_evaluator_utilities.h"

using drake::multibody::internal::RefFromPtrOrThrow;
using drake::multibody::internal::UpdateContextConfiguration;

namespace drake {
namespace multibody {
const double kEps = std::numeric_limits<double>::epsilon();

PointToLineDistanceConstraint::~PointToLineDistanceConstraint() = default;

PointToLineDistanceConstraint::PointToLineDistanceConstraint(
    const MultibodyPlant<double>* const plant, const Frame<double>& frame_point,
    const Eigen::Ref<const Eigen::Vector3d>& p_B1P,
    const Frame<double>& frame_line,
    const Eigen::Ref<const Eigen::Vector3d>& p_B2Q,
    const Eigen::Ref<const Eigen::Vector3d>& n_B2, double distance_lower,
    double distance_upper, systems::Context<double>* plant_context)
    : solvers::Constraint(1, RefFromPtrOrThrow(plant).num_positions(),
                          Vector1d(distance_lower * distance_lower),
                          Vector1d(distance_upper * distance_upper)),
      plant_double_(plant),
      frame_point_index_(frame_point.index()),
      frame_line_index_(frame_line.index()),
      p_B1P_(p_B1P),
      p_B2Q_(p_B2Q),
      n_B2_normalized_(n_B2.normalized()),
      project_matrix_(Eigen::Matrix3d::Identity() -
                      n_B2_normalized_ * n_B2_normalized_.transpose()),
      context_double_(plant_context),
      plant_autodiff_(nullptr),
      context_autodiff_(nullptr) {
  if (plant_context == nullptr) {
    throw std::invalid_argument("plant_context is nullptr");
  }
  DRAKE_DEMAND(distance_lower >= 0);
  DRAKE_DEMAND(distance_upper >= distance_lower);
  // Make sure that the user-supplied n_B2 is not close to 0-vector.
  DRAKE_DEMAND(n_B2.norm() > 100 * kEps);
}

PointToLineDistanceConstraint::PointToLineDistanceConstraint(
    const MultibodyPlant<AutoDiffXd>* plant,
    const Frame<AutoDiffXd>& frame_point,
    const Eigen::Ref<const Eigen::Vector3d>& p_B1P,
    const Frame<AutoDiffXd>& frame_line,
    const Eigen::Ref<const Eigen::Vector3d>& p_B2Q,
    const Eigen::Ref<const Eigen::Vector3d>& n_B2, double distance_lower,
    double distance_upper, systems::Context<AutoDiffXd>* plant_context)
    : solvers::Constraint(1, RefFromPtrOrThrow(plant).num_positions(),
                          Vector1d(distance_lower * distance_lower),
                          Vector1d(distance_upper * distance_upper)),
      plant_double_(nullptr),
      frame_point_index_(frame_point.index()),
      frame_line_index_(frame_line.index()),
      p_B1P_(p_B1P),
      p_B2Q_(p_B2Q),
      n_B2_normalized_(n_B2.normalized()),
      project_matrix_(Eigen::Matrix3d::Identity() -
                      n_B2_normalized_ * n_B2_normalized_.transpose()),
      context_double_(nullptr),
      plant_autodiff_(plant),
      context_autodiff_(plant_context) {
  if (plant_context == nullptr) {
    throw std::invalid_argument("plant_context is nullptr");
  }
  DRAKE_DEMAND(distance_lower >= 0);
  DRAKE_DEMAND(distance_upper >= distance_lower);
  DRAKE_DEMAND(n_B2.norm() > 100 * kEps);
}

namespace {

template <typename T>
void EvalPointToLineDistanceConstraintGradient(
    const systems::Context<T>&, const MultibodyPlant<T>&, const Frame<T>&,
    const Frame<T>&, const Eigen::Vector3d&, const Vector3<T>& p_QP_B2,
    const Eigen::Matrix3d& project_matrix, const Eigen::Ref<const VectorX<T>>&,
    VectorX<T>* y) {
  // The squared distance from a point P to a line is uᵀ(I-nnᵀ)u, where u = QP,
  // Q being a point on the line, n being the unit-length vector along the line,
  // project_matrix = I-nnᵀ.
  using std::pow;
  (*y)(0) = p_QP_B2.dot(project_matrix * p_QP_B2);
}

void EvalPointToLineDistanceConstraintGradient(
    const systems::Context<double>& context,
    const MultibodyPlant<double>& plant, const Frame<double>& frame_point,
    const Frame<double>& frame_line, const Eigen::Vector3d& p_B1P,
    const Eigen::Vector3d& p_QP_B2, const Eigen::Matrix3d& project_matrix,
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) {
  Eigen::MatrixXd Jq_V_B2P(3, plant.num_positions());
  plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable::kQDot,
                                          frame_point, p_B1P, frame_line,
                                          frame_line, &Jq_V_B2P);

  *y = math::InitializeAutoDiff(Vector1d(p_QP_B2.dot(project_matrix * p_QP_B2)),
                                2 * p_QP_B2.transpose() * project_matrix *
                                    Jq_V_B2P * math::ExtractGradient(x));
}

template <typename T, typename S>
void DoEvalGeneric(const MultibodyPlant<T>& plant, systems::Context<T>* context,
                   const FrameIndex frame_point_index,
                   const Eigen::Vector3d& p_B1P,
                   const FrameIndex frame_line_index,
                   const Eigen::Vector3d& p_B2Q,
                   const Eigen::Matrix3d& project_matrix,
                   const Eigen::Ref<const VectorX<S>>& x, VectorX<S>* y) {
  y->resize(1);
  UpdateContextConfiguration(context, plant, x);
  const Frame<T>& frame_point = plant.get_frame(frame_point_index);
  const Frame<T>& frame_line = plant.get_frame(frame_line_index);
  // Compute p_B2P.
  Vector3<T> p_B2P;
  plant.CalcPointsPositions(*context, frame_point, p_B1P.cast<T>(), frame_line,
                            &p_B2P);
  const Vector3<T> p_QP_B2 = p_B2P - p_B2Q;
  EvalPointToLineDistanceConstraintGradient(*context, plant, frame_point,
                                            frame_line, p_B1P, p_QP_B2,
                                            project_matrix, x, y);
}

}  // namespace

void PointToLineDistanceConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  if (use_autodiff()) {
    AutoDiffVecXd y_t;
    Eval(x.cast<AutoDiffXd>(), &y_t);
    *y = math::ExtractValue(y_t);
  } else {
    DoEvalGeneric(*plant_double_, context_double_, frame_point_index_, p_B1P_,
                  frame_line_index_, p_B2Q_, project_matrix_, x, y);
  }
}

void PointToLineDistanceConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  if (use_autodiff()) {
    DoEvalGeneric(*plant_autodiff_, context_autodiff_, frame_point_index_,
                  p_B1P_, frame_line_index_, p_B2Q_, project_matrix_, x, y);
  } else {
    DoEvalGeneric(*plant_double_, context_double_, frame_point_index_, p_B1P_,
                  frame_line_index_, p_B2Q_, project_matrix_, x, y);
  }
}
}  // namespace multibody
}  // namespace drake
