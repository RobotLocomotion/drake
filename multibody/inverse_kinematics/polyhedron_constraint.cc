#include "drake/multibody/inverse_kinematics/polyhedron_constraint.h"

#include <limits>

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_evaluator_utilities.h"

using drake::multibody::internal::RefFromPtrOrThrow;
using drake::multibody::internal::UpdateContextConfiguration;

namespace drake {
namespace multibody {
const double kInf = std::numeric_limits<double>::infinity();

PolyhedronConstraint::PolyhedronConstraint(
    const MultibodyPlant<double>* plant, const Frame<double>& frameF,
    const Frame<double>& frameG, const Eigen::Ref<const Eigen::Matrix3Xd>& p_GP,
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b,
    systems::Context<double>* plant_context)
    : solvers::Constraint(A.rows(), RefFromPtrOrThrow(plant).num_positions(),
                          Eigen::VectorXd::Constant(b.rows(), -kInf), b),
      plant_double_{plant},
      frameF_index_{frameF.index()},
      frameG_index_{frameG.index()},
      p_GP_{p_GP},
      A_{A},
      b_{b},
      context_double_{plant_context},
      plant_autodiff_{nullptr},
      context_autodiff_{nullptr} {
  if (plant_context == nullptr) {
    throw std::invalid_argument(
        "PolyhedronConstraint: plant_context is nullptr.");
  }
  DRAKE_DEMAND(A_.cols() == p_GP_.cols() * 3);
}

PolyhedronConstraint::PolyhedronConstraint(
    const MultibodyPlant<AutoDiffXd>* const plant,
    const Frame<AutoDiffXd>& frameF, const Frame<AutoDiffXd>& frameG,
    const Eigen::Ref<const Eigen::Matrix3Xd>& p_GP,
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b,
    systems::Context<AutoDiffXd>* plant_context)
    : solvers::Constraint(A.rows(), RefFromPtrOrThrow(plant).num_positions(),
                          Eigen::VectorXd::Constant(b.rows(), -kInf), b),
      plant_double_{nullptr},
      frameF_index_{frameF.index()},
      frameG_index_{frameG.index()},
      p_GP_{p_GP},
      A_{A},
      b_{b},
      context_double_{nullptr},
      plant_autodiff_{plant},
      context_autodiff_{plant_context} {
  if (plant_context == nullptr) {
    throw std::invalid_argument(
        "PolyhedronConstraint: plant_context is nullptr");
  }
  DRAKE_DEMAND(A_.cols() == p_GP_.cols() * 3);
}

template <typename T, typename S>
void DoEvalGeneric(const MultibodyPlant<T>& plant, systems::Context<T>* context,
                   FrameIndex frameF_index, FrameIndex frameG_index,
                   const Eigen::Matrix3Xd& p_GP, const Eigen::MatrixXd& A,
                   const Eigen::Ref<const VectorX<S>>& x, VectorX<S>* y) {
  UpdateContextConfiguration(context, plant, x);
  const Frame<T>& frameF = plant.get_frame(frameF_index);
  const Frame<T>& frameG = plant.get_frame(frameG_index);

  Matrix3X<T> p_FP(3, p_GP.cols());
  plant.CalcPointsPositions(*context, frameG, p_GP.cast<T>(), frameF, &p_FP);
  Eigen::Map<VectorX<T>> p_FP_stack(p_FP.data(), 3 * p_GP.cols());
  if constexpr (std::is_same_v<T, S>) {
    *y = A.cast<T>() * p_FP_stack;
  } else {
    Eigen::MatrixXd Jq_v_FP(3 * p_GP.cols(), plant.num_positions());
    plant.CalcJacobianTranslationalVelocity(*context,
                                            JacobianWrtVariable::kQDot, frameG,
                                            p_GP, frameF, frameF, &Jq_v_FP);
    *y = math::InitializeAutoDiff(A * p_FP_stack,
                                  A * Jq_v_FP * math::ExtractGradient(x));
  }
}

void PolyhedronConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                  Eigen::VectorXd* y) const {
  if (use_autodiff()) {
    AutoDiffVecXd y_t;
    Eval(x.cast<AutoDiffXd>(), &y_t);
    *y = math::ExtractValue(y_t);
  } else {
    DoEvalGeneric<double, double>(*plant_double_, context_double_,
                                  frameF_index_, frameG_index_, p_GP_, A_, x,
                                  y);
  }
}

void PolyhedronConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                  AutoDiffVecXd* y) const {
  if (use_autodiff()) {
    DoEvalGeneric(*plant_autodiff_, context_autodiff_, frameF_index_,
                  frameG_index_, p_GP_, A_, x, y);
  } else {
    DoEvalGeneric(*plant_double_, context_double_, frameF_index_, frameG_index_,
                  p_GP_, A_, x, y);
  }
}
}  // namespace multibody
}  // namespace drake
