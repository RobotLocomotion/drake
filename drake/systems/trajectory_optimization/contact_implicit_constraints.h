#pragma once

#include <tuple>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/binding.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace systems {
/**
 * This constraint formulates the general nonlinear complementary constraint
 * <pre>
 *   0 ≤ g(z) ⊥ h(z) ≥ 0    (1)
 * </pre>
 * where g(z) and h(z) are column vectors of the same dimension. The inequality
 * is elementwise.
 * To solve a problem with this nonlinear complementary constraint through
 * nonlinear optimization, we formulate (and relax) it as
 * <pre>
 *   α = g(z)
 *   β = h(z)
 *   α, β ≥ 0
 *   αᵢ * βᵢ ≤ ε
 * </pre>
 * where α, β are additional slack variables. ε > 0 is a small positive
 * tolerance. As ε → 0, the relaxation becomes tight as the original constraint
 * (1). Check equation 26 - 29 in the paper
 * A Direct Method for Trajectory Optimization of Rigid Bodies Through Contact
 * Michael Posa, Cecilia Cantu, and Russ Tedrake. IJRR, 2014.
 */
class GeneralNonlinearComplementaryConstraint {
 public:
  typedef void (*nonlinear_fun_double)(const Eigen::Ref<const Eigen::VectorXd>&,
                                       Eigen::Ref<Eigen::VectorXd>);
  typedef void (*nonlinear_fun_autodiff)(const Eigen::Ref<const AutoDiffVecXd>&,
                                         Eigen::Ref<AutoDiffVecXd>);

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GeneralNonlinearComplementaryConstraint)

  /**
   * Construct a container that wraps the information about this complementary
   * constraint.
   * @param g_double The function that evaluates `g` with Eigen double vector.
   * @param g_autodiff The function that evaluates `g` with Eigen autodiff
   * vector.
   * @param h_double The function that evaluates `h` with Eigen double vector.
   * @param h_autodiff The function that evaluates `h` with Eigen autodiff
   * vector.
   * @param num_complementary The number of rows in `g` and `h`.
   * @param z_size The size of the vector `z`.
   * @param complementary_epsilon The threshold for the complementary condition.
   */
  GeneralNonlinearComplementaryConstraint(nonlinear_fun_double g_double,
                                          nonlinear_fun_autodiff g_autodiff,
                                          nonlinear_fun_double h_double,
                                          nonlinear_fun_autodiff h_autodiff,
                                          int num_complementary, int z_size,
                                          double complementary_epsilon)
      : g_double_{g_double},
        g_autodiff_{g_autodiff},
        h_double_{h_double},
        h_autodiff_{h_autodiff},
        num_complementary_{num_complementary},
        z_size_{z_size},
        complementary_epsilon_{complementary_epsilon} {
    DRAKE_ASSERT(num_complementary > 0);
    DRAKE_ASSERT(complementary_epsilon >= 0);
  }

  /**
   * This class contains the nonlinear constraints inside the complementary
   * constraint.
   */
  class NonlinearComplementaryNonlinearConstraint : public solvers::Constraint {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NonlinearComplementaryNonlinearConstraint)

    NonlinearComplementaryNonlinearConstraint(nonlinear_fun_double g_double,
                                              nonlinear_fun_autodiff g_autodiff,
                                              nonlinear_fun_double h_double,
                                              nonlinear_fun_autodiff h_autodiff,
                                              int num_complementary, int z_size,
                                              double complementary_epsilon)
        : solvers::Constraint(num_complementary * 3,
                              z_size + 2 * num_complementary,
                              Eigen::VectorXd::Zero(3 * num_complementary),
                              Eigen::VectorXd::Zero(3 * num_complementary)),
          g_double_{g_double},
          g_autodiff_{g_autodiff},
          h_double_{h_double},
          h_autodiff_{h_autodiff},
          num_complementary_{num_complementary},
          z_size_{z_size} {
      DRAKE_ASSERT(num_complementary_ > 0);
      Eigen::VectorXd ub(3 * num_complementary);
      ub.tail(num_complementary) =
          Eigen::VectorXd::Constant(num_complementary, complementary_epsilon);
      UpdateUpperBound(ub);
    }

    ~NonlinearComplementaryNonlinearConstraint() override {}

   protected:
    void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                Eigen::VectorXd& y) const override;

    void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                AutoDiffVecXd& y) const override;

   private:
    const nonlinear_fun_double g_double_;
    const nonlinear_fun_autodiff g_autodiff_;
    const nonlinear_fun_double h_double_;
    const nonlinear_fun_autodiff h_autodiff_;
    const int num_complementary_;
    const int z_size_;
  };

  /**
   * Add the generalized nonlinear complementary constraint into the
   * optimization program.
   * @param prog The optimization program to be changed.
   * @param z The generalized nonlinear complementary constraint is imposed on
   * variable `z`.
   * @return T. T is a tuple
   * T.get<0>() The newly created bounding box constraint.
   * T.get<1>() The newly created linear constraint.
   * T.get<2>() The newly created nonlinear constraint.
   * T.get<3>() The newly added slack variables.
   */
  std::tuple<solvers::Binding<solvers::BoundingBoxConstraint>,
             solvers::Binding<solvers::LinearConstraint>,
             solvers::Binding<solvers::Constraint>,
             solvers::VectorXDecisionVariable> AddConstraintToProgram(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& z) const {
    return DoAddConstraintToProgram(prog, z);
  }

  virtual ~GeneralNonlinearComplementaryConstraint() {}

 protected:
  const nonlinear_fun_double g_double_;
  const nonlinear_fun_autodiff g_autodiff_;
  const nonlinear_fun_double h_double_;
  const nonlinear_fun_autodiff h_autodiff_;
  const int num_complementary_;
  const int z_size_;
  const double complementary_epsilon_;

 private:
  virtual std::tuple<solvers::Binding<solvers::BoundingBoxConstraint>,
                     solvers::Binding<solvers::LinearConstraint>,
                     solvers::Binding<solvers::Constraint>,
                     solvers::VectorXDecisionVariable>
  DoAddConstraintToProgram(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& z) const;
};
}  // namespace systems
}  // namespace drake
