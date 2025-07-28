#include "drake/planning/iris/iris_common.h"

#include <common_robotics_utilities/parallelism.hpp>

#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"

namespace drake {
namespace planning {

IrisParameterizationFunction::IrisParameterizationFunction(
    const multibody::RationalForwardKinematics* kin,
    const Eigen::Ref<const Eigen::VectorXd>& q_star_val) {
  const int dimension = kin->plant().num_positions();
  DRAKE_DEMAND(dimension > 0);

  parameterization_double_ =
      [kin, q_star_captured =
                Eigen::VectorXd(q_star_val)](const Eigen::VectorXd& s_val) {
        return kin->ComputeQValue(s_val, q_star_captured);
      };
  // TODO(cohnt): Construct a VectorX<AutoDiffXd> parameterization when using
  // this constructor as well.
  parameterization_autodiff_ = nullptr;

  parameterization_is_threadsafe_ = true;
  parameterization_dimension_ = dimension;
}

IrisParameterizationFunction::IrisParameterizationFunction(
    const Eigen::VectorX<symbolic::Expression>& expression_parameterization,
    const Eigen::VectorX<symbolic::Variable>& variables) {
  // First, we check that the variables in expression_parameterization match
  // the user-supplied variables.
  symbolic::Variables expression_variables;
  for (const auto& expression : expression_parameterization) {
    expression_variables.insert(expression.GetVariables());
  }
  symbolic::Variables user_supplied_variables(variables);
  DRAKE_THROW_UNLESS(expression_variables == user_supplied_variables);

  // Check for duplicates in variables.
  DRAKE_THROW_UNLESS(variables.size() == ssize(user_supplied_variables));

  int dimension = ssize(expression_variables);

  // Note that in this lambda, we copy the shared_ptr variables, ensuring that
  // variables is kept alive without making a copy of the individual Variable
  // objects (which would break the substitution machinery).
  parameterization_double_ =
      [expression_parameterization_captured =
           Eigen::VectorX<symbolic::Expression>(expression_parameterization),
       variables_captured = Eigen::VectorX<symbolic::Variable>(variables)](
          const Eigen::VectorXd& q) {
        DRAKE_ASSERT(q.size() == variables_captured.size());
        symbolic::Environment env;
        for (int i = 0; i < q.size(); ++i) {
          env.insert(variables_captured[i], q[i]);
        }
        Eigen::VectorXd out = expression_parameterization_captured.unaryExpr(
            [&env](const symbolic::Expression& expression) {
              return expression.Evaluate(env);
            });
        return out;
      };
  // TODO(cohnt): Construct a VectorX<AutoDiffXd> parameterization when using
  // this constructor as well.
  parameterization_autodiff_ = nullptr;

  parameterization_is_threadsafe_ = true;
  parameterization_dimension_ = dimension;
}

namespace internal {

using common_robotics_utilities::parallelism::DegreeOfParallelism;
using common_robotics_utilities::parallelism::DynamicParallelForIndexLoop;
using common_robotics_utilities::parallelism::ParallelForBackend;
using common_robotics_utilities::parallelism::StaticParallelForRangeLoop;
using common_robotics_utilities::parallelism::ThreadWorkRange;

int unadaptive_test_samples(double epsilon, double delta, double tau) {
  return static_cast<int>(-2 * std::log(delta) / (tau * tau * epsilon) + 0.5);
}
float calc_delta_min(double delta, int max_iterations) {
  return delta * 6 / (M_PI * M_PI * max_iterations * max_iterations);
}

// Add the tangent to the (scaled) ellipsoid at @p point as a
// constraint.
void AddTangentToPolytope(
    const geometry::optimization::Hyperellipsoid& E,
    const Eigen::Ref<const Eigen::VectorXd>& point,
    double configuration_space_margin,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>* A,
    Eigen::VectorXd* b, int* num_constraints) {
  while (*num_constraints >= A->rows()) {
    // Increase pre-allocated polytope size.
    A->conservativeResize(A->rows() * 2, A->cols());
    b->conservativeResize(b->rows() * 2);
  }

  A->row(*num_constraints) =
      (E.A().transpose() * E.A() * (point - E.center())).normalized();
  (*b)[*num_constraints] =
      A->row(*num_constraints) * point - configuration_space_margin;
  if (A->row(*num_constraints) * E.center() > (*b)[*num_constraints]) {
    throw std::logic_error(
        "The current center of the IRIS region is within "
        "options.sampled_iris_options.configuration_space_margin of being "
        "infeasible.  Check your sample point and/or any additional "
        "constraints you've passed in via the options. The configuration "
        "space surrounding the sample point must have an interior.");
  }
  *num_constraints += 1;
}

bool CheckProgConstraints(const solvers::MathematicalProgram* prog_ptr,
                          const Eigen::VectorXd& particle, const double tol) {
  if (!prog_ptr) {
    return true;
  }
  for (const auto& binding : prog_ptr->GetAllConstraints()) {
    DRAKE_ASSERT(binding.evaluator() != nullptr);
    if (!binding.evaluator()->CheckSatisfied(particle, tol)) {
      return false;
    }
  }
  return true;
}

std::vector<uint8_t> CheckProgConstraintsParallel(
    const solvers::MathematicalProgram* prog_ptr,
    const std::vector<Eigen::VectorXd>& particles,
    const Parallelism& parallelism, const double tol,
    std::optional<int> end_index) {
  const int actual_end_index = end_index.value_or(ssize(particles));
  DRAKE_DEMAND(actual_end_index >= 0 && actual_end_index <= ssize(particles));
  std::vector<uint8_t> is_valid(actual_end_index, 1);
  if (!prog_ptr) {
    return is_valid;
  }
  const auto check_particle_work = [&prog_ptr, &particles, &tol, &is_valid](
                                       const int thread_num,
                                       const int64_t index) {
    unused(thread_num);
    is_valid[index] = static_cast<uint8_t>(
        CheckProgConstraints(prog_ptr, particles[index], tol));
  };

  DynamicParallelForIndexLoop(DegreeOfParallelism(parallelism.num_threads()), 0,
                              actual_end_index, check_particle_work,
                              ParallelForBackend::BEST_AVAILABLE);
  return is_valid;
}

void PopulateParticlesByUniformSampling(
    const geometry::optimization::HPolyhedron& P, int number_to_sample,
    int mixing_steps, std::vector<RandomGenerator>* generators,
    std::vector<Eigen::VectorXd>* particles) {
  DRAKE_THROW_UNLESS(number_to_sample <= ssize(*particles));
  const int num_threads = ssize(*generators);

  const auto hit_and_run_sample_work =
      [&P, &particles, &generators,
       &mixing_steps](const ThreadWorkRange& work_range) {
        const int64_t start_index = work_range.GetRangeStart();
        const int64_t end_index = work_range.GetRangeEnd();
        const int64_t thread_num = work_range.GetThreadNum();
        RandomGenerator* generator = &(generators->at(thread_num));
        (*particles)[start_index] = P.UniformSample(generator, mixing_steps);
        for (int j = start_index + 1; j < end_index; ++j) {
          (*particles)[j] =
              P.UniformSample(generator, (*particles)[j - 1], mixing_steps);
        }
      };

  StaticParallelForRangeLoop(DegreeOfParallelism(num_threads), 0,
                             number_to_sample, hit_and_run_sample_work,
                             ParallelForBackend::BEST_AVAILABLE);
}

}  // namespace internal
}  // namespace planning
}  // namespace drake
