#include "drake/multibody/contact_solvers/sap_solver.h"

#include <algorithm>
#include <numeric>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "fmt/format.h"

#include "drake/multibody/contact_solvers/contact_solver_utils.h"
namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using Eigen::SparseMatrix;
using Eigen::SparseVector;

template <typename T>
ContactSolverStatus SapSolver<T>::SolveWithGuess(const T&,
                                                 const SystemDynamicsData<T>&,
                                                 const PointContactData<T>&,
                                                 const VectorX<T>&,
                                                 ContactSolverResults<T>*) {
  throw std::runtime_error("This method must be implemented.");
  DRAKE_UNREACHABLE();
}

template <typename T>
void SapSolver<T>::PackContactResults(const PreProcessedData& data,
                                      const VectorX<T>& v, const VectorX<T>& vc,
                                      const VectorX<T>& gamma,
                                      ContactSolverResults<T>* results) {
  results->Resize(data.nv, data.nc);
  results->v_next = v;
  ExtractNormal(vc, &results->vn);
  ExtractTangent(vc, &results->vt);
  ExtractNormal(gamma, &results->fn);
  ExtractTangent(gamma, &results->ft);
  // N.B. While contact solver works with impulses, results are reported as
  // forces.
  results->fn /= data.time_step;
  results->ft /= data.time_step;
  const auto& Jop = data.Jblock;
  Jop.MultiplyByTranspose(gamma, &results->tau_contact);
  results->tau_contact /= data.time_step;
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

template class ::drake::multibody::contact_solvers::internal::SapSolver<double>;
