#include "drake/multibody/contact_solvers/contact_solver.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
int ContactSolver<T>::num_velocities() const {
  return num_velocities_;
}

template <typename T>
int ContactSolver<T>::num_contacts() const {
  return num_contacts_;
}

template <typename T>
void ContactSolver<T>::SetSystemDynamicsData(SystemDynamicsData<T> data) {
  dynamics_data_ = std::make_unique<SystemDynamicsData<T>>(data);
  num_velocities_ = data.num_velocities();
}

template <typename T>
void ContactSolver<T>::SetPointContactData(PointContactData<T> data) {
  contact_data_ = std::make_unique<PointContactData<T>>(data);
  num_contacts_ = data.num_contacts();
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::ContactSolver)
