#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/solvers/linear_operator.h"

#include <string>

namespace drake {
namespace multibody {
namespace solvers {  

template <typename T>
class SystemDynamicsData {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SystemDynamicsData)

  SystemDynamicsData(const LinearOperator<T>* Minv,
                     const VectorX<T>* v_star)
      : Minv_(Minv), v_star_(v_star) {
    DRAKE_DEMAND(Minv != nullptr);
    DRAKE_DEMAND(v_star != nullptr);
    DRAKE_DEMAND(Minv->rows() == Minv->cols());
    nv_ = Minv->rows();
    DRAKE_DEMAND(v_star->size() == num_velocities());
  }

  int num_velocities() const { return nv_; }

  const LinearOperator<T>& get_Minv() const { return *Minv_; };
  const VectorX<T>& get_v_star() const { return *v_star_; };

 private:
  int nv_;
  const LinearOperator<T>* Minv_{nullptr};
  const VectorX<T>* v_star_{nullptr};
};

}  // namespace solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::solvers::SystemDynamicsData)
