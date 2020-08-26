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
                     const VectorX<T>* v0,
                     const VectorX<T>* tau)
      : Minv_(Minv), v0_(v0), tau_(tau) {
    DRAKE_DEMAND(Minv != nullptr);
    DRAKE_DEMAND(v0 != nullptr);
    DRAKE_DEMAND(tau != nullptr);
    DRAKE_DEMAND(Minv->rows() == Minv->cols());
    nv_ = Minv->rows();
  }

  int num_velocities() const { return nv_; }

  const LinearOperator<T>& get_Minv() const { return *Minv_; };
  const VectorX<T>& get_v0() const { return *v0_; };
  const VectorX<T>& get_tau() const { return *tau_; }

 private:
  int nv_;
  const LinearOperator<T>* Minv_{nullptr};
  const VectorX<T>* v0_{nullptr};
  const VectorX<T>* tau_{nullptr};
};

}  // namespace solvers
}  // namespace multibody
}  // namespace drake