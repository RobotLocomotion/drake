#pragma once

#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {

template <typename T>
class ExternalForceField {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ExternalForceField)

  /* Constructs a field that adds up all the input fields. */
  ExternalForceField(
      std::vector<std::function<Vector3<T>(const Vector3<T>&)>> fields)
      : fields_(std::move(fields)) {}

  /* Constructs an empty field. */
  ExternalForceField() {}

  Vector3<T> Eval(const Vector3<T>& p_WQ) const;

 private:
  std::vector<std::function<Vector3<T>(const Vector3<T>&)>> fields_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::ExternalForceField)
