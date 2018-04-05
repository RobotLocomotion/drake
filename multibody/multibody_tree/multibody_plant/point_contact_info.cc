#include "drake/multibody/multibody_tree/multibody_plant/point_contact_info.h"

#include <cmath>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace multibody {
namespace multibody_plant {

using std::abs;

template <typename T>
PointContactInfo<T>::PointContactInfo(
    const Vector3<T>& nhat_BA, const Vector3<T>& that, const T& phi,
    const T& vn, const T& vt, const Vector3<T>& v_AcBc,
    const Vector3<T>& vt_AcBc, const T& fn_AC, const T& ft_AC,
    const T& mu_stribeck)
    : nhat_BA_W_(nhat_BA),
      that_W_(that),
      phi_(phi),
      vn_(vn),
      vt_(vt),
      v_AcBc_W_(v_AcBc),
      vt_AcBc_W_(vt_AcBc),
      fn_AC_(fn_AC),
      ft_AC_(ft_AC),
      mu_stribeck_(mu_stribeck) {
  DRAKE_ASSERT(abs(nhat_BA.norm() - 1.0) <
      Eigen::NumTraits<T>::dummy_precision());
  DRAKE_ASSERT(abs(that.norm() - 1.0) <
      Eigen::NumTraits<T>::dummy_precision());
}

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::multibody_plant::PointContactInfo)
