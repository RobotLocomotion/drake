#pragma once

#include <type_traits>

#include "drake/common/autodiff.h"

namespace drake {
namespace geometry {

/** Given the two scalar types U and T, returns the most "promoted" type. The
 scalars must be either `double` or `AutoDiffXd`.

 This trait implicitly encodes the logic: if either B or T are AutoDiffXd,
 return AutoDiffXd. The logic is illustrated with this truth table:

 |      T       |      U       |    Result    |
 | :----------: | :----------: | :----------: |
 |  AutoDiffXd  |  AutoDiffXd  |  AutoDiffXd  |
 |    double    |  AutoDiffXd  |  AutoDiffXd  |
 |  AutoDiffXd  |    double    |  AutoDiffXd  |
 |    double    |    double    |    double    |

 This also includes the helper type:

 ```
 template <typename T, typename U>
 using promoted_numerical_t = typename promoted_numerical<T, U>::type;
 ```
 */
template <typename T, typename U>
struct promoted_numerical {
  static_assert(
      std::conjunction_v<std::disjunction<std::is_same<U, double>,
                                          std::is_same<U, AutoDiffXd>>,
                         std::disjunction<std::is_same<T, double>,
                                          std::is_same<T, AutoDiffXd>>>,
      "This utility is only compatible with 'double' and 'AutoDiffXd' scalar "
      "types");
  /* If U is double, then the return type is ultimately determined by T. If
   U is AutoDiff (i.e. "not double"), then we can just return that. If we ever
   change the space of scalar types this supports, this logic will have to
   change. */
  using type = std::conditional_t<std::is_same_v<U, double>, T, U>;
};

template <typename T, typename U>
using promoted_numerical_t = typename promoted_numerical<T, U>::type;

}  // namespace geometry
}  // namespace drake
