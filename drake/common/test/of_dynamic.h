#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/nice_type_name.h"

namespace drake {
/// Checks if @ptr, a pointer to `Base` class, can be safely downcasted to a
/// pointer to `Derived` class. Our motivation is to provide a good diagnostic
/// for what @p ptr _actually_ was when a test fails.
///
/// Here is an illustrative code snippet. We assume that `Prius` and `Camry` are
/// derived classes of `Car`.
///
/// @code
/// const Car* prius = new Prius{};
///
/// // The following assertion fails without diagnostic info.
/// EXPECT_TRUE(dynamic_cast<Camry>(ptr) != nullptr)
///
/// // The following assertion returns `::testing::AssertionFailure()` with
/// // diagnostic info attached.
/// EXPECT_TRUE(of_dynamic<Camry>(prius));
/// // Output:
/// // Value of: of_dynamic<Camry>(prius)
/// // Actual: false (of_dynamic<Camry>(Car* ptr) failed
/// //                because ptr is of dynamic type Prius.)
/// // Expected: true
/// @endcode
template <typename Derived, typename Base>
::testing::AssertionResult of_dynamic(const Base* ptr) {
  if (dynamic_cast<const Derived* const>(ptr) == nullptr) {
    const std::string base_name{NiceTypeName::Get<Base>()};
    const std::string derived_name{NiceTypeName::Get<Derived>()};
    const std::string dynamic_name{NiceTypeName::Canonicalize(
        NiceTypeName::Demangle(typeid(*ptr).name()))};
    return ::testing::AssertionFailure()
           << "of_dynamic<" << derived_name << ">(" << base_name << "* ptr)"
           << " failed because ptr is of dynamic type " << dynamic_name << ".";
  }
  return ::testing::AssertionSuccess();
}

/// Checks if @ptr, a shared pointer to `Base` class, can be safely downcasted
/// to a shared pointer to `Derived` class. Our motivation is to provide a good
/// diagnostic for what @p ptr _actually_ was when a test fails.
template <typename Derived, typename Base>
::testing::AssertionResult of_dynamic(std::shared_ptr<Base> ptr) {
  return of_dynamic<Derived, Base>(ptr.get());
}
}  // namespace drake
