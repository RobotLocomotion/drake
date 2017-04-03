#include <iostream>
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
/// // The following assertion fails *with* diagnostic info.
/// EXPECT_FALSE(of_dynamic<Camry>(prius));
/// // Output:
/// // of_dynamic<Camry>(Car* ptr) failed because ptr is of dynamic type Prius.
/// @endcode
template <typename Derived, typename Base>
bool of_dynamic(const Base* const ptr) {
  if (dynamic_cast<const Derived* const>(ptr) == nullptr) {
    const std::string base_name{NiceTypeName::Demangle(typeid(Base).name())};
    const std::string derived_name{
        NiceTypeName::Demangle(typeid(Derived).name())};
    const std::string dynamic_name{NiceTypeName::Demangle(typeid(*ptr).name())};
    std::cerr << "of_dynamic<" << derived_name << ">(" << base_name << "* ptr)"
              << " failed because ptr is of dynamic type " << dynamic_name
              << "." << std::endl;
    return false;
  } else {
    return true;
  }
}

/// Checks if @ptr, a shared pointer to `Base` class, can be safely downcasted
/// to a shared pointer to `Derived` class. Our motivation is to provide a good
/// diagnostic for what @p ptr _actually_ was when a test fails.
template <typename Derived, typename Base>
bool of_dynamic(const std::shared_ptr<Base> ptr) {
  return of_dynamic<Derived, Base>(ptr.get());
}
}  // namespace drake
