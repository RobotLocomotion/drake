#include "drake/common/nice_type_name.h"

#include <complex>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/autodiff.h"

using std::string;

namespace drake {

// Need a non-anonymous namespace here for testing; can't canonicalize
// names in anonymous namespaces.
namespace nice_type_name_test {
enum class Color { Red, Green, Blue };

struct ForTesting {
  enum MyEnum { One, Two, Three };
  enum class MyEnumClass { Four, Five, Six };
};

class Base {
 public:
  virtual ~Base() = default;
};
class Derived : public Base {};
}  // namespace nice_type_name_test

namespace {
// Can't test much of NiceTypeName::Demangle because its behavior is compiler-
// and platform-specific. Everyone should agree on simple types though.
GTEST_TEST(NiceTypeNameTest, Demangle) {
  // NOLINTNEXTLINE(readability/casting) False positive.
  EXPECT_EQ(NiceTypeName::Demangle(typeid(bool).name()), "bool");
  // NOLINTNEXTLINE(readability/casting) False positive.
  EXPECT_EQ(NiceTypeName::Demangle(typeid(int).name()), "int");
  EXPECT_EQ(NiceTypeName::Demangle(typeid(unsigned).name()), "unsigned int");
}

// Standalone tests of the method that is used by NiceTypeName::Get<T>::Get()
// to clean up the demangled names on various platforms.
GTEST_TEST(NiceTypeNameTest, Canonicalize) {
  // Get rid of extra spaces and useless names like "class".
  EXPECT_EQ(NiceTypeName::Canonicalize("class std :: vector < double   >"),
            "std::vector<double>");
  // OSX's stl like to throw in these extra namespaces.
  EXPECT_EQ(NiceTypeName::Canonicalize("std:: __1 :: __23 :: set<T>"),
            "std::set<T>");

  // Should leave spaces between words.
  EXPECT_EQ(NiceTypeName::Canonicalize("lunch bucket"), "lunch bucket");
  // And keep funny looking namespaces if they aren't __something.
  EXPECT_EQ(NiceTypeName::Canonicalize("std::my__1::__23x::resigned char"),
            "std::my__1::resigned char");
}

GTEST_TEST(NiceTypeNameTest, BuiltIns) {
  EXPECT_EQ(NiceTypeName::Get<bool>(), "bool");
  EXPECT_EQ(NiceTypeName::Get<signed char>(), "signed char");
  EXPECT_EQ(NiceTypeName::Get<unsigned char>(), "unsigned char");
  EXPECT_EQ(NiceTypeName::Get<short>(), "short");  // NOLINT(runtime/int)
  EXPECT_EQ(NiceTypeName::Get<unsigned short>(),   // NOLINT(runtime/int)
            "unsigned short");
  EXPECT_EQ(NiceTypeName::Get<int>(), "int");
  EXPECT_EQ(NiceTypeName::Get<unsigned int>(), "unsigned int");
  EXPECT_EQ(NiceTypeName::Get<unsigned>(), "unsigned int");
  EXPECT_EQ(NiceTypeName::Get<long>(), "long");      // NOLINT(runtime/int)
  EXPECT_EQ(NiceTypeName::Get<long int>(), "long");  // NOLINT(runtime/int)
  EXPECT_EQ(NiceTypeName::Get<unsigned long>(),      // NOLINT(runtime/int)
            "unsigned long");
  EXPECT_EQ(NiceTypeName::Get<long long>(),  // NOLINT(runtime/int)
            "long long");
  EXPECT_EQ(NiceTypeName::Get<long long int>(),  // NOLINT(runtime/int)
            "long long");
  EXPECT_EQ(NiceTypeName::Get<unsigned long long>(),  // NOLINT(runtime/int)
            "unsigned long long");
  EXPECT_EQ(NiceTypeName::Get<float>(), "float");
  EXPECT_EQ(NiceTypeName::Get<double>(), "double");
  EXPECT_EQ(NiceTypeName::Get<long double>(), "long double");
  EXPECT_EQ(NiceTypeName::Get<std::complex<float>>(), "std::complex<float>");
  EXPECT_EQ(NiceTypeName::Get<std::complex<double>>(), "std::complex<double>");
  EXPECT_EQ(NiceTypeName::Get<std::complex<long double>>(),
            "std::complex<long double>");
}

GTEST_TEST(NiceTypeNameTest, StdClasses) {
  EXPECT_EQ(NiceTypeName::Get<std::string>(), "std::string");
  EXPECT_EQ(NiceTypeName::Get<string>(), "std::string");

  // Shouldn't be fooled by an alias or typedef.
  using MyStringAlias = std::string;
  typedef std::string MyStringTypedef;
  EXPECT_EQ(NiceTypeName::Get<MyStringAlias>(), "std::string");
  EXPECT_EQ(NiceTypeName::Get<MyStringTypedef>(), "std::string");

  // std::vector with default allocator.
  EXPECT_EQ(NiceTypeName::Get<std::vector<int>>(),
            "std::vector<int,std::allocator<int>>");
  EXPECT_EQ(NiceTypeName::Get<std::vector<std::string>>(),
            "std::vector<std::string,std::allocator<std::string>>");

  // Try non-standard allocator.
  using NonStdAlloc = std::vector<unsigned, std::allocator<unsigned>>;
  EXPECT_EQ(NiceTypeName::Get<NonStdAlloc>(),
            "std::vector<unsigned int,std::allocator<unsigned int>>");
}

GTEST_TEST(NiceTypeNameTest, Eigen) {
  EXPECT_EQ(NiceTypeName::Get<Eigen::Matrix2d>(), "Eigen::Matrix2d");
  EXPECT_EQ(NiceTypeName::Get<Eigen::Matrix3i>(), "Eigen::Matrix3i");
  EXPECT_EQ(NiceTypeName::Get<Eigen::Matrix3f>(), "Eigen::Matrix3f");
  EXPECT_EQ(NiceTypeName::Get<Eigen::Matrix3d>(), "Eigen::Matrix3d");
  EXPECT_EQ(NiceTypeName::Get<Eigen::MatrixXd>(), "Eigen::MatrixXd");

  EXPECT_EQ(NiceTypeName::Get<Eigen::Vector2d>(), "Eigen::Vector2d");
  EXPECT_EQ(NiceTypeName::Get<Eigen::Vector3i>(), "Eigen::Vector3i");
  EXPECT_EQ(NiceTypeName::Get<Eigen::Vector3f>(), "Eigen::Vector3f");
  EXPECT_EQ(NiceTypeName::Get<Eigen::Vector3d>(), "Eigen::Vector3d");
  EXPECT_EQ(NiceTypeName::Get<Eigen::VectorXd>(), "Eigen::VectorXd");

  EXPECT_EQ(NiceTypeName::Get<AutoDiffXd>(), "drake::AutoDiffXd");

  using PairType = std::pair<Eigen::Vector2i, Eigen::Vector3d>;
  EXPECT_EQ(NiceTypeName::Get<PairType>(),
            "std::pair<Eigen::Vector2i,Eigen::Vector3d>");
}

GTEST_TEST(NiceTypeNameTest, Enum) {
  EXPECT_EQ(NiceTypeName::Get<nice_type_name_test::Color>(),
            "drake::nice_type_name_test::Color");
  EXPECT_EQ(NiceTypeName::Get<nice_type_name_test::ForTesting>(),
            "drake::nice_type_name_test::ForTesting");
  EXPECT_EQ(NiceTypeName::Get<nice_type_name_test::ForTesting::MyEnum>(),
            "drake::nice_type_name_test::ForTesting::MyEnum");
  EXPECT_EQ(NiceTypeName::Get<nice_type_name_test::ForTesting::MyEnumClass>(),
            "drake::nice_type_name_test::ForTesting::MyEnumClass");

  EXPECT_EQ(NiceTypeName::Get<decltype(nice_type_name_test::ForTesting::One)>(),
            "drake::nice_type_name_test::ForTesting::MyEnum");
  EXPECT_EQ(NiceTypeName::Get<decltype(
                nice_type_name_test::ForTesting::MyEnumClass::Four)>(),
            "drake::nice_type_name_test::ForTesting::MyEnumClass");
}

// Test the expression-accepting form of NiceTypeName::Get().
GTEST_TEST(NiceTypeNameTest, Expressions) {
  using nice_type_name_test::Derived;
  using nice_type_name_test::Base;
  const int i = 10;
  const double d = 3.14;
  EXPECT_EQ(NiceTypeName::Get(i), "int");
  EXPECT_EQ(NiceTypeName::Get(d), "double");
  EXPECT_EQ(NiceTypeName::Get(i * d), "double");

  Derived derived;
  Base* base = &derived;

  // These both have the same name despite different declarations because
  // they resolve to the same concrete type.
  EXPECT_EQ(NiceTypeName::Get(derived), "drake::nice_type_name_test::Derived");
  EXPECT_EQ(NiceTypeName::Get(*base), "drake::nice_type_name_test::Derived");

  // OTOH, these differ because we get only the declared types.
  EXPECT_NE(NiceTypeName::Get<decltype(*base)>(),
            NiceTypeName::Get<decltype(derived)>());

  auto derived_uptr = std::make_unique<Derived>();
  auto base_uptr = std::unique_ptr<Base>(new Derived());
  EXPECT_EQ(NiceTypeName::Get(*derived_uptr),
            "drake::nice_type_name_test::Derived");
  EXPECT_EQ(NiceTypeName::Get(*base_uptr),
            "drake::nice_type_name_test::Derived");

  // unique_ptr is not polymorphic (unlike its contents) so its declared type
  // and runtime type are the same.
  EXPECT_EQ(NiceTypeName::Get<decltype(base_uptr)>(),
            NiceTypeName::Get(base_uptr));
}

}  // namespace
}  // namespace drake
