#include "drake/common/nice_type_name.h"

#include <complex>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

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
}

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
  EXPECT_EQ(NiceTypeName::Get<Eigen::Matrix3f>(),
            "Eigen::Matrix<float,3,3,0,3,3>");

  using PairType = std::pair<Eigen::Vector2i, Eigen::Vector3d>;
  EXPECT_EQ(NiceTypeName::Get<PairType>(),
            "std::pair<Eigen::Matrix<int,2,1,0,2,1>,"
            "Eigen::Matrix<double,3,1,0,3,1>>");
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

}  // namespace
}  // namespace drake
