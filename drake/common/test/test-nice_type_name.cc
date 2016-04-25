#include "drake/common/nice_type_name.h"

#include <complex>
#include <string>
#include <utility>
#include <vector>

#include <iostream>

#include <Eigen/Dense>
#include "gtest/gtest.h"

using std::string;

namespace drake {
namespace common {

// Need a non-anonymous namespace here for testing; can't canonicalize
// names in anonymous namespaces.
namespace test_nice_type_name { 
enum class Color {Red, Green, Blue};

struct ForTesting {
    enum MyEnum {One,Two,Three};
    enum class MyEnumClass {Four,Five,Six};
};
}

namespace {
// Can't test much of DemangleTypeName because its behavior is compiler-
// and platform-specific. Everyone should agree on simple types though.
GTEST_TEST(TestNiceTypeName,Demangle) {
    EXPECT_EQ(DemangleTypeName(typeid(bool).name()), "bool");
    EXPECT_EQ(DemangleTypeName(typeid(int).name()), "int");
    EXPECT_EQ(DemangleTypeName(typeid(unsigned).name()), "unsigned int");
}

// Standalone tests of the method that is used by NiceTypeName<T>::Get()
// to clean up the demangled names on various platforms.
GTEST_TEST(TestNiceTypeName,Canonicalize) {
    // Get rid of extra spaces and useless names like "class".
    EXPECT_EQ(CanonicalizeTypeName("class std :: vector < double   >"),
              "std::vector<double>");
    // OSX's stl like to throw in these extra namespaces.
    EXPECT_EQ(CanonicalizeTypeName("std:: __1 :: __23 :: set<T>"),
              "std::set<T>");

    // Should leaves spaces between words.
    EXPECT_EQ(CanonicalizeTypeName("lunch bucket"), "lunch bucket");
    // And keep funny looking namespaces if they aren't __digits.
    EXPECT_EQ(CanonicalizeTypeName("std::my__1::__23x::resigned char"),
              "std::my__1::__23x::resigned char");
}

GTEST_TEST(TestNiceTypeName, BuiltIns) {
  EXPECT_EQ(NiceTypeName<bool>::Get(), "bool");
  EXPECT_EQ(NiceTypeName<signed char>::Get(), "signed char");
  EXPECT_EQ(NiceTypeName<unsigned char>::Get(), "unsigned char");
  EXPECT_EQ(NiceTypeName<short>::Get(), "short");
  EXPECT_EQ(NiceTypeName<unsigned short>::Get(), "unsigned short");
  EXPECT_EQ(NiceTypeName<int>::Get(), "int");
  EXPECT_EQ(NiceTypeName<unsigned int>::Get(), "unsigned int");
  EXPECT_EQ(NiceTypeName<unsigned>::Get(), "unsigned int");
  EXPECT_EQ(NiceTypeName<long>::Get(), "long");
  EXPECT_EQ(NiceTypeName<unsigned long>::Get(), "unsigned long");
  EXPECT_EQ(NiceTypeName<long long>::Get(), "long long");
  EXPECT_EQ(NiceTypeName<unsigned long long>::Get(), "unsigned long long");
  EXPECT_EQ(NiceTypeName<float>::Get(), "float");
  EXPECT_EQ(NiceTypeName<double>::Get(), "double");
  EXPECT_EQ(NiceTypeName<long double>::Get(), "long double");
  EXPECT_EQ(NiceTypeName<std::complex<float>>::Get(), 
            "std::complex<float>");
  EXPECT_EQ(NiceTypeName<std::complex<double>>::Get(),
            "std::complex<double>");
  EXPECT_EQ(NiceTypeName<std::complex<long double>>::Get(),
            "std::complex<long double>");
}

GTEST_TEST(TestNiceTypeName,StdClasses) {
  const std::string string_name =  
    "std::basic_string<char,std::char_traits<char>,std::allocator<char>>";

  EXPECT_EQ(NiceTypeName<std::string>::Get(), string_name);
  EXPECT_EQ(NiceTypeName<string>::Get(), string_name);

  // Shouldn't be fooled by an alias or typedef.
  using MyStringAlias = std::string;
  typedef std::string MyStringTypedef;
  EXPECT_EQ(NiceTypeName<MyStringAlias>::Get(), string_name);
  EXPECT_EQ(NiceTypeName<MyStringTypedef>::Get(), string_name);

  // std::vector with default allocator.
  EXPECT_EQ(NiceTypeName<std::vector<int>>::Get(), 
    "std::vector<int,std::allocator<int>>");
  EXPECT_EQ(NiceTypeName<std::vector<std::string>>::Get(), 
    "std::vector<" + string_name + ",std::allocator<" + string_name + ">>");

  // Try non-standard allocator.
  using NonStdAlloc = std::vector<int,std::allocator<unsigned>>;
  EXPECT_EQ(NiceTypeName<NonStdAlloc>::Get(), 
    "std::vector<int,std::allocator<unsigned int>>");
}


GTEST_TEST(TestNiceTypeName,Eigen) {
  EXPECT_EQ(NiceTypeName<Eigen::Matrix3f>::Get(), 
    "Eigen::Matrix<float,3,3,0,3,3>");

  EXPECT_EQ(NiceTypeName<NiceTypeName<Eigen::Vector2i>>::Get(),
    "drake::common::NiceTypeName<Eigen::Matrix<int,2,1,0,2,1>>");
}

GTEST_TEST(TestNiceTypeName,Enum) {
  EXPECT_EQ(NiceTypeName<test_nice_type_name::Color>::Get(),
    "drake::common::test_nice_type_name::Color");
  EXPECT_EQ(NiceTypeName<test_nice_type_name::ForTesting>::Get(),
    "drake::common::test_nice_type_name::ForTesting");
  EXPECT_EQ(NiceTypeName<test_nice_type_name::ForTesting::MyEnum>::Get(),
    "drake::common::test_nice_type_name::ForTesting::MyEnum");
  EXPECT_EQ(NiceTypeName<test_nice_type_name::ForTesting::MyEnumClass>::Get(),
    "drake::common::test_nice_type_name::ForTesting::MyEnumClass");

  EXPECT_EQ(NiceTypeName<decltype(test_nice_type_name::ForTesting::One)>::Get(), 
    "drake::common::test_nice_type_name::ForTesting::MyEnum");
  EXPECT_EQ(NiceTypeName<decltype(test_nice_type_name::ForTesting
                                  ::MyEnumClass::Four)>::Get(), 
    "drake::common::test_nice_type_name::ForTesting::MyEnumClass");
}

}  // namespace
}  // namespace common
}  // namespace drake
