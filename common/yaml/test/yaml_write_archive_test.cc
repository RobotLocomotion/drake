#include "drake/common/yaml/yaml_write_archive.h"

#include <vector>

#include <fmt/args.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/name_value.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/test/example_structs.h"

// This test suite is the C++ flavor of the Python test suite at
//  drake/bindings/pydrake/common/test/yaml_typed_test.py
// and should be roughly kept in sync with the test cases in that file.

namespace drake {
namespace yaml {
namespace test {
namespace {

using internal::YamlWriteArchive;

// A test fixture with common helpers.
class YamlWriteArchiveTest : public ::testing::Test {
 public:
  template <typename Serializable>
  static std::string Save(const Serializable& data,
                          const std::string& root_name) {
    YamlWriteArchive archive;
    archive.Accept(data);
    return archive.EmitString(root_name);
  }

  template <typename Serializable>
  static std::string Save(const Serializable& data) {
    return Save(data, "doc");
  }

  static std::string WrapDoc(const std::string& value) {
    return "doc:\n  value: " + value + "\n";
  }
};

TEST_F(YamlWriteArchiveTest, Double) {
  const auto test = [](const double value, const std::string& expected) {
    const DoubleStruct x{value};
    EXPECT_EQ(Save(x), WrapDoc(expected));
  };

  test(0.0, "0.0");
  test(1.0, "1.0");
  test(-1.0, "-1.0");

  test(0.009, "0.009");

  test(1.2, "1.2");
  test(-1.2, "-1.2");

  test(5.6e+16, "5.6e+16");
  test(5.6e-12, "5.6e-12");
  test(-5.6e+16, "-5.6e+16");
  test(-5.6e-12, "-5.6e-12");

  // See https://yaml.org/spec/1.2.2/#10214-floating-point for the specs.
  test(std::numeric_limits<double>::quiet_NaN(), ".nan");
  test(std::numeric_limits<double>::infinity(), ".inf");
  test(-std::numeric_limits<double>::infinity(), "-.inf");
}

TEST_F(YamlWriteArchiveTest, String) {
  const auto test = [](const std::string& value, const std::string& expected) {
    const StringStruct x{value};
    EXPECT_EQ(Save(x), WrapDoc(expected));
  };

  // We'll use these named fmt args to help make our expected values clear.
  fmt::dynamic_format_arg_store<fmt::format_context> args;
  args.push_back(fmt::arg("bs", '\\'));  // backslash
  args.push_back(fmt::arg("dq", '"'));   // double quote

  // Plain string.
  test("a", "a");

  // Needs quoting for special characters. Note that there are several valid
  // ways to quote and/or escape these, but for now we just check against the
  // exact choice that yaml-cpp uses. In the future if we see new outputs, we
  // could allow them too.
  test("'", fmt::vformat("{dq}'{dq}", args));
  test("\"", fmt::vformat("{dq}{bs}{dq}{dq}", args));

  // Needs quoting to avoid being misinterpreted as another data type.
  test("1", "'1'");
  test("1.0", "'1.0'");
  test(".NaN", "'.NaN'");
  test("true", "'true'");
  test("null", "'null'");
}

TEST_F(YamlWriteArchiveTest, StdArray) {
  const auto test = [](const std::array<double, 3>& value,
                       const std::string& expected) {
    const ArrayStruct x{value};
    EXPECT_EQ(Save(x), expected);
  };

  test({1.0, 2.0, 3.0}, R"""(doc:
  value: [1.0, 2.0, 3.0]
)""");
}

TEST_F(YamlWriteArchiveTest, StdVector) {
  const auto test = [](const std::vector<double>& value,
                       const std::string& expected) {
    const VectorStruct x{value};
    EXPECT_EQ(Save(x), expected);
  };

  // When the vector items are simple YAML scalars, we should use
  // "flow" style, where they all appear on a single line.
  test({}, R"""(doc:
  value: []
)""");

  test({1.0, 2.0, 3.0}, R"""(doc:
  value: [1.0, 2.0, 3.0]
)""");

  // When the vector items are not simple scalars, we should use
  // "block" style, where each gets its own line(s).
  NonPodVectorStruct x;
  x.value = {
      {.value = "foo"},
      {.value = "bar"},
  };
  EXPECT_EQ(Save(x), R"""(doc:
  value:
    - value: foo
    - value: bar
)""");
}

TEST_F(YamlWriteArchiveTest, StdMap) {
  const auto test = [](const std::map<std::string, double>& value,
                       const std::string& expected) {
    const MapStruct x{value};
    EXPECT_EQ(Save(x), expected);
  };

  test({}, R"""(doc:
  value: {}
)""");

  test({{"foo", 0.0}}, R"""(doc:
  value:
    foo: 0.0
)""");

  test({{"foo", 0.0}, {"bar", 1.0}}, R"""(doc:
  value:
    bar: 1.0
    foo: 0.0
)""");
}

TEST_F(YamlWriteArchiveTest, StdUnorderedMap) {
  const auto test = [](const std::unordered_map<std::string, double>& value,
                       const std::string& expected) {
    const UnorderedMapStruct x{value};
    EXPECT_EQ(Save(x), expected);
  };

  // The YamlWriteArchive API promises that the output must be deterministic.
  // Here, we test a stronger condition that it's always sorted.  (If in the
  // future we decide to use a different order, we can update here to match.)
  const std::unordered_map<std::string, double> value{
      // Use many values to increase the chance that we'll detect
      // implementations that fail to sort.
      {"gulf", 6.0},    {"fox", 5.0},   {"echo", 4.0},  {"delta", 3.0},
      {"charlie", 2.0}, {"bravo", 1.0}, {"alpha", 0.0},
  };
  test(value,
       R"""(doc:
  value:
    alpha: 0.0
    bravo: 1.0
    charlie: 2.0
    delta: 3.0
    echo: 4.0
    fox: 5.0
    gulf: 6.0
)""");
}

TEST_F(YamlWriteArchiveTest, StdMapDirectly) {
  const auto test = [](const std::map<std::string, double>& value,
                       const std::string& expected) {
    EXPECT_EQ(Save(value), expected);
  };

  test({}, R"""(doc:
)""");

  test({{"foo", 0.0}}, R"""(doc:
  foo: 0.0
)""");

  test({{"foo", 0.0}, {"bar", 1.0}}, R"""(doc:
  bar: 1.0
  foo: 0.0
)""");
}

TEST_F(YamlWriteArchiveTest, Optional) {
  const auto test = [](const std::optional<double>& value,
                       const std::string& expected) {
    const OptionalStruct x{value};
    EXPECT_EQ(Save(x), expected);
  };

  test(std::nullopt, "doc:\n");
  test(1.0, "doc:\n  value: 1.0\n");
}

TEST_F(YamlWriteArchiveTest, Variant) {
  const auto test = [](const Variant4& value, const std::string& expected) {
    const VariantStruct x{value};
    EXPECT_EQ(Save(x), WrapDoc(expected));
  };

  test(Variant4(std::string()), "''");
  test(Variant4(std::string("foo")), "foo");
  test(Variant4(1.0), "!!float 1.0");
  test(Variant4(DoubleStruct{1.0}), "!DoubleStruct\n    value: 1.0");
  test(Variant4(EigenVecStruct{Eigen::Vector2d(1.0, 2.0)}),
       "!EigenStruct\n    value: [1.0, 2.0]");
}

TEST_F(YamlWriteArchiveTest, PrimitiveVariant) {
  const auto test = [](const PrimitiveVariant& value,
                       const std::string& expected) {
    const PrimitiveVariantStruct x{value};
    EXPECT_EQ(Save(x), WrapDoc(expected));
  };

  test(std::vector<double>{1.0, 2.0}, "[1.0, 2.0]");
  test(true, "!!bool true");
  test(10, "!!int 10");
  test(1.0, "!!float 1.0");
  test(std::string("foo"), "!!str foo");
}

TEST_F(YamlWriteArchiveTest, EigenVector) {
  const auto test = [](const Eigen::VectorXd& value,
                       const std::string& expected) {
    const EigenVecStruct x{value};
    EXPECT_EQ(Save(x), expected);
    const EigenVec3Struct x3{value};
    EXPECT_EQ(Save(x3), expected);
  };

  test(Eigen::Vector3d(1.0, 2.0, 3.0), R"""(doc:
  value: [1.0, 2.0, 3.0]
)""");
}

TEST_F(YamlWriteArchiveTest, EigenVectorX) {
  const auto test = [](const Eigen::VectorXd& value,
                       const std::string& expected) {
    const EigenVecStruct x{value};
    EXPECT_EQ(Save(x), expected);
  };

  test(Eigen::VectorXd(), R"""(doc:
  value: []
)""");
  test(Eigen::Matrix<double, 1, 1>(1.0), R"""(doc:
  value: [1.0]
)""");
}

TEST_F(YamlWriteArchiveTest, EigenMatrix) {
  using Matrix34d = Eigen::Matrix<double, 3, 4>;
  const auto test = [](const Matrix34d& value, const std::string& expected) {
    const EigenMatrixStruct x{value};
    EXPECT_EQ(Save(x), expected);
    const EigenMatrix34Struct x3{value};
    EXPECT_EQ(Save(x3), expected);
  };

  test((Matrix34d{} << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11).finished(),
       R"""(doc:
  value:
    - [0.0, 1.0, 2.0, 3.0]
    - [4.0, 5.0, 6.0, 7.0]
    - [8.0, 9.0, 10.0, 11.0]
)""");
}

TEST_F(YamlWriteArchiveTest, EigenMatrixUpTo6) {
  using Matrix34d = Eigen::Matrix<double, 3, 4>;
  const auto test = [](const Matrix34d& value, const std::string& expected) {
    const EigenMatrixUpTo6Struct x{value};
    EXPECT_EQ(Save(x), expected);
  };

  test((Matrix34d{} << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11).finished(),
       R"""(doc:
  value:
    - [0.0, 1.0, 2.0, 3.0]
    - [4.0, 5.0, 6.0, 7.0]
    - [8.0, 9.0, 10.0, 11.0]
)""");
}

TEST_F(YamlWriteArchiveTest, EigenMatrix00) {
  const auto test = [](const std::string& expected) {
    const Eigen::MatrixXd empty;
    const EigenMatrixStruct x{empty};
    EXPECT_EQ(Save(x), expected);
    const EigenMatrix00Struct x00;
    EXPECT_EQ(Save(x00), expected);
  };

  test(R"""(doc:
  value: []
)""");
}

TEST_F(YamlWriteArchiveTest, Nested) {
  OuterStruct x;
  x.inner_struct.inner_value = 2.0;
  x.outer_value = 1.0;

  const std::string saved = Save(x);
  const std::string expected = R"""(doc:
  outer_value: 1.0
  inner_struct:
    inner_value: 2.0
)""";
  EXPECT_EQ(saved, expected);
}

TEST_F(YamlWriteArchiveTest, BlankInner) {
  OuterWithBlankInner x;
  x.outer_value = 1.0;

  const std::string saved = Save(x);
  const std::string expected = R"""(doc:
  outer_value: 1.0
  inner_struct: {}
)""";
  EXPECT_EQ(saved, expected);
}

// Test the Emitter function given different key names and a provided
// serializable content.
TEST_F(YamlWriteArchiveTest, EmitterWithProvidedSerializable) {
  const auto test = [](const std::string& root_name,
                       const std::string& expected) {
    const DoubleStruct x{1.0};
    EXPECT_EQ(Save(x, root_name), expected);
  };

  // Test default key name.
  const std::string expected_default_root_name = R"""(root:
  value: 1.0
)""";
  test("root", expected_default_root_name);

  // Test empty key name.
  const std::string expected_empty_root_name = R"""(value: 1.0
)""";
  test("", expected_empty_root_name);
}

// Test the Emitter function given different key names. The serializable
// content is empty.
TEST_F(YamlWriteArchiveTest, EmitterNoProvidedSerializable) {
  const auto test = [](const std::string& root_name,
                       const std::string& expected) {
    YamlWriteArchive archive;
    EXPECT_EQ(archive.EmitString(root_name), expected);
  };

  // Test non-empty key name will return a node with empty value.
  const std::string expected_default_root_name = R"""(root:
)""";
  test("root", expected_default_root_name);

  // Test empty key name will yield an empty map.
  const std::string expected_empty_root_name = R"""({}
)""";
  test("", expected_empty_root_name);
}

}  // namespace
}  // namespace test
}  // namespace yaml
}  // namespace drake
