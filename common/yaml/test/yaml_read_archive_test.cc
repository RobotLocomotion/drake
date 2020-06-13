#include "drake/common/yaml/yaml_read_archive.h"

#include <cmath>
#include <limits>
#include <sstream>
#include <vector>

#include <Eigen/Core>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/name_value.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

// TODO(jwnimmer-tri) All of these regexps would be better off using the
// std::regex::basic grammar, where () and {} are not special characters.

namespace {

// These unit tests use a variety of sample Serializable structs, showing what
// a user may write for their own schemas.

struct DoubleStruct {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  double value = NAN;
};

bool operator==(const DoubleStruct& a, const DoubleStruct& b) {
  return a.value == b.value;
}

struct ArrayStruct {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  ArrayStruct() {
    value.fill(NAN);
  }

  std::array<double, 3> value;
};

struct VectorStruct {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  VectorStruct() {
    value.resize(1, NAN);
  }

  std::vector<double> value;
};

struct MapStruct {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  MapStruct() {
    value["NAN"] = NAN;
  }

  std::map<std::string, double> value;
};

struct OptionalStruct {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  OptionalStruct() {
    value = NAN;
  }

  std::optional<double> value;
};

using Variant3 = std::variant<std::string, double, DoubleStruct>;

std::ostream& operator<<(std::ostream& os, const Variant3& value) {
  if (value.index() == 0) {
    os << "std::string{" << std::get<0>(value) << "}";
  } else if (value.index() == 1) {
    os << "double{" << std::get<1>(value) << "}";
  } else {
    os << "DoubleStruct{" << std::get<2>(value).value << "}";
  }
  return os;
}

struct VariantStruct {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  VariantStruct() {
    value = NAN;
  }

  Variant3 value;
};

struct VariantWrappingStruct {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(inner));
  }

  VariantStruct inner;
};

template <int Rows, int Cols>
struct EigenStruct {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  EigenStruct() {
    if (value.size() == 0) {
      value.resize(1, 1);
    }
    value.setConstant(NAN);
  }

  Eigen::Matrix<double, Rows, Cols> value;
};

using EigenVecStruct = EigenStruct<Eigen::Dynamic, 1>;
using EigenVec3Struct = EigenStruct<3, 1>;
using EigenMatrixStruct = EigenStruct<Eigen::Dynamic, Eigen::Dynamic>;
using EigenMatrix34Struct = EigenStruct<3, 4>;

struct OuterStruct {
  struct InnerStruct {
    double inner_value = NAN;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(DRAKE_NVP(inner_value));
    }
  };

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(outer_value));
    a->Visit(DRAKE_NVP(inner_struct));
  }

  double outer_value = NAN;
  InnerStruct inner_struct;
};

}  // namespace

namespace drake {
namespace yaml {
namespace {

// A test fixture with common helpers.
class YamlReadArchiveTest
    : public ::testing::TestWithParam<YamlReadArchive::Options> {
 public:
  // Loads a single "doc: { ... }" map from `contents` and returns the nested
  // map (i.e., just the "{ ... }" part, not the "doc" part).  It is an error
  // for the "{ ... }" part not to be a map node.
  static YAML::Node Load(const std::string& contents) {
    const YAML::Node loaded = YAML::Load(contents);
    if (loaded.Type() != YAML::NodeType::Map) {
      throw std::runtime_error("Bad contents parse " + contents);
    }
    const YAML::Node doc = loaded["doc"];
    if (doc.Type() != YAML::NodeType::Map) {
      throw std::runtime_error("Bad doc parse " + contents);
    }
    return doc;
  }

  // Loads a single "{ value: something }" map node.  If the argument is the
  // empty string, the result is a map from "value" to Null (not an empty map,
  // nor Null itself, etc.)
  static YAML::Node LoadSingleValue(const std::string& value) {
    return Load("doc:\n  value: " + value + "\n");
  }

  // Parses root into a Serializable and returns the result of the parse.
  // Any exceptions raised are reported as errors.
  template <typename Serializable>
  static Serializable AcceptNoThrow(const YAML::Node& root) {
    SCOPED_TRACE("for type " + NiceTypeName::Get<Serializable>());
    Serializable result{};
    bool raised = false;
    std::string what;
    try {
      YamlReadArchive(root, GetParam()).Accept(&result);
    } catch (const std::exception& e) {
      raised = true;
      what = e.what();
    }
    EXPECT_FALSE(raised);
    EXPECT_EQ(what, "");
    return result;
  }

  // Parses root into a Serializable and discards the result.
  // This is usually used to check that an exception is raised.
  template <typename Serializable>
  static void AcceptIntoDummy(const YAML::Node& root) {
    Serializable dummy{};
    YamlReadArchive(root, GetParam()).Accept(&dummy);
  }

  // Parses root into a Serializable and returns the result of the parse.
  // If allow_cpp_with_no_yaml is set, then any exceptions are errors.
  // If allow_cpp_with_no_yaml is not set, then lack of exception is an error.
  template <typename Serializable>
  static Serializable AcceptEmptyDoc() {
    SCOPED_TRACE("for type " + NiceTypeName::Get<Serializable>());
    const YAML::Node root = Load("doc: {}");
    Serializable result{};
    bool raised = false;
    std::string what;
    try {
      YamlReadArchive(root, GetParam()).Accept(&result);
    } catch (const std::exception& e) {
      raised = true;
      what = e.what();
    }
    if (GetParam().allow_cpp_with_no_yaml) {
      EXPECT_FALSE(raised);
      EXPECT_EQ(what, "");
    } else {
      EXPECT_TRUE(raised);
      EXPECT_THAT(what, testing::MatchesRegex(
          ".*missing entry for [^ ]* value.*"));
    }
    return result;
  }
};

TEST_P(YamlReadArchiveTest, Double) {
  const auto test = [](const std::string& value, double expected) {
    const auto& x = AcceptNoThrow<DoubleStruct>(LoadSingleValue(value));
    EXPECT_EQ(x.value, expected);
  };

  test("0", 0.0);
  test("1", 1.0);
  test("-1", -1.0);

  test("0.0", 0.0);
  test("1.2", 1.2);
  test("-1.2", -1.2);

  test("3e4", 3e4);
  test("3e-4", 3e-4);
  test("5.6e7", 5.6e7);
  test("5.6e-7", 5.6e-7);
  test("-5.6e7", -5.6e7);
  test("-5.6e-7", -5.6e-7);

  test("3E4", 3e4);
  test("3E-4", 3e-4);
  test("5.6E7", 5.6e7);
  test("5.6E-7", 5.6e-7);
  test("-5.6E7", -5.6e7);
  test("-5.6E-7", -5.6e-7);
}

TEST_P(YamlReadArchiveTest, DoubleMissing) {
  const auto& x = AcceptEmptyDoc<DoubleStruct>();
  EXPECT_THAT(x.value, testing::NanSensitiveDoubleEq(NAN));
}

TEST_P(YamlReadArchiveTest, StdArray) {
  const auto test = [](const std::string& value,
                       const std::array<double, 3>& expected) {
    const auto& x = AcceptNoThrow<ArrayStruct>(LoadSingleValue(value));
    EXPECT_EQ(x.value, expected);
  };

  test("[1.0, 2.0, 3.0]", {1.0, 2.0, 3.0});
}

TEST_P(YamlReadArchiveTest, StdArrayMissing) {
  const auto& x = AcceptEmptyDoc<ArrayStruct>();
  EXPECT_THAT(x.value[0], testing::NanSensitiveDoubleEq(NAN));
  EXPECT_THAT(x.value[1], testing::NanSensitiveDoubleEq(NAN));
  EXPECT_THAT(x.value[2], testing::NanSensitiveDoubleEq(NAN));
}

TEST_P(YamlReadArchiveTest, StdVector) {
  const auto test = [](const std::string& value,
                       const std::vector<double>& expected) {
    const auto& x = AcceptNoThrow<VectorStruct>(LoadSingleValue(value));
    EXPECT_EQ(x.value, expected);
  };

  test("[1.0, 2.0, 3.0]", {1.0, 2.0, 3.0});
}

TEST_P(YamlReadArchiveTest, StdVectorMissing) {
  const auto& x = AcceptEmptyDoc<VectorStruct>();
  ASSERT_EQ(x.value.size(), 1);
  EXPECT_THAT(x.value[0], testing::NanSensitiveDoubleEq(NAN));
}

TEST_P(YamlReadArchiveTest, StdMap) {
  const auto test = [](const std::string& doc,
                       const std::map<std::string, double>& expected) {
    const auto& x = AcceptNoThrow<MapStruct>(Load(doc));
    EXPECT_EQ(x.value, expected) << doc;
  };

  test("doc:\n  value:\n    foo: 0.0\n    bar: 1.0\n",
       {{"foo", 0.0}, {"bar", 1.0}});
}

TEST_P(YamlReadArchiveTest, StdMapMissing) {
  const auto& x = AcceptEmptyDoc<MapStruct>();
  ASSERT_EQ(x.value.size(), 1);
  EXPECT_THAT(x.value.at("NAN"), testing::NanSensitiveDoubleEq(NAN));
}

TEST_P(YamlReadArchiveTest, StdMapWithMergeKeys) {
  const auto test = [](const std::string& doc,
                       const std::map<std::string, double>& expected) {
    const auto& x = AcceptNoThrow<MapStruct>(Load(doc));
    EXPECT_EQ(x.value, expected) << doc;
  };

  // Use merge keys to populate some keys.
  test(R"R(
_template: &template
  foo: 1.0

doc:
  value:
    << : *template
    bar: 2.0
)R", {{"foo", 1.0}, {"bar", 2.0}});

  // A pre-existing value should win, though.
  test(R"R(
_template: &template
  foo: 3.0

doc:
  value:
    << : *template
    foo: 1.0
    bar: 2.0
)R", {{"foo", 1.0}, {"bar", 2.0}});

  // A list of merges should also work.
  test(R"R(
_template: &template
  - foo: 1.0
  - baz: 3.0

doc:
  value:
    << : *template
    bar: 2.0
)R", {{"foo", 1.0}, {"bar", 2.0}, {"baz", 3.0}});
}

TEST_P(YamlReadArchiveTest, StdMapWithBadMergeKey) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<MapStruct>(Load(R"R(
_template: &template 99.0

doc:
  value:
    << : *template
    bar: 2.0
)R")),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " has invalid merge key type \\(Scalar\\) within entry"
      " for std::map<[^ ]*> value\\.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<MapStruct>(Load(R"R(
_template: &template

doc:
  value:
    << : *template
    bar: 2.0
)R")),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " has invalid merge key type \\(Null\\) within entry"
      " for std::map<[^ ]*> value\\.");
}

TEST_P(YamlReadArchiveTest, Optional) {
  const auto test = [](const std::string& doc,
                       const std::optional<double>& expected) {
    const auto& x = AcceptNoThrow<OptionalStruct>(Load(doc));
    if (expected.has_value()) {
      ASSERT_TRUE(x.value.has_value()) << *expected;
      EXPECT_EQ(x.value.value(), expected.value()) << *expected;
    } else {
      EXPECT_FALSE(x.value.has_value());
    }
  };

  test("doc: {}", std::nullopt);
  if (GetParam().allow_yaml_with_no_cpp) {
    test("doc:\n  foo: bar", std::nullopt);
  }
  test("doc:\n  value:", std::nullopt);
  test("doc:\n  value: 1.0", 1.0);
}

TEST_P(YamlReadArchiveTest, Variant) {
  const auto test = [](const std::string& doc,
                       const Variant3& expected) {
    const auto& x = AcceptNoThrow<VariantStruct>(Load(doc));
    EXPECT_EQ(x.value, expected) << doc;
  };

  test("doc:\n  value: !!str foo", "foo");
  test("doc:\n  value: !!float 1.0", 1.0);
  test("doc:\n  value: !DoubleStruct { value: 1.0 }", DoubleStruct{1.0});
}

TEST_P(YamlReadArchiveTest, VariantMissing) {
  const auto& x = AcceptEmptyDoc<VariantStruct>();
  EXPECT_THAT(std::get<double>(x.value), testing::NanSensitiveDoubleEq(NAN));
}

TEST_P(YamlReadArchiveTest, EigenVector) {
  const auto test = [](const std::string& value,
                       const Eigen::VectorXd& expected) {
    const auto& vec = AcceptNoThrow<EigenVecStruct>(LoadSingleValue(value));
    const auto& vec3 = AcceptNoThrow<EigenVec3Struct>(LoadSingleValue(value));
    EXPECT_TRUE(drake::CompareMatrices(vec.value, expected));
    EXPECT_TRUE(drake::CompareMatrices(vec3.value, expected));
  };

  test("[1.0, 2.0, 3.0]", Eigen::Vector3d(1.0, 2.0, 3.0));
}

TEST_P(YamlReadArchiveTest, EigenVectorX) {
  const auto test = [](const std::string& value,
                       const Eigen::VectorXd& expected) {
    const auto& x = AcceptNoThrow<EigenVecStruct>(LoadSingleValue(value));
    EXPECT_TRUE(drake::CompareMatrices(x.value, expected));
  };

  test("[]", Eigen::VectorXd());
  test("[1.0]", Eigen::Matrix<double, 1, 1>(1.0));
}

TEST_P(YamlReadArchiveTest, EigenMatrix) {
  using Matrix34d = Eigen::Matrix<double, 3, 4>;
  const auto test = [](const std::string& doc,
                       const Eigen::MatrixXd& expected) {
    const auto& mat = AcceptNoThrow<EigenMatrixStruct>(Load(doc));
    const auto& mat34 = AcceptNoThrow<EigenMatrix34Struct>(Load(doc));
    EXPECT_TRUE(drake::CompareMatrices(mat.value, expected));
    EXPECT_TRUE(drake::CompareMatrices(mat34.value, expected));
  };

  test(R"R(
doc:
  value:
  - [0.0, 1.0, 2.0, 3.0]
  - [4.0, 5.0, 6.0, 7.0]
  - [8.0, 9.0, 10.0, 11.0]
)R", (Matrix34d{} << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11).finished());
}

TEST_P(YamlReadArchiveTest, EigenMissing) {
  const auto& vx = AcceptEmptyDoc<EigenVecStruct>();
  const auto& vn = AcceptEmptyDoc<EigenVec3Struct>();
  const auto& mx = AcceptEmptyDoc<EigenMatrixStruct>();
  const auto& mn = AcceptEmptyDoc<EigenMatrix34Struct>();
  ASSERT_EQ(vx.value.size(), 1);
  ASSERT_EQ(mx.value.size(), 1);
  EXPECT_THAT(vx.value(0), testing::NanSensitiveDoubleEq(NAN));
  EXPECT_THAT(vn.value(0), testing::NanSensitiveDoubleEq(NAN));
  EXPECT_THAT(mx.value(0, 0), testing::NanSensitiveDoubleEq(NAN));
  EXPECT_THAT(mn.value(0, 0), testing::NanSensitiveDoubleEq(NAN));
}

TEST_P(YamlReadArchiveTest, Nested) {
  const auto& x = AcceptNoThrow<OuterStruct>(Load(R"R(
doc:
  outer_value: 1.0
  inner_struct:
    inner_value: 2.0
)R"));
  EXPECT_EQ(x.outer_value, 1.0);
  EXPECT_EQ(x.inner_struct.inner_value, 2.0);
}

TEST_P(YamlReadArchiveTest, NestedWithMergeKeys) {
  const auto test = [](const std::string& orig_doc) {
    std::string doc = orig_doc;
    if (!GetParam().allow_yaml_with_no_cpp) {
      doc = std::regex_replace(
          orig_doc, std::regex(" *ignored_key: ignored_value"), "");
    }
    SCOPED_TRACE("With doc = " + doc);
    const auto& x = AcceptNoThrow<OuterStruct>(Load(doc));
    EXPECT_EQ(x.outer_value, 1.0);
    EXPECT_EQ(x.inner_struct.inner_value, 2.0);
  };

  // Use merge keys to populate InnerStruct.
  test(R"R(
_template: &template
  inner_value: 2.0
  ignored_key: ignored_value

doc:
  inner_struct:
    << : *template
  outer_value: 1.0
)R");

  // Use merge keys to populate InnerStruct, though to no effect because the
  // existing value wins.
  test(R"R(
_template: &template
  inner_value: 3.0
  ignored_key: ignored_value

doc:
  inner_struct:
    << : *template
    inner_value: 2.0
  outer_value: 1.0
)R");

  // Use merge keys to populate OuterStruct.
  test(R"R(
_template: &template
  inner_struct:
    inner_value: 2.0
    ignored_key: ignored_value

doc:
  << : *template
  outer_value: 1.0
)R");

  // Use array of merge keys to populate OuterStruct.
  // First array with a value wins.
  test(R"R(
_template: &template
  - inner_struct:
      inner_value: 2.0
      ignored_key: ignored_value
  - inner_struct:
      inner_value: 3.0

doc:
  << : *template
  outer_value: 1.0
)R");
}

TEST_P(YamlReadArchiveTest, NestedWithBadMergeKey) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<OuterStruct>(Load(R"R(
_template: &template 99.0

doc:
  inner_struct:
    << : *template
  outer_value: 1.0
)R")),
      std::runtime_error,
      "YAML node of type Map"
      " \\(with size 2 and keys \\{inner_struct, outer_value\\}\\)"
      " has invalid merge key type \\(Scalar\\) within entry"
      " for .*::OuterStruct::InnerStruct inner_struct\\.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<OuterStruct>(Load(R"R(
_template: &template

doc:
  inner_struct:
    << : *template
  outer_value: 1.0
)R")),
      std::runtime_error,
      "YAML node of type Map"
      " \\(with size 2 and keys \\{inner_struct, outer_value\\}\\)"
      " has invalid merge key type \\(Null\\) within entry"
      " for .*::OuterStruct::InnerStruct inner_struct\\.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<OuterStruct>(Load(R"R(
_template: &template
  - inner_value
  - 2.0

doc:
  << : *template
  outer_value: 1.0
)R")),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{outer_value\\}\\)"
      " has invalid merge key type \\(Sequence-of-non-Map\\) within entry"
      " for <root>\\.");
}

// This finds nothing when a scalar was wanted, because the name had a typo.
TEST_P(YamlReadArchiveTest, VisitScalarFoundNothing) {
  // This has a "_TYPO" in a field name.
  const YAML::Node node = Load(R"R(
doc:
  outer_value: 1.0
  inner_struct:
    inner_value_TYPO: 2.0
)R");
  if (GetParam().allow_cpp_with_no_yaml &&
      GetParam().allow_yaml_with_no_cpp) {
    const auto& x = AcceptNoThrow<OuterStruct>(node);
    EXPECT_EQ(x.outer_value, 1.0);
    EXPECT_THAT(x.inner_struct.inner_value, testing::NanSensitiveDoubleEq(NAN));
  } else if (GetParam().allow_cpp_with_no_yaml) {
    DRAKE_EXPECT_THROWS_MESSAGE(
        AcceptIntoDummy<OuterStruct>(node),
        std::runtime_error,
        "YAML node of type Map"
        " \\(with size 1 and keys \\{inner_value_TYPO\\}\\)"
        " key inner_value_TYPO did not match any visited value entry for <root>"
        " while accepting YAML node of type Map"
        " \\(with size 2 and keys \\{inner_struct, outer_value\\}\\)"
        " while visiting [^ ]*InnerStruct inner_struct\\.");
  } else {
    DRAKE_EXPECT_THROWS_MESSAGE(
        AcceptIntoDummy<OuterStruct>(node),
        std::runtime_error,
        "YAML node of type Map"
        " \\(with size 1 and keys \\{inner_value_TYPO\\}\\)"
        " is missing entry for double inner_value"
        " while accepting YAML node of type Map"
        " \\(with size 2 and keys \\{inner_struct, outer_value\\}\\)"
        " while visiting [^ ]*InnerStruct inner_struct\\.");
  }
}

// This finds an array when a scalar was wanted.
TEST_P(YamlReadArchiveTest, VisitScalarFoundArray) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<OuterStruct>(Load(R"R(
doc:
  outer_value: 1.0
  inner_struct:
    inner_value: [2.0, 3.0]
)R")),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{inner_value\\}\\)"
      " has non-Scalar \\(Sequence\\) entry for double inner_value"
      " while accepting YAML node of type Map"
      " \\(with size 2 and keys \\{inner_struct, outer_value\\}\\)"
      " while visiting [^ ]*InnerStruct inner_struct\\.");
}

// This finds a struct when a scalar was wanted.
TEST_P(YamlReadArchiveTest, VisitScalarFoundStruct) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<OuterStruct>(Load(R"R(
doc:
  outer_value: 1.0
  inner_struct:
    inner_value:
       key: 2.0
)R")),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{inner_value\\}\\)"
      " has non-Scalar \\(Map\\) entry for double inner_value"
      " while accepting YAML node of type Map"
      " \\(with size 2 and keys \\{inner_struct, outer_value\\}\\)"
      " while visiting [^ ]*InnerStruct inner_struct\\.");
}

// This finds nothing when a std::array was wanted.
TEST_P(YamlReadArchiveTest, VisitArrayFoundNothing) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<ArrayStruct>(LoadSingleValue("")),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Null\\) entry for std::array<.*> value\\.");
}

// This finds a scalar when a std::array was wanted.
TEST_P(YamlReadArchiveTest, VisitArrayFoundScalar) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<ArrayStruct>(LoadSingleValue("1.0")),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Scalar\\) entry for std::array<.*> value\\.");
}

// This finds a sub-structure when a std::array was wanted.
TEST_P(YamlReadArchiveTest, VisitArrayFoundStruct) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<ArrayStruct>(Load(R"R(
doc:
  value:
    inner_value: 1.0
)R")),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Map\\) entry for std::array<.*> value\\.");
}

// This finds nothing when a std::vector was wanted.
TEST_P(YamlReadArchiveTest, VisitVectorFoundNothing) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<VectorStruct>(LoadSingleValue("")),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Null\\) entry for std::vector<.*> value\\.");
}

// This finds a scalar when a std::vector was wanted.
TEST_P(YamlReadArchiveTest, VisitVectorFoundScalar) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<VectorStruct>(LoadSingleValue("1.0")),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Scalar\\) entry for std::vector<.*> value\\.");
}

// This finds a sub-structure when a std::vector was wanted.
TEST_P(YamlReadArchiveTest, VisitVectorFoundStruct) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<VectorStruct>(Load(R"R(
doc:
  value:
    inner_value: 1.0
)R")),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Map\\) entry for std::vector<.*> value\\.");
}

// This finds a sequence when an optional<double> was wanted.
TEST_P(YamlReadArchiveTest, VisitOptionalScalarFoundSequence) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<OptionalStruct>(LoadSingleValue("[1.0]")),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " has non-Scalar \\(Sequence\\) entry for std::optional<double>"
      " value\\.");
}

// This finds various untagged things when a variant was wanted.
TEST_P(YamlReadArchiveTest, VisitVariantFoundNoTag) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<VariantWrappingStruct>(
          Load("doc:\n  inner:\n    value:")),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " has non-Scalar \\(Null\\) entry for std::string value"
      " while accepting YAML node of type Map \\(with size 1 and keys"
      " \\{inner\\}\\) while visiting \\(anonymous\\)::VariantStruct inner.");

  // std::string values should load correctly even without a YAML type tag.
  const auto& str = AcceptNoThrow<VariantWrappingStruct>(
      Load("doc:\n  inner:\n    value: foo"));
  EXPECT_EQ(str.inner.value, Variant3("foo"));

  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<VariantWrappingStruct>(
          Load("doc:\n  inner:\n    value: [foo, bar]")),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " has non-Scalar \\(Sequence\\) entry for std::string value"
      " while accepting YAML node of type Map \\(with size 1 and keys"
      " \\{inner\\}\\) while visiting \\(anonymous\\)::VariantStruct inner.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<VariantWrappingStruct>(
          Load("doc:\n  inner:\n    value: {foo: bar}")),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " has non-Scalar \\(Map\\) entry for std::string value\\"
      " while accepting YAML node of type Map \\(with size 1 and keys"
      " \\{inner\\}\\) while visiting \\(anonymous\\)::VariantStruct inner.");
}

// This finds an unknown tag when a variant was wanted.
TEST_P(YamlReadArchiveTest, VisitVariantFoundUnknownTag) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<VariantStruct>(Load("doc:\n  value: !UnknownTag foo")),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\) "
      "has unsupported type tag !UnknownTag "
      "while selecting a variant<> entry for "
      "st.::variant<std::string,double,\\(anonymous\\)::DoubleStruct> value.");
}

// This finds nothing when an Eigen::Vector or Eigen::Matrix was wanted.
TEST_P(YamlReadArchiveTest, VisitEigenFoundNothing) {
  const std::string value;
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenVecStruct>(LoadSingleValue(value)),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Null\\) entry for Eigen::VectorXd value\\.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenVec3Struct>(LoadSingleValue(value)),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Null\\) entry for Eigen::Vector3d value\\.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenMatrixStruct>(LoadSingleValue(value)),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Null\\) entry for Eigen::MatrixXd value\\.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenMatrix34Struct>(LoadSingleValue(value)),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Null\\) entry for Eigen::Matrix.*3,4.* value\\.");
}

// This finds a scalar when an Eigen::Vector or Eigen::Matrix was wanted.
TEST_P(YamlReadArchiveTest, VisitEigenFoundScalar) {
  const std::string value{"1.0"};
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenVecStruct>(LoadSingleValue(value)),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Scalar\\) entry for Eigen::VectorXd value\\.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenVec3Struct>(LoadSingleValue(value)),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Scalar\\) entry for Eigen::Vector3d value\\.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenMatrixStruct>(LoadSingleValue(value)),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Scalar\\) entry for Eigen::MatrixXd value\\.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenMatrix34Struct>(LoadSingleValue(value)),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Scalar\\) entry for Eigen::Matrix.* value\\.");
}

// This finds a one-dimensional Sequence when a 2-d Eigen::Matrix was wanted.
TEST_P(YamlReadArchiveTest, VisitEigenMatrixFoundOneDimensional) {
  const std::string value{"[1.0, 2.0, 3.0, 4.0]"};
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenMatrixStruct>(LoadSingleValue(value)),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " is Sequence-of-Scalar \\(not Sequence-of-Sequence\\)"
      " entry for Eigen::MatrixXd value\\.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenMatrix34Struct>(LoadSingleValue(value)),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " is Sequence-of-Scalar \\(not Sequence-of-Sequence\\)"
      " entry for Eigen::Matrix.* value\\.");
}

// This finds a non-square (4+4+3) matrix, when an Eigen::Matrix was wanted.
TEST_P(YamlReadArchiveTest, VisitEigenMatrixFoundNonSquare) {
  const std::string doc(R"R(
doc:
  value:
  - [0.0, 1.0, 2.0, 3.0]
  - [4.0, 5.0, 6.0, 7.0]
  - [8.0, 9.0, 0.0]
)R");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenMatrixStruct>(Load(doc)),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " has inconsistent cols dimensions entry for Eigen::MatrixXd value\\.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenMatrix34Struct>(Load(doc)),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " has inconsistent cols dimensions entry for Eigen::Matrix.* value\\.");
}

// This finds nothing when a sub-structure was wanted.
TEST_P(YamlReadArchiveTest, VisitStructFoundNothing) {
  const YAML::Node node = Load(R"R(
  doc:
    outer_value: 1.0
  )R");
  if (GetParam().allow_cpp_with_no_yaml) {
    const auto& x = AcceptNoThrow<OuterStruct>(node);
    EXPECT_EQ(x.outer_value, 1.0);
    EXPECT_THAT(x.inner_struct.inner_value, testing::NanSensitiveDoubleEq(NAN));
  } else {
    DRAKE_EXPECT_THROWS_MESSAGE(
        AcceptIntoDummy<OuterStruct>(node),
        std::runtime_error,
        "YAML node of type Map \\(with size 1 and keys \\{outer_value\\}\\)"
        " is missing entry for [^ ]*InnerStruct inner_struct\\.");
  }
}

// This finds a scalar when a sub-structure was wanted.
TEST_P(YamlReadArchiveTest, VisitStructFoundScalar) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<OuterStruct>(Load(R"R(
doc:
  outer_value: 1.0
  inner_struct: 2.0
)R")),
      std::runtime_error,
      "YAML node of type Map"
      " \\(with size 2 and keys \\{inner_struct, outer_value\\}\\)"
      " has non-Map \\(Scalar\\) entry for [^ ]*InnerStruct inner_struct\\.");
}

// This finds an array when a sub-structure was wanted.
TEST_P(YamlReadArchiveTest, VisitStructFoundArray) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<OuterStruct>(Load(R"R(
doc:
  outer_value: 1.0
  inner_struct: [2.0, 3.0]
)R")),
      std::runtime_error,
      "YAML node of type Map"
      " \\(with size 2 and keys \\{inner_struct, outer_value\\}\\)"
      " has non-Map \\(Sequence\\) entry for [^ ]*InnerStruct inner_struct\\.");
}

INSTANTIATE_TEST_SUITE_P(
    AllOptions, YamlReadArchiveTest,
    ::testing::Values(
        YamlReadArchive::Options{
            .allow_yaml_with_no_cpp = false,
            .allow_cpp_with_no_yaml = false},
        YamlReadArchive::Options{
            .allow_yaml_with_no_cpp = false,
            .allow_cpp_with_no_yaml = true},
        YamlReadArchive::Options{
            .allow_yaml_with_no_cpp = true,
            .allow_cpp_with_no_yaml = false},
        YamlReadArchive::Options{
            .allow_yaml_with_no_cpp = true,
            .allow_cpp_with_no_yaml = true}));

}  // namespace
}  // namespace yaml
}  // namespace drake
