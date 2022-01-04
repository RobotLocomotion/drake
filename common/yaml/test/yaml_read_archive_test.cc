#include "drake/common/yaml/yaml_read_archive.h"

#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "drake/common/name_value.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/test/example_structs.h"

// TODO(jwnimmer-tri) All of these regexps would be better off using the
// std::regex::basic grammar, where () and {} are not special characters.

namespace drake {
namespace yaml {
namespace test {
namespace {

// TODO(jwnimmer-tri) Add a test case for reading NonPodVectorStruct.
// TODO(jwnimmer-tri) Add a test case for reading OuterWithBlankInner.
// TODO(jwnimmer-tri) Add a test case for reading StringStruct.
// TODO(jwnimmer-tri) Add a test case for reading UnorderedMapStruct.

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
      throw std::invalid_argument("Bad contents parse " + contents);
    }
    const YAML::Node doc = loaded["doc"];
    if (doc.Type() != YAML::NodeType::Map) {
      throw std::invalid_argument("Bad doc parse " + contents);
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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      // TODO(jwnimmer-tri) Once the deprecation 2022-03-01 date hits,
      // port this to use the drake::yaml::internal::Node API instead.
      YamlReadArchive(root, GetParam()).Accept(&result);
#pragma GCC diagnostic pop
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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      // TODO(jwnimmer-tri) Once the deprecation 2022-03-01 date hits,
      // port this to use the drake::yaml::internal::Node API instead.
    YamlReadArchive(root, GetParam()).Accept(&dummy);
#pragma GCC diagnostic pop
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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      // TODO(jwnimmer-tri) Once the deprecation 2022-03-01 date hits,
      // port this to use the drake::yaml::internal::Node API instead.
      YamlReadArchive(root, GetParam()).Accept(&result);
#pragma GCC diagnostic pop
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
  EXPECT_EQ(x.value, kNominalDouble);
}

TEST_P(YamlReadArchiveTest, AllScalars) {
  const std::string doc = R"""(
doc:
  some_bool: true
  some_double: 100.0
  some_float: 101.0
  some_int32: 102
  some_uint32: 103
  some_int64: 104
  some_uint64: 105
  some_string: foo
)""";
  const auto& x = AcceptNoThrow<AllScalarsStruct>(Load(doc));
  EXPECT_EQ(x.some_bool, true);
  EXPECT_EQ(x.some_double, 100.0);
  EXPECT_EQ(x.some_float, 101.0);
  EXPECT_EQ(x.some_int32, 102);
  EXPECT_EQ(x.some_uint32, 103);
  EXPECT_EQ(x.some_int64, 104);
  EXPECT_EQ(x.some_uint64, 105);
  EXPECT_EQ(x.some_string, "foo");
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
  EXPECT_EQ(x.value[0], kNominalDouble);
  EXPECT_EQ(x.value[1], kNominalDouble);
  EXPECT_EQ(x.value[2], kNominalDouble);
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
  EXPECT_EQ(x.value[0], kNominalDouble);
}

TEST_P(YamlReadArchiveTest, StdMap) {
  const auto test = [](const std::string& doc,
                       const std::map<std::string, double>& expected) {
    const auto& x = AcceptNoThrow<MapStruct>(Load(doc));
    std::map<std::string, double> adjusted_expected = expected;
    if (GetParam().retain_map_defaults) {
      adjusted_expected["kNominalDouble"] = kNominalDouble;
    }
    EXPECT_EQ(x.value, adjusted_expected) << doc;
  };

  test("doc:\n  value:\n    foo: 0.0\n    bar: 1.0\n",
       {{"foo", 0.0}, {"bar", 1.0}});
}

TEST_P(YamlReadArchiveTest, BigStdMapAppend) {
  if (!GetParam().allow_cpp_with_no_yaml) {
    // The parser would raise an uninteresting exception in this case.
    return;
  }
  std::string doc = R"""(
doc:
  value:
    bar:
      outer_value: 3.0
      inner_struct:
        inner_value: 4.0
)""";
  const auto& x = AcceptNoThrow<BigMapStruct>(Load(doc));
  if (GetParam().retain_map_defaults) {
    EXPECT_EQ(x.value.size(), 2);
    EXPECT_EQ(x.value.at("foo").outer_value, 1.0);
    EXPECT_EQ(x.value.at("foo").inner_struct.inner_value, 2.0);
  } else {
    EXPECT_EQ(x.value.size(), 1);
  }
  EXPECT_EQ(x.value.at("bar").outer_value, 3.0);
  EXPECT_EQ(x.value.at("bar").inner_struct.inner_value, 4.0);
}

TEST_P(YamlReadArchiveTest, BigStdMapMergeNewOuterValue) {
  if (!GetParam().allow_cpp_with_no_yaml) {
    // The parser would raise an uninteresting exception in this case.
    return;
  }
  std::string doc = R"""(
doc:
  value:
    foo:
      outer_value: 3.0
)""";
  const auto& x = AcceptNoThrow<BigMapStruct>(Load(doc));
  EXPECT_EQ(x.value.size(), 1);
  EXPECT_EQ(x.value.at("foo").outer_value, 3.0);
  if (GetParam().retain_map_defaults) {
    EXPECT_EQ(x.value.at("foo").inner_struct.inner_value, 2.0);
  } else {
    EXPECT_EQ(x.value.at("foo").inner_struct.inner_value, kNominalDouble);
  }
}

TEST_P(YamlReadArchiveTest, BigStdMapMergeNewInnerValue) {
  if (!GetParam().allow_cpp_with_no_yaml) {
    // The parser would raise an uninteresting exception in this case.
    return;
  }
  std::string doc = R"""(
doc:
  value:
    foo:
      inner_struct:
        inner_value: 4.0
)""";
  const auto& x = AcceptNoThrow<BigMapStruct>(Load(doc));
  EXPECT_EQ(x.value.size(), 1);
  if (GetParam().retain_map_defaults) {
    EXPECT_EQ(x.value.at("foo").outer_value, 1.0);
  } else {
    EXPECT_EQ(x.value.at("foo").outer_value, kNominalDouble);
  }
  EXPECT_EQ(x.value.at("foo").inner_struct.inner_value, 4.0);
}

TEST_P(YamlReadArchiveTest, BigStdMapMergeEmpty) {
  if (!GetParam().allow_cpp_with_no_yaml) {
    // The parser would raise an uninteresting exception in this case.
    return;
  }
  std::string doc = R"""(
doc:
  value:
    foo: {}
)""";
  const auto& x = AcceptNoThrow<BigMapStruct>(Load(doc));
  EXPECT_EQ(x.value.size(), 1);
  if (GetParam().retain_map_defaults) {
    EXPECT_EQ(x.value.at("foo").outer_value, 1.0);
    EXPECT_EQ(x.value.at("foo").inner_struct.inner_value, 2.0);
  } else {
    EXPECT_EQ(x.value.at("foo").outer_value, kNominalDouble);
    EXPECT_EQ(x.value.at("foo").inner_struct.inner_value, kNominalDouble);
  }
}

TEST_P(YamlReadArchiveTest, StdMapMissing) {
  const auto& x = AcceptEmptyDoc<MapStruct>();
  ASSERT_EQ(x.value.size(), 1);
  EXPECT_EQ(x.value.at("kNominalDouble"), kNominalDouble);
}

TEST_P(YamlReadArchiveTest, StdMapWithMergeKeys) {
  const auto test = [](const std::string& doc,
                       const std::map<std::string, double>& expected) {
    const auto& x = AcceptNoThrow<MapStruct>(Load(doc));
    std::map<std::string, double> adjusted_expected = expected;
    if (GetParam().retain_map_defaults) {
      adjusted_expected["kNominalDouble"] = kNominalDouble;
    }
    EXPECT_EQ(x.value, adjusted_expected) << doc;
  };

  // Use merge keys to populate some keys.
  test(R"""(
_template: &template
  foo: 1.0

doc:
  value:
    << : *template
    bar: 2.0
)""", {{"foo", 1.0}, {"bar", 2.0}});

  // A pre-existing value should win, though.
  test(R"""(
_template: &template
  foo: 3.0

doc:
  value:
    << : *template
    foo: 1.0
    bar: 2.0
)""", {{"foo", 1.0}, {"bar", 2.0}});

  // A list of merges should also work.
  test(R"""(
_template: &template
  - foo: 1.0
  - baz: 3.0

doc:
  value:
    << : *template
    bar: 2.0
)""", {{"foo", 1.0}, {"bar", 2.0}, {"baz", 3.0}});
}

TEST_P(YamlReadArchiveTest, StdMapWithBadMergeKey) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<MapStruct>(Load(R"""(
_template: &template 99.0

doc:
  value:
    << : *template
    bar: 2.0
)""")),
      "YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has invalid merge key type \\(Scalar\\)\\.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<MapStruct>(Load(R"""(
_template: &template

doc:
  value:
    << : *template
    bar: 2.0
)""")),
      "YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has invalid merge key type \\(Null\\).");
}

TEST_P(YamlReadArchiveTest, StdMapDirectly) {
  const std::string doc = R"""(
doc:
  key_1:
    value: 1.0
  key_2:
    value: 2.0
)""";
  const auto& x = AcceptNoThrow<std::map<std::string, DoubleStruct>>(Load(doc));
  EXPECT_EQ(x.size(), 2);
  EXPECT_EQ(x.at("key_1").value, 1.0);
  EXPECT_EQ(x.at("key_2").value, 2.0);
}

TEST_P(YamlReadArchiveTest, StdMapDirectlyWithDefaults) {
  const std::string doc = R"""(
doc:
  key_1:
    value: 1.0
  key_with_defaulted_value: {}
)""";
  const auto node = Load(doc);
  if (GetParam().allow_cpp_with_no_yaml) {
    std::map<std::string, DoubleStruct> result;
    result.emplace("pre_existing_default", DoubleStruct{.value = 0.0});
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    // TODO(jwnimmer-tri) Once the deprecation 2022-03-01 date hits,
    // port this to use the drake::yaml::internal::Node API instead.
    YamlReadArchive(node, GetParam()).Accept(&result);
#pragma GCC diagnostic pop
    if (GetParam().retain_map_defaults) {
      EXPECT_EQ(result.size(), 3);
      EXPECT_EQ(result.at("pre_existing_default").value, 0.0);
    } else {
      EXPECT_EQ(result.size(), 2);
    }
    EXPECT_EQ(result.at("key_1").value, 1.0);
    EXPECT_EQ(result.at("key_with_defaulted_value").value, kNominalDouble);
  } else {
    DRAKE_EXPECT_THROWS_MESSAGE(
        (AcceptIntoDummy<std::map<std::string, DoubleStruct>>(node)),
        ".*missing entry for double value.*");
  }
}

TEST_P(YamlReadArchiveTest, Optional) {
  const auto test_with_default = [](const std::string& doc,
                                    const std::optional<double>& expected) {
    const auto& x = AcceptNoThrow<OptionalStruct>(Load(doc));
    if (expected.has_value()) {
      ASSERT_TRUE(x.value.has_value()) << *expected;
      EXPECT_EQ(x.value.value(), expected.value()) << *expected;
    } else {
      EXPECT_FALSE(x.value.has_value());
    }
  };
  const auto test_no_default = [](const std::string& doc,
                                  const std::optional<double>& expected) {
    const auto& x = AcceptNoThrow<OptionalStructNoDefault>(Load(doc));
    if (expected.has_value()) {
      ASSERT_TRUE(x.value.has_value()) << *expected;
      EXPECT_EQ(x.value.value(), expected.value()) << *expected;
    } else {
      EXPECT_FALSE(x.value.has_value());
    }
  };

  /*
    If the YAML data provided a value for the optional key, then we should
    always take that value (1-4). If the YAML data provided an explicit null for
    the optional key, then we should always take that null (5-8). If the YAML
    data did not mention the key (9-12), the situation is more subtle. In the
    case where allow_cpp_with_no_yaml is false, then every C++ member field must
    match up with YAML -- with the caveat that optional members can be omitted;
    in that case, an absent YAML node must interpreted as nullopt so that C++
    and YAML remain congruent (9, 11). In the case where allow_cpp_with_no_yaml
    is true, we should only be changing C++ values when the yaml data mentions
    the key; unmentioned values should stay undisturbed; in that case, a missing
    key should have no effect (10, 12); only an explicit null key (7, 8) should
    evidence a change.

     # | yaml   | store | acwny || want
    ===+========+=======+=======++========
     1 | Scalar | False | False || Scalar
     2 | Scalar | False | True  || Scalar
     3 | Scalar | True  | False || Scalar
     4 | Scalar | True  | True  || Scalar
     5 | Null   | False | False || False
     6 | Null   | False | True  || False
     7 | Null   | True  | False || False
     8 | Null   | True  | True  || False
     9 | Absent | False | False || False
    10 | Absent | False | True  || False
    11 | Absent | True  | False || False
    12 | Absent | True  | True  || True

    yaml = node type present in yaml file
    store = nvp.value().has_value() initial condition
    acwny = Options.allow_cpp_with_no_yaml
    want = nvp.value() desired final condition
  */

  test_no_default("doc:\n  value: 1.0", 1.0);         // # 1, 2
  test_with_default("doc:\n  value: 1.0", 1.0);       // # 3, 4
  test_no_default("doc:\n  value:", std::nullopt);    // # 5, 6
  test_with_default("doc:\n  value:", std::nullopt);  // # 7, 8

  test_no_default("doc: {}", std::nullopt);  // # 9, 10
  if (GetParam().allow_cpp_with_no_yaml) {
    test_with_default("doc: {}", kNominalDouble);  // # 12
  } else {
    test_with_default("doc: {}", std::nullopt);  // # 11
  }

  if (GetParam().allow_yaml_with_no_cpp) {
    test_no_default("doc:\n  foo: bar\n  value: 1.0", 1.0);         // # 1, 2
    test_with_default("doc:\n  foo: bar\n  value: 1.0", 1.0);       // # 3, 4
    test_no_default("doc:\n  foo: bar\n  value:", std::nullopt);    // # 5, 6
    test_with_default("doc:\n  foo: bar\n  value:", std::nullopt);  // # 7, 8

    test_no_default("doc:\n  foo: bar", std::nullopt);  // # 9, 10
    if (GetParam().allow_cpp_with_no_yaml) {
      test_with_default("doc:\n  foo: bar", kNominalDouble);  // # 12
    } else {
      test_with_default("doc:\n  foo: bar", std::nullopt);  // # 11
    }
  }
}

TEST_P(YamlReadArchiveTest, Variant) {
  const auto test = [](const std::string& doc,
                       const Variant4& expected) {
    const auto& x = AcceptNoThrow<VariantStruct>(Load(doc));
    EXPECT_EQ(x.value, expected) << doc;
  };

  test("doc:\n  value: !!str foo", "foo");
  test("doc:\n  value: !!float 1.0", 1.0);
  test("doc:\n  value: !DoubleStruct { value: 1.0 }", DoubleStruct{1.0});
}

TEST_P(YamlReadArchiveTest, VariantMissing) {
  const auto& x = AcceptEmptyDoc<VariantStruct>();
  EXPECT_EQ(std::get<double>(x.value), kNominalDouble);
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

  test(R"""(
doc:
  value:
  - [0.0, 1.0, 2.0, 3.0]
  - [4.0, 5.0, 6.0, 7.0]
  - [8.0, 9.0, 10.0, 11.0]
)""", (Matrix34d{} << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11).finished());
}

TEST_P(YamlReadArchiveTest, EigenMatrixUpTo6) {
  using Matrix34d = Eigen::Matrix<double, 3, 4>;
  const auto test = [](const std::string& doc,
                       const Matrix34d& expected) {
    const auto& mat = AcceptNoThrow<EigenMatrixUpTo6Struct>(Load(doc));
    EXPECT_TRUE(drake::CompareMatrices(mat.value, expected));
  };

  test(R"""(
doc:
  value:
  - [0.0, 1.0, 2.0, 3.0]
  - [4.0, 5.0, 6.0, 7.0]
  - [8.0, 9.0, 10.0, 11.0]
)""", (Matrix34d{} << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11).finished());
}

TEST_P(YamlReadArchiveTest, EigenMatrix00) {
  const auto test = [](const std::string& doc) {
    const auto& mat = AcceptNoThrow<EigenMatrixStruct>(Load(doc));
    const auto& mat00 = AcceptNoThrow<EigenMatrix00Struct>(Load(doc));
    const Eigen::MatrixXd empty;
    EXPECT_TRUE(drake::CompareMatrices(mat.value, empty));
    EXPECT_TRUE(drake::CompareMatrices(mat00.value, empty));
  };

  test(R"""(
doc:
  value: []
)""");
  test(R"""(
doc:
  value: [[]]
)""");
}

TEST_P(YamlReadArchiveTest, EigenMissing) {
  const auto& vx = AcceptEmptyDoc<EigenVecStruct>();
  const auto& vn = AcceptEmptyDoc<EigenVec3Struct>();
  const auto& mx = AcceptEmptyDoc<EigenMatrixStruct>();
  const auto& mn = AcceptEmptyDoc<EigenMatrix34Struct>();
  ASSERT_EQ(vx.value.size(), 1);
  ASSERT_EQ(mx.value.size(), 1);
  EXPECT_EQ(vx.value(0), kNominalDouble);
  EXPECT_EQ(vn.value(0), kNominalDouble);
  EXPECT_EQ(mx.value(0, 0), kNominalDouble);
  EXPECT_EQ(mn.value(0, 0), kNominalDouble);
}

TEST_P(YamlReadArchiveTest, Nested) {
  const auto& x = AcceptNoThrow<OuterStruct>(Load(R"""(
doc:
  outer_value: 1.0
  inner_struct:
    inner_value: 2.0
)"""));
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
  test(R"""(
_template: &template
  inner_value: 2.0
  ignored_key: ignored_value

doc:
  inner_struct:
    << : *template
  outer_value: 1.0
)""");

  // Use merge keys to populate InnerStruct, though to no effect because the
  // existing value wins.
  test(R"""(
_template: &template
  inner_value: 3.0
  ignored_key: ignored_value

doc:
  inner_struct:
    << : *template
    inner_value: 2.0
  outer_value: 1.0
)""");

  // Use merge keys to populate OuterStruct.
  test(R"""(
_template: &template
  inner_struct:
    inner_value: 2.0
    ignored_key: ignored_value

doc:
  << : *template
  outer_value: 1.0
)""");

  // Use array of merge keys to populate OuterStruct.
  // First array with a value wins.
  test(R"""(
_template: &template
  - inner_struct:
      inner_value: 2.0
      ignored_key: ignored_value
  - inner_struct:
      inner_value: 3.0

doc:
  << : *template
  outer_value: 1.0
)""");
}

TEST_P(YamlReadArchiveTest, NestedWithBadMergeKey) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<OuterStruct>(Load(R"""(
_template: &template 99.0

doc:
  inner_struct:
    << : *template
  outer_value: 1.0
)""")),
      "YAML node of type Mapping"
      " \\(with size 2 and keys \\{inner_struct, outer_value\\}\\)"
      " has invalid merge key type \\(Scalar\\)\\.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<OuterStruct>(Load(R"""(
_template: &template

doc:
  inner_struct:
    << : *template
  outer_value: 1.0
)""")),
      "YAML node of type Mapping"
      " \\(with size 2 and keys \\{inner_struct, outer_value\\}\\)"
      " has invalid merge key type \\(Null\\)\\.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<OuterStruct>(Load(R"""(
_template: &template
  - inner_value
  - 2.0

doc:
  << : *template
  outer_value: 1.0
)""")),
      "YAML node of type Mapping \\(with size 1 and keys \\{outer_value\\}\\)"
      " has invalid merge key type \\(Sequence-of-non-Mapping\\)\\.");
}

// This finds nothing when a scalar was wanted, because the name had a typo.
TEST_P(YamlReadArchiveTest, VisitScalarFoundNothing) {
  // This has a "_TYPO" in a field name.
  const YAML::Node node = Load(R"""(
doc:
  outer_value: 1.0
  inner_struct:
    inner_value_TYPO: 2.0
)""");
  if (GetParam().allow_cpp_with_no_yaml &&
      GetParam().allow_yaml_with_no_cpp) {
    const auto& x = AcceptNoThrow<OuterStruct>(node);
    EXPECT_EQ(x.outer_value, 1.0);
    EXPECT_EQ(x.inner_struct.inner_value, kNominalDouble);
  } else if (GetParam().allow_cpp_with_no_yaml) {
    DRAKE_EXPECT_THROWS_MESSAGE(
        AcceptIntoDummy<OuterStruct>(node),
        "YAML node of type Mapping"
        " \\(with size 1 and keys \\{inner_value_TYPO\\}\\)"
        " key inner_value_TYPO did not match any visited value entry for <root>"
        " while accepting YAML node of type Mapping"
        " \\(with size 2 and keys \\{inner_struct, outer_value\\}\\)"
        " while visiting [^ ]*InnerStruct inner_struct\\.");
  } else {
    DRAKE_EXPECT_THROWS_MESSAGE(
        AcceptIntoDummy<OuterStruct>(node),
        "YAML node of type Mapping"
        " \\(with size 1 and keys \\{inner_value_TYPO\\}\\)"
        " is missing entry for double inner_value"
        " while accepting YAML node of type Mapping"
        " \\(with size 2 and keys \\{inner_struct, outer_value\\}\\)"
        " while visiting [^ ]*InnerStruct inner_struct\\.");
  }
}

// This finds an array when a scalar was wanted.
TEST_P(YamlReadArchiveTest, VisitScalarFoundArray) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<OuterStruct>(Load(R"""(
doc:
  outer_value: 1.0
  inner_struct:
    inner_value: [2.0, 3.0]
)""")),
      "YAML node of type Mapping \\(with size 1 and keys \\{inner_value\\}\\)"
      " has non-Scalar \\(Sequence\\) entry for double inner_value"
      " while accepting YAML node of type Mapping"
      " \\(with size 2 and keys \\{inner_struct, outer_value\\}\\)"
      " while visiting [^ ]*InnerStruct inner_struct\\.");
}

// This finds a struct when a scalar was wanted.
TEST_P(YamlReadArchiveTest, VisitScalarFoundStruct) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<OuterStruct>(Load(R"""(
doc:
  outer_value: 1.0
  inner_struct:
    inner_value:
       key: 2.0
)""")),
      "YAML node of type Mapping \\(with size 1 and keys \\{inner_value\\}\\)"
      " has non-Scalar \\(Mapping\\) entry for double inner_value"
      " while accepting YAML node of type Mapping"
      " \\(with size 2 and keys \\{inner_struct, outer_value\\}\\)"
      " while visiting [^ ]*InnerStruct inner_struct\\.");
}

// This finds nothing when a std::array was wanted.
TEST_P(YamlReadArchiveTest, VisitArrayFoundNothing) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<ArrayStruct>(LoadSingleValue("")),
      "YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Null\\) entry for std::array<.*> value\\.");
}

// This finds a scalar when a std::array was wanted.
TEST_P(YamlReadArchiveTest, VisitArrayFoundScalar) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<ArrayStruct>(LoadSingleValue("1.0")),
      "YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Scalar\\) entry for std::array<.*> value\\.");
}

// This finds a sub-structure when a std::array was wanted.
TEST_P(YamlReadArchiveTest, VisitArrayFoundStruct) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<ArrayStruct>(Load(R"""(
doc:
  value:
    inner_value: 1.0
)""")),
      "YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Mapping\\) entry for std::array<.*> value\\.");
}

// This finds nothing when a std::vector was wanted.
TEST_P(YamlReadArchiveTest, VisitVectorFoundNothing) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<VectorStruct>(LoadSingleValue("")),
      "YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Null\\) entry for std::vector<.*> value\\.");
}

// This finds a scalar when a std::vector was wanted.
TEST_P(YamlReadArchiveTest, VisitVectorFoundScalar) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<VectorStruct>(LoadSingleValue("1.0")),
      "YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Scalar\\) entry for std::vector<.*> value\\.");
}

// This finds a sub-structure when a std::vector was wanted.
TEST_P(YamlReadArchiveTest, VisitVectorFoundStruct) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<VectorStruct>(Load(R"""(
doc:
  value:
    inner_value: 1.0
)""")),
      "YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Mapping\\) entry for std::vector<.*> value\\.");
}

// This finds a sequence when an optional<double> was wanted.
TEST_P(YamlReadArchiveTest, VisitOptionalScalarFoundSequence) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<OptionalStruct>(LoadSingleValue("[1.0]")),
      "YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Scalar \\(Sequence\\) entry for std::optional<double>"
      " value\\.");
}

// This finds various untagged things when a variant was wanted.
TEST_P(YamlReadArchiveTest, VisitVariantFoundNoTag) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<VariantWrappingStruct>(
          Load("doc:\n  inner:\n    value:")),
      "YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Scalar \\(Null\\) entry for std::string value"
      " while accepting YAML node of type Mapping \\(with size 1 and keys"
      " \\{inner\\}\\) while visiting drake::yaml::test::VariantStruct inner.");

  // std::string values should load correctly even without a YAML type tag.
  const auto& str = AcceptNoThrow<VariantWrappingStruct>(
      Load("doc:\n  inner:\n    value: foo"));
  EXPECT_EQ(str.inner.value, Variant4("foo"));

  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<VariantWrappingStruct>(
          Load("doc:\n  inner:\n    value: [foo, bar]")),
      "YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Scalar \\(Sequence\\) entry for std::string value"
      " while accepting YAML node of type Mapping \\(with size 1 and keys"
      " \\{inner\\}\\) while visiting drake::yaml::test::VariantStruct inner.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<VariantWrappingStruct>(
          Load("doc:\n  inner:\n    value: {foo: bar}")),
      "YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Scalar \\(Mapping\\) entry for std::string value\\"
      " while accepting YAML node of type Mapping \\(with size 1 and keys"
      " \\{inner\\}\\) while visiting drake::yaml::test::VariantStruct inner.");
}

// This finds an unknown tag when a variant was wanted.
TEST_P(YamlReadArchiveTest, VisitVariantFoundUnknownTag) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<VariantStruct>(Load("doc:\n  value: !UnknownTag foo")),
      "YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\) "
      "has unsupported type tag !UnknownTag "
      "while selecting a variant<> entry for "
      "std::variant<std::string,double,drake::yaml::test::DoubleStruct,"
      "drake::yaml::test::EigenStruct<-1,1,-1,1>> value.");
}

// This finds nothing when an Eigen::Vector or Eigen::Matrix was wanted.
TEST_P(YamlReadArchiveTest, VisitEigenFoundNothing) {
  const std::string value;
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenVecStruct>(LoadSingleValue(value)),
      "YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Null\\) entry for Eigen::VectorXd value\\.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenVec3Struct>(LoadSingleValue(value)),
      "YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Null\\) entry for Eigen::Vector3d value\\.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenMatrixStruct>(LoadSingleValue(value)),
      "YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Null\\) entry for Eigen::MatrixXd value\\.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenMatrix34Struct>(LoadSingleValue(value)),
      "YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Null\\) entry for Eigen::Matrix.*3,4.* value\\.");
}

// This finds a scalar when an Eigen::Vector or Eigen::Matrix was wanted.
TEST_P(YamlReadArchiveTest, VisitEigenFoundScalar) {
  const std::string value{"1.0"};
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenVecStruct>(LoadSingleValue(value)),
      "YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Scalar\\) entry for Eigen::VectorXd value\\.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenVec3Struct>(LoadSingleValue(value)),
      "YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Scalar\\) entry for Eigen::Vector3d value\\.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenMatrixStruct>(LoadSingleValue(value)),
      "YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Scalar\\) entry for Eigen::MatrixXd value\\.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenMatrix34Struct>(LoadSingleValue(value)),
      "YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Scalar\\) entry for Eigen::Matrix.* value\\.");
}

// This finds a one-dimensional Sequence when a 2-d Eigen::Matrix was wanted.
TEST_P(YamlReadArchiveTest, VisitEigenMatrixFoundOneDimensional) {
  const std::string value{"[1.0, 2.0, 3.0, 4.0]"};
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenMatrixStruct>(LoadSingleValue(value)),
      "YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " is Sequence-of-Scalar \\(not Sequence-of-Sequence\\)"
      " entry for Eigen::MatrixXd value\\.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenMatrix34Struct>(LoadSingleValue(value)),
      "YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " is Sequence-of-Scalar \\(not Sequence-of-Sequence\\)"
      " entry for Eigen::Matrix.* value\\.");
}

// This finds a non-square (4+4+3) matrix, when an Eigen::Matrix was wanted.
TEST_P(YamlReadArchiveTest, VisitEigenMatrixFoundNonSquare) {
  const std::string doc(R"""(
doc:
  value:
  - [0.0, 1.0, 2.0, 3.0]
  - [4.0, 5.0, 6.0, 7.0]
  - [8.0, 9.0, 0.0]
)""");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenMatrixStruct>(Load(doc)),
      "YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has inconsistent cols dimensions entry for Eigen::MatrixXd value\\.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenMatrix34Struct>(Load(doc)),
      "YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has inconsistent cols dimensions entry for Eigen::Matrix.* value\\.");
}

// This finds nothing when a sub-structure was wanted.
TEST_P(YamlReadArchiveTest, VisitStructFoundNothing) {
  const YAML::Node node = Load(R"""(
  doc:
    outer_value: 1.0
  )""");
  if (GetParam().allow_cpp_with_no_yaml) {
    const auto& x = AcceptNoThrow<OuterStruct>(node);
    EXPECT_EQ(x.outer_value, 1.0);
    EXPECT_EQ(x.inner_struct.inner_value, kNominalDouble);
  } else {
    DRAKE_EXPECT_THROWS_MESSAGE(
        AcceptIntoDummy<OuterStruct>(node),
        "YAML node of type Mapping \\(with size 1 and keys \\{outer_value\\}\\)"
        " is missing entry for [^ ]*InnerStruct inner_struct\\.");
  }
}

// This finds a scalar when a sub-structure was wanted.
TEST_P(YamlReadArchiveTest, VisitStructFoundScalar) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<OuterStruct>(Load(R"""(
doc:
  outer_value: 1.0
  inner_struct: 2.0
)""")),
      "YAML node of type Mapping"
      " \\(with size 2 and keys \\{inner_struct, outer_value\\}\\)"
      " has non-Mapping \\(Scalar\\) entry for"
      " [^ ]*InnerStruct inner_struct\\.");
}

// This finds an array when a sub-structure was wanted.
TEST_P(YamlReadArchiveTest, VisitStructFoundArray) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<OuterStruct>(Load(R"""(
doc:
  outer_value: 1.0
  inner_struct: [2.0, 3.0]
)""")),
      "YAML node of type Mapping"
      " \\(with size 2 and keys \\{inner_struct, outer_value\\}\\)"
      " has non-Mapping \\(Sequence\\) entry for"
      " [^ ]*InnerStruct inner_struct\\.");
}

std::vector<YamlReadArchive::Options> MakeAllPossibleOptions() {
  std::vector<YamlReadArchive::Options> all;
  for (const bool i : {false, true}) {
    for (const bool j : {false, true}) {
      for (const bool k : {false, true}) {
        all.push_back(YamlReadArchive::Options{i, j, k});
      }
    }
  }
  return all;
}

INSTANTIATE_TEST_SUITE_P(
    AllOptions, YamlReadArchiveTest,
    ::testing::ValuesIn(MakeAllPossibleOptions()));

}  // namespace
}  // namespace test
}  // namespace yaml
}  // namespace drake
