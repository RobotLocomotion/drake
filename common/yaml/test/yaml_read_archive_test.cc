#include "drake/common/yaml/yaml_read_archive.h"

#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/name_value.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/test/example_structs.h"

// This test suite is the C++ flavor of the Python test suite at
//  drake/bindings/pydrake/common/test/yaml_typed_test.py
// and should be roughly kept in sync with the test cases in that file.

// TODO(jwnimmer-tri) All of these regexps would be better off using the
// std::regex::basic grammar, where () and {} are not special characters.

namespace drake {
namespace yaml {
namespace test {
namespace {

namespace fs = std::filesystem;
using internal::YamlReadArchive;

// TODO(jwnimmer-tri) Add a test case for reading NonPodVectorStruct.
// TODO(jwnimmer-tri) Add a test case for reading OuterWithBlankInner.
// TODO(jwnimmer-tri) Add a test case for reading UnorderedMapStruct.

// A test fixture with common helpers.
class YamlReadArchiveTest : public ::testing::TestWithParam<LoadYamlOptions> {
 public:
  // Loads a single "doc: { ... }" map from `contents` and returns the nested
  // map (i.e., just the "{ ... }" part, not the "doc" part).  It is an error
  // for the "{ ... }" part not to be a map node.
  static internal::Node Load(std::string contents) {
    while (contents.size() > 0 && contents.at(0) == '\n') {
      // Strip off leading newlines.
      contents.erase(0, 1);
    }
    const internal::Node loaded =
        YamlReadArchive::LoadStringAsNode(contents, std::nullopt);
    if (!loaded.IsMapping()) {
      throw std::logic_error("Bad contents parse " + contents);
    }
    const string_map<internal::Node>& mapping = loaded.GetMapping();
    if (!mapping.contains("doc")) {
      throw std::logic_error("Missing doc parse " + contents);
    }
    const internal::Node doc = mapping.at("doc");
    if (!doc.IsMapping()) {
      throw std::logic_error("Bad doc parse " + contents);
    }
    return doc;
  }

  // Loads a single "{ value: something }" map node.  If the argument is the
  // empty string, the result is a map from "value" to Null (not an empty map,
  // nor Null itself, etc.)
  static internal::Node LoadSingleValue(const std::string& value) {
    // The corresponding test in python lacks the `doc:` context. Therefore, the
    // python-generated yaml has two fewer indenting spaces than the
    // C++-generated yaml. So, that the test call sites can look the same, we
    // account for the disparity in indentation here by replacing "\n" in
    // `value` with "\n  ".
    std::string indented = value;
    std::string::size_type n = 0;
    while ((n = indented.find("\n", n)) != std::string::npos) {
      indented.replace(n, 1, "\n  ");
      n += 3;
    }

    return Load("doc:\n  value: " + indented + "\n");
  }

  // Parses root into a Serializable and returns the result of the parse.
  // Any exceptions raised are reported as errors.
  template <typename Serializable>
  static Serializable AcceptNoThrow(const internal::Node& root) {
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
  static void AcceptIntoDummy(const internal::Node& root) {
    Serializable dummy{};
    YamlReadArchive(root, GetParam()).Accept(&dummy);
  }

  // Parses root into a Serializable and returns the result of the parse.
  // If allow_cpp_with_no_yaml is set, then any exceptions are errors.
  // If allow_cpp_with_no_yaml is not set, then lack of exception is an error.
  template <typename Serializable>
  static Serializable AcceptEmptyDoc() {
    SCOPED_TRACE("for type " + NiceTypeName::Get<Serializable>());
    const internal::Node root = Load("doc: {}");
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
      EXPECT_THAT(what,
                  testing::MatchesRegex(".*missing entry for [^ ]* value.*"));
    }
    return result;
  }
};

// TODO(jwnimmer-tri) This test case is extremely basic. We should add many more
// corner cases & etc. here.
TEST_P(YamlReadArchiveTest, String) {
  const auto test = [](const std::string& value, const std::string& expected) {
    const auto& x = AcceptNoThrow<StringStruct>(LoadSingleValue(value));
    EXPECT_EQ(x.value, expected);
  };

  test("foo", "foo");
  test("''''", "'");
  test("'\"'", "\"");
}

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

TEST_P(YamlReadArchiveTest, Int) {
  const auto test = [](const std::string& value, int expected) {
    const auto& x = AcceptNoThrow<IntStruct>(LoadSingleValue(value));
    EXPECT_EQ(x.value, expected);
  };

  // Plain scalars.
  test("0", 0);
  test("1", 1);
  test("-1", -1);
  test("30000", 30000);

  // Strings.
  test("'0'", 0);
  test("'1'", 1);
  test("'-1'", -1);
  test("'30000'", 30000);

  // Float scalars.
  // TODO(jwnimmer-tri) We'd like these to work. (They do in Python.)
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<IntStruct>(LoadSingleValue("0.0")), ".*not parse.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<IntStruct>(LoadSingleValue("1.0")), ".*not parse.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<IntStruct>(LoadSingleValue("-1.0")), ".*not parse.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<IntStruct>(LoadSingleValue("3.0e+4")), ".*not parse.*");
}

TEST_P(YamlReadArchiveTest, Bool) {
  const auto test = [](const std::string& value, bool expected) {
    const auto& x = AcceptNoThrow<BoolStruct>(LoadSingleValue(value));
    EXPECT_EQ(x.value, expected);
  };

  // Plain scalars in canonical form.
  test("true", true);
  test("false", false);

  // Strings.
  test("'true'", true);
  test("'false'", false);

  // Yaml's insane non-canonical plain scalars. (This is not the complete set,
  // rather just a couple as a sanity check.)
  // TODO(jwnimmer-tri) Our Python parser uses YAML 1.2 not YAML 1.1, so ends
  // up rejecting these. We should probably agree one way or another?
  test("yes", true);
  test("no", false);
}

TEST_P(YamlReadArchiveTest, Bytes) {
  const auto test = [](const std::string& value, const std::string& expected) {
    const auto& x = AcceptNoThrow<BytesStruct>(LoadSingleValue(value));
    EXPECT_EQ(x.value, StringToByteVector(expected))
        << "Expected string: '" << expected << "'";
  };

  // Using !!binary on a schema whose type is bytes.
  test("!!binary A3Rlc3Rfc3RyAw==", "\x03test_str\x03");
  test("!!binary |\n  A3Rlc3Rfc3RyAw==", "\x03test_str\x03");
  test("!!binary |\n  A3Rlc3R\n  fc3RyAw==", "\x03test_str\x03");
  test("!!binary ", "");

  // Malformed base64 value.
  {
    // Proper encoding of "\x03t_str\x03" is 'A3Rfc3RyAw=='.

    // Missing character.
    DRAKE_EXPECT_THROWS_MESSAGE(
        AcceptIntoDummy<BytesStruct>(LoadSingleValue("!!binary A3Rfc3RyAw=")),
        ".*invalid base64.*");

    // Invalid character.
    DRAKE_EXPECT_THROWS_MESSAGE(
        AcceptIntoDummy<BytesStruct>(LoadSingleValue("!!binary A3Rfc*RyAw==")),
        ".*invalid base64.*");
  }

  // Assigning any other type to bytes is rejected.
  // Note: these various value strings should be converted to various primitive
  // types (string, int, etc.) before we process the scalar value. However,
  // this doesn't currently happen so the error message can't complain that the
  // wrong *type* has been passed to value. When we aggressively convert and
  // check for mismatch, these error matches will shift to match what is done
  // in yaml.py.
  const auto reject = [](const std::string& value,
                         const std::string& error_regex =
                             ".*must be base64.*") {
    DRAKE_EXPECT_THROWS_MESSAGE(
        AcceptIntoDummy<BytesStruct>(LoadSingleValue(value)), error_regex);
  };
  // String.
  reject("test string");
  reject("!!str 1234");
  // Int.
  reject("12");
  reject("!!int 12");
  reject("0x3");
  reject("0o3");
  reject("00:03");
  // Float.
  reject("1234.5");
  reject("!!float 1234.5");
  reject(".inf");
  reject("00:03.3");
  // Null. Null triggers the more general exception of assigning null to scalar.
  reject("null", ".*has non-Scalar .Null.*");
  reject("", ".*has non-Scalar .Null.*");
  // Bool.
  reject("true");
  reject("!!bool true");

  // Using !!binary for non-binary types is bad.
  const auto reject_bad_target = []<class T>(const std::string& value,
                                             const T&) {
    DRAKE_EXPECT_THROWS_MESSAGE(AcceptIntoDummy<T>(LoadSingleValue(value)),
                                ".*incompatible !!binary tag.*");
  };  // NOLINT  -- templated lambda confuses cpplint about the semicolon.
  // These all use valid base64 encoding of what would otherwise be valid string
  // values for the serializable type.
  reject_bad_target("!!binary LmluZg==", DoubleStruct{});      // .inf
  reject_bad_target("!!binary aW5m", DoubleStruct{});          // inf
  reject_bad_target("!!binary MTIzNC41", DoubleStruct{});      // 1234.5
  reject_bad_target("!!binary MTIzNA==", IntStruct{});         // 1234
  reject_bad_target("!!binary dGVzdC9wYXRo", PathStruct{});    // test/path
  reject_bad_target("!!binary YSBzdHJpbmc=", StringStruct{});  // "a string"
}

TEST_P(YamlReadArchiveTest, Path) {
  const auto test_valid = [](const std::string& value,
                             const std::string& expected) {
    const auto& x = AcceptNoThrow<PathStruct>(LoadSingleValue(value));
    EXPECT_EQ(x.value.string(), expected);
  };
  // These plain strings are unchanged by parsing them into a fs::path.
  test_valid(".", ".");
  test_valid("no_directory.txt", "no_directory.txt");
  test_valid("/absolute/path/file.txt", "/absolute/path/file.txt");
  test_valid("/quoted\"/path", "/quoted\"/path");
  // These strings end up changing to a greater or lesser degree.
  test_valid("\"\"", "");
  test_valid("!!str", "");
  test_valid("/non_lexical//path", "/non_lexical/path");

  // Values that *might* have been plain scalars of other types explicitly
  // declared as strings (via various mechanisms) all become paths.
  test_valid("'1234'", "1234");
  test_valid("\"1234\"", "1234");
  test_valid("!!str 1234", "1234");

  // C++ parsing is lazy in determining the type. Rather than aggressively
  // parsing plain types into primitives, it defers until it sees the type it
  // is being written to. This make's Drake's YAML parsing more permissive.
  // This tests tracks the permissive behavior so when we fix it up (and make
  // it stricter), we'll be reminded to say something in release notes.
  test_valid("1234", "1234");

  // The following should all throw.
  const auto test_throw = [](const std::string& value) {
    SCOPED_TRACE(fmt::format("Should throw for {}\n", value));
    // TODO(SeanCurtis-TRI) Fix the code so that these throw and change this to
    // EXPECT_THROW.
    EXPECT_NO_THROW(AcceptIntoDummy<PathStruct>(LoadSingleValue(value)));
  };
  test_throw("!!float 1234.5");
  test_throw("1234.5");
  test_throw("1234");
  test_throw("!!int 1234");
  test_throw("!!bool true");
  test_throw("true");
}

TEST_P(YamlReadArchiveTest, PathMissing) {
  const PathStruct default_path;
  const auto& x = AcceptEmptyDoc<PathStruct>();
  EXPECT_EQ(x.value, default_path.value);
}

TEST_P(YamlReadArchiveTest, AllScalars) {
  const std::string doc = R"""(
doc:
  some_bool: true
  some_float: 100.0
  some_double: 101.0
  some_int32: 102
  some_uint32: 103
  some_int64: 104
  some_uint64: 105
  some_string: foo
  some_path: /alternative/path
  some_bytes: !!binary BQYH
)""";
  const auto& x = AcceptNoThrow<AllScalarsStruct>(Load(doc));
  EXPECT_EQ(x.some_bool, true);
  EXPECT_EQ(x.some_float, 100.0);
  EXPECT_EQ(x.some_double, 101.0);
  EXPECT_EQ(x.some_int32, 102);
  EXPECT_EQ(x.some_uint32, 103);
  EXPECT_EQ(x.some_int64, 104);
  EXPECT_EQ(x.some_uint64, 105);
  EXPECT_EQ(x.some_string, "foo");
  EXPECT_EQ(x.some_path, "/alternative/path");
  EXPECT_EQ(x.some_bytes, StringToByteVector("\x05\x06\x07"));
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
    string_map<double> adjusted_expected{expected.begin(), expected.end()};
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
    string_map<double> adjusted_expected{expected.begin(), expected.end()};
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
)""",
       {{"foo", 1.0}, {"bar", 2.0}});

  // A pre-existing value should win, though.
  test(R"""(
_template: &template
  foo: 3.0

doc:
  value:
    << : *template
    foo: 1.0
    bar: 2.0
)""",
       {{"foo", 1.0}, {"bar", 2.0}});

  // A list of merges should also work.
  test(R"""(
_template: &template
  - foo: 1.0
  - baz: 3.0

doc:
  value:
    << : *template
    bar: 2.0
)""",
       {{"foo", 1.0}, {"bar", 2.0}, {"baz", 3.0}});
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
    YamlReadArchive(node, GetParam()).Accept(&result);
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

/* Smoke test for compatibility for the odd scalar: vector<byte>. */
TEST_P(YamlReadArchiveTest, OptionalBytes) {
  const auto& x = AcceptNoThrow<OptionalBytesStruct>(
      Load("doc:\n  value: !!binary b3RoZXID/3N0dWZm"));
  ASSERT_TRUE(x.value.has_value());
  EXPECT_EQ(x.value.value(), StringToByteVector("other\x03\xffstuff"));
}

TEST_P(YamlReadArchiveTest, Variant) {
  const auto test = [](const std::string& doc, const Variant4& expected) {
    const auto& x = AcceptNoThrow<VariantStruct>(Load(doc));
    EXPECT_EQ(x.value, expected) << doc;
  };

  test("doc:\n  value: \"\"", "");
  test("doc:\n  value: foo", "foo");
  test("doc:\n  value: \"foo\"", "foo");
  test("doc:\n  value: !!str foo", "foo");
  test("doc:\n  value: !!float 1.0", 1.0);
  test("doc:\n  value: !DoubleStruct { value: 1.0 }", DoubleStruct{1.0});
}

TEST_P(YamlReadArchiveTest, PrimitiveVariant) {
  const auto test = [](const std::string& doc,
                       const PrimitiveVariant& expected) {
    const auto& x = AcceptNoThrow<PrimitiveVariantStruct>(Load(doc));
    EXPECT_EQ(x.value, expected) << doc;
  };

  test("doc:\n  value: [1.0, 2.0]", std::vector<double>{1.0, 2.0});
  test("doc:\n  value: !!bool true", true);
  test("doc:\n  value: !!bool 'true'", true);
  test("doc:\n  value: !!int 10", 10);
  test("doc:\n  value: !!int '10'", 10);
  test("doc:\n  value: !!float 1.0", 1.0);
  test("doc:\n  value: !!float '1.0'", 1.0);
  test("doc:\n  value: !!str foo", std::string("foo"));
  test("doc:\n  value: !!str 'foo'", std::string("foo"));
  test("doc:\n  value: !!binary A3Rlc3Rfc3RyAw==",
       StringToByteVector("\x03test_str\x03"));

  // It might be sensible for this case to pass, but for now we'll require that
  // non-0'th variant indices always use a tag even where it could be inferred.
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<PrimitiveVariantStruct>(Load("doc:\n  value: 1.0")),
      ".*vector.*double.*");
}

// When loading a variant, the default value should remain intact in cases where
// the type tag is unchanged.
TEST_P(YamlReadArchiveTest, VariantNestedDefaults) {
  const LoadYamlOptions param = GetParam();
  VariantStruct result{Variant4{DoubleStruct{.value = 22.0}}};
  const std::string doc = "doc: { value: !DoubleStruct {} }";
  auto parse = [&]() {
    YamlReadArchive(Load(doc), param).Accept(&result);
  };
  if (param.allow_cpp_with_no_yaml) {
    EXPECT_NO_THROW(parse());
  } else {
    EXPECT_THROW(parse(), std::exception);
  }
  EXPECT_EQ(std::get<DoubleStruct>(result.value).value, 22.0);
}

TEST_P(YamlReadArchiveTest, VariantMissing) {
  const auto& x = AcceptEmptyDoc<VariantStruct>();
  EXPECT_EQ(std::get<double>(x.value), kNominalDouble);
}

TEST_P(YamlReadArchiveTest, VariantTypePromotion) {
  const std::string doc = R"""(
doc:
  double_type: 1
  eigen_type: [1]
  path_type: /path/to/somewhere
  truthy_type: true
)""";

  const double expected_double = 1.0;
  const Eigen::VectorXd expected_eigen = Eigen::VectorXd::Constant(1, 1.0);
  const std::filesystem::path expected_path("/path/to/somewhere");
  const bool expected_bool = true;

  // Check that loading the above document into non-variant values promotes them
  // appropriately.
  const auto& basic = AcceptNoThrow<PromotionBasicStruct>(Load(doc));
  EXPECT_EQ(basic.double_type, expected_double);
  EXPECT_EQ(basic.eigen_type, expected_eigen);
  EXPECT_EQ(basic.path_type, expected_path);
  EXPECT_EQ(basic.truthy_type, expected_bool);

  // Check that loading the same document into variant values promotes them
  // exactly the same way.
  const auto& variant1 = AcceptNoThrow<PromotionVariantStruct>(Load(doc));
  EXPECT_EQ(std::get<0>(variant1.double_type), expected_double);
  EXPECT_EQ(std::get<0>(variant1.eigen_type), expected_eigen);
  EXPECT_EQ(std::get<0>(variant1.path_type), expected_path);
  EXPECT_EQ(std::get<0>(variant1.truthy_type), expected_bool);

  // Check the same thing but with the struct defaults initialized to the second
  // type in the variant instead of the first.
  PromotionVariantStruct variant2{
      .double_type = DoubleStruct{},
      .eigen_type = DoubleStruct{},
      .path_type = DoubleStruct{},
      .truthy_type = DoubleStruct{},
  };
  EXPECT_NO_THROW(YamlReadArchive(Load(doc), GetParam()).Accept(&variant2));
  EXPECT_EQ(std::get<0>(variant2.double_type), expected_double);
  EXPECT_EQ(std::get<0>(variant2.eigen_type), expected_eigen);
  EXPECT_EQ(std::get<0>(variant2.path_type), expected_path);
  EXPECT_EQ(std::get<0>(variant2.truthy_type), expected_bool);
}

TEST_P(YamlReadArchiveTest, EigenVector) {
  const auto test = [](const std::string& value,
                       const Eigen::VectorXd& expected) {
    const auto& vec = AcceptNoThrow<EigenVecStruct>(LoadSingleValue(value));
    const auto& vec3 = AcceptNoThrow<EigenVec3Struct>(LoadSingleValue(value));
    const auto& upto3 =
        AcceptNoThrow<EigenVecUpTo3Struct>(LoadSingleValue(value));
    EXPECT_TRUE(drake::CompareMatrices(vec.value, expected));
    EXPECT_TRUE(drake::CompareMatrices(vec3.value, expected));
    EXPECT_TRUE(drake::CompareMatrices(upto3.value, expected));
  };

  test("[1.0, 2.0, 3.0]", Eigen::Vector3d(1.0, 2.0, 3.0));
}

TEST_P(YamlReadArchiveTest, EigenArraySingleColumn) {
  const auto test = [](const std::string& value,
                       const Eigen::ArrayXd& expected) {
    const auto& vec =
        AcceptNoThrow<EigenArrayStruct<1>>(LoadSingleValue(value));
    EXPECT_TRUE(drake::CompareMatrices(vec.value.matrix(), expected.matrix()));
  };

  test("[1.0, 2.0, 3.0]", Eigen::Array3d(1.0, 2.0, 3.0));
}

TEST_P(YamlReadArchiveTest, EigenVectorX) {
  const auto test = [](const std::string& value,
                       const Eigen::VectorXd& expected) {
    const auto& x = AcceptNoThrow<EigenVecStruct>(LoadSingleValue(value));
    EXPECT_TRUE(drake::CompareMatrices(x.value, expected));
    const auto& upto3 =
        AcceptNoThrow<EigenVecUpTo3Struct>(LoadSingleValue(value));
    EXPECT_TRUE(drake::CompareMatrices(upto3.value, expected));
  };

  test("[]", Eigen::VectorXd());
  test("[1.0]", Eigen::Matrix<double, 1, 1>(1.0));
}

TEST_P(YamlReadArchiveTest, EigenVectorOverflow) {
  DRAKE_EXPECT_THROWS_MESSAGE(AcceptIntoDummy<EigenVecUpTo3Struct>(Load(R"""(
doc:
  value: [0, 0, 0, 0]
)""")),
                              ".*maximum size is 3.*");
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
)""",
       (Matrix34d{} << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11).finished());
}

TEST_P(YamlReadArchiveTest, EigenMatrixFixedSizeBadValue) {
  DRAKE_EXPECT_THROWS_MESSAGE(AcceptIntoDummy<EigenMatrix34Struct>(Load(R"""(
doc:
  value:
  - [1.0, 2.0, 3.0]
  - [4.0, 5.0, 6.0]
  - [7.0, 8.0, 9.0]
)""")),
                              ".*has dimension 3x3.*wanted 3x4.*");
}

TEST_P(YamlReadArchiveTest, EigenArrayRectangular) {
  using Array34d = Eigen::Array<double, 3, 4>;
  const auto test = [](const std::string& doc, const Array34d& expected) {
    const auto& mat = AcceptNoThrow<EigenArrayStruct<4>>(Load(doc));
    EXPECT_TRUE(drake::CompareMatrices(mat.value.matrix(), expected.matrix()));
  };

  test(R"""(
doc:
  value:
  - [1.0, 2.0, 3.0, 4.0]
  - [5.0, 6.0, 7.0, 8.0]
  - [9.0, 10.0, 11.0, 12.0]
)""",
       (Array34d{} << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12).finished());
}

TEST_P(YamlReadArchiveTest, EigenMatrixUpTo6) {
  using Matrix34d = Eigen::Matrix<double, 3, 4>;
  const auto test = [](const std::string& doc, const Matrix34d& expected) {
    const auto& mat = AcceptNoThrow<EigenMatrixUpTo6Struct>(Load(doc));
    EXPECT_TRUE(drake::CompareMatrices(mat.value, expected));
  };

  test(R"""(
doc:
  value:
  - [0.0, 1.0, 2.0, 3.0]
  - [4.0, 5.0, 6.0, 7.0]
  - [8.0, 9.0, 10.0, 11.0]
)""",
       (Matrix34d{} << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11).finished());
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
      doc = std::regex_replace(orig_doc,
                               std::regex(" *ignored_key: ignored_value"), "");
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
  const internal::Node node = Load(R"""(
doc:
  outer_value: 1.0
  inner_struct:
    inner_value_TYPO: 2.0
)""");
  if (GetParam().allow_cpp_with_no_yaml && GetParam().allow_yaml_with_no_cpp) {
    const auto& x = AcceptNoThrow<OuterStruct>(node);
    EXPECT_EQ(x.outer_value, 1.0);
    EXPECT_EQ(x.inner_struct.inner_value, kNominalDouble);
  } else if (GetParam().allow_cpp_with_no_yaml) {
    DRAKE_EXPECT_THROWS_MESSAGE(
        AcceptIntoDummy<OuterStruct>(node),
        "<string>:4:5:"
        " YAML node of type Mapping"
        " \\(with size 1 and keys \\{inner_value_TYPO\\}\\)"
        " key 'inner_value_TYPO' did not match any visited value entry for"
        " <root> while accepting YAML node of type Mapping"
        " \\(with size 2 and keys \\{inner_struct, outer_value\\}\\)"
        " while visiting [^ ]*InnerStruct inner_struct\\.");
  } else {
    DRAKE_EXPECT_THROWS_MESSAGE(
        AcceptIntoDummy<OuterStruct>(node),
        "<string>:4:5:"
        " YAML node of type Mapping"
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
      "<string>:4:5:"
      " YAML node of type Mapping \\(with size 1 and keys \\{inner_value\\}\\)"
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
      "<string>:4:5:"
      " YAML node of type Mapping \\(with size 1 and keys \\{inner_value\\}\\)"
      " has non-Scalar \\(Mapping\\) entry for double inner_value"
      " while accepting YAML node of type Mapping"
      " \\(with size 2 and keys \\{inner_struct, outer_value\\}\\)"
      " while visiting [^ ]*InnerStruct inner_struct\\.");
}

// This finds nothing when a std::array was wanted.
TEST_P(YamlReadArchiveTest, VisitArrayFoundNothing) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<ArrayStruct>(LoadSingleValue("")),
      "<string>:2:3:"
      " YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Null\\) entry for std::array<.*> value\\.");
}

// This finds a scalar when a std::array was wanted.
TEST_P(YamlReadArchiveTest, VisitArrayFoundScalar) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<ArrayStruct>(LoadSingleValue("1.0")),
      "<string>:2:3:"
      " YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
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
      "<string>:2:3:"
      " YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Mapping\\) entry for std::array<.*> value\\.");
}

// This finds nothing when a std::vector was wanted.
TEST_P(YamlReadArchiveTest, VisitVectorFoundNothing) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<VectorStruct>(LoadSingleValue("")),
      "<string>:2:3:"
      " YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Null\\) entry for std::vector<.*> value\\.");
}

// This finds a scalar when a std::vector was wanted.
TEST_P(YamlReadArchiveTest, VisitVectorFoundScalar) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<VectorStruct>(LoadSingleValue("1.0")),
      "<string>:2:3:"
      " YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
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
      "<string>:2:3:"
      " YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Mapping\\) entry for std::vector<.*> value\\.");
}

// This finds a sequence when an optional<double> was wanted.
TEST_P(YamlReadArchiveTest, VisitOptionalScalarFoundSequence) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<OptionalStruct>(LoadSingleValue("[1.0]")),
      "<string>:2:3:"
      " YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Scalar \\(Sequence\\) entry for std::optional<double>"
      " value\\.");
}

// This finds various untagged things when a variant was wanted.
TEST_P(YamlReadArchiveTest, VisitVariantFoundNoTag) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<VariantWrappingStruct>(
          Load("doc:\n  inner:\n    value:")),
      "<string>:3:5:"
      " YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Scalar \\(Null\\) entry for std::string value"
      " while accepting YAML node of type Mapping"
      " \\(with size 1 and keys \\{inner\\}\\)"
      " while visiting drake::yaml::test::VariantStruct inner.");

  // std::string values should load correctly even without a YAML type tag.
  const auto& str = AcceptNoThrow<VariantWrappingStruct>(
      Load("doc:\n  inner:\n    value: foo"));
  EXPECT_EQ(str.inner.value, Variant4("foo"));

  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<VariantWrappingStruct>(
          Load("doc:\n  inner:\n    value: [foo, bar]")),
      "<string>:3:5:"
      " YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Scalar \\(Sequence\\) entry for std::string value"
      " while accepting YAML node of type Mapping"
      " \\(with size 1 and keys \\{inner\\}\\)"
      " while visiting drake::yaml::test::VariantStruct inner.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<VariantWrappingStruct>(
          Load("doc:\n  inner:\n    value: {foo: bar}")),
      "<string>:3:5:"
      " YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Scalar \\(Mapping\\) entry for std::string value\\"
      " while accepting YAML node of type Mapping"
      " \\(with size 1 and keys \\{inner\\}\\)"
      " while visiting drake::yaml::test::VariantStruct inner.");
}

// This finds an unknown tag when a variant was wanted.
TEST_P(YamlReadArchiveTest, VisitVariantFoundUnknownTag) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<VariantStruct>(Load("doc:\n  value: !UnknownTag foo")),
      "<string>:2:3: "
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
      "<string>:2:3:"
      " YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Null\\) entry for Eigen::VectorXd value\\.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenVec3Struct>(LoadSingleValue(value)),
      "<string>:2:3:"
      " YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Null\\) entry for Eigen::Vector3d value\\.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenMatrixStruct>(LoadSingleValue(value)),
      "<string>:2:3:"
      " YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Null\\) entry for Eigen::MatrixXd value\\.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenMatrix34Struct>(LoadSingleValue(value)),
      "<string>:2:3:"
      " YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Null\\) entry for Eigen::Matrix.*3,4.* value\\.");
}

// This finds a scalar when an Eigen::Vector or Eigen::Matrix was wanted.
TEST_P(YamlReadArchiveTest, VisitEigenFoundScalar) {
  const std::string value{"1.0"};
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenVecStruct>(LoadSingleValue(value)),
      "<string>:2:3:"
      " YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Scalar\\) entry for Eigen::VectorXd value\\.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenVec3Struct>(LoadSingleValue(value)),
      "<string>:2:3:"
      " YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Scalar\\) entry for Eigen::Vector3d value\\.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenMatrixStruct>(LoadSingleValue(value)),
      "<string>:2:3:"
      " YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Scalar\\) entry for Eigen::MatrixXd value\\.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenMatrix34Struct>(LoadSingleValue(value)),
      "<string>:2:3:"
      " YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Scalar\\) entry for Eigen::Matrix.* value\\.");
}

// This finds a one-dimensional Sequence when a 2-d Eigen::Matrix was wanted.
TEST_P(YamlReadArchiveTest, VisitEigenMatrixFoundOneDimensional) {
  const std::string value{"[1.0, 2.0, 3.0, 4.0]"};
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenMatrixStruct>(LoadSingleValue(value)),
      "<string>:2:3:"
      " YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " is Sequence-of-Scalar \\(not Sequence-of-Sequence\\)"
      " entry for Eigen::MatrixXd value\\.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenMatrix34Struct>(LoadSingleValue(value)),
      "<string>:2:3:"
      " YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
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
      "<string>:2:3:"
      " YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has inconsistent cols dimensions entry for Eigen::MatrixXd value\\.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<EigenMatrix34Struct>(Load(doc)),
      "<string>:2:3:"
      " YAML node of type Mapping \\(with size 1 and keys \\{value\\}\\)"
      " has inconsistent cols dimensions entry for Eigen::Matrix.* value\\.");
}

// This finds nothing when a sub-structure was wanted.
TEST_P(YamlReadArchiveTest, VisitStructFoundNothing) {
  const internal::Node node = Load(R"""(
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
        "<string>:2:5:"
        " YAML node of type Mapping"
        " \\(with size 1 and keys \\{outer_value\\}\\)"
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
      "<string>:2:3:"
      " YAML node of type Mapping"
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
      "<string>:2:3:"
      " YAML node of type Mapping"
      " \\(with size 2 and keys \\{inner_struct, outer_value\\}\\)"
      " has non-Mapping \\(Sequence\\) entry for"
      " [^ ]*InnerStruct inner_struct\\.");
}

// The user input a top-level array instead of a mapping.
TEST_P(YamlReadArchiveTest, VisitRootStructFoundArray) {
  const std::string doc = R"""(
- foo
- bar
)""";
  const internal::Node root =
      YamlReadArchive::LoadStringAsNode(doc, std::nullopt);
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<OuterStruct>(root),
      ".*top level element should be a Mapping.*not a Sequence.*");
}

std::vector<LoadYamlOptions> MakeAllPossibleOptions() {
  std::vector<LoadYamlOptions> all;
  for (const bool i : {false, true}) {
    for (const bool j : {false, true}) {
      for (const bool k : {false, true}) {
        all.push_back(LoadYamlOptions{i, j, k});
      }
    }
  }
  return all;
}

INSTANTIATE_TEST_SUITE_P(AllOptions, YamlReadArchiveTest,
                         ::testing::ValuesIn(MakeAllPossibleOptions()));

}  // namespace
}  // namespace test
}  // namespace yaml
}  // namespace drake
