#include "drake/common/yaml/yaml_read_archive.h"

#include <cmath>
#include <limits>
#include <sstream>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "drake/common/name_value.h"
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

  drake::optional<double> value;
};

using Variant3 = drake::variant<std::string, double, DoubleStruct>;

std::ostream& operator<<(std::ostream& os, const Variant3& value) {
  if (value.index() == 0) {
    os << "std::string{" << drake::get<0>(value) << "}";
  } else if (value.index() == 1) {
    os << "double{" << drake::get<1>(value) << "}";
  } else {
    os << "DoubleStruct{" << drake::get<2>(value).value << "}";
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
class YamlReadArchiveTest : public ::testing::Test {
 public:
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

  static YAML::Node LoadSingleValue(const std::string& value) {
    return Load("doc:\n  value: " + value + "\n");
  }

  template <typename Serializable>
  static Serializable AcceptNoThrow(const YAML::Node& root) {
    Serializable result{};
    bool raised = false;
    std::string what;
    try {
      YamlReadArchive(root).Accept(&result);
    } catch (const std::exception& e) {
      raised = true;
      what = e.what();
    }
    EXPECT_FALSE(raised);
    EXPECT_EQ(what, "");
    return result;
  }

  template <typename Serializable>
  static void AcceptIntoDummy(const YAML::Node& root) {
    Serializable dummy{};
    YamlReadArchive(root).Accept(&dummy);
  }
};

TEST_F(YamlReadArchiveTest, Double) {
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

TEST_F(YamlReadArchiveTest, StdArray) {
  const auto test = [](const std::string& value,
                       const std::array<double, 3>& expected) {
    const auto& x = AcceptNoThrow<ArrayStruct>(LoadSingleValue(value));
    EXPECT_EQ(x.value, expected);
  };

  test("[1.0, 2.0, 3.0]", {1.0, 2.0, 3.0});
}

TEST_F(YamlReadArchiveTest, StdVector) {
  const auto test = [](const std::string& value,
                       const std::vector<double>& expected) {
    const auto& x = AcceptNoThrow<VectorStruct>(LoadSingleValue(value));
    EXPECT_EQ(x.value, expected);
  };

  test("[1.0, 2.0, 3.0]", {1.0, 2.0, 3.0});
}

TEST_F(YamlReadArchiveTest, StdMap) {
  const auto test = [](const std::string& doc,
                       const std::map<std::string, double>& expected) {
    const auto& x = AcceptNoThrow<MapStruct>(Load(doc));
    EXPECT_EQ(x.value, expected) << doc;
  };

  test("doc:\n  value:\n    foo: 0.0\n    bar: 1.0\n",
       {{"foo", 0.0}, {"bar", 1.0}});
}

TEST_F(YamlReadArchiveTest, Optional) {
  const auto test = [](const std::string& doc,
                       const drake::optional<double>& expected) {
    const auto& x = AcceptNoThrow<OptionalStruct>(Load(doc));
    if (expected.has_value()) {
      ASSERT_TRUE(x.value.has_value()) << *expected;
      EXPECT_EQ(x.value.value(), expected.value()) << *expected;
    } else {
      EXPECT_FALSE(x.value.has_value());
    }
  };

  test("doc:\n  foo: bar", drake::nullopt);
  test("doc:\n  value:", drake::nullopt);
  test("doc:\n  value: 1.0", 1.0);
}

TEST_F(YamlReadArchiveTest, Variant) {
  const auto test = [](const std::string& doc,
                       const Variant3& expected) {
    const auto& x = AcceptNoThrow<VariantStruct>(Load(doc));
    EXPECT_EQ(x.value, expected) << doc;
  };

  test("doc:\n  value: !!str foo", "foo");
  test("doc:\n  value: !!float 1.0", 1.0);
  test("doc:\n  value: !DoubleStruct { value: 1.0 }", DoubleStruct{1.0});
}

TEST_F(YamlReadArchiveTest, EigenVector) {
  const auto test = [](const std::string& value,
                       const Eigen::VectorXd& expected) {
    const auto& vec = AcceptNoThrow<EigenVecStruct>(LoadSingleValue(value));
    const auto& vec3 = AcceptNoThrow<EigenVec3Struct>(LoadSingleValue(value));
    EXPECT_TRUE(drake::CompareMatrices(vec.value, expected));
    EXPECT_TRUE(drake::CompareMatrices(vec3.value, expected));
  };

  test("[1.0, 2.0, 3.0]", Eigen::Vector3d(1.0, 2.0, 3.0));
}

TEST_F(YamlReadArchiveTest, EigenVectorX) {
  const auto test = [](const std::string& value,
                       const Eigen::VectorXd& expected) {
    const auto& x = AcceptNoThrow<EigenVecStruct>(LoadSingleValue(value));
    EXPECT_TRUE(drake::CompareMatrices(x.value, expected));
  };

  test("[]", Eigen::VectorXd());
  test("[1.0]", Eigen::Matrix<double, 1, 1>(1.0));
}

TEST_F(YamlReadArchiveTest, EigenMatrix) {
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

TEST_F(YamlReadArchiveTest, Nested) {
  const auto& x = AcceptNoThrow<OuterStruct>(Load(R"R(
doc:
  outer_value: 1.0
  inner_struct:
    inner_value: 2.0
)R"));
  EXPECT_EQ(x.outer_value, 1.0);
  EXPECT_EQ(x.inner_struct.inner_value, 2.0);
}

// This finds nothing when a scalar was wanted, because the name had a typo.
TEST_F(YamlReadArchiveTest, VisitScalarFoundNothing) {
  // This has a "_TYPO" in a field name.
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<OuterStruct>(Load(R"R(
doc:
  outer_value: 1.0
  inner_struct:
    inner_value_TYPO: 2.0
)R")),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{inner_value_TYPO\\}\\)"
      " is missing entry for double inner_value"
      " while accepting YAML node of type Map"
      " \\(with size 2 and keys \\{inner_struct, outer_value\\}\\)"
      " while visiting [^ ]*InnerStruct inner_struct\\.");
}

// This finds an array when a scalar was wanted.
TEST_F(YamlReadArchiveTest, VisitScalarFoundArray) {
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
TEST_F(YamlReadArchiveTest, VisitScalarFoundStruct) {
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
TEST_F(YamlReadArchiveTest, VisitArrayFoundNothing) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<ArrayStruct>(LoadSingleValue("")),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Null\\) entry for std::array<.*> value\\.");
}

// This finds a scalar when a std::array was wanted.
TEST_F(YamlReadArchiveTest, VisitArrayFoundScalar) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<ArrayStruct>(LoadSingleValue("1.0")),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Scalar\\) entry for std::array<.*> value\\.");
}

// This finds a sub-structure when a std::array was wanted.
TEST_F(YamlReadArchiveTest, VisitArrayFoundStruct) {
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
TEST_F(YamlReadArchiveTest, VisitVectorFoundNothing) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<VectorStruct>(LoadSingleValue("")),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Null\\) entry for std::vector<.*> value\\.");
}

// This finds a scalar when a std::vector was wanted.
TEST_F(YamlReadArchiveTest, VisitVectorFoundScalar) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<VectorStruct>(LoadSingleValue("1.0")),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " has non-Sequence \\(Scalar\\) entry for std::vector<.*> value\\.");
}

// This finds a sub-structure when a std::vector was wanted.
TEST_F(YamlReadArchiveTest, VisitVectorFoundStruct) {
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
TEST_F(YamlReadArchiveTest, VisitOptionalScalarFoundSequence) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<OptionalStruct>(LoadSingleValue("[1.0]")),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\)"
      " has non-Scalar \\(Sequence\\) entry for .*optional<double> value\\.");
}

// This finds something untagged tag when a variant was wanted.
TEST_F(YamlReadArchiveTest, VisitVariantFoundNoTag) {
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
TEST_F(YamlReadArchiveTest, VisitVariantFoundUnknownTag) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<VariantStruct>(Load("doc:\n  value: !UnknownTag foo")),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{value\\}\\) "
      "has unsupported type tag !UnknownTag "
      "while selecting a variant<> entry for "
      "st.::variant<std::string,double,\\(anonymous\\)::DoubleStruct> value.");
}

// This finds nothing when an Eigen::Vector or Eigen::Matrix was wanted.
TEST_F(YamlReadArchiveTest, VisitEigenFoundNothing) {
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
TEST_F(YamlReadArchiveTest, VisitEigenFoundScalar) {
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
TEST_F(YamlReadArchiveTest, VisitEigenMatrixFoundOneDimensional) {
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
TEST_F(YamlReadArchiveTest, VisitEigenMatrixFoundNonSquare) {
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
TEST_F(YamlReadArchiveTest, VisitStructFoundNothing) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      AcceptIntoDummy<OuterStruct>(Load(R"R(
doc:
  outer_value: 1.0
)R")),
      std::runtime_error,
      "YAML node of type Map \\(with size 1 and keys \\{outer_value\\}\\)"
      " is missing entry for [^ ]*InnerStruct inner_struct\\.");
}

// This finds a scalar when a sub-structure was wanted.
TEST_F(YamlReadArchiveTest, VisitStructFoundScalar) {
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
TEST_F(YamlReadArchiveTest, VisitStructFoundArray) {
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

}  // namespace
}  // namespace yaml
}  // namespace drake
