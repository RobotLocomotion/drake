#pragma once

#include <array>
#include <map>
#include <optional>
#include <ostream>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

#include <Eigen/Core>

#include "drake/common/name_value.h"

namespace drake {
namespace yaml {
namespace test {

// A value used in the test data below to include a default (placeholder) value
// when initializing struct data members.
constexpr double kNominalDouble = 1.2345;

// These unit tests use a variety of sample Serializable structs, showing what
// a user may write for their own schemas.

struct DoubleStruct {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  double value = kNominalDouble;
};

// This is used only for EXPECT_EQ, not by any YAML operations.
bool operator==(const DoubleStruct& a, const DoubleStruct& b) {
  return a.value == b.value;
}

struct StringStruct {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  std::string value = "kNominalDouble";
};

// This is used only for EXPECT_EQ, not by any YAML operations.
bool operator==(const StringStruct& a, const StringStruct& b) {
  return a.value == b.value;
}

struct AllScalarsStruct {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(some_bool));
    a->Visit(DRAKE_NVP(some_float));
    a->Visit(DRAKE_NVP(some_double));
    a->Visit(DRAKE_NVP(some_int32));
    a->Visit(DRAKE_NVP(some_uint32));
    a->Visit(DRAKE_NVP(some_int64));
    a->Visit(DRAKE_NVP(some_uint64));
    a->Visit(DRAKE_NVP(some_string));
  }

  bool some_bool = false;
  double some_double = kNominalDouble;
  float some_float = static_cast<float>(kNominalDouble);
  int32_t some_int32 = 12;
  uint32_t some_uint32 = 12;
  int64_t some_int64 = 14;
  uint64_t some_uint64 = 15;
  std::string some_string = "kNominalString";
};

struct ArrayStruct {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  ArrayStruct() {
    value.fill(kNominalDouble);
  }

  explicit ArrayStruct(const std::array<double, 3>& value_in)
      : value(value_in) {}

  std::array<double, 3> value;
};

struct VectorStruct {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  VectorStruct() {
    value.resize(1, kNominalDouble);
  }

  explicit VectorStruct(const std::vector<double>& value_in)
      : value(value_in) {}

  std::vector<double> value;
};

struct NonPodVectorStruct {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  NonPodVectorStruct() {
    value.resize(1, {"kNominalDouble"});
  }

  std::vector<StringStruct> value;
};

struct MapStruct {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  MapStruct() {
    value["kNominalDouble"] = kNominalDouble;
  }

  explicit MapStruct(const std::map<std::string, double>& value_in)
      : value(value_in) {}

  std::map<std::string, double> value;
};

struct UnorderedMapStruct {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  UnorderedMapStruct() {
    value["kNominalDouble"] = kNominalDouble;
  }

  explicit UnorderedMapStruct(
      const std::unordered_map<std::string, double>& value_in)
      : value(value_in) {}

  std::unordered_map<std::string, double> value;
};

struct OptionalStruct {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  OptionalStruct() {
    value = kNominalDouble;
  }

  explicit OptionalStruct(const double value_in)
      : OptionalStruct(std::optional<double>(value_in)) {}

  explicit OptionalStruct(const std::optional<double>& value_in)
      : value(value_in) {}

  std::optional<double> value;
};

struct OptionalStructNoDefault {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  OptionalStructNoDefault()
      : value(std::nullopt) {}

  explicit OptionalStructNoDefault(const double value_in)
      : OptionalStructNoDefault(std::optional<double>(value_in)) {}

  explicit OptionalStructNoDefault(const std::optional<double>& value_in)
      : value(value_in) {}

  std::optional<double> value;
};

template <int Rows, int Cols, int MaxRows = Rows, int MaxCols = Cols>
struct EigenStruct {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  EigenStruct() {
    if ((value.size() == 0) && (Rows != 0) && (Cols != 0)) {
      value.resize(1, 1);
    }
    value.setConstant(kNominalDouble);
  }

  explicit EigenStruct(const Eigen::Matrix<double, Rows, Cols>& value_in)
      : value(value_in) {}

  // We'd like to test a non-default value for Options, but it's awkward to have
  // to specify one as part of our template arguments.  Instead, we'll use our
  // variation in values of existing template arguments to establish variation
  // of Options as well.  The particular choice of Rows and DontAlign here is
  // irrelevant -- we just need any kind of variation.
  static constexpr int Options = (MaxRows != Rows) ? Eigen::DontAlign : 0;

  Eigen::Matrix<double, Rows, Cols, Options, MaxRows, MaxCols> value;
};

// This is used only for EXPECT_EQ, not by any YAML operations.
template <int Rows, int Cols>
bool operator==(const EigenStruct<Rows, Cols>& a,
                const EigenStruct<Rows, Cols>& b) {
  return a.value == b.value;
}

using EigenVecStruct = EigenStruct<Eigen::Dynamic, 1>;
using EigenVec3Struct = EigenStruct<3, 1>;
using EigenMatrixStruct = EigenStruct<Eigen::Dynamic, Eigen::Dynamic>;
using EigenMatrix34Struct = EigenStruct<3, 4>;
using EigenMatrix00Struct = EigenStruct<0, 0>;
using EigenMatrixUpTo6Struct =
    EigenStruct<Eigen::Dynamic, Eigen::Dynamic, 6, 6>;

using Variant4 = std::variant<
    std::string, double, DoubleStruct, EigenVecStruct>;

std::ostream& operator<<(std::ostream& os, const Variant4& value) {
  if (value.index() == 0) {
    os << "std::string{" << std::get<0>(value) << "}";
  } else if (value.index() == 1) {
    os << "double{" << std::get<1>(value) << "}";
  } else if (value.index() == 2) {
    os << "DoubleStruct{" << std::get<2>(value).value << "}";
  } else {
    os << "EigenVecStruct{" << std::get<3>(value).value << "}";
  }
  return os;
}

struct VariantStruct {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  VariantStruct() {
    value = kNominalDouble;
  }

  explicit VariantStruct(const Variant4& value_in)
      : value(value_in) {}

  Variant4 value;
};

struct VariantWrappingStruct {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(inner));
  }

  VariantStruct inner;
};

struct OuterStruct {
  struct InnerStruct {
    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(DRAKE_NVP(inner_value));
    }

    double inner_value = kNominalDouble;
  };

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(outer_value));
    a->Visit(DRAKE_NVP(inner_struct));
  }

  double outer_value = kNominalDouble;
  InnerStruct inner_struct;
};

struct OuterStructOpposite {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(inner_struct));
    a->Visit(DRAKE_NVP(outer_value));
  }

  // N.B. The opposite member order of OuterStruct.
  OuterStruct::InnerStruct inner_struct;
  double outer_value = kNominalDouble;
};

struct OuterWithBlankInner {
  struct Blank {
    template <typename Archive>
    void Serialize(Archive* a) {}
  };

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(outer_value));
    a->Visit(DRAKE_NVP(inner_struct));
  }

  double outer_value = kNominalDouble;
  Blank inner_struct;
};

struct BigMapStruct {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  BigMapStruct() {
    value["foo"].outer_value = 1.0;
    value["foo"].inner_struct.inner_value = 2.0;
  }

  std::map<std::string, OuterStruct> value;
};

}  // namespace test
}  // namespace yaml
}  // namespace drake
