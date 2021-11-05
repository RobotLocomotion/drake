#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/name_value.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/common/yaml/yaml_read_archive.h"

using drake::yaml::SaveYamlString;
using drake::yaml::YamlReadArchive;

namespace drake {
namespace yaml {
namespace {

struct Inner {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(doubles));
  }

  std::vector<double> doubles;
};

struct Outer {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(inners));
  }

  std::vector<Inner> inners;
};

struct Map {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(items));
  }

  std::map<std::string, Outer> items;
};

GTEST_TEST(YamlPerformanceTest, VectorNesting) {
  // Populate a resonably-sized but non-trival set of data -- 50,000 numbers
  // arranged into a map with nested vectors.
  const int kDim = 100;
  Map data;
  double dummy = 1.0;
  const std::vector keys{"a", "b", "c", "d", "e"};
  for (const std::string& key : keys) {
    Outer& outer = data.items[key];
    outer.inners.resize(kDim);
    for (Inner& inner : outer.inners) {
      inner.doubles.resize(kDim, dummy);
      dummy += 1.0;
    }
  }

  // Convert the `Map data` into yaml string format.
  const std::string yaml_data = SaveYamlString(data, "doc");

  // Parse the yaml string into a node tree while checking that resource
  // usage is somewhat bounded.
  internal::Node yaml_root = internal::Node::MakeNull();
  {
    test::LimitMalloc guard({.max_num_allocations = 5'000'000});
    yaml_root = YamlReadArchive::LoadStringAsNode(yaml_data, "doc");
  }

  // Transfer the node tree into a C++ structure while checking that resource
  // usage is sane.
  Map new_data;
  {
    // When the performance of parsing was fixed and this test was added, this
    // Accept operation used about 51,000 allocations and took about 1 second
    // of wall clock time (for both release and debug builds).
    //
    // The prior implementation with gratuitous copies used over 2.6 billion
    // allocations and took more than 10 minutes of wall clock time in a
    // release build.
    //
    // We'll set the hard limit ~20x higher than currently observed to allow
    // some flux as library implementations evolve, etc.
    test::LimitMalloc guard({.max_num_allocations = 1'000'000});
    const YamlReadArchive::Options default_options;
    YamlReadArchive archive(std::move(yaml_root), default_options);
    archive.Accept(&new_data);
  }

  // Double-check that we actually did the work.
  ASSERT_EQ(new_data.items.size(), keys.size());
  for (const std::string& key : keys) {
    Outer& outer = new_data.items[key];
    ASSERT_EQ(outer.inners.size(), kDim);
    for (Inner& inner : outer.inners) {
      ASSERT_EQ(inner.doubles.size(), kDim);
    }
  }
}

}  // namespace
}  // namespace yaml
}  // namespace drake

using ADS1 = Eigen::AutoDiffScalar<drake::VectorX<double>>;
using ADS2 = Eigen::AutoDiffScalar<drake::VectorX<ADS1>>;

// Add ADL Serialize method to Eigen::AutoDiffScalar.
namespace Eigen {
template <typename Archive>
void Serialize(Archive* a, ADS2* x) {
  a->Visit(drake::MakeNameValue("value", &(x->value())));
  a->Visit(drake::MakeNameValue("derivatives", &(x->derivatives())));
}
template <typename Archive>
void Serialize(Archive* a, ADS1* x) {
  a->Visit(drake::MakeNameValue("value", &(x->value())));
  a->Visit(drake::MakeNameValue("derivatives", &(x->derivatives())));
}
}  // namespace Eigen

namespace drake {
namespace yaml {
namespace {

struct BigEigen {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  MatrixX<ADS2> value;
};

GTEST_TEST(YamlPerformanceTest, EigenMatrix) {
  // Populate a resonably-sized but non-trival set of data, about ~10,000
  // numbers stored at various levels of nesting.
  BigEigen data;
  const int kDim = 10;
  data.value.resize(kDim, kDim);
  double dummy = 1.0;
  for (int i = 0; i < data.value.rows(); ++i) {
    for (int j = 0; j < data.value.cols(); ++j) {
      ADS2& x = data.value(i, j);
      x.value() = dummy;
      dummy += 1.0;
      x.derivatives().resize(kDim);
      for (int k = 0; k < x.derivatives().size(); ++k) {
        ADS1& y = x.derivatives()(k);
        y.value() = dummy;
        dummy += 1.0;
        y.derivatives() = Eigen::VectorXd::Zero(kDim);
      }
    }
  }

  // Convert the `BigEigen data` into yaml string format.
  const std::string yaml_data = SaveYamlString(data, "doc");

  // Parse the yaml string into a node tree while checking that resource
  // usage is somewhat bounded.
  internal::Node yaml_root = internal::Node::MakeNull();
  {
    test::LimitMalloc guard({.max_num_allocations = 5'000'000});
    yaml_root = YamlReadArchive::LoadStringAsNode(yaml_data, "doc");
  }

  // Transfer the node tree into a C++ structure while checking that resource
  // usage is sane.
  BigEigen new_data;
  {
    // When the performance of parsing was fixed and this test was added, this
    // Accept operation used about 12,000 allocations and took less than 1
    // second of wall clock time (for both release and debug builds).
    //
    // The prior implementation with gratuitous copies used over 1.6 million
    // allocations.
    //
    // We'll set the hard limit ~20x higher than currently observed to allow
    // some flux as library implementations evolve, etc.
    test::LimitMalloc guard({.max_num_allocations = 250000});
    const YamlReadArchive::Options default_options;
    YamlReadArchive archive(std::move(yaml_root), default_options);
    archive.Accept(&new_data);
  }

  // Double-check that we actually did the work.
  ASSERT_EQ(new_data.value.rows(), kDim);
  ASSERT_EQ(new_data.value.cols(), kDim);
  for (int i = 0; i < kDim; ++i) {
    for (int j = 0; j < kDim; ++j) {
      ADS2& x = new_data.value(i, j);
      ASSERT_EQ(x.derivatives().size(), kDim);
      for (int k = 0; k < kDim; ++k) {
        ADS1& y = x.derivatives()(k);
        ASSERT_EQ(y.derivatives().size(), kDim);
      }
    }
  }
}

}  // namespace
}  // namespace yaml
}  // namespace drake
