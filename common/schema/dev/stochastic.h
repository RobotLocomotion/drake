#pragma once

#include <memory>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/name_value.h"
#include "drake/common/random.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace schema {

// TODO(ggould-tri) The documentation on DistributionVector{,Variant} isn't
// really adequate to Drake standards. It's misleadingly minimal and doesn't
// capture the subtlety of the semantics, in particular with respect to the
// homogeneity of the vector -- it is really a distribution over vectors more
// than a vector of distributions, since you can't have a DistributionVector
// that is [Gaussian, Gaussian, Uniform] or similarly non-homogeneous. This
// will become acute as people try to create stochastic free body states where
// the translational and rotational coordinates require different approaches:
// The type names will become the classic "maze of twisty little passages, all
// alike" in the style of BasicVector/VectorBase unless there is a worked
// example in hand for them to copy from that will avoid ever touching those
// pitfalls.
//
// When transform.h comes over from Anzu, then its documentation will become
// the most acute place where this information is missing. This class really
// needs a more doxygen-oriented explanation that includes examples.

/// Base class for a single distribution, to be used with YAML archives.
/// (See struct DistributionVector for vector-valued distributions.)
struct Distribution {
  virtual ~Distribution();

  virtual double Sample(drake::RandomGenerator* generator) const = 0;
  virtual double Mean() const = 0;
  virtual drake::symbolic::Expression ToSymbolic() const = 0;

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Distribution)
  Distribution();
};

/// A single deterministic `value`.
struct Deterministic final : public Distribution {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Deterministic)

  Deterministic();
  explicit Deterministic(double value);

  ~Deterministic() final;

  double Sample(drake::RandomGenerator*) const final;
  double Mean() const final;
  drake::symbolic::Expression ToSymbolic() const final;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  double value{};
};

/// A gaussian distribution with `mean` and `std`.
struct Gaussian final : public Distribution {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Gaussian)

  Gaussian();
  Gaussian(double mean, double std);
  ~Gaussian() final;

  double Sample(drake::RandomGenerator*) const final;
  double Mean() const final;
  drake::symbolic::Expression ToSymbolic() const final;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(mean));
    a->Visit(DRAKE_NVP(std));
  }

  double mean{};
  double std{};
};

/// A uniform distribution with `min` inclusive and `max` exclusive.
struct Uniform final : public Distribution {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Uniform)

  Uniform();
  Uniform(double min, double max);
  ~Uniform() final;

  double Sample(drake::RandomGenerator*) const final;
  double Mean() const final;
  drake::symbolic::Expression ToSymbolic() const final;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(min));
    a->Visit(DRAKE_NVP(max));
  }

  double min{};
  double max{};
};

/// Chooses from among discrete `values` with equal probability.
struct UniformDiscrete final : public Distribution {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(UniformDiscrete)

  UniformDiscrete();
  explicit UniformDiscrete(std::vector<double> values);
  ~UniformDiscrete() final;

  double Sample(drake::RandomGenerator*) const final;
  double Mean() const final;
  drake::symbolic::Expression ToSymbolic() const final;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(values));
  }

  std::vector<double> values;
};

/// Variant over all kinds of distributions.
using DistributionVariant = std::variant<
    double, Deterministic, Gaussian, Uniform, UniformDiscrete>;

/// Copies the given variant into a Distribution base class.
std::unique_ptr<Distribution> ToDistribution(
    const DistributionVariant& var);

/// Like Distribution::Sample, but on a DistributionVariant instead.
double Sample(const DistributionVariant& var,
              drake::RandomGenerator* generator);

/// Like Distribution::Mean, but on a DistributionVariant instead.
double Mean(const DistributionVariant& var);

/// Like Distribution::ToSymbolic, but on a DistributionVariant instead.
drake::symbolic::Expression ToSymbolic(const DistributionVariant& var);

/// Like Distribution::Sample, but elementwise over a collection of
/// possibly-heterogenous DistributionVariant instead.
Eigen::VectorXd Sample(const std::vector<DistributionVariant>& vec,
                       drake::RandomGenerator* generator);

/// Like Distribution::Mean, but elementwise over a collection of
/// possibly-heterogenous DistributionVariant instead.
Eigen::VectorXd Mean(const std::vector<DistributionVariant>& vec);

/// Like Distribution::ToSymbolic, but elementwise over a collection of
/// possibly-heterogenous DistributionVariant instead.
drake::VectorX<drake::symbolic::Expression> ToSymbolic(
    const std::vector<DistributionVariant>& vec);

/// Returns true iff `var` is set to a deterministic value.
bool IsDeterministic(const DistributionVariant& var);

/// If `var` is deterministic, retrieves its value.
/// @throws exception if `var` is not deterministic.
double GetDeterministicValue(const DistributionVariant& var);

// ---------------------------------------------------------------------------

/// Base class for a vector of distributions, to be used with YAML archives.
/// (See struct Distribution for single distributions.)
struct DistributionVector {
  virtual ~DistributionVector();

  virtual Eigen::VectorXd Sample(drake::RandomGenerator* generator) const = 0;
  virtual Eigen::VectorXd Mean() const = 0;
  virtual drake::VectorX<drake::symbolic::Expression> ToSymbolic() const = 0;

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DistributionVector)
  DistributionVector();
};

/// A single deterministic vector `value`.
/// @tparam Size rows at compile time (max 6) or else Eigen::Dynamic.
template <int Size>
struct DeterministicVector final : public DistributionVector {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DeterministicVector)

  DeterministicVector();
  explicit DeterministicVector(const drake::Vector<double, Size>& value);
  ~DeterministicVector() final;

  Eigen::VectorXd Sample(drake::RandomGenerator* generator) const final;
  Eigen::VectorXd Mean() const final;
  drake::VectorX<drake::symbolic::Expression> ToSymbolic() const final;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  drake::Vector<double, Size> value;
};

/// A gaussian distribution with vector `mean` and vector or scalar `std`.
/// @tparam Size rows at compile time (max 6) or else Eigen::Dynamic.
template <int Size>
struct GaussianVector final : public DistributionVector {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GaussianVector)

  GaussianVector();
  GaussianVector(const drake::Vector<double, Size>& mean,
                 const drake::VectorX<double>& std);
  ~GaussianVector() final;

  Eigen::VectorXd Sample(drake::RandomGenerator*) const final;
  Eigen::VectorXd Mean() const final;
  drake::VectorX<drake::symbolic::Expression> ToSymbolic() const final;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(mean));
    a->Visit(DRAKE_NVP(std));
  }

  drake::Vector<double, Size> mean;
  drake::VectorX<double> std;
};

/// A uniform distribution with vector `min` inclusive and vector `max`
/// exclusive.
/// @tparam Size rows at compile time (max 6) or else Eigen::Dynamic.
template <int Size>
struct UniformVector final : public DistributionVector {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(UniformVector)

  UniformVector();
  UniformVector(const drake::Vector<double, Size>& min,
                const drake::Vector<double, Size>& max);
  ~UniformVector() final;

  Eigen::VectorXd Sample(drake::RandomGenerator* generator) const final;
  Eigen::VectorXd Mean() const final;
  drake::VectorX<drake::symbolic::Expression> ToSymbolic() const final;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(min));
    a->Visit(DRAKE_NVP(max));
  }

  drake::Vector<double, Size> min;
  drake::Vector<double, Size> max;
};

namespace internal {
struct InvalidVariantSelection {
  template <typename Archive>
  void Serialize(Archive*) {
    DRAKE_UNREACHABLE();
  }
};
}  // namespace internal

/// Variant over all kinds of vector distributions.
///
/// If the Size parameter allows for 1-element vectors (i.e, is either 1 or
/// Eigen::Dynamic), then this variant also offers the single distributions.
///
/// @tparam Size rows at compile time (max 6) or else Eigen::Dynamic.
template <int Size>
using DistributionVectorVariant = std::variant<
  drake::Vector<double, Size>,
  DeterministicVector<Size>,
  GaussianVector<Size>,
  UniformVector<Size>,
  std::conditional_t<(Size == Eigen::Dynamic || Size == 1),
    Deterministic,
    internal::InvalidVariantSelection>,
  std::conditional_t<(Size == Eigen::Dynamic || Size == 1),
    Gaussian,
    internal::InvalidVariantSelection>,
  std::conditional_t<(Size == Eigen::Dynamic || Size == 1),
    Uniform,
    internal::InvalidVariantSelection>>;

/// DistributionVectorVariant that permits any vector size dynamically.
using DistributionVectorVariantX = DistributionVectorVariant<Eigen::Dynamic>;

/// Copies the given variant into a DistributionVector base class.
/// @tparam Size rows at compile time (max 6) or else Eigen::Dynamic.
template <int Size>
std::unique_ptr<DistributionVector> ToDistributionVector(
    const DistributionVectorVariant<Size>& vec);

/// Returns true iff this is set to a deterministic value.
/// @tparam Size rows at compile time (max 6) or else Eigen::Dynamic.
template <int Size>
bool IsDeterministic(const DistributionVectorVariant<Size>& vec);

/// If `vec` is deterministic, retrieves its value.
/// @throws exception if `vec` is not deterministic.
/// @tparam Size rows at compile time (max 6) or else Eigen::Dynamic.
template <int Size>
Eigen::VectorXd GetDeterministicValue(
    const DistributionVectorVariant<Size>& vec);

}  // namespace schema
}  // namespace drake
