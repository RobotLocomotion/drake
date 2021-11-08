#pragma once

#include <memory>
#include <type_traits>
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

/** @defgroup schema_stochastic Configuring distributions
@ingroup stochastic_systems
@{

This page describes how to use classes such as schema::Distribution to denote
stochastic quantities, as a bridge between loading a scenario specification and
populating the corresponding symbolic::Expression quantities into a
systems::System.

# Stochastic variables

We'll explain uses of schema::Distribution and related types using the matching
YAML syntax as parsed by yaml::LoadYamlFile.

Given this C++ data structure:

```
struct MyStuff {
  schema::DistributionVariant value;
};

MyStuff stuff;
```

You might load a YAML file such as this:

```
stuff:
  value: 1.0
```

The `stuff.value` is set to a constant (not stochastic) value 1.0.

Alternatively, you might load a YAML file such as this:

```
stuff:
  value: !Gaussian
    mean: 1.0
    stddev: 0.5
```

Now, `stuff.value` is set to gaussian variable with the given mean and standard
deviation.

The exclamation point syntax is a YAML type tag, which we use to specify the
type choice within an `std::variant`.  The schema::DistributionVariant is a
typedef for a specific `std::variant`.

There are a few other choices for the type.

Here, you might specify a real-valued uniform range:

```
stuff:
  value: !Uniform
    min: 1.0
    max: 5.0
```

Or, you might choose from a set of equally-likely options:

```
stuff:
  value: !UniformDiscrete
    values: [1.0, 1.5, 2.0]
```

You may also use YAML's flow style to fit everything onto a single line.
These one-line spellings are the equivalent to those above.

```
stuff:
  value: !Gaussian { mean: 1.0, stddev: 0.5 }
```

```
stuff:
  value: !Uniform { min: 1.0, max: 5.0 }
```

```
stuff:
  value: !UniformDiscrete { values: [1.0, 1.5, 2.0] }
```

# Vectors of stochastic variables

For convenience, we also provide the option to specify a vector of independent
stochastic variables with the same type.

We'll explain uses of schema::DistributionVector and related types using the
matching YAML syntax as parsed by yaml::LoadYamlFile.

Given this C++ data structure:

```
struct MyThing {
  schema::DistributionVectorVariantX value;
};

MyThing thing;
```

You might load a YAML file such as this:

```
thing:
  value: [1.0, 2.0, 3.0]
```

The `thing.value` is set to a constant (not stochastic) vector with three
elements.

You might also choose to constrain the vector to be a fixed size:

```
struct MyThing3 {
  schema::DistributionVectorVariant3 value;
};

MyThing3 thing3;
```

Whether fixed or dynamic size, you might specify stochastic variables:

```
thing:
  value: !GaussianVector
    mean: [2.1, 2.2, 2.3]
    stddev: [1.0]            # Same stddev each.
```

Or:

```
thing:
  value: !GaussianVector
    mean: [2.1, 2.2, 2.3]
    stddev: [1.0, 0.5, 0.2]  # Different stddev each.
```

Or:

```
thing:
  value: !UniformVector
    min: [10.0, 20.0]
    max: [11.0, 22.0]
```

@note You cannot mix, e.g., %Gaussian and %Uniform within the same vector; all
elements must be a homogeneous type.

All distributions still support constants for some elements and stochastic for
others by specifying a zero-sized range for the constant elements:

```
thing:
  value: !UniformVector   # The first element is a constant 2.0, not stochastic.
    min: [2.0, -1.0]
    max: [2.0,  1.0]
```

Or:

```
thing:
  value: !GaussianVector  # The first element is a constant 2.0, not stochastic.
    mean: [2.0, 3.0]
    stddev: [0.0, 1.0]
```

# See also

See @ref schema_transform for one practical application, of specifying
rotations, translations, and transforms using stochastic schemas.

@} */

/// Base class for a single distribution, to be used with YAML archives.
/// (See class DistributionVector for vector-valued distributions.)
///
/// See @ref serialize_tips for implementation details, especially the
/// unusually public member fields in our subclasses.
class Distribution {
 public:
  virtual ~Distribution();

  virtual double Sample(drake::RandomGenerator* generator) const = 0;
  virtual double Mean() const = 0;
  virtual drake::symbolic::Expression ToSymbolic() const = 0;

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Distribution)
  Distribution();
};

/// A single deterministic `value`.
class Deterministic final : public Distribution {
 public:
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

/// A gaussian distribution with `mean` and `stddev`.
class Gaussian final : public Distribution {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Gaussian)

  Gaussian();
  Gaussian(double mean, double stddev);
  ~Gaussian() final;

  double Sample(drake::RandomGenerator*) const final;
  double Mean() const final;
  drake::symbolic::Expression ToSymbolic() const final;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(mean));
    a->Visit(DRAKE_NVP(stddev));
  }

  double mean{};
  double stddev{};
};

/// A uniform distribution with `min` inclusive and `max` exclusive.
class Uniform final : public Distribution {
 public:
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
class UniformDiscrete final : public Distribution {
 public:
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
/// @throws std::exception if `var` is not deterministic.
double GetDeterministicValue(const DistributionVariant& var);

// ---------------------------------------------------------------------------

/// Base class for a vector of distributions, to be used with YAML archives.
/// (See class Distribution for scalar-valued distributions.)
///
/// See @ref serialize_tips for implementation details, especially the
/// unusually public member fields in our subclasses.
class DistributionVector {
 public:
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
class DeterministicVector final : public DistributionVector {
 public:
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

/// A gaussian distribution with vector `mean` and vector or scalar `stddev`.
///
/// When `mean` and `stddev` both have the same number of elements, that
/// denotes an elementwise pairing of the 0th mean with 0th stddev, 1st mean
/// with 1st stddev, etc.
///
/// Alternatively, `stddev` can be a vector with a single element, no matter
/// the size of `mean`; that denotes the same `stddev` value applied to every
/// element of `mean`.
///
/// @tparam Size rows at compile time (max 6) or else Eigen::Dynamic.
template <int Size>
class GaussianVector final : public DistributionVector {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GaussianVector)

  GaussianVector();
  GaussianVector(const drake::Vector<double, Size>& mean,
                 const drake::VectorX<double>& stddev);
  ~GaussianVector() final;

  Eigen::VectorXd Sample(drake::RandomGenerator*) const final;
  Eigen::VectorXd Mean() const final;
  drake::VectorX<drake::symbolic::Expression> ToSymbolic() const final;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(mean));
    a->Visit(DRAKE_NVP(stddev));
  }

  drake::Vector<double, Size> mean;
  drake::VectorX<double> stddev;
};

/// A uniform distribution with vector `min` inclusive and vector `max`
/// exclusive.
/// @tparam Size rows at compile time (max 6) or else Eigen::Dynamic.
template <int Size>
class UniformVector final : public DistributionVector {
 public:
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
/// Eigen::Dynamic), then this variant also offers the single distribution
/// types (Deterministic, Gaussian, Uniform).  If the Size parameter is 2 or
/// greater, the single distribution types are not allowed.
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

/// Returns true iff all of `vec`'s elements are set to a deterministic value.
/// @tparam Size rows at compile time (max 6) or else Eigen::Dynamic.
template <int Size>
bool IsDeterministic(const DistributionVectorVariant<Size>& vec);

/// If `vec` is deterministic, retrieves its value.
/// @throws std::exception if `vec` is not deterministic.
/// @tparam Size rows at compile time (max 6) or else Eigen::Dynamic.
template <int Size>
Eigen::VectorXd GetDeterministicValue(
    const DistributionVectorVariant<Size>& vec);

}  // namespace schema
}  // namespace drake
