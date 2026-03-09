#include "drake/common/schema/stochastic.h"

#include <stdexcept>
#include <utility>

#include <fmt/format.h>

#include "drake/common/nice_type_name.h"
#include "drake/common/overloaded.h"
#include "drake/common/unused.h"

using drake::symbolic::Expression;
using std::unique_ptr;

namespace drake {
namespace schema {

Distribution::Distribution() {}

Distribution::~Distribution() {}

Deterministic::Deterministic() {}

Deterministic::Deterministic(double value_in) : value(value_in) {}

Deterministic::~Deterministic() {}

double Deterministic::Sample(drake::RandomGenerator* generator) const {
  unused(generator);
  return value;
}

double Deterministic::Mean() const {
  return value;
}

Expression Deterministic::ToSymbolic() const {
  return value;
}

Gaussian::Gaussian() {}

Gaussian::Gaussian(double mean_in, double stddev_in)
    : mean(mean_in), stddev(stddev_in) {}

Gaussian::~Gaussian() {}

double Gaussian::Sample(drake::RandomGenerator* generator) const {
  std::normal_distribution<double> distribution(mean, stddev);
  return distribution(*generator);
}

double Gaussian::Mean() const {
  return mean;
}

Expression Gaussian::ToSymbolic() const {
  std::normal_distribution<Expression> distribution(mean, stddev);
  return distribution();
}

Uniform::Uniform() {}

Uniform::Uniform(double min_in, double max_in)
    : min(min_in), max(max_in) {}  // NOLINT(build/include_what_you_use)

Uniform::~Uniform() {}

double Uniform::Sample(drake::RandomGenerator* generator) const {
  std::uniform_real_distribution<double> distribution(min, max);
  return distribution(*generator);
}

double Uniform::Mean() const {
  return (min + max) / 2.0;
}

Expression Uniform::ToSymbolic() const {
  std::uniform_real_distribution<Expression> distribution(min, max);
  return distribution();
}

UniformDiscrete::UniformDiscrete() {}

UniformDiscrete::UniformDiscrete(std::vector<double> values_in)
    : values(std::move(values_in)) {}

UniformDiscrete::~UniformDiscrete() {}

double UniformDiscrete::Sample(drake::RandomGenerator* generator) const {
  if (values.empty()) {
    throw std::logic_error(
        "Cannot Sample() empty UniformDiscrete distribution.");
  }
  const std::vector<double> weights(values.size(), 1.);
  const int index = std::discrete_distribution<int>(weights.begin(),
                                                    weights.end())(*generator);
  return values.at(index);
}

double UniformDiscrete::Mean() const {
  if (values.empty()) {
    throw std::logic_error("Cannot Mean() empty UniformDiscrete distribution.");
  }
  double sum = 0;
  for (double value : values) {
    sum += value;
  }
  return sum / values.size();
}

Expression UniformDiscrete::ToSymbolic() const {
  if (values.empty()) {
    throw std::logic_error(
        "Cannot ToSymbolic() empty UniformDiscrete distribution.");
  }
  // Sample a real number distribution over [0, num_values).
  const int num_values = values.size();
  std::uniform_real_distribution<Expression> distribution(0, num_values);
  const Expression real_index = distribution();
  // Loop to compute the result as if we were doing:
  //   result = (real_index < 1.0) ? values[0] :
  //            (real_index < 2.0) ? values[1] :
  //            ...
  //            values.back();
  Expression result = values.back();
  for (int i = num_values - 2; i >= 0; --i) {
    result = if_then_else(real_index < (i + 1), values[i], result);
  }
  return result;
}

unique_ptr<Distribution> ToDistribution(const DistributionVariant& var) {
  return std::visit(
      [](auto&& arg) -> unique_ptr<Distribution> {
        using ContainedType = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<ContainedType, double>) {
          return std::make_unique<Deterministic>(arg);
        } else {
          return std::make_unique<ContainedType>(arg);
        }
      },
      var);
}

double Sample(const DistributionVariant& var,
              drake::RandomGenerator* generator) {
  return ToDistribution(var)->Sample(generator);
}

double Mean(const DistributionVariant& var) {
  return ToDistribution(var)->Mean();
}

Expression ToSymbolic(const DistributionVariant& var) {
  return ToDistribution(var)->ToSymbolic();
}

Eigen::VectorXd Sample(const std::vector<DistributionVariant>& vec,
                       drake::RandomGenerator* generator) {
  Eigen::VectorXd result(vec.size());
  for (size_t i = 0; i < vec.size(); ++i) {
    result(i) = Sample(vec[i], generator);
  }
  return result;
}

Eigen::VectorXd Mean(const std::vector<DistributionVariant>& vec) {
  Eigen::VectorXd result(vec.size());
  for (size_t i = 0; i < vec.size(); ++i) {
    result(i) = Mean(vec[i]);
  }
  return result;
}

drake::VectorX<Expression> ToSymbolic(
    const std::vector<DistributionVariant>& vec) {
  drake::VectorX<Expression> result(vec.size());
  for (size_t i = 0; i < vec.size(); ++i) {
    result(i) = ToSymbolic(vec[i]);
  }
  return result;
}

bool IsDeterministic(const DistributionVariant& var) {
  return std::visit<bool>(  // BR
      overloaded{
          [](const double&) {
            return true;
          },
          [](const Deterministic&) {
            return true;
          },
          [](const Gaussian& arg) {
            return arg.stddev == 0.0;
          },
          [](const Uniform& arg) {
            return arg.min == arg.max;
          },
          [](const UniformDiscrete& arg) {
            return arg.values.size() == 1;
          },
      },
      var);
}

double GetDeterministicValue(const DistributionVariant& var) {
  if (!IsDeterministic(var)) {
    std::visit(
        [](auto&& arg) {
          using ContainedType = std::decay_t<decltype(arg)>;
          throw std::logic_error(fmt::format(
              "Attempt to GetDeterministicValue() on a variant that contains "
              "a {}",
              drake::NiceTypeName::Get<ContainedType>()));
        },
        var);
  }
  return ToDistribution(var)->Mean();
}

DistributionVector::DistributionVector() {}

DistributionVector::~DistributionVector() {}

template <int Size>
DeterministicVector<Size>::DeterministicVector() {}

template <int Size>
DeterministicVector<Size>::DeterministicVector(
    const drake::Vector<double, Size>& value_in)
    : value(value_in) {}

template <int Size>
DeterministicVector<Size>::~DeterministicVector() {}

template <int Size>
Eigen::VectorXd DeterministicVector<Size>::Sample(
    drake::RandomGenerator* generator) const {
  unused(generator);
  return value;
}

template <int Size>
Eigen::VectorXd DeterministicVector<Size>::Mean() const {
  return value;
}

template <int Size>
drake::VectorX<Expression> DeterministicVector<Size>::ToSymbolic() const {
  return value.template cast<Expression>();
}

template <int Size>
GaussianVector<Size>::GaussianVector() {}

template <int Size>
GaussianVector<Size>::GaussianVector(const drake::Vector<double, Size>& mean_in,
                                     const drake::VectorX<double>& stddev_in)
    : mean(mean_in), stddev(stddev_in) {}

template <int Size>
GaussianVector<Size>::~GaussianVector() {}

template <int Size>
Eigen::VectorXd GaussianVector<Size>::Sample(
    drake::RandomGenerator* generator) const {
  if (!(stddev.size() == mean.size() || stddev.size() == 1)) {
    throw std::logic_error(fmt::format(
        "Cannot Sample() a GaussianVector distribution with size {} mean "
        "but size {} dev",
        mean.size(), stddev.size()));
  }
  Eigen::VectorXd result(mean.size());
  for (int i = 0; i < mean.size(); ++i) {
    const double stddev_i = (stddev.size() == 1) ? stddev(0) : stddev(i);
    result(i) = Gaussian(mean(i), stddev_i).Sample(generator);
  }
  return result;
}

template <int Size>
Eigen::VectorXd GaussianVector<Size>::Mean() const {
  return mean;
}

template <int Size>
drake::VectorX<Expression> GaussianVector<Size>::ToSymbolic() const {
  if (!(stddev.size() == mean.size() || stddev.size() == 1)) {
    throw std::logic_error(fmt::format(
        "Cannot ToSymbolic() a GaussianVector distribution with size {} mean "
        "but size {} dev",
        mean.size(), stddev.size()));
  }
  drake::VectorX<Expression> result(mean.size());
  for (int i = 0; i < mean.size(); ++i) {
    const double stddev_i = (stddev.size() == 1) ? stddev(0) : stddev(i);
    result(i) = Gaussian(mean(i), stddev_i).ToSymbolic();
  }
  return result;
}

template <int Size>
UniformVector<Size>::UniformVector() {}

template <int Size>
UniformVector<Size>::UniformVector(const drake::Vector<double, Size>& min_in,
                                   const drake::Vector<double, Size>& max_in)
    : min(min_in), max(max_in) {}  // NOLINT(build/include_what_you_use)

template <int Size>
UniformVector<Size>::~UniformVector() {}

template <int Size>
Eigen::VectorXd UniformVector<Size>::Sample(
    drake::RandomGenerator* generator) const {
  if (min.size() != max.size()) {
    throw std::logic_error(fmt::format(
        "Cannot Sample() a UniformVector distribution with size {} min "
        "but size {} max",
        min.size(), max.size()));
  }
  Eigen::VectorXd result(max.size());
  for (int i = 0; i < max.size(); ++i) {
    result(i) = Uniform(min(i), max(i)).Sample(generator);
  }
  return result;
}

template <int Size>
Eigen::VectorXd UniformVector<Size>::Mean() const {
  if (min.size() != max.size()) {
    throw std::logic_error(fmt::format(
        "Cannot Mean() a UniformVector distribution with size {} min "
        "but size {} max",
        min.size(), max.size()));
  }
  return (min + max) / 2.0;
}

template <int Size>
drake::VectorX<Expression> UniformVector<Size>::ToSymbolic() const {
  if (min.size() != max.size()) {
    throw std::logic_error(fmt::format(
        "Cannot ToSymbolic() a UniformVector distribution with size {} min "
        "but size {} max",
        min.size(), max.size()));
  }
  drake::VectorX<Expression> result(max.size());
  for (int i = 0; i < max.size(); ++i) {
    // NOLINTNEXTLINE(build/include_what_you_use)
    result(i) = Uniform(min(i), max(i)).ToSymbolic();
  }
  return result;
}

template <int Size>
unique_ptr<DistributionVector> ToDistributionVector(
    const DistributionVectorVariant<Size>& vec) {
  return std::visit<unique_ptr<DistributionVector>>(
      overloaded{
          // NOLINTNEXTLINE(whitespace/line_length)
          [](const drake::Vector<double, Size>& arg) {
            return std::make_unique<DeterministicVector<Size>>(arg);
          },
          [](const DeterministicVector<Size>& arg) {
            return std::make_unique<DeterministicVector<Size>>(arg);
          },
          [](const GaussianVector<Size>& arg) {
            return std::make_unique<GaussianVector<Size>>(arg);
          },
          [](const UniformVector<Size>& arg) {
            return std::make_unique<UniformVector<Size>>(arg);
          },
          [](const Deterministic& arg) {
            return std::make_unique<DeterministicVector<Size>>(
                Eigen::VectorXd::Constant(1, arg.value));
          },
          [](const Gaussian& arg) {
            return std::make_unique<GaussianVector<Size>>(
                Eigen::VectorXd::Constant(1, arg.mean),
                Eigen::VectorXd::Constant(1, arg.stddev));
          },
          [](const Uniform& arg) {
            return std::make_unique<UniformVector<Size>>(
                Eigen::VectorXd::Constant(1, arg.min),
                Eigen::VectorXd::Constant(1, arg.max));
          },
          []<typename T>(
              const internal::InvalidVariantSelection<T>&) -> std::nullptr_t {
            DRAKE_UNREACHABLE();
          },
      },
      vec);
}

template <int Size>
bool IsDeterministic(const DistributionVectorVariant<Size>& vec) {
  return std::visit<bool>(
      overloaded{
          [](const drake::Vector<double, Size>&) {
            return true;
          },
          [](const DeterministicVector<Size>&) {
            return true;
          },
          [](const GaussianVector<Size>& arg) {
            return arg.stddev.isZero(0.0);
          },
          [](const UniformVector<Size>& arg) {
            return arg.min == arg.max;
          },
          [](const Deterministic&) {
            return true;
          },
          [](const Gaussian& arg) {
            return arg.stddev == 0.0;
          },
          [](const Uniform& arg) {
            return arg.min == arg.max;
          },
          []<typename T>(
              const internal::InvalidVariantSelection<T>&) -> std::false_type {
            DRAKE_UNREACHABLE();
          },
      },
      vec);
}

template <int Size>
Eigen::VectorXd GetDeterministicValue(
    const DistributionVectorVariant<Size>& vec) {
  if (!IsDeterministic(vec)) {
    std::visit(
        [](auto&& arg) {
          using ContainedType = std::decay_t<decltype(arg)>;
          throw std::logic_error(fmt::format(
              "Attempt to GetDeterministicValue() on a variant that contains "
              "a {}",
              drake::NiceTypeName::Get<ContainedType>()));
        },
        vec);
  }
  return ToDistributionVector(vec)->Mean();
}

#define DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_ALL_SIZES(Clazz) \
  template class Clazz<Eigen::Dynamic>;                                \
  template class Clazz<1>;                                             \
  template class Clazz<2>;                                             \
  template class Clazz<3>;                                             \
  template class Clazz<4>;                                             \
  template class Clazz<5>;                                             \
  template class Clazz<6>;

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_ALL_SIZES(DeterministicVector)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_ALL_SIZES(GaussianVector)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_ALL_SIZES(UniformVector)

#undef DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_ALL_SIZES

#define DRAKE_DEFINE_TEMPLATE_INSTANTIATIONS_ON_ALL_SIZES(Func) \
  template Func(const DistributionVectorVariantX&);             \
  template Func(const DistributionVectorVariant<1>&);           \
  template Func(const DistributionVectorVariant<2>&);           \
  template Func(const DistributionVectorVariant<3>&);           \
  template Func(const DistributionVectorVariant<4>&);           \
  template Func(const DistributionVectorVariant<5>&);           \
  template Func(const DistributionVectorVariant<6>&);

DRAKE_DEFINE_TEMPLATE_INSTANTIATIONS_ON_ALL_SIZES(
    unique_ptr<DistributionVector> ToDistributionVector)
DRAKE_DEFINE_TEMPLATE_INSTANTIATIONS_ON_ALL_SIZES(bool IsDeterministic)
DRAKE_DEFINE_TEMPLATE_INSTANTIATIONS_ON_ALL_SIZES(
    Eigen::VectorXd GetDeterministicValue)

#undef DRAKE_DEFINE_TEMPLATE_INSTANTIATIONS_ON_ALL_SIZES

}  // namespace schema
}  // namespace drake
