#include <memory>

// See https://eigen.tuxfamily.org/dox-devel/TopicPreprocessorDirectives.html.
#define EIGEN_RUNTIME_NO_MALLOC
#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/common/parameterized_type_map.h"

namespace drake {

namespace {

using drake::CompareMatrices;
using drake::MatrixCompareType;

GTEST_TEST(ParameterizedTypeMapTest, TestBasics) {
  ParameterizedTypeMap<Vector3> map;

  auto vec_int = Vector3<int>(1, 2, 3);
  map.emplace<int>(vec_int);

  auto vec_double = Vector3<double>(2., 3., 4.);
  map.emplace<double>(vec_double);

  ASSERT_EQ(*map.get<int>(), vec_int);
  ASSERT_EQ(*map.get<double>(), vec_double);

  ASSERT_TRUE(map.has_key<int>());
  ASSERT_TRUE(map.has_key<double>());
  ASSERT_FALSE(map.has_key<float>());
}

namespace parameterized_type_map_test {

template <typename T>
class DestructorTestType {
 public:
  DestructorTestType(T value, bool& destructed_flag)
      : value_(value), destructed_flag_(destructed_flag) {
    destructed_flag_ = false;
  }

  ~DestructorTestType() { destructed_flag_ = true; }

  const T& get_value() const { return value_; }

 private:
  T value_;
  bool& destructed_flag_;
};

}  // parameterized_type_map_test

/*
 * Ensure that values get destructed properly.
 */
GTEST_TEST(ParameterizedTypeMapTest, TestLifeTime) {
  using parameterized_type_map_test::DestructorTestType;

  bool bool_destructed;
  bool double_destructed;

  {
    ParameterizedTypeMap<DestructorTestType> map;

    bool bool_value = true;
    map.emplace<bool>(bool_value, bool_destructed);

    double double_value = 5.9;
    map.emplace<double>(double_value, double_destructed);

    ASSERT_FALSE(bool_destructed);
    ASSERT_EQ(bool_value, map.get<bool>()->get_value());

    ASSERT_FALSE(double_destructed);
    ASSERT_EQ(double_value, map.get<double>()->get_value());
  }
  ASSERT_TRUE(bool_destructed);
  ASSERT_TRUE(double_destructed);
}

template <typename T>
using remove_const_and_reference =
    typename std::remove_const<typename std::remove_reference<T>::type>::type;

GTEST_TEST(ParameterizedTypeMapTest, TestIntendedUseCase) {
  int n = 10;

  VectorX<float> vec = VectorX<float>::LinSpaced(n, 0, n - 1);
  ParameterizedTypeMap<VectorX> map;

  auto fun = [&](const auto& x) {
    using Scalar = remove_const_and_reference<decltype(x)>;
    if (!map.has_key<Scalar>()) {
      map.emplace<Scalar>(vec.cast<Scalar>());
    }
    const auto& vec_of_correct_type = *map.get<Scalar>();
    return vec_of_correct_type * x;
  };

  float x_float_1(2);
  ASSERT_TRUE(CompareMatrices(fun(x_float_1), vec * x_float_1, 0.,
                              MatrixCompareType::absolute));

  double x_double_1 = 3.;
  ASSERT_TRUE(CompareMatrices(fun(x_double_1), vec.cast<double>() * x_double_1,
                              1e-12, MatrixCompareType::absolute));

  float x_float_2(5);
  VectorX<float> expected_float_2 = vec * x_float_2;

  double x_double_2 = 15.;
  VectorX<double> expected_double_2 = vec.cast<double>() * x_double_2;

  Eigen::internal::set_is_malloc_allowed(false);
  ASSERT_TRUE(CompareMatrices(fun(x_float_2), expected_float_2, 0.,
                              MatrixCompareType::absolute));
  ASSERT_TRUE(CompareMatrices(fun(x_double_2), expected_double_2, 0.,
                              MatrixCompareType::absolute));
  Eigen::internal::set_is_malloc_allowed(true);
}

}  // namespace
}  // namespace drake
