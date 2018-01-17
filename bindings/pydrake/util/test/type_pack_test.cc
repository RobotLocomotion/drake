#include "drake/bindings/pydrake/util/type_pack.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/nice_type_name.h"

using std::string;
using std::vector;

namespace drake {
namespace {

template <typename... Ts>
struct SimpleTemplate {};

using Pack = type_pack<int, double, char, void>;

// Mostly, this just checks for compilation failures.
GTEST_TEST(TypeUtilTest, TypeAt) {
  using T_0 = type_at<0, int, double, char, void>::type;
  EXPECT_TRUE((std::is_same<T_0, int>::value));
  using T_1 = type_at<1, int, double, char, void>::type;
  EXPECT_TRUE((std::is_same<T_1, double>::value));
  using T_2 = type_at<2, int, double, char, void>::type;
  EXPECT_TRUE((std::is_same<T_2, char>::value));
  using T_3 = type_at<3, int, double, char, void>::type;
  EXPECT_TRUE((std::is_same<T_3, void>::value));

  EXPECT_TRUE((std::is_same<Pack::type_at<0>, int>::value));
  EXPECT_TRUE((std::is_same<Pack::type_at<1>, double>::value));
  EXPECT_TRUE((std::is_same<Pack::type_at<2>, char>::value));
  EXPECT_TRUE((std::is_same<Pack::type_at<3>, void>::value));
}

GTEST_TEST(TypeUtilTest, TypeTags) {
  // Ensure that we can default-construct tags for types that are not
  // default-constructible.
  auto tag_check = type_tag<void>{};
  EXPECT_TRUE((std::is_same<
      decltype(tag_check), type_tag<void>>::value));
  auto pack_check_empty = type_pack<>{};
  EXPECT_TRUE((std::is_same<
      decltype(pack_check_empty), type_pack<>>::value));
  auto pack_check = type_pack<void, void>{};
  EXPECT_TRUE((std::is_same<
      decltype(pack_check), type_pack<void, void>>::value));
}

GTEST_TEST(TypeUtilTest, Bind) {
  using T_0 = Pack::bind<SimpleTemplate>;
  EXPECT_TRUE((std::is_same<
      T_0, SimpleTemplate<int, double, char, void>>::value));
  Pack pack;
  using T_1 = decltype(type_bind<SimpleTemplate>(pack));
  EXPECT_TRUE((std::is_same<
      T_1, SimpleTemplate<int, double, char, void>>::value));
}

GTEST_TEST(TypeUtilTest, Extract) {
  using T = SimpleTemplate<int, double, char, void>;
  using TPack = type_pack_extract<T>;
  EXPECT_TRUE((std::is_same<TPack, Pack>::value));
}

GTEST_TEST(TypeUtilTest, Visit) {
  using PackTags = type_pack<
      type_tag<int>, type_tag<double>, type_tag<char>, type_tag<void>>;
  vector<string> names;
  const vector<string> names_expected = {"int", "double", "char", "void"};

  auto visitor = [&names](auto tag) {
    using T = typename decltype(tag)::type;
    names.push_back(NiceTypeName::Get<T>());
  };
  names.clear();
  type_visit(visitor, PackTags{});
  EXPECT_EQ(names, names_expected);

  names.clear();
  type_visit<type_visit_with_tag<>>(visitor, Pack{});
  EXPECT_EQ(names, names_expected);

  // N.B. `Check` will operate types contained within the pack as dictated by
  // the `VisitWith` parameter in `type_visit`.
  // As an example, see below that the two visit calls will be visiting the
  // same types (with the same results), but `Check` will operate on
  // (a) the direct types with `type_visit_with_tag<>` and (b) the tag types
  // with `type_visit_with_default` (the default parameter).
  const vector<string> names_expected_sub = {"int", "char", "void"};

  names.clear();
  type_visit<type_visit_with_tag<>, type_check_different_from<double>::type>(
      visitor, Pack{});
  EXPECT_EQ(names, names_expected_sub);

  names.clear();
  using CheckTag =
      template_single_tag<type_check_different_from<type_tag<double>>::type>;
  type_visit(visitor, PackTags{}, CheckTag{});
  EXPECT_EQ(names, names_expected_sub);
};

GTEST_TEST(TypeUtilTest, Hash) {
  using T = int;
  using U = int;
  using V = double;
  EXPECT_EQ(type_hash<T>(), type_hash<U>());
  EXPECT_NE(type_hash<T>(), type_hash<V>());
}

}  // namespace
}  // namespace drake
