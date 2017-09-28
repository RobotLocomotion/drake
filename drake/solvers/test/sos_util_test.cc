#include "drake/solvers/sos_util.h"

#include <array>
#include <iostream>
#include <map>
#include <set>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace solvers {
namespace build_gram_basis {

using std::array;
using std::set;
using std::vector;
using std::make_pair;
using drake::symbolic::Expression;
using drake::symbolic::Monomial;
using drake::symbolic::Polynomial;
using drake::symbolic::Variable;

typedef std::map<Variable, int> map;

class SosUtilTest : public ::testing::Test {
 public:
  SosUtilTest()
      : exprs_({2, x_, 5 * x_, y_, x_ * y_, 2 * x_ * x_, 6 * x_ * y_,
                x_ * x_ + x_ * y_, 3 * x_ * x_ * y_ + 4 * pow(y_, 3) * z_ + 2,
                pow(x_, 4) * pow(y_, 2) + pow(y_, 4) * pow(x_, 2) + 1}),
        polys_(exprs_.begin(), exprs_.end()),
        supports_{support_0, support_1, support_2, support_3, support_4,
                  support_5, support_6, support_7, support_8, support_9},
        polytopes_{polytope_0, polytope_1, polytope_2, polytope_3, polytope_4,
                   polytope_5, polytope_6, polytope_7, polytope_8, polytope_9},
        sos_supports_{sos_support_0, sos_support_1, sos_support_2,
                      sos_support_3, sos_support_4, sos_support_5,
                      sos_support_6, sos_support_7, sos_support_8,
                      sos_support_9} {}

 protected:
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};
  const Expression x_{var_x_};
  const Expression y_{var_y_};
  const Expression z_{var_z_};
  const vector<Expression> exprs_;
  const vector<Polynomial> polys_;

  /*support_0 is empty*/
  const Eigen::Matrix<int, 1, 0> support_0;
  const int support_data_1[1]{1};
  const Eigen::Matrix<int, 1, 1> support_1{&support_data_1[0]};
  const int support_data_2[1]{1};
  const Eigen::Matrix<int, 1, 1> support_2{&support_data_2[0]};
  const int support_data_3[1]{1};
  const Eigen::Matrix<int, 1, 1> support_3{&support_data_3[0]};
  const int support_data_4[2]{1, 1};
  const Eigen::Matrix<int, 1, 2> support_4{&support_data_4[0]};
  const int support_data_5[1]{2};
  const Eigen::Matrix<int, 1, 1> support_5{&support_data_5[0]};
  const int support_data_6[2]{1, 1};
  const Eigen::Matrix<int, 1, 2> support_6{&support_data_6[0]};
  const int support_data_7[4]{1, 2, 1, 0};
  const Eigen::Matrix<int, 2, 2> support_7{&support_data_7[0]};
  const int support_data_8[9]{0, 2, 0, 3, 1, 0, 1, 0, 0};
  const Eigen::Matrix<int, 3, 3> support_8{&support_data_8[0]};
  const int support_data_9[6]{2, 4, 0, 4, 2, 0};
  const Eigen::Matrix<int, 3, 2> support_9{&support_data_9[0]};
  const vector<Eigen::MatrixXi> supports_;

  const Eigen::Matrix<int, 0, 0> polytope_0;
  const Eigen::Matrix<int, 0, 0> polytope_1;
  const Eigen::Matrix<int, 0, 0> polytope_2;
  const Eigen::Matrix<int, 0, 0> polytope_3;
  const int polytope_data_4[6]{0, 1, 0, 1, 0, 0};
  const Eigen::Matrix<int, 3, 2> polytope_4{&polytope_data_4[0]};
  const int polytope_data_5[2]{1, 0};
  const Eigen::Matrix<int, 2, 1> polytope_5{&polytope_data_5[0]};
  const int polytope_data_6[6]{0, 1, 0, 1, 0, 0};
  const Eigen::Matrix<int, 3, 2> polytope_6{&polytope_data_6[0]};
  const int polytope_data_7[6]{0, 1, 0, 1, 0, 0};
  const Eigen::Matrix<int, 3, 2> polytope_7{&polytope_data_7[0]};
  const int polytope_data_8[30]{0, 0, 0, 1, 1, 2, 0, 0, 1, 0, 0, 1, 2, 0, 1,
                                0, 0, 1, 0, 0, 2, 1, 0, 1, 0, 0, 1, 0, 0, 0};
  const Eigen::Matrix<int, 10, 3> polytope_8{&polytope_data_8[0]};
  const int polytope_data_9[20]{0, 1, 2, 3, 0, 1, 2, 0, 1, 0,
                                3, 2, 1, 0, 2, 1, 0, 1, 0, 0};
  const Eigen::Matrix<int, 10, 2> polytope_9{&polytope_data_9[0]};
  const vector<Eigen::MatrixXi> polytopes_;

  const Eigen::Matrix<int, 0, 0> sos_support_0;
  const Eigen::Matrix<int, 0, 0> sos_support_1;
  const Eigen::Matrix<int, 0, 0> sos_support_2;
  const Eigen::Matrix<int, 0, 0> sos_support_3;
  const Eigen::Matrix<int, 0, 2> sos_support_4;
  const int sos_support_data_5[1]{1};
  const Eigen::Matrix<int, 1, 1> sos_support_5{&sos_support_data_5[0]};
  const Eigen::Matrix<int, 0, 2> sos_support_6;
  const int sos_support_data_7[2]{1, 0};
  const Eigen::Matrix<int, 1, 2> sos_support_7{&sos_support_data_7[0]};
  const int sos_support_data_8[3]{0, 0, 0};
  const Eigen::Matrix<int, 1, 3> sos_support_8{&sos_support_data_8[0]};
  const int sos_support_data_9[8]{1, 2, 1, 0, 2, 1, 1, 0};
  const Eigen::Matrix<int, 4, 2> sos_support_9{&sos_support_data_9[0]};
  const vector<Eigen::MatrixXi> sos_supports_;
};

TEST_F(SosUtilTest, GenerateCrossTerms) {
  const Eigen::Matrix<int, 0, 0> base_matrix_00;
  const Eigen::Matrix<int, 0, 0> base_matrix_10;
  const Eigen::Matrix<int, 0, 0> base_matrix_01;
  const int base_matrix_data_11[1]{1};
  const Eigen::Matrix<int, 1, 1> base_matrix_11{&base_matrix_data_11[0]};
  const int base_matrix_data_21[2]{1, 2};
  const Eigen::Matrix<int, 2, 1> base_matrix_21{&base_matrix_data_21[0]};
  const int base_matrix_data_12[2]{1, 2};
  const Eigen::Matrix<int, 1, 2> base_matrix_12{&base_matrix_data_12[0]};
  const int base_matrix_data_22[4]{1, 0, 0, 1};
  const Eigen::Matrix<int, 2, 2> base_matrix_22{&base_matrix_data_22[0]};
  const int base_matrix_data_32[6]{1, 0, 0, 0, 1, 0};
  const Eigen::Matrix<int, 3, 2> base_matrix_32{&base_matrix_data_32[0]};
  const int base_matrix_data_23[6]{1, 0, 0, 1, 0, 0};
  const Eigen::Matrix<int, 2, 3> base_matrix_23{&base_matrix_data_23[0]};
  const int base_matrix_data_33[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};
  const Eigen::Matrix<int, 3, 3> base_matrix_33{&base_matrix_data_33[0]};
  const vector<Eigen::MatrixXi> base_matrices_{
      base_matrix_00, base_matrix_10, base_matrix_01, base_matrix_11,
      base_matrix_21, base_matrix_12, base_matrix_22, base_matrix_32,
      base_matrix_23, base_matrix_33};

  const Eigen::Matrix<int, 0, 0> cross_matrix_00;
  const Eigen::Matrix<int, 0, 0> cross_matrix_10;
  const Eigen::Matrix<int, 0, 0> cross_matrix_01;
  const Eigen::Matrix<int, 0, 0> cross_matrix_11;
  const int cross_matrix_data_21[1]{3};
  const Eigen::Matrix<int, 1, 1> cross_matrix_21{&cross_matrix_data_21[0]};
  const Eigen::Matrix<int, 0, 0> cross_matrix_12;
  const int cross_matrix_data_22[2]{1, 1};
  const Eigen::Matrix<int, 1, 2> cross_matrix_22{&cross_matrix_data_22[0]};
  const int cross_matrix_data_32[6]{1, 1, 0, 1, 0, 1};
  const Eigen::Matrix<int, 3, 2> cross_matrix_32{&cross_matrix_data_32[0]};
  const int cross_matrix_data_23[3]{1, 1, 0};
  const Eigen::Matrix<int, 1, 3> cross_matrix_23{&cross_matrix_data_23[0]};
  const int cross_matrix_data_33[9]{1, 1, 0, 1, 0, 1, 0, 1, 1};
  const Eigen::Matrix<int, 3, 3> cross_matrix_33{&cross_matrix_data_33[0]};
  const vector<Eigen::MatrixXi> cross_matrices_{
      cross_matrix_00, cross_matrix_10, cross_matrix_01, cross_matrix_11,
      cross_matrix_21, cross_matrix_12, cross_matrix_22, cross_matrix_32,
      cross_matrix_23, cross_matrix_33};

  for (int i = 0; i < static_cast<int>(base_matrices_.size()); ++i) {
    EXPECT_EQ(
        RowIntersect(GenerateCrossTerms(base_matrices_[i]), cross_matrices_[i]),
        cross_matrices_[i]);
    //    EXPECT_EQ(RowIntersect(GenerateCrossTerms(base_matrices_[i]),
    //    cross_matrices_[i]), cross_matrices_[i]);
  }
}

TEST_F(SosUtilTest, DiagConsistentTest) {
  // only for last test case
  // TODO(FischerGundlach) Add more test cases where the crossterms are actually
  // tested within CheckDiagonalConsistency
  EXPECT_EQ(CheckDiagonalConsistency(support_9, sos_support_9), support_9 / 2);
}

TEST_F(SosUtilTest, ExponentBoundPolytopeTest) {
  int i{0};
  for (auto& p : polys_) {
    // catches constant and odd polynomial => don't compute
    // GenerateExponentBoundPolytope
    if (!((p.TotalDegree() % 2 == 1) || p.indeterminates().empty())) {
      auto pair_ = PolynomialToSupport(p);
      EXPECT_EQ(GenerateExponentBoundPolytope(pair_.first), polytopes_[i]);
    }
    ++i;
  }
}

TEST_F(SosUtilTest, PolynomialToSupportTest) {
  map map_0;
  map map_1;
  map_1.insert(make_pair(var_x_, 0));
  map map_2;
  map_2.insert(make_pair(var_x_, 0));
  map map_3;
  map_3.insert(make_pair(var_y_, 0));
  map map_4;
  map_4.insert(make_pair(var_x_, 0));
  map_4.insert(make_pair(var_y_, 1));
  map map_5;
  map_5.insert(make_pair(var_x_, 0));
  map map_6;
  map_6.insert(make_pair(var_x_, 0));
  map_6.insert(make_pair(var_y_, 1));
  map map_7;
  map_7.insert(make_pair(var_x_, 0));
  map_7.insert(make_pair(var_y_, 1));
  map map_8;
  map_8.insert(make_pair(var_x_, 0));
  map_8.insert(make_pair(var_y_, 1));
  map_8.insert(make_pair(var_z_, 2));
  map map_9;
  map_9.insert(make_pair(var_x_, 0));
  map_9.insert(make_pair(var_y_, 1));

  const vector<map> maps_{map_0, map_1, map_2, map_3, map_4,
                          map_5, map_6, map_7, map_8, map_9};

  int i{0};
  for (auto& p : polys_) {
    auto pair_ = PolynomialToSupport(p);
    EXPECT_EQ(pair_.first, supports_[i]);
    EXPECT_EQ(pair_.second, maps_[i]);
    ++i;
  }
}

TEST_F(SosUtilTest, RandomPruneTest) {
  // The rows in mat_X correspond to the support by mpow, where 2mpow is
  // inside/on the boundary of the support pow. Therefore these rows need to be
  // maintained after pruning!

  int i{0};
  for (auto& p : polys_) {
    // catches constant and odd polynomial => don't compute
    // GenerateExponentBoundPolytope
    if (!((p.TotalDegree() % 2 == 1) || p.indeterminates().empty())) {
      Eigen::MatrixXi pow{PolynomialToSupport(p).first};
      Eigen::MatrixXi mpow{GenerateExponentBoundPolytope(pow)};
      // performe the test 100 times, as the pruning is a random operation.
      for (int testruns = 0; testruns < 1; ++testruns) {
        mpow = RandomlyPruneSupport(pow, mpow, 1);
        bool contains_interior{false};

        if (RowIntersect(mpow, sos_supports_[i]) == sos_supports_[i]) {
          contains_interior = true;
        }

        EXPECT_TRUE(contains_interior);
      }
    }
    ++i;
  }
}

TEST_F(SosUtilTest, RemoveRowsTest) {
  set<int> set_remove_0;
  set<int> set_remove_1;
  set<int> set_remove_2;
  set<int> set_remove_3;
  array<int, 3> data_remove_4{{0, 1, 2}};
  set<int> set_remove_4{data_remove_4.begin(), data_remove_4.end()};
  array<int, 1> data_remove_5{{1}};
  set<int> set_remove_5{data_remove_5.begin(), data_remove_5.end()};
  array<int, 3> data_remove_6{{0, 1, 2}};
  set<int> set_remove_6{data_remove_6.begin(), data_remove_6.end()};
  array<int, 2> data_remove_7{{0, 2}};
  set<int> set_remove_7{data_remove_7.begin(), data_remove_7.end()};
  array<int, 9> data_remove_8{{0, 1, 2, 3, 4, 5, 6, 7, 8}};
  set<int> set_remove_8{data_remove_8.begin(), data_remove_8.end()};
  array<int, 6> data_remove_9{{0, 3, 4, 6, 7, 8}};
  set<int> set_remove_9{data_remove_9.begin(), data_remove_9.end()};
  const vector<set<int>> vec_remove_{
      set_remove_0, set_remove_1, set_remove_2, set_remove_3, set_remove_4,
      set_remove_5, set_remove_6, set_remove_7, set_remove_8, set_remove_9};

  for (int i = 0; i < static_cast<int>(polytopes_.size()); ++i) {
    EXPECT_EQ(RemoveRows(polytopes_[i], vec_remove_[i]), sos_supports_[i]);
  }
}

TEST_F(SosUtilTest, RowIntersectTest) {
  for (int i = 0; i < static_cast<int>(polytopes_.size()); ++i) {
    EXPECT_EQ(RowIntersect(sos_supports_[i], polytopes_[i]), sos_supports_[i]);
    EXPECT_EQ(RowIntersect(polytopes_[i], sos_supports_[i]), sos_supports_[i]);
  }
}

TEST_F(SosUtilTest, SupportToGramBasisTest) {
  const Monomial m_0{pow(x_, 2) * pow(y_, 4)};
  const Monomial m_1{pow(x_, 4) * pow(y_, 2)};
  const Monomial m_2{1};
  const vector<Monomial> ms_{m_0, m_1, m_2};

  auto pair_0 = PolynomialToSupport(polys_[0]);
  auto gramBasis_0 = SupportToGramBasis(pair_0.first, pair_0.second);
  EXPECT_EQ(gramBasis_0[0], Monomial());

  // TODO(FischerGundlach) Add more SOS problems
  auto pair_9 = PolynomialToSupport(polys_[9]);
  auto gramBasis_9 = SupportToGramBasis(pair_9.first, pair_9.second);

  for (int i = 0; i < static_cast<int>(ms_.size()); ++i) {
    EXPECT_EQ(gramBasis_9[i], ms_[i]);
  }
}

TEST_F(SosUtilTest, UniqueRowsTest) {
  for (int i = 1; i < static_cast<int>(sos_supports_.size()); ++i) {
    Eigen::MatrixXi double_support(2 * sos_supports_[i].rows(),
                                   sos_supports_[i].cols());
    double_support << sos_supports_[i], sos_supports_[i];
    EXPECT_EQ(UniqueRows(double_support), sos_supports_[i]);
  }
}

}  // namespace build_gram_basis
}  // namespace solvers
}  // namespace drake
