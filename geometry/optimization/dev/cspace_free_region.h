#pragma once

#include <map>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/geometry/optimization/dev/collision_geometry.h"
#include "drake/multibody/rational/rational_forward_kinematics.h"

namespace drake {
namespace geometry {
namespace optimization {
/** The separating plane aᵀx + b ≥ δ, aᵀx+b ≤ −δ has parameters a and b. These
 parameters a polynomial function of s with the specified degree.
 */
enum class SeparatingPlaneOrder {
  kAffine,  ///< a and b are affine function of s.
};

/**
 One collision geometry is on the "positive" side of the separating plane,
 namely {x| aᵀx + b ≥ δ} (with δ ≥ 0}, and the other collision geometry is on
 the "negative" side of the separating plane, namely {x|aᵀx+b ≤ −δ}.
 @tparam T The type of decision_variables. T= symbolic::Variable or double.
 */
template <typename T>
struct SeparatingPlane {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SeparatingPlane)

  SeparatingPlane() = default;

  SeparatingPlane(Vector3<symbolic::Polynomial> m_a, symbolic::Polynomial m_b,
                  const CollisionGeometry* m_positive_side_geometry,
                  const CollisionGeometry* m_negative_side_geometry,
                  multibody::BodyIndex m_expressed_body,
                  SeparatingPlaneOrder m_plane_order,
                  const Eigen::Ref<const VectorX<T>>& m_decision_variables)
      : a{std::move(m_a)},
        b{std::move(m_b)},
        positive_side_geometry{m_positive_side_geometry},
        negative_side_geometry{m_negative_side_geometry},
        expressed_body{m_expressed_body},
        plane_order{m_plane_order},
        decision_variables{m_decision_variables} {}

  Vector3<symbolic::Polynomial> a;
  symbolic::Polynomial b;
  const CollisionGeometry* positive_side_geometry;
  const CollisionGeometry* negative_side_geometry;
  multibody::BodyIndex expressed_body;
  SeparatingPlaneOrder plane_order;
  VectorX<T> decision_variables;
};

/**
 Computes the parameter a, b in the plane { x | aᵀx+b=0 }.
 a and b are both polynomials of `s_for_plane`. The coefficient of these
 polynomials are in `decision_variables`.
 The possible combination of T, U, V are
 1. T=symbolic::Variable, U=symbolic::Variable, V=symbolic::Polynomial.
 2. T=double, U=symbolic::Variable, V=symbolic::Polynomial
 3. T=double, U=double, V=double
 */
template <typename T, typename U, typename V>
void CalcPlane(const VectorX<T>& decision_variables,
               const VectorX<U>& s_for_plane, SeparatingPlaneOrder order,
               Vector3<V>* a_val, V* b_val) {
  static_assert(
      (std::is_same_v<T, symbolic::Variable> &&
       std::is_same_v<U, symbolic::Variable> &&
       std::is_same_v<V, symbolic::Polynomial>) ||
          (std::is_same_v<T, double> && std::is_same_v<U, symbolic::Variable> &&
           std::is_same_v<V, symbolic::Polynomial>) ||
          (std::is_same_v<T, double> && std::is_same_v<U, double> &&
           std::is_same_v<V, double>),
      "CalcPlane: unsupported scalar type");

  switch (order) {
    case SeparatingPlaneOrder::kAffine: {
      DRAKE_DEMAND(decision_variables.rows() == 4 * s_for_plane.rows() + 4);
      Eigen::Matrix<T, 3, Eigen::Dynamic> a_coeff(3, s_for_plane.rows());
      int var_count = 0;
      for (int i = 0; i < 3; ++i) {
        a_coeff.row(i) =
            decision_variables.segment(var_count, s_for_plane.rows());
        var_count += s_for_plane.rows();
      }
      const Vector3<T> a_constant =
          decision_variables.template segment<3>(var_count);
      const VectorX<T> b_coeff =
          decision_variables.segment(var_count, s_for_plane.rows());
      var_count += s_for_plane.rows();
      const T& b_constant = decision_variables(var_count);
      var_count++;
      if constexpr (std::is_same_v<T, double> && std::is_same_v<U, double> &&
                    std::is_same_v<V, double>) {
        *a_val = a_coeff * s_for_plane + a_constant;
        *b_val = b_coeff.dot(s_for_plane) + b_constant;
        return;
      }
      if constexpr (std::is_same_v<U, symbolic::Variable> &&
                    std::is_same_v<V, symbolic::Polynomial>) {
        const symbolic::Monomial monomial_one{};
        for (int i = 0; i < 3; ++i) {
          symbolic::Polynomial::MapType monomial_to_coeff_map;
          for (int j = 0; j < s_for_plane.rows(); ++j) {
            monomial_to_coeff_map.emplace(symbolic::Monomial(s_for_plane(j)),
                                          a_coeff(i, j));
          }
          monomial_to_coeff_map.emplace(monomial_one, a_constant(i));
          (*a_val)(i) = symbolic::Polynomial(monomial_to_coeff_map);
        }
        symbolic::Polynomial::MapType monomial_to_coeff_map;
        for (int j = 0; j < s_for_plane.rows(); ++j) {
          monomial_to_coeff_map.emplace(symbolic::Monomial(s_for_plane(j)),
                                        b_coeff(j));
        }
        monomial_to_coeff_map.emplace(monomial_one, b_constant);
        *b_val = symbolic::Polynomial(monomial_to_coeff_map);
        return;
      }
    }
  }
}

/**
 This class tries to find large convex region in the configuration space, such
 that this whole convex set is collision free.
 For more details, refer to the paper
 "Finding and Optimizing Certified, Colision-Free Regions in Configuration Space
 for Robot Manipulators" by Alexandre Amice, Hongkai Dai, Peter Werner, Annan
 Zhang and Russ Tedrake.
 */
class CspaceFreeRegion {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CspaceFreeRegion)

  ~CspaceFreeRegion() {}

  /**
   @param plant The plant for which we compute the C-space free regions. It must
   outlive this CspaceFreeRegion object.
   @param scene_graph The scene graph has been connected with `plant`. It must
   outlive this CspaceFreeRegion object.
   @param plane_order The order of the polynomials in the plane to separate a
   pair of collision geometries.
   */
  CspaceFreeRegion(const multibody::MultibodyPlant<double>* plant,
                   const geometry::SceneGraph<double>* scene_graph,
                   SeparatingPlaneOrder plane_order);

  [[nodiscard]] const multibody::RationalForwardKinematics&
  rational_forward_kin() const {
    return rational_forward_kin_;
  }

  /** separating_planes()[map_geometries_to_separating_planes.at(geometry1_id,
   geometry2_id)] is the separating plane that separates geometry1 and
   geometry 2.
   */
  [[nodiscard]] const std::unordered_map<SortedPair<geometry::GeometryId>, int>&
  map_geometries_to_separating_planes() const {
    return map_geometries_to_separating_planes_;
  }

  [[nodiscard]] const std::vector<SeparatingPlane<symbolic::Variable>>&
  separating_planes() const {
    return separating_planes_;
  }

 private:
  multibody::RationalForwardKinematics rational_forward_kin_;
  const geometry::SceneGraph<double>& scene_graph_;
  std::map<multibody::BodyIndex,
           std::vector<std::unique_ptr<CollisionGeometry>>>
      link_geometries_;

  SeparatingPlaneOrder plane_order_;
  std::vector<SeparatingPlane<symbolic::Variable>> separating_planes_;
  std::unordered_map<SortedPair<geometry::GeometryId>, int>
      map_geometries_to_separating_planes_;
};

/**
 * Given a diagram (which contains the plant and the scene_graph), returns all
 * the collision geometries.
 */
[[nodiscard]] std::map<multibody::BodyIndex,
                       std::vector<std::unique_ptr<CollisionGeometry>>>
GetCollisionGeometries(const multibody::MultibodyPlant<double>& plant,
                       const geometry::SceneGraph<double>& scene_graph);

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
