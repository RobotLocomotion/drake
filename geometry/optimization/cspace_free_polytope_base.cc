#include "drake/geometry/optimization/cspace_free_polytope_base.h"

#include <limits>
#include <utility>

#include "drake/geometry/optimization/cspace_free_internal.h"
#include "drake/multibody/rational/rational_forward_kinematics_internal.h"

namespace drake {
namespace geometry {
namespace optimization {
const double kInf = std::numeric_limits<double>::infinity();
namespace {
/*
 The monomials in the numerator of body point position has the form ∏ᵢ pow(tᵢ,
 dᵢ) * ∏ⱼ pow(xⱼ, cⱼ), where tᵢ is the configuration variable tᵢ = tan(Δθᵢ/2)
 for the revolute joint, dᵢ = 0, 1, or 2, on the kinematic chain between the
 pair of bodies; xⱼ is the prismatic joint displacement on the same kinematic
 chain, cⱼ = 0 or 1. When we use SeparatingPlaneOrder = kAffine, the monomial
 basis would include all the monomials in the form ∏ᵢ pow(sᵢ, nᵢ) with nᵢ = 0
 or 1, s ∈ {tᵢ, xⱼ}. Let's denote this monomial basis as m(s).

 When we have non-polytopic collision geometries, we will also impose a matrix
 of rational functions being positive semidefinite. This requires a matrix-sos
 constraint with slack variable y. We will need the monomial basis in the form
 yᵢ*m(s) with i=0,1,2.

 @param[in/out] map_body_to_monomial_basis_array stores all the monomials basis
 already computed.
 @param[out] monomial_basis_array The monomial basis array for this pair of
 body, monomial_basis_array = [m(s), y₀*m(s), y₁*m(s), y₂*m(s)]
 */
void FindMonomialBasisArray(
    const multibody::RationalForwardKinematics& rational_forward_kin,
    const Vector3<symbolic::Variable>& y_slack,
    const SortedPair<multibody::BodyIndex>& body_pair,
    const std::unordered_map<SortedPair<multibody::BodyIndex>,
                             std::vector<int>>& map_body_pair_to_s_on_chain,
    std::unordered_map<SortedPair<multibody::BodyIndex>,
                       std::array<VectorX<symbolic::Monomial>, 4>>*
        map_body_to_monomial_basis_array,
    std::array<VectorX<symbolic::Monomial>, 4>* monomial_basis_array) {
  auto body_pair_it = map_body_to_monomial_basis_array->find(body_pair);
  if (body_pair_it == map_body_to_monomial_basis_array->end()) {
    symbolic::Variables s_set;
    const std::vector<int>& s_indices =
        map_body_pair_to_s_on_chain.at(body_pair);
    for (const int s_index : s_indices) {
      s_set.insert(rational_forward_kin.s()[s_index]);
    }
    if (s_set.empty()) {
      // No s variable. The monomial basis is just [1].
      (*monomial_basis_array)[0].resize(1);
      (*monomial_basis_array)[0](0) = symbolic::Monomial();
    } else {
      (*monomial_basis_array)[0] =
          symbolic::CalcMonomialBasisOrderUpToOne(s_set);
    }
    // monomial_basis_array[i+1] = y(i) * monomial_basis_array[0]
    for (int i = 0; i < 3; ++i) {
      const symbolic::Monomial yi(y_slack(i));
      (*monomial_basis_array)[i + 1].resize((*monomial_basis_array)[0].rows());
      for (int j = 0; j < (*monomial_basis_array)[0].rows(); ++j) {
        (*monomial_basis_array)[i + 1](j) = yi * (*monomial_basis_array)[0](j);
      }
    }
    map_body_to_monomial_basis_array->emplace(body_pair, *monomial_basis_array);
  } else {
    *monomial_basis_array = body_pair_it->second;
  }
}

// TODO(hongkai.dai): move this function to a header file for general usage.
template <typename T>
void SymmetricMatrixFromLowerTriangularPart(
    int rows, const Eigen::Ref<const VectorX<T>>& lower_triangle,
    MatrixX<T>* mat) {
  mat->resize(rows, rows);
  DRAKE_THROW_UNLESS(lower_triangle.rows() == rows * (rows + 1) / 2);
  int count = 0;
  for (int j = 0; j < rows; ++j) {
    (*mat)(j, j) = lower_triangle(count++);
    for (int i = j + 1; i < rows; ++i) {
      (*mat)(i, j) = lower_triangle(count);
      (*mat)(j, i) = lower_triangle(count);
      count++;
    }
  }
}

// TODO(hongkai.dai): move this change to MathematicalProgram.
void AddPsdConstraint(solvers::MathematicalProgram* prog,
                      const MatrixX<symbolic::Variable>& X) {
  DRAKE_THROW_UNLESS(X.rows() == X.cols());
  if (X.rows() == 1) {
    prog->AddBoundingBoxConstraint(0, kInf, X(0, 0));
  } else if (X.rows() == 2) {
    prog->AddRotatedLorentzConeConstraint(
        Vector3<symbolic::Variable>(X(0, 0), X(1, 1), X(0, 1)));
  } else {
    prog->AddPositiveSemidefiniteConstraint(X);
  }
}
}  // namespace

CspaceFreePolytopeBase::CspaceFreePolytopeBase(
    const multibody::MultibodyPlant<double>* plant,
    const geometry::SceneGraph<double>* scene_graph,
    SeparatingPlaneOrder plane_order, SForPlane s_for_plane_enum,
    const Options& options)
    : rational_forward_kin_(plant),
      scene_graph_{scene_graph},
      link_geometries_{internal::GetCollisionGeometries(*plant, *scene_graph)},
      plane_order_{plane_order},
      s_set_{rational_forward_kin_.s()},
      with_cross_y_{options.with_cross_y} {
  DRAKE_DEMAND(scene_graph_ != nullptr);
  // Create separating planes.
  // collision_pairs maps each pair of body to the pair of collision geometries
  // on that pair of body.
  std::map<SortedPair<multibody::BodyIndex>,
           std::vector<std::pair<const CIrisCollisionGeometry*,
                                 const CIrisCollisionGeometry*>>>
      collision_pairs;
  int num_collision_pairs = internal::GenerateCollisionPairs(
      rational_forward_kin_.plant(), *scene_graph_, link_geometries_,
      &collision_pairs);

  // Initialize map_body_pair_to_s_on_chain.
  for (const auto& [body_pair, collisions] : collision_pairs) {
    SetIndicesOfSOnChainForBodyPair(body_pair);
  }

  separating_planes_.reserve(num_collision_pairs);
  const symbolic::Monomial monomial_one{};
  for (const auto& [link_pair, geometry_pairs] : collision_pairs) {
    for (const auto& geometry_pair : geometry_pairs) {
      Vector3<symbolic::Polynomial> a;
      symbolic::Polynomial b;
      VectorX<symbolic::Variable> plane_decision_vars;
      switch (plane_order) {
        case SeparatingPlaneOrder::kAffine: {
          const VectorX<symbolic::Variable> s_for_plane =
              GetSForPlane(link_pair, s_for_plane_enum);
          plane_decision_vars.resize(4 * s_for_plane.rows() + 4);
          for (int i = 0; i < plane_decision_vars.rows(); ++i) {
            plane_decision_vars(i) =
                symbolic::Variable(fmt::format("plane_var{}", i));
          }
          CalcPlane<symbolic::Variable, symbolic::Variable,
                    symbolic::Polynomial>(plane_decision_vars, s_for_plane,
                                          plane_order_, &a, &b);
        }
      }
      const multibody::BodyIndex expressed_body =
          multibody::internal::FindBodyInTheMiddleOfChain(
              rational_forward_kin_.plant(), link_pair.first(),
              link_pair.second());
      separating_planes_.emplace_back(a, b, geometry_pair.first,
                                      geometry_pair.second, expressed_body,
                                      plane_order_, plane_decision_vars);

      map_geometries_to_separating_planes_.emplace(
          SortedPair<geometry::GeometryId>(geometry_pair.first->id(),
                                           geometry_pair.second->id()),
          static_cast<int>(separating_planes_.size()) - 1);

      // Later we will also need the s variables on the kinematics chain from
      // the expressed body to either ends of link_pair (for example, when we
      // compute the monomial basis). Hence we compute it here.
      SetIndicesOfSOnChainForBodyPair(
          SortedPair<multibody::BodyIndex>(link_pair.first(), expressed_body));
      SetIndicesOfSOnChainForBodyPair(
          SortedPair<multibody::BodyIndex>(link_pair.second(), expressed_body));
    }
  }

  for (int i = 0; i < 3; ++i) {
    y_slack_(i) = symbolic::Variable("y" + std::to_string(i));
  }

  this->CalcMonomialBasis();
}

CspaceFreePolytopeBase::~CspaceFreePolytopeBase() {}

void CspaceFreePolytopeBase::CalcMonomialBasis() {
  for (int plane_index = 0;
       plane_index < static_cast<int>(separating_planes_.size());
       ++plane_index) {
    const auto& plane = separating_planes_[plane_index];
    for (const auto collision_geometry :
         {plane.positive_side_geometry, plane.negative_side_geometry}) {
      const SortedPair<multibody::BodyIndex> body_pair(
          plane.expressed_body, collision_geometry->body_index());
      std::array<VectorX<symbolic::Monomial>, 4> monomial_basis_array;
      FindMonomialBasisArray(rational_forward_kin_, y_slack_, body_pair,
                             map_body_pair_to_s_on_chain_,
                             &map_body_to_monomial_basis_array_,
                             &monomial_basis_array);
    }
  }
}

void CspaceFreePolytopeBase::CalcSBoundsPolynomial(
    const Eigen::VectorXd& s_lower, const Eigen::VectorXd& s_upper,
    VectorX<symbolic::Polynomial>* s_minus_s_lower,
    VectorX<symbolic::Polynomial>* s_upper_minus_s) const {
  const int s_size = rational_forward_kin().s().rows();
  s_minus_s_lower->resize(s_size);
  s_upper_minus_s->resize(s_size);
  const symbolic::Monomial monomial_one{};
  const auto& s = rational_forward_kin().s();
  for (int i = 0; i < s_size; ++i) {
    (*s_minus_s_lower)(i) = symbolic::Polynomial(symbolic::Polynomial::MapType(
        {{symbolic::Monomial(s(i)), 1}, {monomial_one, -s_lower(i)}}));
    (*s_upper_minus_s)(i) = symbolic::Polynomial(symbolic::Polynomial::MapType(
        {{monomial_one, s_upper(i)}, {symbolic::Monomial(s(i)), -1}}));
  }
}

void CspaceFreePolytopeBase::SetIndicesOfSOnChainForBodyPair(
    const SortedPair<multibody::BodyIndex>& body_pair) {
  if (map_body_pair_to_s_on_chain_.count(body_pair) == 0) {
    const std::vector<multibody::internal::MobilizerIndex> mobilizer_indices =
        multibody::internal::FindMobilizersOnPath(rational_forward_kin_.plant(),
                                                  body_pair.first(),
                                                  body_pair.second());
    const auto& tree =
        multibody::internal::GetInternalTree(rational_forward_kin_.plant());
    std::vector<int> s_indices;
    for (const auto& mobilizer_index : mobilizer_indices) {
      const auto& mobilizer = tree.get_mobilizer(mobilizer_index);
      if ((mobilizer.num_positions() == 1 && mobilizer.num_velocities() == 1) &&
          ((mobilizer.can_rotate() && !mobilizer.can_translate()) ||
           (mobilizer.can_translate() && !mobilizer.can_rotate()))) {
        // This is a revolute or prismatic joint.
        s_indices.push_back(
            rational_forward_kin_.map_mobilizer_to_s_index()[mobilizer_index]);
      } else if (mobilizer.num_velocities() > 0) {
        throw std::runtime_error(
            "FindMonomialBasis: we only support revolute, prismatic or weld "
            "mobilizers.");
      }
    }
    map_body_pair_to_s_on_chain_.emplace(body_pair, s_indices);
  }
}

VectorX<symbolic::Variable> CspaceFreePolytopeBase::GetSForPlane(
    const SortedPair<multibody::BodyIndex>& body_pair,
    SForPlane s_for_plane_enum) const {
  switch (s_for_plane_enum) {
    case SForPlane::kAll: {
      return rational_forward_kin_.s();
    }
    case SForPlane::kOnChain: {
      const std::vector<int>& s_indices =
          map_body_pair_to_s_on_chain_.at(body_pair);
      VectorX<symbolic::Variable> s_for_plane(s_indices.size());
      for (int i = 0; i < s_for_plane.rows(); ++i) {
        s_for_plane(i) = rational_forward_kin_.s()(s_indices[i]);
      }
      return s_for_plane;
    }
  }
  DRAKE_UNREACHABLE();
}

int CspaceFreePolytopeBase::GetSeparatingPlaneIndex(
    const SortedPair<geometry::GeometryId>& pair) const {
  return (map_geometries_to_separating_planes_.count(pair) == 0)
             ? -1
             : map_geometries_to_separating_planes_.at(pair);
}

namespace internal {
int GetGramVarSize(
    const std::array<VectorX<symbolic::Monomial>, 4>& monomial_basis_array,
    bool with_cross_y, int num_y) {
  auto gram_lower_size = [](int gram_rows) {
    return gram_rows * (gram_rows + 1) / 2;
  };
  if (num_y == 0) {
    // We only need to use monomial_basis_array[0].
    return gram_lower_size(monomial_basis_array[0].rows());
  } else {
    // We will use the monomials that contain y for the psd_mat.
    // We will denote monomial_basis_array[0] as m(s), and
    // monomial_basis_array[i+1] as  yᵢ * m(s).
    if (with_cross_y) {
      // The monomials basis we use are [m(s); y₀*m(s), ..., yₙ * m(s)] where n
      // = num_y - 1.
      int gram_rows = monomial_basis_array[0].rows();
      for (int i = 0; i < num_y; ++i) {
        gram_rows += monomial_basis_array[i + 1].rows();
      }
      return gram_lower_size(gram_rows);
    } else {
      // Use multiple monomial basis, each monomials basis is [m(s); yᵢ*m(s)].
      int ret = 0;
      for (int i = 0; i < num_y; ++i) {
        ret += gram_lower_size(monomial_basis_array[0].rows() +
                               monomial_basis_array[i + 1].rows());
      }
      return ret;
    }
  }
}
GramAndMonomialBasis::GramAndMonomialBasis(
    const std::array<VectorX<symbolic::Monomial>, 4>& monomial_basis_array,
    bool with_cross_y, int num_y) {
  this->gram_var_size =
      internal::GetGramVarSize(monomial_basis_array, with_cross_y, num_y);
  if (num_y == 0) {
    // We only need to use monomial_basis_array[0].
    this->grams.emplace_back(monomial_basis_array[0].rows(),
                             monomial_basis_array[0].rows());
    this->monomial_basis.push_back(monomial_basis_array[0]);
  } else {
    // We will use the monomials that contain y for the psd_mat.
    // We will denote monomial_basis_array[0] as m(s), and
    // monomial_basis_array[i+1] as  yᵢ * m(s).
    if (with_cross_y) {
      // The monomials basis we use is [m(s); y₀*m(s), ..., yₙ * m(s)] where
      // n = num_y - 1.
      int gram_rows = monomial_basis_array[0].rows();
      for (int i = 0; i < num_y; ++i) {
        gram_rows += monomial_basis_array[i + 1].rows();
      }
      this->grams.emplace_back(gram_rows, gram_rows);
      this->monomial_basis.emplace_back(gram_rows);
      this->monomial_basis[0].topRows(monomial_basis_array[0].rows()) =
          monomial_basis_array[0];
      gram_rows = monomial_basis_array[0].rows();
      for (int i = 0; i < num_y; ++i) {
        this->monomial_basis[0].segment(gram_rows,
                                        monomial_basis_array[i + 1].rows()) =
            monomial_basis_array[i + 1];
        gram_rows += monomial_basis_array[i + 1].rows();
      }
    } else {
      // Use multiple monomial bases, each monomial basis is [m(s); yᵢ*m(s)].
      for (int i = 0; i < num_y; ++i) {
        const int gram_rows =
            monomial_basis_array[0].rows() + monomial_basis_array[i + 1].rows();
        this->grams.emplace_back(gram_rows, gram_rows);
        this->monomial_basis.emplace_back(gram_rows);
        this->monomial_basis.back().topRows(monomial_basis_array[0].rows()) =
            monomial_basis_array[0];
        this->monomial_basis.back().bottomRows(
            monomial_basis_array[i + 1].rows()) = monomial_basis_array[i + 1];
      }
    }
  }
}

void GramAndMonomialBasis::AddSos(
    solvers::MathematicalProgram* prog,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& gram_lower,
    symbolic::Polynomial* poly) {
  int gram_var_count = 0;
  for (auto& gram : this->grams) {
    const int gram_lower_size = gram.rows() * (gram.rows() + 1) / 2;
    SymmetricMatrixFromLowerTriangularPart<symbolic::Variable>(
        gram.rows(), gram_lower.segment(gram_var_count, gram_lower_size),
        &gram);
    gram_var_count += gram_lower_size;
  }
  *poly = symbolic::Polynomial();
  gram_var_count = 0;
  for (int i = 0; i < static_cast<int>(this->grams.size()); ++i) {
    AddPsdConstraint(prog, this->grams[i]);
    const int gram_lower_size =
        this->grams[i].rows() * (this->grams[i].rows() + 1) / 2;
    *poly += symbolic::CalcPolynomialWLowerTriangularPart(
        this->monomial_basis[i],
        gram_lower.segment(gram_var_count, gram_lower_size));
    gram_var_count += gram_lower_size;
  }
}
}  // namespace internal
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
