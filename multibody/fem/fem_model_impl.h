#pragma once

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"
#include "drake/multibody/fem/fem_element.h"
#include "drake/multibody/fem/fem_indexes.h"
#include "drake/multibody/fem/fem_model.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* See FemModel for documentation of this class.
 @tparam Element  The type of FEM elements that makes up this FemModelImpl.
 This template parameter must be an instantiation of FemElement, which provides
 the scalar type and the compile time constants such as the natural dimension
 and the number of nodes/quadrature points in each element. See FemElements for
 more details. */
template <class Element>
class FemModelImpl : public FemModel<typename Element::T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FemModelImpl);

  static_assert(
      std::is_base_of_v<FemElement<Element>, Element>,
      "The template parameter Element should be derived from FemElement. ");
  using T = typename Element::T;
  using Data = typename Element::Data;

  /* Returns the number of FEM elements owned by this FEM model. */
  int num_elements() const final { return elements_.size(); }

 protected:
  /* Creates an empty FemModelImpl with no elements. */
  explicit FemModelImpl(const Vector3<T>& tangent_matrix_weights)
      : FemModel<T>(tangent_matrix_weights) {}

  ~FemModelImpl() = default;

  /* Moves the input `element` into the vector of elements held by this
   FemModelImpl. */
  void AddElement(Element&& element) {
    elements_.emplace_back(std::move(element));
  }

  /* Moves the input `elements`' entries into the vector of elements owned by
   this FemModelImpl. The entries in `elements` will be left in the moved-from
   state, but the size of `elements` does not change.
   @pre elements != nullptr */
  void AddElements(std::vector<Element>* elements) {
    DRAKE_DEMAND(elements != nullptr);
    elements_.insert(elements_.end(),
                     std::make_move_iterator(elements->begin()),
                     std::make_move_iterator(elements->end()));
  }

  /* Returns all elements stored in this model. */
  const std::vector<Element>& elements() const { return elements_; }

 protected:
  /** Returns the cache index for the per-element data in this model. */
  systems::CacheIndex element_data_index() const { return element_data_index_; }

  /** Copies all data members, except those modified in DeclareCacheEntries(),
   from the `other` FemModelImpl to `this` FemModelImpl . */
  void SetFrom(const FemModelImpl<Element>& other) {
    elements_ = other.elements_;
  }

 private:
  Vector3<T> DoCalcCenterOfMassPositionInWorld(
      const FemState<T>& fem_state) const final {
    /* p_WoScm_W = ∑ (mᵢ pᵢ) / mₛ, where mᵢ is the mass of the iᵗʰ element,
     mₛ = ∑ mᵢ is the total mass,  and pᵢ is the position vector from Wo to
     the quadrature point of the iᵗʰ element, expressed in the world frame W,
     and Scm is the center of mass of this FemModel S. */
    Vector3<T> sum_mi_pi = Vector3<T>::Zero();
    const T total_mass = this->get_total_mass();
    const std::vector<Data>& element_data =
        fem_state.template EvalElementData<Data>(element_data_index_);
    for (int e = 0; e < num_elements(); ++e) {
      sum_mi_pi += elements_[e].CalcMassTimesPositionForQuadraturePoints(
          element_data[e]);
    }
    DRAKE_DEMAND(total_mass > 0.0);
    return sum_mi_pi / total_mass;
  }

  Vector3<T> DoCalcCenterOfMassTranslationalVelocityInWorld(
      const FemState<T>& fem_state) const final {
    /* For a system S with center of mass Scm, Scm's translational velocity in
     the world frame W is calculated as v_WScm_W = ∑ (mᵢ vᵢ) / mₛ, where mₛ = ∑
     mᵢ, mᵢ is the mass of the iᵗʰ element, and vᵢ is the velocity evaluated at
     the quadrature point of the iᵗʰ element, expressed in the world frame W. */
    const T total_mass = this->get_total_mass();
    Vector3<T> sum_mi_vi = Vector3<T>::Zero();
    const std::vector<Data>& element_data =
        fem_state.template EvalElementData<Data>(element_data_index_);
    for (int e = 0; e < num_elements(); ++e) {
      sum_mi_vi += elements_[e].CalcTranslationalMomentumForQuadraturePoints(
          element_data[e]);
    }
    DRAKE_DEMAND(total_mass > 0.0);
    return sum_mi_vi / total_mass;
  }

  Vector3<T> DoCalcEffectiveAngularVelocity(
      const FemState<T>& fem_state) const final {
    const T total_mass = this->get_total_mass();
    Vector3<T> sum_mi_pi = Vector3<T>::Zero();
    Vector3<T> sum_mi_vi = Vector3<T>::Zero();
    Vector3<T> H_WSWo_W = Vector3<T>::Zero();
    Matrix3<T> I_SWo_W = Matrix3<T>::Zero();

    const std::vector<Data>& element_data =
        fem_state.template EvalElementData<Data>(element_data_index_);
    for (int e = 0; e < num_elements(); ++e) {
      sum_mi_pi += elements_[e].CalcMassTimesPositionForQuadraturePoints(
          element_data[e]);
      sum_mi_vi += elements_[e].CalcTranslationalMomentumForQuadraturePoints(
          element_data[e]);
      H_WSWo_W +=
          elements_[e].CalcAngularMomentumAboutWorldOrigin(element_data[e]);
      I_SWo_W +=
          elements_[e].CalcRotationalInertiaAboutWorldOrigin(element_data[e]);
    }
    DRAKE_DEMAND(total_mass > 0.0);
    const Vector3<T> p_WoScm_W = sum_mi_pi / total_mass;
    const Vector3<T> v_WScm_W = sum_mi_vi / total_mass;
    /* Shift angular momentum from world origin to center of mass
       H_WScm = H_WO - p_WoScm × mv_WScm. */
    const Vector3<T> H_WScm_W =
        H_WSWo_W - p_WoScm_W.cross(total_mass * v_WScm_W);

    /* Shift inertia from world origin to center of mass using the "parallel
     axis theorem" below, with I₃₃ being the 3x3 identity matrix. I_SScm = I_SWO
     - m(p_WoScm · p_WoScm) I₃₃ + m(p_WoScm ⊗ p_WoScm) */
    const Matrix3<T> I_SScm_W =
        I_SWo_W -
        total_mass * (p_WoScm_W.dot(p_WoScm_W) * Matrix3<T>::Identity() -
                      p_WoScm_W * p_WoScm_W.transpose());
    return I_SScm_W.ldlt().solve(H_WScm_W);
  }

  void DoCalcResidual(const FemState<T>& fem_state,
                      const FemPlantData<T>& plant_data,
                      EigenPtr<VectorX<T>> residual) const final {
    /* The values are accumulated in the residual, so it is important to clear
     the old data. */
    residual->setZero();
    constexpr int kDim = 3;
    /* Scratch space to store the contribution to the residual from each
     element. */
    Vector<T, Element::num_dofs> element_residual;
    const std::vector<Data>& element_data =
        fem_state.template EvalElementData<Data>(element_data_index_);
    for (int e = 0; e < num_elements(); ++e) {
      /* residual = Ma-fₑ(x)-fᵥ(x, v)-fₑₓₜ. */
      /* The Ma-fₑ(x)-fᵥ(x, v) term. */
      element_residual = element_data[e].inverse_dynamics;
      /* The -fₑₓₜ term. */
      elements_[e].AddScaledExternalForces(element_data[e], plant_data, -1.0,
                                           &element_residual);
      const std::array<FemNodeIndex, Element::num_nodes>& element_node_indices =
          elements_[e].node_indices();
      for (int a = 0; a < Element::num_nodes; ++a) {
        const int global_node = element_node_indices[a];
        residual->template segment<kDim>(global_node * kDim) +=
            element_residual.template segment<kDim>(a * kDim);
      }
    }
  }

  void DoCalcTangentMatrix(
      const FemState<T>& fem_state,
      contact_solvers::internal::BlockSparseSymmetricMatrix3d* tangent_matrix)
      const final {
    /* We already check for the scalar type in `CalcTangentMatrix()` but the `if
     constexpr` here is still needed to make the compiler happy. */
    if constexpr (std::is_same_v<T, double>) {
      /* Clears the old data. */
      tangent_matrix->SetZero();

      const std::vector<Data>& element_data =
          fem_state.template EvalElementData<Data>(element_data_index_);
      for (int e = 0; e < num_elements(); ++e) {
        const Eigen::Matrix<T, Element::num_dofs, Element::num_dofs>&
            element_tangent_matrix = element_data[e].tangent_matrix;
        const std::array<FemNodeIndex, Element::num_nodes>&
            element_node_indices = elements_[e].node_indices();
        for (int a = 0; a < Element::num_nodes; ++a) {
          const int i = element_node_indices[a];
          for (int b = 0; b <= a; ++b) {
            const int j = element_node_indices[b];
            if (i >= j) {
              tangent_matrix->AddToBlock(
                  i, j,
                  element_tangent_matrix.template block<3, 3>(3 * a, 3 * b));
            } else {
              tangent_matrix->AddToBlock(
                  j, i,
                  element_tangent_matrix.template block<3, 3>(3 * b, 3 * a));
            }
          }
        }
      }
    } else {
      unused(fem_state, tangent_matrix);
      DRAKE_UNREACHABLE();
    }
  }

  std::unique_ptr<contact_solvers::internal::BlockSparseSymmetricMatrix3d>
  DoMakeTangentMatrix() const final {
    /* We already check for the scalar type in `MakeTangentMatrix()` but the `if
     constexpr` here is still needed to make the compiler happy. */
    if constexpr (std::is_same_v<T, double>) {
      std::vector<std::unordered_set<int>> neighbor_nodes(this->num_nodes());
      /* Create a nonzero block for each pair of nodes that are connected by an
       edge in the mesh. */
      for (int e = 0; e < num_elements(); ++e) {
        const std::array<FemNodeIndex, Element::num_nodes>&
            element_node_indices = elements_[e].node_indices();
        for (int a = 0; a < Element::num_nodes; ++a) {
          for (int b = a; b < Element::num_nodes; ++b) {
            /* SymmetricBlockSparseMatrix only needs to allocate for the
             lower triangular part of the matrix. So instead of allocating for
             both (element_node_indices[a], element_node_indices[b]) and
             (element_node_indices[b], element_node_indices[a]) blocks, we only
             allocate for one of them. See Block3x3SparseSymmetricMatrix. */
            const int j =
                std::min(element_node_indices[a], element_node_indices[b]);
            const int i =
                std::max(element_node_indices[a], element_node_indices[b]);
            neighbor_nodes[j].insert(i);
          }
        }
      }
      std::vector<std::vector<int>> sparsity_pattern;
      sparsity_pattern.reserve(this->num_nodes());
      for (int j = 0; j < this->num_nodes(); ++j) {
        sparsity_pattern.emplace_back(neighbor_nodes[j].begin(),
                                      neighbor_nodes[j].end());
      }
      contact_solvers::internal::BlockSparsityPattern block_pattern(
          std::vector<int>(this->num_nodes(), 3), std::move(sparsity_pattern));
      return std::make_unique<
          contact_solvers::internal::BlockSparseSymmetricMatrix3d>(
          std::move(block_pattern));
    } else {
      DRAKE_UNREACHABLE();
    }
  }

  void DeclareCacheEntries(
      internal::FemStateSystem<T>* fem_state_system) final {
    element_data_index_ =
        fem_state_system
            ->DeclareCacheEntry(
                "element data",
                systems::ValueProducer(this,
                                       &FemModelImpl<Element>::CalcElementData))
            .cache_index();
  }

  bool do_is_linear() const final { return Element::is_linear; }

  T DoCalcTotalMass() const final {
    T total_mass = 0.0;
    for (int e = 0; e < num_elements(); ++e) {
      total_mass += elements_[e].mass();
    }
    return total_mass;
  }

  /* Computes the element data for each element in this FEM model. */
  void CalcElementData(const systems::Context<T>& context,
                       std::vector<Data>* data) const {
    DRAKE_DEMAND(data != nullptr);
    data->resize(num_elements());
    const FemState<T> fem_state(&(this->fem_state_system()), &context);

    // TODO(xuchenhan-tri) We should switch to the CRU helper eventually, so
    // that we gain std::async support.
    [[maybe_unused]] const int num_threads = this->parallelism().num_threads();
#if defined(_OPENMP)
#pragma omp parallel for num_threads(num_threads)
#endif
    for (int i = 0; i < num_elements(); ++i) {
      (*data)[i] =
          elements_[i].ComputeData(fem_state, this->tangent_matrix_weights());
    }
  }

  /* FemElements owned by this model. */
  std::vector<Element> elements_;
  systems::CacheIndex element_data_index_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
