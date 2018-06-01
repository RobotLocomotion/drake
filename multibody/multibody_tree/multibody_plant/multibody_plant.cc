#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

#include <algorithm>
#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_throw.h"
#include "drake/common/extract_double.h"
#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/math/orthonormal_basis.h"

#include <fstream>
#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;
//#define PRINT_VAR(a) (void)a;
//#define PRINT_VARn(a) (void)a;

namespace drake {
namespace multibody {
namespace multibody_plant {

// Helper macro to throw an exception within methods that should not be called
// post-finalize.
#define DRAKE_MBP_THROW_IF_FINALIZED() ThrowIfFinalized(__func__)

// Helper macro to throw an exception within methods that should not be called
// pre-finalize.
#define DRAKE_MBP_THROW_IF_NOT_FINALIZED() ThrowIfNotFinalized(__func__)

using geometry::FrameId;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::PenetrationAsPointPair;
using geometry::SceneGraph;
using geometry::SourceId;
using systems::InputPortDescriptor;
using systems::OutputPort;
using systems::State;

using drake::multibody::MultibodyForces;
using drake::multibody::MultibodyTree;
using drake::multibody::MultibodyTreeContext;
using drake::multibody::PositionKinematicsCache;
using drake::multibody::SpatialAcceleration;
using drake::multibody::SpatialForce;
using drake::multibody::VelocityKinematicsCache;
using systems::BasicVector;
using systems::Context;
using systems::InputPortDescriptor;

template<typename T>
MultibodyPlant<T>::MultibodyPlant(double time_step) :
    systems::LeafSystem<T>(systems::SystemTypeTag<
        drake::multibody::multibody_plant::MultibodyPlant>()),
    time_step_(time_step) {
  DRAKE_THROW_UNLESS(time_step >= 0);
  model_ = std::make_unique<MultibodyTree<T>>();
  visual_geometries_.emplace_back();  // Entries for the "world" body.
  collision_geometries_.emplace_back();
}

template <typename T>
geometry::SourceId MultibodyPlant<T>::RegisterAsSourceForSceneGraph(
    SceneGraph<T>* scene_graph) {
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  DRAKE_THROW_UNLESS(!geometry_source_is_registered());
  source_id_ = scene_graph->RegisterSource();
  // Save the GS pointer so that on later geometry registrations we can verify
  // the user is making calls on the same GS instance. Only used for that
  // purpose, it gets nullified at Finalize().
  scene_graph_ = scene_graph;
  return source_id_.value();
}

template <typename T>
void MultibodyPlant<T>::RegisterVisualGeometry(const Body<T>& body,
                                               const Isometry3<double>& X_BG,
                                               const geometry::Shape& shape,
                                               SceneGraph<T>* scene_graph) {
  DRAKE_MBP_THROW_IF_FINALIZED();
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  DRAKE_THROW_UNLESS(geometry_source_is_registered());
  if (scene_graph != scene_graph_) {
    throw std::logic_error(
        "Geometry registration calls must be performed on the SAME instance of "
        "SceneGraph used on the first call to "
        "RegisterAsSourceForSceneGraph()");
  }
  GeometryId id;
  // TODO(amcastro-tri): Consider doing this after finalize so that we can
  // register anchored geometry on ANY body welded to the world.
  if (body.index() == world_index()) {
    id = RegisterAnchoredGeometry(X_BG, shape, scene_graph);
  } else {
    id = RegisterGeometry(body, X_BG, shape, scene_graph);
  }
  const int visual_index = geometry_id_to_visual_index_.size();
  geometry_id_to_visual_index_[id] = visual_index;
  DRAKE_ASSERT(num_bodies() == static_cast<int>(visual_geometries_.size()));
  visual_geometries_[body.index()].push_back(id);
}

template <typename T>
const std::vector<geometry::GeometryId>&
MultibodyPlant<T>::GetVisualGeometriesForBody(const Body<T>& body) const {
  return visual_geometries_[body.index()];
}

template <typename T>
geometry::GeometryId MultibodyPlant<T>::RegisterCollisionGeometry(
    const Body<T>& body, const Isometry3<double>& X_BG,
    const geometry::Shape& shape,
    const CoulombFriction<double>& coulomb_friction,
    SceneGraph<T>* scene_graph) {
  DRAKE_MBP_THROW_IF_FINALIZED();
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  DRAKE_THROW_UNLESS(geometry_source_is_registered());
  if (scene_graph != scene_graph_) {
    throw std::logic_error(
        "Geometry registration calls must be performed on the SAME instance of "
        "SceneGraph used on the first call to "
        "RegisterAsSourceForSceneGraph()");
  }
  GeometryId id;
  // TODO(amcastro-tri): Consider doing this after finalize so that we can
  // register anchored geometry on ANY body welded to the world.
  if (body.index() == world_index()) {
    id = RegisterAnchoredGeometry(X_BG, shape, scene_graph);
  } else {
    id = RegisterGeometry(body, X_BG, shape, scene_graph);
  }
  const int collision_index = geometry_id_to_collision_index_.size();
  geometry_id_to_collision_index_[id] = collision_index;
  DRAKE_ASSERT(
      static_cast<int>(default_coulomb_friction_.size()) == collision_index);
  default_coulomb_friction_.push_back(coulomb_friction);
  DRAKE_ASSERT(num_bodies() == static_cast<int>(collision_geometries_.size()));
  collision_geometries_[body.index()].push_back(id);
  return id;
}

template <typename T>
const std::vector<geometry::GeometryId>&
MultibodyPlant<T>::GetCollisionGeometriesForBody(const Body<T>& body) const {
  DRAKE_ASSERT(body.index() < num_bodies());
  return collision_geometries_[body.index()];
}

template <typename T>
geometry::GeometryId MultibodyPlant<T>::RegisterGeometry(
    const Body<T>& body, const Isometry3<double>& X_BG,
    const geometry::Shape& shape, SceneGraph<T>* scene_graph) {
  DRAKE_ASSERT(!is_finalized());
  DRAKE_ASSERT(geometry_source_is_registered());
  DRAKE_ASSERT(scene_graph == scene_graph_);
  // If not already done, register a frame for this body.
  if (!body_has_registered_frame(body)) {
    body_index_to_frame_id_[body.index()] = scene_graph->RegisterFrame(
        source_id_.value(),
        GeometryFrame(
            body.name(),
            /* Initial pose: Not really used by GS. Will get removed. */
            Isometry3<double>::Identity()));
  }

  // Register geometry in the body frame.
  GeometryId geometry_id = scene_graph->RegisterGeometry(
      source_id_.value(), body_index_to_frame_id_[body.index()],
      std::make_unique<GeometryInstance>(X_BG, shape.Clone()));
  geometry_id_to_body_index_[geometry_id] = body.index();
  return geometry_id;
}

template <typename T>
geometry::GeometryId MultibodyPlant<T>::RegisterAnchoredGeometry(
    const Isometry3<double>& X_WG, const geometry::Shape& shape,
    SceneGraph<T>* scene_graph) {
  DRAKE_ASSERT(!is_finalized());
  DRAKE_ASSERT(geometry_source_is_registered());
  DRAKE_ASSERT(scene_graph == scene_graph_);
  GeometryId geometry_id = scene_graph->RegisterAnchoredGeometry(
      source_id_.value(),
      std::make_unique<GeometryInstance>(X_WG, shape.Clone()));
  geometry_id_to_body_index_[geometry_id] = world_index();
  return geometry_id;
}

template<typename T>
void MultibodyPlant<T>::Finalize() {
  model_->Finalize();
  FinalizePlantOnly();
}

template<typename T>
void MultibodyPlant<T>::FinalizePlantOnly() {
  DeclareStateAndPorts();
  // Only declare ports to communicate with a SceneGraph if the plant is
  // provided with a valid source id.
  if (source_id_) DeclareSceneGraphPorts();
  DeclareCacheEntries();
  scene_graph_ = nullptr;  // must not be used after Finalize().
  if (num_collision_geometries() > 0 &&
      penalty_method_contact_parameters_.time_scale < 0)
    set_penetration_allowance();
  if (num_collision_geometries() > 0 &&
      stribeck_model_.stiction_tolerance() < 0)
    set_stiction_tolerance();
}

template<typename T>
std::unique_ptr<systems::LeafContext<T>>
MultibodyPlant<T>::DoMakeLeafContext() const {
  DRAKE_THROW_UNLESS(is_finalized());
  return std::make_unique<MultibodyTreeContext<T>>(
      model_->get_topology(), is_discrete());
}

template<typename T>
MatrixX<T> MultibodyPlant<T>::CalcNormalSeparationVelocitiesJacobian(
    const Context<T>& context,
    const std::vector<PenetrationAsPointPair<T>>& point_pairs_set) const {
  const int num_contacts = point_pairs_set.size();
  MatrixX<T> N(num_contacts, num_velocities());

  for (int icontact = 0; icontact < num_contacts; ++icontact) {
    const auto& point_pair = point_pairs_set[icontact];

    const GeometryId geometryA_id = point_pair.id_A;
    const GeometryId geometryB_id = point_pair.id_B;

    BodyIndex bodyA_index = geometry_id_to_body_index_.at(geometryA_id);
    const Body<T>& bodyA = model().get_body(bodyA_index);
    BodyIndex bodyB_index = geometry_id_to_body_index_.at(geometryB_id);
    const Body<T>& bodyB = model().get_body(bodyB_index);

    // Penetration depth, > 0 if bodies interpenetrate.
    const Vector3<T>& nhat_BA_W = point_pair.nhat_BA_W;
    const Vector3<T>& p_WCa = point_pair.p_WCa;
    const Vector3<T>& p_WCb = point_pair.p_WCb;

    // Geometric Jacobian for the velocity of the contact point C as moving with
    // body A, s.t.: v_WAc = Jv_WAc * v
    // where v is the vector of generalized velocities.
    MatrixX<T> Jv_WAc(3, this->num_velocities());
    model().CalcPointsGeometricJacobianExpressedInWorld(
        context, bodyA.body_frame(), p_WCa, &Jv_WAc);

    // Geometric Jacobian for the velocity of the contact point C as moving with
    // body B, s.t.: v_WBc = Jv_WBc * v.
    MatrixX<T> Jv_WBc(3, this->num_velocities());
    model().CalcPointsGeometricJacobianExpressedInWorld(
        context, bodyB.body_frame(), p_WCb, &Jv_WBc);

    // The velocity of Bc relative to Ac is
    //   v_AcBc_W = v_WBc - v_WAc.
    // From where the separation velocity is computed as
    //   vn = -v_AcBc_W.dot(nhat_BA_W) = -nhat_BA_Wᵀ⋅v_AcBc_W
    // where the negative sign stems from the sign convention for vn and xdot.
    // This can be written in terms of the Jacobians as
    //   vn = -nhat_BA_Wᵀ⋅(Jv_WBc - Jv_WAc)⋅v

    N.row(icontact) = nhat_BA_W.transpose() * (Jv_WAc - Jv_WBc);
  }

  return N;
}

template<typename T>
MatrixX<T> MultibodyPlant<T>::CalcTangentVelocitiesJacobian(
    const Context<T>& context,
    const std::vector<PenetrationAsPointPair<T>>& point_pairs_set,
    std::vector<Matrix3<T>>* R_WC_set) const {
  const int num_contacts = point_pairs_set.size();
  // D is defined such that vt = D * v, with vt of size 2nc.
  MatrixX<T> D(2 * num_contacts, num_velocities());

  if (R_WC_set != nullptr) R_WC_set->reserve(point_pairs_set.size());
  for (int icontact = 0; icontact < num_contacts; ++icontact) {
    const auto& point_pair = point_pairs_set[icontact];

    const GeometryId geometryA_id = point_pair.id_A;
    const GeometryId geometryB_id = point_pair.id_B;

    BodyIndex bodyA_index = geometry_id_to_body_index_.at(geometryA_id);
    const Body<T>& bodyA = model().get_body(bodyA_index);
    BodyIndex bodyB_index = geometry_id_to_body_index_.at(geometryB_id);
    const Body<T>& bodyB = model().get_body(bodyB_index);

    // Penetration depth, > 0 if bodies interpenetrate.
    const T& x = point_pair.depth;
    DRAKE_ASSERT(x >= 0);
    const Vector3<T>& nhat_BA_W = point_pair.nhat_BA_W;
    const Vector3<T>& p_WCa = point_pair.p_WCa;
    const Vector3<T>& p_WCb = point_pair.p_WCb;

    // Compute the orientation of a contact frame C at the contact point such
    // that the z-axis Cz equals to nhat_BA_W. The tangent vectors are
    // arbitrary, with the only requirement being that they form a valid right
    // handed basis with nhat_BA.
    const Matrix3<T> R_WC = math::ComputeBasisFromAxis(2, nhat_BA_W);
    if (R_WC_set != nullptr) {
      R_WC_set->push_back(R_WC);
    }

    const Vector3<T> that1_W = R_WC.col(0);  // that1 = Cx.
    const Vector3<T> that2_W = R_WC.col(1);  // that2 = Cy.

    // TODO(amcastro-tri): Consider using the midpoint between Ac and Bc for
    // stability reasons. Besides that, there is no other reason to use the
    // midpoint (or any other point between Ac and Bc for that matter) since,
    // in the limit to rigid contact, Ac = Bc.

    MatrixX<T> Jv_WAc(3, this->num_velocities());  // s.t.: v_WAc = Jv_WAc * v.
    model().CalcPointsGeometricJacobianExpressedInWorld(
        context, bodyA.body_frame(), p_WCa, &Jv_WAc);

    MatrixX<T> Jv_WBc(3, this->num_velocities());  // s.t.: v_WBc = Jv_WBc * v.
    model().CalcPointsGeometricJacobianExpressedInWorld(
        context, bodyB.body_frame(), p_WCb, &Jv_WBc);

    // The velocity of Bc relative to Ac is
    //   v_AcBc_W = v_WBc - v_WAc.
    // The first two components of this velocity in C corresponds to the
    // tangential velocities in a plane normal to nhat_BA.
    //   vx_AcBc_C = that1⋅v_AcBc = that1ᵀ⋅(Jv_WBc - Jv_WAc)⋅v
    //   vy_AcBc_C = that2⋅v_AcBc = that2ᵀ⋅(Jv_WBc - Jv_WAc)⋅v

    D.row(2 * icontact)     = that1_W.transpose() * (Jv_WBc - Jv_WAc);
    D.row(2 * icontact + 1) = that2_W.transpose() * (Jv_WBc - Jv_WAc);
  }
  return D;
}

template<typename T>
void MultibodyPlant<T>::set_penetration_allowance(
    double penetration_allowance) {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  // Default to Earth's gravity for this estimation.
  const double g = gravity_field_.has_value() ?
                   gravity_field_.value()->gravity_vector().norm() : 9.81;

  // TODO(amcastro-tri): Improve this heuristics in future PR's for when there
  // are several flying objects and fixed base robots (E.g.: manipulation
  // cases.)

  // The heuristic now is very simple. We should update it to:
  //  - Only scan free bodies for weight.
  //  - Consider an estimate of maximum velocities (context dependent).
  // Right now we are being very conservative and use the maximum mass in the
  // system.
  double mass = 0.0;
  for (BodyIndex body_index(0); body_index < num_bodies(); ++body_index) {
    const Body<T>& body = model().get_body(body_index);
    mass = std::max(mass, body.get_default_mass());
  }

  // For now, we use the model of a critically damped spring mass oscillator
  // to estimate these parameters: mẍ+cẋ+kx=mg
  // Notice however that normal forces are computed according to: fₙ=kx(1+dẋ)
  // which translate to a second order oscillator of the form:
  // mẍ+(kdx)ẋ+kx=mg
  // Therefore, for this more complex, non-linear, oscillator, we estimate the
  // damping constant d using a time scale related to the free oscillation
  // (omega below) and the requested penetration allowance as a length scale.

  // We first estimate the stiffness based on static equilibrium.
  const double stiffness = mass * g / penetration_allowance;
  // Frequency associated with the stiffness above.
  const double omega = sqrt(stiffness / mass);

  // Estimated contact time scale. The relative velocity of objects coming into
  // contact goes to zero in this time scale.
  const double time_scale = 1.0 / omega;

  // Damping ratio for a critically damped model. We could allow users to set
  // this. Right now, critically damp the normal direction.
  // This corresponds to a non-penetraion constraint in the limit for
  // contact_penetration_allowance_ goint to zero (no bounce off).
  const double damping_ratio = 1.0;
  // We form the damping (with units of 1/velocity) using dimensional analysis.
  // Thus we use 1/omega for the time scale and penetration_allowance for the
  // length scale. We then scale it by the damping ratio.
  const double damping = damping_ratio * time_scale / penetration_allowance;

  // Final parameters used in the penalty method:
  penalty_method_contact_parameters_.stiffness = stiffness;
  penalty_method_contact_parameters_.damping = damping;
  // The time scale can be requested to hint the integrator's time step.
  penalty_method_contact_parameters_.time_scale = time_scale;
}

template <>
std::vector<PenetrationAsPointPair<double>>
MultibodyPlant<double>::CalcPointPairPenetrations(
    const systems::Context<double>& context) const {
  const geometry::QueryObject<double>& query_object =
      this->EvalAbstractInput(context, geometry_query_port_)
          ->template GetValue<geometry::QueryObject<double>>();
  std::vector<PenetrationAsPointPair<double>> penetrations =
      query_object.ComputePointPairPenetration();

  // TODO(amcastro-tri): Request SceneGraph to do this filtering for us
  // when that capability lands.
  // TODO(amcastro-tri): consider allowing this id's to belong to a third
  // external system when they correspond to anchored geometry.
  std::vector<PenetrationAsPointPair<double>> filtered_penetrations;
  for (const auto& penetration : penetrations) {
    const GeometryId geometryA_id = penetration.id_A;
    const GeometryId geometryB_id = penetration.id_B;

    // Filter visuals.
    if (!is_collision_geometry(geometryA_id) ||
        !is_collision_geometry(geometryB_id))
      continue;

    BodyIndex bodyA_index = geometry_id_to_body_index_.at(geometryA_id);
    BodyIndex bodyB_index = geometry_id_to_body_index_.at(geometryB_id);

    // Filter out same body collisions.
    if (bodyA_index == bodyB_index) continue;

    filtered_penetrations.push_back(penetration);
  }
  return filtered_penetrations;
}

template<typename T>
std::vector<PenetrationAsPointPair<T>> MultibodyPlant<T>::CalcPointPairPenetrations(
    const systems::Context<T>&) const {
  DRAKE_ABORT_MSG("Only <double> is supported.");
}

template<typename T>
std::vector<CoulombFriction<T>>
MultibodyPlant<T>::CalcCombinedFrictionCoefficients(
    const std::vector<PenetrationAsPointPair<T>>& point_pairs) const {
  std::vector<CoulombFriction<T>> combined_frictions(point_pairs.size());
  for (size_t ic = 0; ic < point_pairs.size(); ++ic) {
    const auto& pair = point_pairs[ic];
    const GeometryId geometryA_id = pair.id_A;
    const GeometryId geometryB_id = pair.id_B;

    const int collision_indexA =
        geometry_id_to_collision_index_.at(geometryA_id);
    const int collision_indexB =
        geometry_id_to_collision_index_.at(geometryB_id);
    const CoulombFriction<double> &geometryA_friction =
        default_coulomb_friction_[collision_indexA];
    const CoulombFriction<double> &geometryB_friction =
        default_coulomb_friction_[collision_indexB];

    combined_frictions[ic] =
        CalcContactFrictionFromSurfaceProperties(
            geometryA_friction, geometryB_friction).template cast<T>();
  }
  return combined_frictions;
}

template<typename T>
void MultibodyPlant<T>::CalcAndAddContactForcesByPenaltyMethod(
    const systems::Context<T>& context,
    const PositionKinematicsCache<T>& pc,
    const VelocityKinematicsCache<T>& vc,
    std::vector<SpatialForce<T>>* F_BBo_W_array, bool include_friction,
    const std::vector<PenetrationAsPointPair<T>>& point_pairs,
    VectorX<T>* fn) const {
  if (num_collision_geometries() == 0) return;

  std::vector<CoulombFriction<T>> combined_friction_pairs;
  if (include_friction) {
    combined_friction_pairs = CalcCombinedFrictionCoefficients(point_pairs);
  }

  int ic = 0;
  for (const auto& pair : point_pairs) {
    const GeometryId geometryA_id = pair.id_A;
    const GeometryId geometryB_id = pair.id_B;

    BodyIndex bodyA_index = geometry_id_to_body_index_.at(geometryA_id);
    BodyIndex bodyB_index = geometry_id_to_body_index_.at(geometryB_id);

    BodyNodeIndex bodyA_node_index =
        model().get_body(bodyA_index).node_index();
    BodyNodeIndex bodyB_node_index =
        model().get_body(bodyB_index).node_index();

    // Penetration depth, > 0 during pair.
    const T& x = pair.depth;
    DRAKE_ASSERT(x >= 0);
    const Vector3<T>& nhat_BA_W = pair.nhat_BA_W;
    const Vector3<T>& p_WCa = pair.p_WCa;
    const Vector3<T>& p_WCb = pair.p_WCb;

    // Contact point C.
    const Vector3<T> p_WC = 0.5 * (p_WCa + p_WCb);

    // Contact point position on body A.
    const Vector3<T>& p_WAo =
        pc.get_X_WB(bodyA_node_index).translation();
    const Vector3<T>& p_CoAo_W = p_WAo - p_WC;

    // Contact point position on body B.
    const Vector3<T>& p_WBo =
        pc.get_X_WB(bodyB_node_index).translation();
    const Vector3<T>& p_CoBo_W = p_WBo - p_WC;

    // Separation velocity, > 0  if objects separate.
    const Vector3<T> v_WAc =
        vc.get_V_WB(bodyA_node_index).Shift(-p_CoAo_W).translational();
    const Vector3<T> v_WBc =
        vc.get_V_WB(bodyB_node_index).Shift(-p_CoBo_W).translational();
    const Vector3<T> v_AcBc_W = v_WBc - v_WAc;

    // if xdot = vn > 0 ==> they are getting closer.
    const T vn = v_AcBc_W.dot(nhat_BA_W);

    // Magnitude of the normal force on body A at contact point C.
    const T k = penalty_method_contact_parameters_.stiffness;
    const T d = penalty_method_contact_parameters_.damping;
    const T fn_AC = k * x * (1.0 + d * vn);

    (*fn)(ic) = fn_AC;

    if (fn_AC > 0) {
      // Normal force on body A, at C, expressed in W.
      const Vector3<T> fn_AC_W = fn_AC * nhat_BA_W;

      Vector3<T> ft_AC_W = Vector3<T>::Zero();
      // Since the normal force is positive (non-zero), compute the friction
      // force.
      if (include_friction) {
        // Compute tangential velocity, that is, v_AcBc projected onto the tangent
        // plane with normal nhat_BA:
        const Vector3<T> vt_AcBc_W = v_AcBc_W - vn * nhat_BA_W;
        // Tangential speed (squared):
        const T vt_squared = vt_AcBc_W.squaredNorm();

        // Consider a value indistinguishable from zero if it is smaller
        // then 1e-14 and test against that value squared.
        const T kNonZeroSqd = 1e-14 * 1e-14;
        // Tangential friction force on A at C, expressed in W.
        if (vt_squared > kNonZeroSqd) {
          const T vt = sqrt(vt_squared);
          // Stribeck friction coefficient.
          const T mu_stribeck = stribeck_model_.ComputeFrictionCoefficient(
              vt, combined_friction_pairs[ic]);
          // Tangential direction.
          const Vector3<T> that_W = vt_AcBc_W / vt;

          // Magnitude of the friction force on A at C.
          const T ft_AC = mu_stribeck * fn_AC;
          ft_AC_W = ft_AC * that_W;
        }
      } // include friction

      // Spatial force on body A at C, expressed in the world frame W.
      const SpatialForce<T> F_AC_W(Vector3<T>::Zero(),
                                   fn_AC_W + ft_AC_W);

      if (F_BBo_W_array != nullptr) {
        if (bodyA_index != world_index()) {
          // Spatial force on body A at Ao, expressed in W.
          const SpatialForce<T> F_AAo_W = F_AC_W.Shift(p_CoAo_W);
          F_BBo_W_array->at(bodyA_node_index) += F_AAo_W;
        }

        if (bodyB_index != world_index()) {
          // Spatial force on body B at Bo, expressed in W.
          const SpatialForce<T> F_BBo_W = -F_AC_W.Shift(p_CoBo_W);
          F_BBo_W_array->at(bodyB_node_index) += F_BBo_W;
        }
      }
    }
    ++ic;
  }
}

template<typename T>
void MultibodyPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  // No derivatives to compute if state is discrete.
  if (is_discrete()) return;

  const auto x =
      dynamic_cast<const systems::BasicVector<T>&>(
          context.get_continuous_state_vector()).get_value();
  const int nv = this->num_velocities();

  // Allocate workspace. We might want to cache these to avoid allocations.
  // Mass matrix.
  MatrixX<T> M(nv, nv);
  // Forces.
  MultibodyForces<T> forces(*model_);
  // Bodies' accelerations, ordered by BodyNodeIndex.
  std::vector<SpatialAcceleration<T>> A_WB_array(model_->num_bodies());
  // Generalized accelerations.
  VectorX<T> vdot = VectorX<T>::Zero(nv);

  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const VelocityKinematicsCache<T>& vc = EvalVelocityKinematics(context);

  // Compute forces applied through force elements. This effectively resets
  // the forces to zero and adds in contributions due to force elements.
  model_->CalcForceElementsContribution(context, pc, vc, &forces);

  // If there is any input actuation, add it to the multibody forces.
  if (num_actuators() > 0) {
    Eigen::VectorBlock<const VectorX<T>> u =
        this->EvalEigenVectorInput(context, actuation_port_);
    for (JointActuatorIndex actuator_index(0);
         actuator_index < num_actuators(); ++actuator_index) {
      const JointActuator<T>& actuator =
          model().get_joint_actuator(actuator_index);
      // We only support actuators on single dof joints for now.
      DRAKE_DEMAND(actuator.joint().num_dofs() == 1);
      for (int joint_dof = 0;
           joint_dof < actuator.joint().num_dofs(); ++joint_dof) {
        actuator.AddInOneForce(context, joint_dof, u[actuator_index], &forces);
      }
    }
  }

  model_->CalcMassMatrixViaInverseDynamics(context, &M);

  // WARNING: to reduce memory foot-print, we use the input applied arrays also
  // as output arrays. This means that both the array of applied body forces and
  // the array of applied generalized forces get overwritten on output. This is
  // not important in this case since we don't need their values anymore.
  // Please see the documentation for CalcInverseDynamics() for details.

  // With vdot = 0, this computes:
  //   tau = C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_Bo_W.
  std::vector<SpatialForce<T>>& F_BBo_W_array = forces.mutable_body_forces();
  VectorX<T>& tau_array = forces.mutable_generalized_forces();

  // Compute contact forces on each body by penalty method.
  if (num_collision_geometries() > 0) {
    std::vector<PenetrationAsPointPair<T>> penetrations =
        CalcPointPairPenetrations(context);
    VectorX<T> fn(penetrations.size());
    CalcAndAddContactForcesByPenaltyMethod(context, pc, vc, &F_BBo_W_array,
                                           true, penetrations, &fn);
  }

  model_->CalcInverseDynamics(
      context, pc, vc, vdot,
      F_BBo_W_array, tau_array,
      &A_WB_array,
      &F_BBo_W_array, /* Notice these arrays gets overwritten on output. */
      &tau_array);

  vdot = M.ldlt().solve(-tau_array);

  auto v = x.bottomRows(nv);
  VectorX<T> xdot(this->num_multibody_states());
  VectorX<T> qdot(this->num_positions());
  model_->MapVelocityToQDot(context, v, &qdot);
  xdot << qdot, vdot;
  derivatives->SetFromVector(xdot);
}

template<typename T>
void MultibodyPlant<T>::DoCalcDiscreteVariableUpdates(
    const drake::systems::Context<T>& context0,
    const std::vector<const drake::systems::DiscreteUpdateEvent<T>*>& events,
    drake::systems::DiscreteValues<T>* updates) const {
  // Assert this method was called on a context storing discrete state.
  DRAKE_ASSERT(context0.get_num_discrete_state_groups() == 1);
  DRAKE_ASSERT(context0.get_continuous_state().size() == 0);

  using std::sqrt;
  using std::max;
  using std::abs;
  // If plant state is continuous, no discrete state to update.
  if (!is_discrete()) return;

  const double dt = time_step_;  // just a shorter alias.

  const int nq = this->num_positions();
  const int nv = this->num_velocities();

  int istep = ExtractDoubleOrThrow(context0.get_time()) / time_step_;
  (void) istep;

  // Get the system state as raw Eigen vectors
  // (solution at the previous time step).
  auto x0 = context0.get_discrete_state(0).get_value();
  VectorX<T> q0 = x0.topRows(nq);
  VectorX<T> v0 = x0.bottomRows(nv);

  // Mass matrix and its factorization.
  MatrixX<T> M0(nv, nv);
  model_->CalcMassMatrixViaInverseDynamics(context0, &M0);
  auto M0_llt = M0.llt();

  // Forces at the previous time step.
  MultibodyForces<T> forces0(*model_);

  const PositionKinematicsCache<T>& pc0 = EvalPositionKinematics(context0);
  const VelocityKinematicsCache<T>& vc0 = EvalVelocityKinematics(context0);

  // Compute forces applied through force elements.
  model_->CalcForceElementsContribution(context0, pc0, vc0, &forces0);

  // If there is any input actuation, add it to the multibody forces.
  if (num_actuators() > 0) {
    Eigen::VectorBlock<const VectorX<T>> u =
        this->EvalEigenVectorInput(context0, actuation_port_);
    for (JointActuatorIndex actuator_index(0);
         actuator_index < num_actuators(); ++actuator_index) {
      const JointActuator<T>& actuator =
          model().get_joint_actuator(actuator_index);
      // We only support actuators on single dof joints for now.
      DRAKE_DEMAND(actuator.joint().num_dofs() == 1);
      for (int joint_dof = 0;
           joint_dof < actuator.joint().num_dofs(); ++joint_dof) {
        actuator.AddInOneForce(
            context0, joint_dof, u[actuator_index], &forces0);
      }
    }
  }

  std::vector<PenetrationAsPointPair<T>> point_pairs0 =
      CalcPointPairPenetrations(context0);
  const int num_contacts = point_pairs0.size();
  VectorX<T> fn(num_contacts);

  std::vector<SpatialForce<T>>& F_BBo_W_array = forces0.mutable_body_forces();
  // Compute contact forces0 on each body by penalty method. No friction, only normal forces0.
  if (num_collision_geometries() > 0) {
    CalcAndAddContactForcesByPenaltyMethod(
        context0, pc0, vc0, &F_BBo_W_array, false, point_pairs0, &fn);
  }

  // Workspace for inverse dynamics:
  // Bodies' accelerations, ordered by BodyNodeIndex.
  std::vector<SpatialAcceleration<T>> A_WB_array(model_->num_bodies());
  // Generalized accelerations.
  VectorX<T> vdot = VectorX<T>::Zero(nv);

  // With vdot = 0, this computes (includes normal forces):
  //   -tau = C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_Bo_W.
  VectorX<T>& minus_tau = forces0.mutable_generalized_forces();
  model_->CalcInverseDynamics(
      context0, pc0, vc0, vdot,
      F_BBo_W_array, minus_tau,
      &A_WB_array,
      &F_BBo_W_array, /* Note: these arrays get overwritten on output. */
      &minus_tau);

  // Compute discrete update before applying friction forces.
  // We denote this state x* = [q*, v*], the "star" state.
  VectorX<T> v_star = v0 + dt * M0_llt.solve(-minus_tau);
  VectorX<T> qdot_star(this->num_positions());
  model_->MapVelocityToQDot(context0, v_star, &qdot_star);
  VectorX<T> q_star = q0 + dt * qdot_star;

  // Compute normal and tangential velocity Jacobians at t0.
  MatrixX<T> D(2 * num_contacts, nv);
  MatrixX<T> Minv_times_Dtrans(nv, 2 * num_contacts);
  MatrixX<T> Wtt(2 * num_contacts, 2 * num_contacts);
  VectorX<T> vt_star(2 * num_contacts);
  if (num_contacts > 0) {
    D = CalcTangentVelocitiesJacobian(context0, point_pairs0);
    vt_star = D * v_star;
    // M⁻¹Dᵀ
    Minv_times_Dtrans = M0_llt.solve(D.transpose());
    // Delassus operator coupling tangential velocities with (tangential)
    // friction forces: Wtt = DM⁻¹Dᵀ
    Wtt = D * Minv_times_Dtrans;
  }

  // Get friction coefficient into a single vector. Dynamic friction is ignored.
  std::vector<CoulombFriction<T>> combined_friction_pairs =
      CalcCombinedFrictionCoefficients(point_pairs0);
  VectorX<T> mu(num_contacts);
  std::transform(combined_friction_pairs.begin(), combined_friction_pairs.end(),
                 mu.data(),
                 [](const CoulombFriction<T>& coulomb_friction) {
                   return coulomb_friction.static_friction();
                 });

  int iter(0);
  T residual(0);
  const int num_unknowns = 2 * num_contacts;

  VectorX<T> ftk(num_unknowns);
  if (num_contacts > 0) {

    const int max_iterations = 200;
    const T tolerance = 1.0e-6;
    residual = 2 * tolerance;

    VectorX<T> vtk(num_unknowns);
    VectorX<T> Rk(vtk.size());
    MatrixX<T> Jk(vtk.size(), vtk.size());
    VectorX<T> Delta_vtk(vtk.size());
    VectorX<T> that(vtk.size());
    VectorX<T> v_slip(num_contacts);
    VectorX<T> mus(num_contacts); // Stribeck friction.
    VectorX<T> dmudv(num_contacts);
    std::vector<Matrix2<T>> dft_dv(num_contacts);
    vtk = vt_star;  // Initial guess with zero friction forces0.

    // The stiction tolerance.
    const double v_stribeck = stribeck_model_.stiction_tolerance();

    // We use the stiction tolerance as a reference scale to estimate a small
    // velocity v_epsilon. With v_epsilon we define a "soft norm" which we
    // use to compute a "soft" tangent vector to avoid the singularity at
    // vt = 0.
    const double epsilon_v = v_stribeck * 1.0e-4;
    const double epsilon_v2 = epsilon_v * epsilon_v;

    for (iter = 0; iter < max_iterations; ++iter) {
      // Compute 2D tangent vectors.
      // To avoid the singularity at v_slip = ‖vt‖ = 0 we use a "soft norm". The
      // idea is to replace the norm in the definition of slip velocity by a
      // "soft norm":
      //    ‖v‖ ≜ sqrt(vᵀv + εᵥ²)
      // We use this to redefine the slip velocity:
      //   v_slip = sqrt(vtᵀvt + v_epsilon)
      // and a "soft" tangent vector:
      //   t̂ = vₜ / sqrt(vₜᵀvₜ + εᵥ²)
      // which now is not only well defined but it has well defined derivatives.
      // We use these softened quantities all throuout our derivations for
      // consistency.
      for (int ic = 0; ic < num_contacts; ++ic) {
        const int ik = 2 * ic;
        const auto vt_ic = vtk.template segment<2>(ik);
        // "soft norm":
        v_slip(ic) = sqrt(vt_ic.squaredNorm() + epsilon_v2);
        // "soft" tangent vector:
        const Vector2<T> that_ic =  vt_ic / v_slip(ic);
        that.template segment<2>(ik) = that_ic;
        mus(ic) =
            stribeck_model_.ModifiedStribeck(v_slip(ic) / v_stribeck, mu(ic));
        // Friction force.
        // Note: minus sign not included in this definition.
        ftk.template segment<2>(ik) = mus(ic) * that_ic * fn(ic);
      }

      // After the previous iteration, we allow updating ftk above to have its
      // latest value before leaving.
      if (residual < tolerance) {
        break;
      }

      // NR residual
      Rk = vtk - vt_star;
      for (int ic = 0; ic < num_contacts; ++ic) {
        const int ik = 2 * ic;
        auto Rk_ic = Rk.template segment(ik, 2);

        for (int jc = 0; jc < num_contacts; ++jc) {
          const int jk = 2 * jc;
          const auto Wij = Wtt.template block<2, 2>(ik, jk);
          const auto ft_jc = ftk.template segment<2>(jk);
          Rk_ic += time_step_ * Wij * ft_jc;
        }

        // Compute dmu/dv = (1/v_stribeck) * dmu/dx
        // where x = v_slip / v_stribeck is the dimensionless slip velocity.
        dmudv(ic) = stribeck_model_.ModifiedStribeckPrime(
            v_slip(ic) / v_stribeck, mu(ic)) / v_stribeck;

        const auto that_ic = that.template segment<2>(ik);

        // Projection matrix. It projects in the direction of that.
        // Notice it is a symmetric 2x2 matrix.
        const Matrix2<T> P_ic = that_ic * that_ic.transpose();

        // Removes the projected direction along that.
        // This is also a symmetric 2x2 matrix.
        const Matrix2<T> Pperp_ic = Matrix2<T>::Identity() - P_ic;

        // Some notes about projection matrices P:
        //  - They are symmetric, positive semi-definite.
        //  - All their eigenvalues are either one or zero.
        //  - Their rank equals the number of non-zero eigenvales.
        //  - From the previous item we have rank(P) = trace(P).
        //  - If P is a projection matrix, so is (I - P).
        // From the above we then know that P and Pperp are both projection
        // matrices of rank one (i.e. rank deficient) and are symmetric
        // semi-positive definite. This has very important consequences for the
        // Jacobian of the residual.

        // We now compute dft/dvt as:
        //   dft/dvt = fn * (
        //     mu_stribeck(‖vₜ‖) / ‖vₜ‖ * Pperp(t̂) +
        //     dmu_stribeck/dx * P(t̂) / v_stribeck )
        // where x = v_slip / v_stribeck is the dimensionless slip velocity.
        // Therefore dft/dvt (in ℝ²ˣ²) is a linear combination of PSD matrices
        // (P and Pperp) where the coefficients of the combination are positive
        // scalars. Therefore,
        // IMPORTANT NOTE: dft/dvt also PSD.
        // IMPORTANT NOTE 2: The derivation for dft/dvt leads to exactly the
        // same result when using the "softened" definitions for v_slip and
        // that where each occurrence of these quantities is replaced by their
        // softened counterpart.

        // Compute dft/dv:
        // Changes of vt in the direction perpendicular to that.
        dft_dv[ic] = Pperp_ic * mus(ic) / v_slip(ic);

        // Changes in the magnitude of vt (which in turns makes mu_stribeck
        // change), in the direction of that.
        dft_dv[ic] += P_ic * dmudv(ic);

        // Note: dft_dv is a symmetric 2x2 matrix.
        dft_dv[ic] *= fn(ic);
      }

      // NR Jacobian
      Jk.setIdentity();  // Identity I2 blocks.
      for (int ic = 0; ic < num_contacts; ++ic) {
        const int ik = 2 * ic;

        for (int jc = 0; jc < num_contacts; ++jc) {
          const int jk = 2 * jc;

          const auto Wij = Wtt.template block<2, 2>(ik, jk);
          auto Jij = Jk.template block<2, 2>(ik, jk);

          Jij += time_step_ * Wij * dft_dv[jc];
        }
      }

      Delta_vtk = Jk.lu().solve(-Rk);
      residual = Delta_vtk.norm();

      // Limit the angle change
      const  T theta_max = 0.25;  // about 15 degs
      const T cmin = cos(theta_max);
      for (int ic = 0; ic < num_contacts; ++ic) {
        const int ik = 2 * ic;
        auto v = vtk.template segment<2>(ik);
        const auto dv = Delta_vtk.template segment<2>(ik);

        T A = v.norm();
        T B = v.dot(dv);
        T DD = dv.norm();

        //if (A < 1e-14 && D < 1e-14) continue;

        Vector2<T> v1 = v+dv;  // for alpha = 1
        const T v1_norm = v1.norm();
        const T v_norm = v.norm();
        const T cos_init = v1.dot(v) / (v1_norm+1e-10) / (v_norm+1e-10);

        Vector2<T> valpha;
        T alpha;

        if (istep==598){
          PRINT_VAR(iter);
          PRINT_VAR(cos_init);
          PRINT_VAR(v.transpose());
          PRINT_VAR(v1.transpose());
          PRINT_VAR(dv.transpose());
        }

        const T x = v_norm / v_stribeck;
        const T x1 = v1_norm / v_stribeck;

        // 180 degrees direction change.
        if ( abs(1.0+cos_init) < 1.0e-10 ) {
          // Clip to near the origin since we know for sure we crossed it.
          valpha = v / (v.norm()+1e-14) * v_stribeck / 2.0;
        } else if (cos_init > cmin || (v1_norm*v_norm) < 1.0e-14 || x < 1.0 || x1 < 1.0) {  // the angle change is small enough
          alpha = 1.0;
          valpha = v1;
        } else { // Limit the angle change
          T A2 = A * A;
          T A4 = A2 * A2;
          T cmin2 = cmin * cmin;

          T a = A2 * DD * DD * cmin2 - B * B;
          T b = 2 * A2 * B * (cmin2 - 1.0);
          T c = A4 * (cmin2 - 1.0);

          T delta = b * b - 4 * a * c;
          if ( delta <  0 || istep == 598) {
            PRINT_VAR(iter);
            PRINT_VAR(istep);
            PRINT_VAR(delta);
            PRINT_VAR(A);
            PRINT_VAR(B);
            PRINT_VAR(cmin);
            PRINT_VAR(DD);
            PRINT_VAR(cos_init);
            PRINT_VAR(abs(1.0+cos_init));
            PRINT_VAR(v.transpose());
            PRINT_VAR(v1.transpose());
            PRINT_VAR(dv.transpose());
            DRAKE_DEMAND(delta > -1e-16);
          }

          T sqrt_delta = sqrt(max(delta, 0.0));

          // There should be a positive and a negative root.
          alpha = (-b + sqrt_delta) / a / 2.0;
          //double alpha2 = (-b - sqrt_delta)/a/2.0;
          if (alpha <= 0) {
            PRINT_VAR(alpha);
            PRINT_VAR(iter);
            PRINT_VAR(istep);
            PRINT_VAR(delta);
            PRINT_VAR(A);
            PRINT_VAR(B);
            PRINT_VAR(cmin);
            PRINT_VAR(DD);
            PRINT_VAR(cos_init);
            PRINT_VAR(abs(1.0+cos_init));
            PRINT_VAR(a);
            PRINT_VAR(b);
            PRINT_VAR(c);
            PRINT_VAR(v.transpose());
            PRINT_VAR(v1.transpose());
            PRINT_VAR(dv.transpose());
          }

          DRAKE_DEMAND(alpha > 0);

          valpha = v + alpha * dv;
        }

        // clip v
        v = valpha;

      }

      // See if worth detection crosses about the line perpendicular to the
      // velocity change, ala Sherm.
      //vtk += Delta_vtk;
    }
  }

  std::ofstream outfile;
  outfile.open("nr_iteration.dat", std::ios_base::app);
  outfile <<
          fmt::format("{0:14.6e} {1:d} {2:d} {3:d} {4:14.6e}\n",
                      context0.get_time(), istep, iter, num_contacts,residual);
  outfile.close();

  // Compute velocity at next time step.
  VectorX<T> v_next(this->num_velocities());
  v_next = v_star;
  if (num_contacts > 0) {
    v_next -= time_step_ * Minv_times_Dtrans * ftk;
  }

  VectorX<T> qdot_next(this->num_positions());
  model_->MapVelocityToQDot(context0, v_next, &qdot_next);

  // qn = q + dt*qdot.
  VectorX<T> x_next(this->num_multibody_states());
  x_next << q0 + dt * qdot_next, v_next;
  updates->get_mutable_vector(0).SetFromVector(x_next);
}

template<typename T>
void MultibodyPlant<T>::DoMapQDotToVelocity(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& qdot,
    systems::VectorBase<T>* generalized_velocity) const {
  const int nq = model_->num_positions();
  const int nv = model_->num_velocities();

  DRAKE_ASSERT(qdot.size() == nq);
  DRAKE_DEMAND(generalized_velocity != nullptr);
  DRAKE_DEMAND(generalized_velocity->size() == nv);

  VectorX<T> v(nv);
  model_->MapQDotToVelocity(context, qdot, &v);
  generalized_velocity->SetFromVector(v);
}

template<typename T>
void MultibodyPlant<T>::DoMapVelocityToQDot(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& generalized_velocity,
    systems::VectorBase<T>* positions_derivative) const {
  const int nq = model_->num_positions();
  const int nv = model_->num_velocities();

  DRAKE_ASSERT(generalized_velocity.size() == nv);
  DRAKE_DEMAND(positions_derivative != nullptr);
  DRAKE_DEMAND(positions_derivative->size() == nq);

  VectorX<T> qdot(nq);
  model_->MapVelocityToQDot(context, generalized_velocity, &qdot);
  positions_derivative->SetFromVector(qdot);
}

template<typename T>
void MultibodyPlant<T>::DeclareStateAndPorts() {
  // The model must be finalized.
  DRAKE_DEMAND(this->is_finalized());

  if (is_discrete()) {
    this->DeclarePeriodicDiscreteUpdate(time_step_);
    this->DeclareDiscreteState(num_multibody_states());
  } else {
    this->DeclareContinuousState(
        BasicVector<T>(model_->num_states()),
        model_->num_positions(),
        model_->num_velocities(), 0 /* num_z */);
  }

  if (num_actuators() > 0) {
    actuation_port_ =
        this->DeclareVectorInputPort(
            systems::BasicVector<T>(num_actuated_dofs())).get_index();
  }

  continuous_state_output_port_ =
      this->DeclareVectorOutputPort(
          BasicVector<T>(num_multibody_states()),
          &MultibodyPlant::CopyContinuousStateOut).get_index();
}

template <typename T>
void MultibodyPlant<T>::CopyContinuousStateOut(
    const Context<T>& context, BasicVector<T>* state_vector) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  state_vector->SetFrom(context.get_continuous_state_vector());
}

template <typename T>
const systems::InputPortDescriptor<T>&
MultibodyPlant<T>::get_actuation_input_port() const {
  DRAKE_THROW_UNLESS(is_finalized());
  DRAKE_THROW_UNLESS(num_actuators() > 0);
  return systems::System<T>::get_input_port(actuation_port_);
}

template <typename T>
const systems::OutputPort<T>&
MultibodyPlant<T>::get_continuous_state_output_port() const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  return this->get_output_port(continuous_state_output_port_);
}

template <typename T>
void MultibodyPlant<T>::DeclareSceneGraphPorts() {
  geometry_query_port_ = this->DeclareAbstractInputPort().get_index();
  // This presupposes that the source id has been assigned and _all_ frames have
  // been registered.
  std::vector<FrameId> ids;
  for (auto it : body_index_to_frame_id_) {
    ids.push_back(it.second);
  }
  geometry_pose_port_ =
      this->DeclareAbstractOutputPort(
          FramePoseVector<T>(*source_id_, ids),
          &MultibodyPlant::CalcFramePoseOutput).get_index();
}

template <typename T>
void MultibodyPlant<T>::CalcFramePoseOutput(
    const Context<T>& context, FramePoseVector<T>* poses) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_ASSERT(source_id_ != nullopt);
  DRAKE_ASSERT(
      poses->size() == static_cast<int>(body_index_to_frame_id_.size()));
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);

  // TODO(amcastro-tri): Make use of Body::EvalPoseInWorld(context) once caching
  // lands.
  poses->clear();
  for (const auto it : body_index_to_frame_id_) {
    const BodyIndex body_index = it.first;
    const Body<T>& body = model_->get_body(body_index);

    // NOTE: The GeometryFrames for each body were registered in the world
    // frame, so we report poses in the world frame.
    poses->set_value(body_index_to_frame_id_.at(body_index),
                     pc.get_X_WB(body.node_index()));
  }
}

template <typename T>
const OutputPort<T>& MultibodyPlant<T>::get_geometry_poses_output_port()
const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_DEMAND(geometry_source_is_registered());
  return systems::System<T>::get_output_port(geometry_pose_port_);
}

template <typename T>
const systems::InputPortDescriptor<T>&
MultibodyPlant<T>::get_geometry_query_input_port() const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_DEMAND(geometry_source_is_registered());
  return systems::System<T>::get_input_port(geometry_query_port_);
}

template<typename T>
void MultibodyPlant<T>::DeclareCacheEntries() {
  // TODO(amcastro-tri): User proper System::Declare() infrastructure to
  // declare cache entries when that lands.
  pc_ = std::make_unique<PositionKinematicsCache<T>>(model_->get_topology());
  vc_ = std::make_unique<VelocityKinematicsCache<T>>(model_->get_topology());
}

template<typename T>
const PositionKinematicsCache<T>& MultibodyPlant<T>::EvalPositionKinematics(
    const systems::Context<T>& context) const {
  // TODO(amcastro-tri): Replace Calc() for an actual Eval() when caching lands.
  model_->CalcPositionKinematicsCache(context, pc_.get());
  return *pc_;
}

template<typename T>
const VelocityKinematicsCache<T>& MultibodyPlant<T>::EvalVelocityKinematics(
    const systems::Context<T>& context) const {
  // TODO(amcastro-tri): Replace Calc() for an actual Eval() when caching lands.
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  model_->CalcVelocityKinematicsCache(context, pc, vc_.get());
  return *vc_;
}

template <typename T>
void MultibodyPlant<T>::ThrowIfFinalized(const char* source_method) const {
  if (is_finalized()) {
    throw std::logic_error(
        "Post-finalize calls to '" + std::string(source_method) + "()' are "
        "not allowed; calls to this method must happen before Finalize().");
  }
}

template <typename T>
void MultibodyPlant<T>::ThrowIfNotFinalized(const char* source_method) const {
  if (!is_finalized()) {
    throw std::logic_error(
        "Pre-finalize calls to '" + std::string(source_method) + "()' are "
        "not allowed; you must call Finalize() first.");
  }
}

template <typename T>
T MultibodyPlant<T>::StribeckModel::ComputeFrictionCoefficient(
    const T& speed_BcAc,
    const CoulombFriction<T>& friction) const {
  DRAKE_ASSERT(speed_BcAc >= 0);
  const T& mu_d = friction.dynamic_friction();
  const T& mu_s = friction.static_friction();
  const T v = speed_BcAc * inv_v_stiction_tolerance_;
  if (v >= 3) {
    return mu_d;
  } else if (v >= 1) {
    return mu_s - (mu_s - mu_d) * step5((v - 1) / 2);
  } else {
    return mu_s * step5(v);
  }
}

template <typename T>
T MultibodyPlant<T>::StribeckModel::step5(const T& x) {
  DRAKE_ASSERT(0 <= x && x <= 1);
  const T x3 = x * x * x;
  return x3 * (10 + x * (6 * x - 15));  // 10x³ - 15x⁴ + 6x⁵
}

template <typename T>
T MultibodyPlant<T>::StribeckModel::ModifiedStribeck(const T& x, const T& mu) {
  DRAKE_ASSERT(x >= 0);
  if (x >= 1) {
    return mu;
  } else {
    return mu * x * (2.0 - x);
  }
}

template <typename T>
T MultibodyPlant<T>::StribeckModel::ModifiedStribeckPrime(
    const T& x, const T& mu) {
  DRAKE_ASSERT(x >= 0);
  if (x >= 1) {
    return 0;
  } else {
    return mu * (2 * (1 - x));
  }
}

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::multibody_plant::MultibodyPlant)
