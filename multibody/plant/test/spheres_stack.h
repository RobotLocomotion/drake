#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"

using drake::geometry::ProximityProperties;
using drake::geometry::SceneGraph;
using drake::math::RigidTransformd;
using drake::systems::Context;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

// TODO(amcastro-tri): Implement AutoDiffXd testing.

namespace drake {
namespace multibody {
namespace internal {

// Helper function for NaN initialization.
static constexpr double nan() {
  return std::numeric_limits<double>::quiet_NaN();
}

// TODO(amcastro-tri): This testing class grew to cover a wide range of
// functionalities that used to be part of CompliantContactManger previous to
// #17741. Now this functionality has been split among driver classes such as
// SapDriver, and more will be introduced. Therefore there is a chance here to
// simplify the setup to only test functionality specific to each driver and/or
// manager.

// In this fixture we set a simple model consisting of a flat ground,
// a sphere (sphere 1) on top of the ground, and another sphere (sphere 2)
// on top the first sphere. They are assigned to be rigid-hydroelastic,
// compliant-hydroelastic, or non-hydroelastic to test various cases of
// contact quantities computed by the CompliantContactManager.
//
// N.B. This testing class only exercises the SapDriver code paths. In
// particular, it uses the DiscreteContactApproximation::kSap contact model
// approximation.
class SpheresStack {
 public:
  // Contact model parameters.
  struct ContactParameters {
    // Point contact stiffness. If nullopt, this property is not added to the
    // model.
    std::optional<double> point_stiffness;
    // Hydroelastic modulus. If nullopt, this property is not added to the
    // model.
    std::optional<double> hydro_modulus;
    // Relaxation time constant τ is used to setup the linear dissipation model
    // where dissipation is c = τ⋅k, with k the point pair stiffness.
    // If nullopt, no dissipation is specified, i.e. the corresponding
    // ProximityProperties will not have dissipation defined.
    std::optional<double> relaxation_time;
    // Coefficient of dynamic friction.
    double friction_coefficient{nan()};
  };

  // Parameters used to setup the model of a compliant sphere.
  struct SphereParameters {
    const std::string name;
    double mass;
    double radius;
    // No contact geometry is registered if nullopt.
    std::optional<ContactParameters> contact_parameters;
  };

  // Sets up this fixture to have:
  //   - A rigid-hydroelastic half-space for the ground.
  //   - A compliant-hydroelastic sphere on top of the ground.
  //   - A non-hydroelastic sphere on top of the first sphere.
  //   - Both spheres also have point contact compliance.
  //   - We set MultibodyPlant to use hydroelastic contact with fallback.
  //   - Sphere 1 penetrates into the ground penetration_distance_.
  //   - Sphere 1 and 2 penetrate penetration_distance_.
  //   - Velocities are zero.
  void SetupRigidGroundCompliantSphereAndNonHydroSphere(
      bool sphere1_on_prismatic_joint = false) {
    const ContactParameters compliant_contact{1.0e5, 1.0e5, 0.01, 1.0};
    const ContactParameters non_hydro_contact{1.0e5, {}, 0.01, 1.0};
    const ContactParameters rigid_hydro_contact{
        std::nullopt, std::numeric_limits<double>::infinity(), 0.0, 1.0};
    const SphereParameters sphere1_params{"Sphere1", 10.0 /* mass */,
                                          0.2 /* size */, compliant_contact};
    const SphereParameters sphere2_params{"Sphere2", 10.0 /* mass */,
                                          0.2 /* size */, non_hydro_contact};
    MakeModel(rigid_hydro_contact, sphere1_params, sphere2_params,
              sphere1_on_prismatic_joint);
  }

  void SetupFreeFloatingSpheresWithNoContact() {
    const bool sphere1_on_prismatic_joint = false;
    const SphereParameters sphere1_params{"Sphere1", 10.0 /* mass */,
                                          0.2 /* size */,
                                          std::nullopt /* no contact */};
    const SphereParameters sphere2_params{"Sphere2", 10.0 /* mass */,
                                          0.2 /* size */,
                                          std::nullopt /* no contact */};
    MakeModel(std::nullopt /* no ground */, sphere1_params, sphere2_params,
              sphere1_on_prismatic_joint);
  }

  // Sets up a model with two spheres and the ground.
  // Spheres are defined by their SphereParameters. Ground contact is defined by
  // `ground_params`, no ground is added if std::nullopt.
  // Sphere 1 is mounted on a prismatic joint about the z axis if
  // sphere1_on_prismatic_joint = true.
  void MakeModel(const std::optional<ContactParameters>& ground_params,
                 const SphereParameters& sphere1_params,
                 const SphereParameters& sphere2_params,
                 bool sphere1_on_prismatic_joint = false) {
    systems::DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) =
        AddMultibodyPlantSceneGraph(&builder, time_step_);
    plant_->set_discrete_contact_approximation(
        DiscreteContactApproximation::kSap);

    // Add model of the ground.
    if (ground_params) {
      const ProximityProperties ground_properties =
          MakeProximityProperties(*ground_params);
      plant_->RegisterCollisionGeometry(plant_->world_body(), RigidTransformd(),
                                        geometry::HalfSpace(),
                                        "ground_collision", ground_properties);
      const Vector4<double> green(0.5, 1.0, 0.5, 1.0);
      plant_->RegisterVisualGeometry(plant_->world_body(), RigidTransformd(),
                                     geometry::HalfSpace(), "ground_visual",
                                     green);
    }

    // Add models of the spheres.
    sphere1_ = &AddSphere(sphere1_params);
    sphere2_ = &AddSphere(sphere2_params);

    // Model with sphere 1 mounted on a prismatic joint with lower limits.
    if (sphere1_on_prismatic_joint) {
      slider1_ = &plant_->AddJoint<PrismaticJoint>(
          "Sphere1Slider", plant_->world_body(), std::nullopt, *sphere1_,
          std::nullopt, Vector3<double>::UnitZ(),
          sphere1_params.radius /* lower limit */);
    }

    plant_->mutable_gravity_field().set_gravity_vector(-gravity_ *
                                                       Vector3d::UnitZ());

    plant_->set_contact_model(
        drake::multibody::ContactModel::kHydroelasticWithFallback);

    plant_->Finalize();
    auto owned_contact_manager =
        std::make_unique<CompliantContactManager<double>>();
    contact_manager_ = owned_contact_manager.get();
    plant_->SetDiscreteUpdateManager(std::move(owned_contact_manager));

    diagram_ = builder.Build();
    diagram_context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &plant_->GetMyMutableContextFromRoot(diagram_context_.get());

    SetContactState(sphere1_params, sphere2_params);
  }

  // Sphere 1 is set on top of the ground and sphere 2 sits right on top of
  // sphere 1. We set the state of the model so that sphere 1 penetrates into
  // the ground a distance penetration_distance_ and so that sphere 1 and 2 also
  // interpenetrate a distance penetration_distance_.
  // MakeModel() must have already been called.
  void SetContactState(const SphereParameters& sphere1_params,
                       const SphereParameters& sphere2_params) const {
    DRAKE_DEMAND(plant_ != nullptr);
    const double sphere1_com_z = sphere1_params.radius - penetration_distance_;
    if (slider1_ != nullptr) {
      slider1_->set_translation(plant_context_, sphere1_com_z);
    } else {
      const RigidTransformd X_WB1(Vector3d(0, 0, sphere1_com_z));
      plant_->SetFreeBodyPose(plant_context_, *sphere1_, X_WB1);
    }
    const double sphere2_com_z = 2.0 * sphere1_params.radius +
                                 sphere2_params.radius -
                                 2.0 * penetration_distance_;
    const RigidTransformd X_WB2(Vector3d(0, 0, sphere2_com_z));
    plant_->SetFreeBodyPose(plant_context_, *sphere2_, X_WB2);
  }

  const MultibodyPlant<double>& plant() const { return *plant_; }
  const CompliantContactManager<double>& manager() const {
    return *contact_manager_;
  }

 protected:
  // Arbitrary positive value so that the model is discrete.
  double time_step_{0.001};

  const double gravity_{10.0};  // Acceleration of gravity, in m/s².

  // Default penetration distance. The configuration of the model is set so that
  // ground/sphere1 and sphere1/sphere2 interpenetrate by this amount.
  const double penetration_distance_{1.0e-3};

  std::unique_ptr<systems::Diagram<double>> diagram_;
  MultibodyPlant<double>* plant_{nullptr};
  SceneGraph<double>* scene_graph_{nullptr};
  const RigidBody<double>* sphere1_{nullptr};
  const RigidBody<double>* sphere2_{nullptr};
  const PrismaticJoint<double>* slider1_{nullptr};
  CompliantContactManager<double>* contact_manager_{nullptr};
  std::unique_ptr<Context<double>> diagram_context_;
  Context<double>* plant_context_{nullptr};

 private:
  // Helper to add a spherical body into the model.
  const RigidBody<double>& AddSphere(const SphereParameters& params) {
    // Add rigid body.
    const SpatialInertia<double> M_BBcm =
        SpatialInertia<double>::SolidSphereWithMass(params.mass, params.radius);
    const RigidBody<double>& body = plant_->AddRigidBody(params.name, M_BBcm);

    // Add collision geometry.
    if (params.contact_parameters) {
      const geometry::Sphere shape(params.radius);
      const ProximityProperties properties =
          MakeProximityProperties(*params.contact_parameters);
      plant_->RegisterCollisionGeometry(body, RigidTransformd(), shape,
                                        params.name + "_collision", properties);
    }

    return body;
  }

  // Utility to make ProximityProperties from ContactParameters.
  // params.relaxation_time is ignored if nullopt.
  static ProximityProperties MakeProximityProperties(
      const ContactParameters& params) {
    DRAKE_DEMAND(params.point_stiffness || params.hydro_modulus);
    ProximityProperties properties;
    if (params.hydro_modulus) {
      if (params.hydro_modulus == std::numeric_limits<double>::infinity()) {
        geometry::AddRigidHydroelasticProperties(/* resolution_hint */ 1.0,
                                                 &properties);
      } else {
        geometry::AddCompliantHydroelasticProperties(
            /* resolution_hint */ 1.0, *params.hydro_modulus, &properties);
      }
      // N.B. Add the slab thickness property by default so that we can model a
      // half space (either compliant or rigid).
      properties.AddProperty(geometry::internal::kHydroGroup,
                             geometry::internal::kSlabThickness, 1.0);
    }
    geometry::AddContactMaterial(
        /* "hunt_crossley_dissipation" */ {}, params.point_stiffness,
        CoulombFriction<double>(params.friction_coefficient,
                                params.friction_coefficient),
        &properties);

    if (params.relaxation_time.has_value()) {
      properties.AddProperty(geometry::internal::kMaterialGroup,
                             "relaxation_time", *params.relaxation_time);
    }
    return properties;
  }
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
