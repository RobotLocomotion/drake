#pragma once

#include <map>
#include <optional>
#include <set>
#include <string>
#include <variant>

#include <sdf/Element.hh>
#include <tinyxml2.h>

#include "drake/common/diagnostic_policy.h"
#include "drake/common/drake_copyable.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/parsing/detail_collision_filter_group_resolver.h"
#include "drake/multibody/plant/coulomb_friction.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/linear_bushing_roll_pitch_yaw.h"
#include "drake/multibody/tree/linear_spring_damper.h"
#include "drake/multibody/tree/spatial_inertia.h"

namespace drake {
namespace multibody {
namespace internal {

using ElementNode = std::variant<sdf::ElementPtr, tinyxml2::XMLElement*>;

// @returns true if @p str ends with @p ext. The match is case-insensitive.
bool EndsWithCaseInsensitive(std::string_view str, std::string_view ext);

// Helper class that provides for either a file name xor file contents to be
// passed between our various parsing functions.
class DataSource {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DataSource);

  // The result of calling GetStem on a file-contents data source.
  static constexpr char kContentsPseudoStem[] = "<literal-string>";

  // A data source contains either a file name, or file contents.
  enum DataSourceType { kFilename, kContents };

  // Depending on the DataSourceType value supplied, @p data will be treated as
  // either a file name or contents. The data is aliased, so the lifetime of
  // the passed data must exceed the lifetime of the created object.
  // @pre data cannot be nullptr.
  DataSource(DataSourceType type, const std::string* data);

  // @return true iff the data source is a file name.
  bool IsFilename() const { return type_ == kFilename; }

  // @return true iff the data source is file contents.
  bool IsContents() const { return type_ == kContents; }

  // Returns a reference to the filename.
  // @pre IsFilename().
  const std::string& filename() const;

  // Returns a reference to the contents.
  // @pre IsContents().
  const std::string& contents() const;

  // If the data source is a file name, returns its absolute path. If the
  // absolute path calculation causes errors, throw std::exception. Otherwise,
  // returns an empty string.
  std::string GetAbsolutePath() const;

  // If the data source is a file name, returns its parent path. If the parent
  // path calculation causes errors, throw std::exception. Otherwise, returns
  // an empty string.
  std::string GetRootDir() const;

  // If the data source is a file name, returns its base name, without
  // directory or extension. Otherwise, returns kContentsPseudoStem.
  std::string GetStem() const;

 private:
  DataSourceType type_{};
  const std::string* data_{};
};

// Note:
//   static global variables are strongly discouraged by the C++ style guide:
// http://drake.mit.edu/styleguide/cppguide.html#Static_and_Global_Variables
// For this reason, we create and return an instance of CoulombFriction
// instead of using a static variable.
// Default value of the Coulomb's law coefficients of friction for when they
// are not specified in the URDF/SDFormat file.
inline CoulombFriction<double> default_friction() {
  return CoulombFriction<double>(1.0, 1.0);
}

// Populates an instance of geometry::ProximityProperties from a reading
// interface in a URDF/SDFormat agnostic manner. This unifies the URDF and
// SDFormat parsing logic and eliminates code redundancy. The individual URDF
// and SDFormat parsers have the following responsibilities (based on the simple
// fact that the two parsers use different mechanisms to extract data from the
// file):
//
//   1. Determine if the `<drake:rigid_hydroelastic>` tag is present.
//   2. Determine if the `<drake:compliant_hydroelastic>` tag is present.
//   3. Create a function that will extract an *optional* double-valued scalar
//      from a <drake:some_property> child tag of the
//      <drake:proximity_properties> tag.
//
// This function does limited semantic parsing. For example, if for a
// particular application, a set of coordinated properties are required, this
// parsing method does *not* validate that set. It's sole purpose is to
// parse supported parameters and store them in expected values in the
// properties. Downstream consumers of those properties are responsible for
// confirming that all required properties are present and well formed.
//
// @param diagnostic   The error-reporting channel.
// @param read_double  The function for extracting double values for specific
//                     named tags.
// @param is_rigid     True if the caller detected the presence of the
//                     <drake:rigid_hydroelastic> tag.
// @param is_compliant True if the caller detected the presence of the
//                     <drake:compliant_hydroelastic> tag.
// @return All proximity properties discovered via the `read_double` function.
// @pre At most one of `is_rigid` and `is_compliant` is true.
geometry::ProximityProperties ParseProximityProperties(
    const drake::internal::DiagnosticPolicy& diagnostic,
    const std::function<std::optional<double>(const char*)>& read_double,
    bool is_rigid, bool is_compliant);

// Populates a LinearBushingRollPitchYaw from a reading interface in a
// URDF/SDFormat agnostic manner. This function does no semantic parsing and
// leaves the responsibility of handling errors or missing values to the
// individual parsers. All values are expected to exist and be well formed.
// Through this, the API to specify the linear_bushing_rpy tag in both SDFormat
// and URDF can be controlled/modified in a single function.
//
// __SDFormat__:
//
// <drake:linear_bushing_rpy>
//   <drake:bushing_frameA>frameA</drake:bushing_frameA>
//   <drake:bushing_frameC>frameC</drake:bushing_frameC>
//
//   <drake:bushing_torque_stiffness>0 0 0</drake:bushing_torque_stiffness>
//   <drake:bushing_torque_damping>0 0 0</drake:bushing_torque_damping>
//   <drake:bushing_force_stiffness>0 0 0</drake:bushing_force_stiffness>
//   <drake:bushing_force_damping>0 0 0</drake:bushing_force_damping>
// </drake:linear_bushing_rpy>
//
// __URDF__:
//
//
// <drake:linear_bushing_rpy>
//   <drake:bushing_frameA name="frameA"/>
//   <drake:bushing_frameC name="frameC"/>
//
//   <drake:bushing_torque_stiffness value="0 0 0"/>
//   <drake:bushing_torque_damping   value="0 0 0"/>
//   <drake:bushing_force_stiffness  value="0 0 0"/>
//   <drake:bushing_force_damping    value="0 0 0"/>
// </drake:linear_bushing_rpy>
//
// The @p read_frame functor may (at its option) throw std:exception, or return
// nullptr when frame parsing fails. Similarly,
// ParseLinearBushingRollPitchYaw() may return nullptr when read_frame has
// returned nullptr.
const LinearBushingRollPitchYaw<double>* ParseLinearBushingRollPitchYaw(
    const std::function<Eigen::Vector3d(const char*)>& read_vector,
    const std::function<const Frame<double>*(const char*)>& read_frame,
    MultibodyPlant<double>* plant);

// Populates a LinearSpringDamper from a reading interface in a URDF/SDFormat
// agnostic manner. This function does no semantic parsing and leaves the
// responsibility of handling errors or missing values to the individual
// parsers. All values are expected to exist and be well formed. Through this,
// the API to specify the linear_spring_damper tag in both SDFormat and URDF can
// be controlled/modified in a single function.
//
// __SDFormat__:
//
// <drake:linear_spring_damper>
//   <drake:linear_spring_damper_body_A>body_A</drake:linear_spring_damper_body_A>
//   <drake:linear_spring_damper_p_AP>0 0 0</drake:linear_spring_damper_p_AP>
//   <drake:linear_spring_damper_body_B>body_B</drake:linear_spring_damper_body_B>
//   <drake:linear_spring_damper_p_BQ>0 0 0</drake:linear_spring_damper_p_BQ>
//   <drake:linear_spring_damper_free_length>1.0</drake:linear_spring_damper_free_length>
//   <drake:linear_spring_damper_stiffness>1.0</drake:linear_spring_damper_stiffness>
//   <drake:linear_spring_damper_damping>1.0</drake:linear_spring_damper_damping>
// </drake:linear_spring_damper>
//
// __URDF__:
//
// <drake:linear_spring_damper>
//   <drake:linear_spring_damper_body_A name="body_A"/>
//   <drake:linear_spring_damper_p_AP value="0 0 0"/>
//   <drake:linear_spring_damper_body_B name="body_B"/>
//   <drake:linear_spring_damper_p_BQ value="0 0 0"/>
//   <drake:linear_spring_damper_free_length value="1.0"/>
//   <drake:linear_spring_damper_stiffness value="1.0"/>
//   <drake:linear_spring_damper_damping value="1.0"/>
// </drake:linear_spring_damper>
//
// Each of the various @p read_* functors may (at its option) emit diagnostic
// errors or warnings but should not throw. ParseLinearSpringDamper() may return
// nullptr at its option.
const LinearSpringDamper<double>* ParseLinearSpringDamper(
    const std::function<Eigen::Vector3d(const char*)>& read_vector,
    const std::function<const RigidBody<double>*(const char*)>& read_body,
    const std::function<std::optional<double>(const char*)>& read_double,
    MultibodyPlant<double>* plant);

// Adds a ball constraint to `plant` from a reading interface in a URDF/SDFormat
// agnostic manner. This function does no semantic parsing and leaves the
// responsibility of handling errors or missing values to the individual
// parsers. All values are expected to exist and be well formed. Through this,
// the API to specify the ball_constraint tag in both SDFormat and URDF can be
// controlled/modified in a single function.
//
// __SDFormat__:
//
// <drake:ball_constraint>
//   <drake:ball_constraint_body_A>body_A</drake:ball_constraint_body_A>
//   <drake:ball_constraint_body_B>body_B</drake:ball_constraint_body_B>
//   <drake:ball_constraint_p_AP>0 0 0</drake:ball_constraint_p_AP>
//   <drake:ball_constraint_p_BQ>0 0 0</drake:ball_constraint_p_BQ>
// </drake:ball_constraint>
//
// __URDF__:
//
// <drake:ball_constraint>
//   <drake:ball_constraint_body_A name="body_A"/>
//   <drake:ball_constraint_body_B name="body_B"/>
//   <drake:ball_constraint_p_AP value="0 0 0"/>
//   <drake:ball_constraint_p_BQ value="0 0 0"/>
// </drake:ball_constraint>
//
// The @p read_body functor may (at its option) throw std:exception, or return
// nullptr when body parsing fails. Similarly,
// ParseBallConstraint() may return nullopt when read_body has
// returned nullptr.
std::optional<MultibodyConstraintId> ParseBallConstraint(
    const std::function<Eigen::Vector3d(const char*)>& read_vector,
    const std::function<const RigidBody<double>*(const char*)>& read_body,
    MultibodyPlant<double>* plant);

// Adds a tendon constraint to `plant` from a reading interface in a
// URDF/SDFormat agnostic manner. This function validates that the specified
// joints exist in the model, but otherwise does no semantic parsing and leaves
// the responsibility of handling errors or missing values to the individual
// parsers. All values are expected to exist and be well formed. Through this,
// the API to specify the tendon_constraint tag in both SDFormat and URDF can be
// controlled/modified in a single function.
//
// __SDFormat__:
//
// <drake:tendon_constraint>
//   <drake:tendon_constraint_joint name='joint_A' a='10.0'/>
//   <drake:tendon_constraint_joint name='joint_B' a='20.0'/>
//   <drake:tendon_constraint_offset>0.5</drake:tendon_constraint_offset>
//   <drake:tendon_constraint_lower_limit>-1.0</drake:tendon_constraint_lower_limit>
//   <drake:tendon_constraint_upper_limit>1.0</drake:tendon_constraint_upper_limit>
//   <drake:tendon_constraint_stiffness>0.1</drake:tendon_constraint_stiffness>
//   <drake:tendon_constraint_damping>0.01</drake:tendon_constraint_damping>
// </drake:tendon_constraint>
//
// __URDF__:
//
// <drake:tendon_constraint>
//   <drake:tendon_constraint_joint name='joint_A' a='10.0'/>
//   <drake:tendon_constraint_joint name='joint_B' a='20.0'/>
//   <drake:tendon_constraint_offset value="0.5"/>
//   <drake:tendon_constraint_lower_limit value="-1.0"/>
//   <drake:tendon_constraint_upper_limit value="1.0"/>
//   <drake:tendon_constraint_stiffness value="0.1"/>
//   <drake:tendon_constraint_damping value="0.01"/>
// </drake:tendon_constraint>
//
// The various @p read_* functors may (at its option) emit diagnostic errors or
// warnings but should not throw. ParseTendonConstraint() may return nullopt at
// its option.
std::optional<MultibodyConstraintId> ParseTendonConstraint(
    const drake::internal::DiagnosticPolicy& diagnostic,
    ModelInstanceIndex model_instance, const ElementNode& constraint_node,
    const std::function<std::optional<double>(const char*)>& read_double,
    const std::function<ElementNode(const ElementNode&, const char*)>&
        next_child_element,
    const std::function<ElementNode(const ElementNode&, const char*)>&
        next_sibling_element,
    const std::function<std::string(const ElementNode&, const char*)>&
        read_string_attribute,
    const std::function<double(const ElementNode&, const char*)>&
        read_double_attribute,
    MultibodyPlant<double>* plant);

// TODO(@SeanCurtis-TRI): The real solution here is to create a wrapper
// class that provides a consistent interface to either representation.
// Then instantiate on the caller side and express the code here in terms of
// that type.
//
// Populates collision filter groups from a reading interface in a URDF/SDFormat
// agnostic manner. Through this, the API to specify the collision_filter_group
// tag in both SDFormat and URDF can be controlled/modified in a single
// function.  Functors are allowed to throw an exception when the requested
// quantities are not available.
// @param diagnostic            The error-reporting channel.
// @param model_instance        Model Instance that contains the bodies involved
//                              in the collision filter groups.
// @param model_node            Node used to parse for the
//                              collision_gilter_group tag.
// @param plant                 MultibodyPlant used to register the collision
//                              filter groups.
// @param resolver              Collects the collision filter group data.
// @param next_child_element    Function that returns the next child element
//                              with the specified tag in the ElementNode
//                              provided.
// @param next_sibling_element  Function that returns the next sibling element
//                              with the specified tag in the ElementNode
//                              provided.
// @param has_attribute         Function that checks if an attribute exists
//                              in the ElementNode provided.
// @param read_tag_string       Function that provides a common interface to
//                              extract a tag value. In SDFormat it will be a
//                              tag "value" (the attribute "name" will not be
//                              used), in URDF it will be a named attribute.
// @param read_string_attribute Function that reads a string attribute with the
//                              name provided in the ElementNoded provided.
// @param read_bool_attribute   Function that reads a boolean attribute with
//                              the name provided in the ElementNode provided.
void ParseCollisionFilterGroupCommon(
    const drake::internal::DiagnosticPolicy& diagnostic,
    ModelInstanceIndex model_instance, const ElementNode& model_node,
    MultibodyPlant<double>* plant,
    internal::CollisionFilterGroupResolver* resolver,
    const std::function<ElementNode(const ElementNode&, const char*)>&
        next_child_element,
    const std::function<ElementNode(const ElementNode&, const char*)>&
        next_sibling_element,
    const std::function<bool(const ElementNode&, const char*)>& has_attribute,
    const std::function<std::string(const ElementNode&, const char*)>&
        read_string_attribute,
    const std::function<bool(const ElementNode&, const char*)>&
        read_bool_attribute,
    const std::function<std::string(const ElementNode&, const char*)>&
        read_tag_string);

// This struct helps label and order the rotational inertia inputs at call
// sites of ParseSpatialInertia. Units of all quantities are kg⋅m².
//
// These quantities should have been parsed from the URDF
// `/robot/link/inertial/inertia` tag or from the SDFormat
// `//model/link/inertial/inertia` tag.
struct InertiaInputs {
  // Moments.
  double ixx{};
  double iyy{};
  double izz{};
  // Products of inertia.
  double ixy{};
  double ixz{};
  double iyz{};
};

// Combines the given user inputs into a SpatialInertia. When any data is
// invalid, emits a warning into the diagnostic policy and returns a
// best-effort approximation instead.
//
// The inertia math defines an "inertia frame" (called here Bi), with
// origin at the center of mass of the body, and axes arbitrary, but sometimes
// chosen to zero out the products of inertia.
//
// Conveniently, both URDF and SDFormat inputs support compatible formats for
// specifying the inertia frame. Callers should have previously constructed the
// transform `X_BBi` from the URDF `/robot/link/inertial/origin` tag or from
// the SDFormat `//model/link/inertial/pose` tag.
//
// @param diagnostic  The error-reporting channel.
// @param X_BBi       Pose of the inertia frame expressed in the body's frame.
// @param mass        Mass of the body.
// @param inertia_Bi_Bi  The moments and products of inertia about Bi's origin,
//                      expressed in frame Bi.
SpatialInertia<double> ParseSpatialInertia(
    const drake::internal::DiagnosticPolicy& diagnostic,
    const math::RigidTransformd& X_BBi, double mass,
    const InertiaInputs& inertia_Bi_Bi);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
