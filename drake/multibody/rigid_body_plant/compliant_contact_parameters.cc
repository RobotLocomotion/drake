#include "drake/multibody/rigid_body_plant/compliant_contact_parameters.h"

#include <string>

#include "drake/thirdParty/zlib/tinyxml2/tinyxml2.h"

namespace drake {
namespace systems {

using std::to_string;
using tinyxml2::XMLElement;

double CompliantContactParameters::kDefaultStiffness = 10000.0;
double CompliantContactParameters::kDefaultDissipation = 2;
double CompliantContactParameters::kDefaultStaticFriction = 0.9;
double CompliantContactParameters::kDefaultDynamicFriction = 0.5;

void CompliantContactParameters::set_stiffness(double value) {
  if (value < 0) {
    throw std::runtime_error(
        "Stiffness value must be non-negative. Given " + to_string(value));
  }
  stiffness_ = value;
}

void CompliantContactParameters::set_dissipation(double value) {
  if (value < 0) {
    throw std::runtime_error(
        "Dissipation value must be non-negative. Given " + to_string(value));
  }
  dissipation_ = value;
}

void CompliantContactParameters::set_friction(double value) {
  ThrowForBadFriction(value, value);
  static_friction_ = dynamic_friction_ = value;
}

void CompliantContactParameters::set_friction(double static_friction,
                                              double dynamic_friction) {
  ThrowForBadFriction(static_friction, dynamic_friction);
  static_friction_ = static_friction;
  dynamic_friction_ = dynamic_friction;
}

void CompliantContactParameters::ThrowForBadFriction(double static_friction,
                                                     double dynamic_friction) {
  using std::to_string;
  using std::runtime_error;
  if (dynamic_friction < 0) {
    throw runtime_error("Given dynamic friction is negative: " +
                        to_string(dynamic_friction));
  }
  if (static_friction < 0) {
    throw runtime_error("Given static friction is negative: " +
                        to_string(dynamic_friction));
  }
  if (dynamic_friction > static_friction) {
    throw std::runtime_error("Given dynamic friction (" +
                             std::to_string(dynamic_friction) +
                             ") is greater than given static "
                             "friction (" +
                             to_string(static_friction) + "). Must be less.");
  }
}

void CompliantContactParameters::SetDefaultValues(
    const CompliantContactParameters &values) {
  // NOTE: This doesn't validate the friction values; it relies on the friction
  // setters on the instance to have already done this.
  kDefaultStiffness = values.stiffness();
  kDefaultDissipation = values.dissipation();
  kDefaultStaticFriction = values.static_friction();
  kDefaultDynamicFriction = values.dynamic_friction();
}

CompliantContactParameters ParseCollisionCompliance(XMLElement* node) {
  CompliantContactParameters parameters;
  XMLElement* compliant_node = node->FirstChildElement("drake_compliance");
  if (compliant_node) {
    tinyxml2::XMLElement* child;
    double static_friction{-1};
    double dynamic_friction{-1};
    // Encode the number of friction values read to give appropriate error
    // message. Zero and 2 are acceptable; 1 is an error.
    int friction_read = 0;
    for (child = compliant_node->FirstChildElement(); child;
         child = child->NextSiblingElement()) {
      std::string name(child->Value());
      double val;
      if (child->FirstChild()) {
        // NOTE: This is copied-and-pasted from xml_util.cc. I didn't want the
        // dependency on parsers to do this.
        std::string str(child->FirstChild()->Value());
        std::size_t num_chars = 0;
        val = std::stod(str, &num_chars);

        // Verifies that there are no additional characters after the double
        // value.
        if (str.size() != num_chars) {
          throw std::runtime_error(
              "ERROR: Double value contained additional characters after the "
              "number.");
        }
      } else {
        throw std::runtime_error("Drake compliant parameter \"" + name +
                                 "\" contains no value.");
      }
      if (name == "penetration_stiffness") {
        parameters.set_stiffness(val);
      } else if (name == "dissipation") {
        parameters.set_dissipation(val);
      } else if (name == "static_friction") {
        static_friction = val;
        ++friction_read;
      } else if (name == "dynamic_friction") {
        dynamic_friction = val;
        ++friction_read;
      }
      // TODO(SeanCurtis-TRI): Log warnings on unrecognized parameter names.
    }
    // Handle friction
    if (friction_read) {
      if (friction_read < 2) {
        throw std::runtime_error(
            "When specifying coefficient of friction, "
            "both static and dynamic coefficients must be defined");
      }
      parameters.set_friction(static_friction, dynamic_friction);
    }
  }
  return parameters;
}
}  // namespace systems
}  // namespace drake
