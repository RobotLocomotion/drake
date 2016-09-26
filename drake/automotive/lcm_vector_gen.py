#!/usr/bin/env python

"""Generate c++ and LCM definitions for the LCM Vector concept.
"""

import argparse
import os
import subprocess


def put(fileobj, text, newlines_after=0):
    fileobj.write(text.strip('\n') + '\n' * newlines_after)


INDICES_BEGIN = """
/// Describes the row indices of a %(camel)s.
struct DRAKEAUTOMOTIVE_EXPORT %(indices)s {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = %(nfields)d;

  // The index of each individual coordinate.
"""
INDICES_FIELD = """static const int %(kname)s = %(kvalue)d;"""
INDICES_FIELD_STORAGE = """const int %(indices)s::%(kname)s;"""
INDICES_END = """
};
"""

def to_kname(field):
    return 'k' + ''.join([
        word.capitalize()
        for word in field.split('_')])

def generate_indices(hh, caller_context, fields):
    """
    Args:
        fields is the list of fieldnames in the LCM message.
    """
    context = dict(caller_context)
    context.update(nfields = len(fields))
    context.update(kname = "kNumCoordinates")
    put(hh, INDICES_BEGIN % context, 1)
    for kvalue, field in enumerate(fields):
        # field is the LCM message field name
        # kname is the C++ kConstant name
        # kvalue is the C++ vector row index integer value
        context.update(kname = to_kname(field))
        context.update(kvalue = kvalue)
        put(hh, INDICES_FIELD % context, 1)
    put(hh, INDICES_END % context, 2)

def generate_indices_storage(cc, caller_context, fields):
    """
    Args:
        fields is the list of fieldnames in the LCM message.
    """
    context = dict(caller_context)
    context.update(nfields = len(fields))
    context.update(kname = "kNumCoordinates")
    put(cc, INDICES_FIELD_STORAGE % context, 1)
    for kvalue, field in enumerate(fields):
        # field is the LCM message field name
        # kname is the C++ kConstant name
        # kvalue is the C++ vector row index integer value
        context.update(kname = to_kname(field))
        context.update(kvalue = kvalue)
        put(cc, INDICES_FIELD_STORAGE % context, 1)
    put(cc, '', 1)


DEFAULT_CTOR = """
  /// Default constructor.  Sets all rows to zero.
  %(camel)s() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }
"""

def generate_default_ctor(hh, context, _):
    put(hh, DEFAULT_CTOR % context, 2)


ACCESSOR_BEGIN = """
  /// @name Getters and Setters
  //@{
"""
ACCESSOR = """
    const T %(field)s() const { return this->GetAtIndex(K::%(kname)s); }
    void set_%(field)s(const T& %(field)s) {
      this->SetAtIndex(K::%(kname)s, %(field)s);
    }
"""
ACCESSOR_END = """
  //@}
"""

def generate_accessors(hh, caller_context, fields):
    context = dict(caller_context)
    put(hh, ACCESSOR_BEGIN % context, 1)
    for field in fields:
        context.update(field = field)
        context.update(kname = to_kname(field))
        put(hh, ACCESSOR % context, 1)
    put(hh, ACCESSOR_END % context, 2)


VECTOR_HH_PREAMBLE = """
#pragma once

%(generated_code_warning)s

#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include "drake/drakeAutomotive_export.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace automotive {
"""

VECTOR_CLASS_BEGIN = """

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class %(camel)s : public systems::BasicVector<T> {
 public:
  // An abbreviation for our row index constants.
  typedef %(indices)s K;
"""

VECTOR_CLASS_END = """
};
"""

VECTOR_HH_POSTAMBLE = """
}  // namespace automotive
}  // namespace drake
"""

VECTOR_CC_PREAMBLE = """
#include "drake/automotive/gen/%(snake)s.h"

%(generated_code_warning)s

namespace drake {
namespace automotive {
"""

VECTOR_CC_POSTAMBLE = """
}  // namespace automotive
}  // namespace drake
"""

TRANSLATOR_HH_PREAMBLE = """
#pragma once

%(generated_code_warning)s

#include "drake/automotive/gen/%(snake)s.h"
#include "drake/drakeAutomotiveLcm_export.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"
#include "drake/lcmt_%(snake)s_t.hpp"

namespace drake {
namespace automotive {
"""

TRANSLATOR_CLASS_DECL = """
/**
 * Translates between LCM message objects and VectorBase objects for the
 * %(camel)s type.
 */
class DRAKEAUTOMOTIVELCM_EXPORT %(camel)sTranslator
    : public systems::lcm::LcmAndVectorBaseTranslator {
 public:
  %(camel)sTranslator()
      : LcmAndVectorBaseTranslator(%(indices)s::kNumCoordinates) {}
  std::unique_ptr<systems::BasicVector<double>> AllocateOutputVector()
      const override;
  void Deserialize(const void* lcm_message_bytes, int lcm_message_length,
      systems::VectorBase<double>* vector_base) const override;
  void Serialize(double time, const systems::VectorBase<double>& vector_base,
      std::vector<uint8_t>* lcm_message_bytes) const override;
};
"""

TRANSLATOR_HH_POSTAMBLE = """
}  // namespace automotive
}  // namespace drake
"""

TRANSLATOR_CC_PREAMBLE = """
#include "drake/automotive/gen/%(snake)s_translator.h"

%(generated_code_warning)s

#include <stdexcept>

#include "drake/common/drake_assert.h"

namespace drake {
namespace automotive {
"""

TRANSLATOR_CC_POSTAMBLE = """
}  // namespace automotive
}  // namespace drake
"""

ALLOCATE_OUTPUT_VECTOR = """
std::unique_ptr<systems::BasicVector<double>>
%(camel)sTranslator::AllocateOutputVector() const {
  return std::make_unique<%(camel)s<double>>();
}
"""


def generate_allocate_output_vector(cc, caller_context, fields):
    context = dict(caller_context)
    put(cc, ALLOCATE_OUTPUT_VECTOR % context, 2)

DESERIALIZE_BEGIN = """
void %(camel)sTranslator::Serialize(
    double time, const systems::VectorBase<double>& vector_base,
    std::vector<uint8_t>* lcm_message_bytes) const {
  const auto* const vector =
      dynamic_cast<const %(camel)s<double>*>(&vector_base);
  DRAKE_DEMAND(vector != nullptr);
  drake::lcmt_%(snake)s_t message;
  message.timestamp = static_cast<int64_t>(time * 1000);
"""
DESERIALIZE_FIELD = """
  message.%(field)s = vector->%(field)s();
"""
DESERIALIZE_END = """
  const int lcm_message_length = message.getEncodedSize();
  lcm_message_bytes->resize(lcm_message_length);
  message.encode(lcm_message_bytes->data(), 0, lcm_message_length);
}
"""


def generate_deserialize(cc, caller_context, fields):
    context = dict(caller_context)
    put(cc, DESERIALIZE_BEGIN % context, 1)
    for field in fields:
        context.update(field = field)
        put(cc, DESERIALIZE_FIELD % context, 1)
    put(cc, DESERIALIZE_END % context, 2)

SERIALIZE_BEGIN = """
void %(camel)sTranslator::Deserialize(
    const void* lcm_message_bytes, int lcm_message_length,
    systems::VectorBase<double>* vector_base) const {
  DRAKE_DEMAND(vector_base != nullptr);
  auto* const my_vector = dynamic_cast<%(camel)s<double>*>(vector_base);
  DRAKE_DEMAND(my_vector != nullptr);

  drake::lcmt_%(snake)s_t message;
  int status = message.decode(lcm_message_bytes, 0, lcm_message_length);
  if (status < 0) {
    throw std::runtime_error("Failed to decode LCM message %(snake)s.");
  }
"""
SERIALIZE_FIELD = """  my_vector->set_%(field)s(message.%(field)s);"""
SERIALIZE_END = """
}
"""


def generate_serialize(cc, caller_context, fields):
    context = dict(caller_context)
    put(cc, SERIALIZE_BEGIN % context, 1)
    for field in fields:
        context.update(field = field)
        put(cc, SERIALIZE_FIELD % context, 1)
    put(cc, SERIALIZE_END % context, 2)

LCMTYPE_PREAMBLE = """
%(generated_code_warning)s

package drake;

struct lcmt_%(snake)s_t {
  // The timestamp in milliseconds.
  int64_t timestamp;
"""

LCMTYPE_POSTAMBLE = """
}
"""


def generate_code(args):
    cxx_dir = os.path.abspath(args.cxx_dir)
    lcmtype_dir = os.path.abspath(args.lcmtype_dir)
    drake_dist_dir = subprocess.check_output(
        "git rev-parse --show-toplevel".split()).strip()

    title_phrase = args.title.split()
    camel = ''.join([x.capitalize() for x in title_phrase])
    snake = '_'.join([x.lower() for x in title_phrase])
    screaming_snake = '_'.join([x.upper() for x in title_phrase])

    # The context provides string substitutions for the C++ code blocks in the
    # literal strings throughout this program.
    context = dict()
    context.update(camel = camel)
    context.update(indices = camel + 'Indices')
    context.update(snake = snake)
    context.update(screaming_snake = screaming_snake)

    # This is a specially-formatted code block to warn users not to edit.
    # This disclaimer text is special-cased by our review tool, reviewable.io.
    disclaimer = "// GENERATED FILE " + "DO NOT EDIT"
    generator = os.path.abspath(__file__).replace(
        os.path.join(drake_dist_dir, ''), '')
    context.update(generated_code_warning = '\n'.join([
        disclaimer, "// See " + generator + "."]))

    with open(os.path.join(cxx_dir, "%s.h" % snake), 'w') as hh:
        put(hh, VECTOR_HH_PREAMBLE % context, 2)
        generate_indices(hh, context, args.fields)
        put(hh, VECTOR_CLASS_BEGIN % context, 2)
        generate_default_ctor(hh, context, args.fields)
        generate_accessors(hh, context, args.fields)
        put(hh, VECTOR_CLASS_END % context, 2)
        put(hh, VECTOR_HH_POSTAMBLE % context, 1)

    with open(os.path.join(cxx_dir, "%s.cc" % snake), 'w') as cc:
        put(cc, VECTOR_CC_PREAMBLE % context, 2)
        generate_indices_storage(cc, context, args.fields)
        put(cc, VECTOR_CC_POSTAMBLE % context, 1)

    with open(os.path.join(cxx_dir, "%s_translator.h" % snake), 'w') as hh:
        put(hh, TRANSLATOR_HH_PREAMBLE % context, 2)
        put(hh, TRANSLATOR_CLASS_DECL % context, 2)
        put(hh, TRANSLATOR_HH_POSTAMBLE % context, 1)

    with open(os.path.join(cxx_dir, "%s_translator.cc" % snake), 'w') as cc:
        put(cc, TRANSLATOR_CC_PREAMBLE % context, 2)
        generate_allocate_output_vector(cc, context, args.fields)
        generate_deserialize(cc, context, args.fields)
        generate_serialize(cc, context, args.fields)
        put(cc, TRANSLATOR_CC_POSTAMBLE % context, 1)

    with open(os.path.join(lcmtype_dir, "lcmt_%s_t.lcm" % snake), 'w') as lcm:
        put(lcm, LCMTYPE_PREAMBLE % context, 2)
        for field in args.fields:
            put(lcm, "  double %s;" % field, 1)
        put(lcm, LCMTYPE_POSTAMBLE % context, 1)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--cxx-dir', help="output directory for cxx files", default=".")
    parser.add_argument(
        '--lcmtype-dir', help="output directory for lcm file", default=".")
    parser.add_argument(
        '--title', help="title phrase, from which type names will be made")
    parser.add_argument(
        'fields', metavar='FIELD', nargs='+', help="field names for vector")
    args = parser.parse_args()
    generate_code(args)

if __name__ == "__main__":
    main()
