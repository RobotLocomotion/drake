"""Generate c++ and LCM definitions for the LCM Vector concept.
Run this script using Bazel.
"""

import argparse
import os

import google.protobuf.text_format

from drake.tools import named_vector_pb2


def put(fileobj, text, newlines_after=0):
    fileobj.write(text.strip('\n') + '\n' * newlines_after)


INDICES_BEGIN = """
/// Describes the row indices of a %(camel)s.
struct %(indices)s {
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
    context.update(nfields=len(fields))
    context.update(kname="kNumCoordinates")
    put(hh, INDICES_BEGIN % context, 1)
    for kvalue, field in enumerate(fields):
        # field is the LCM message field name
        # kname is the C++ kConstant name
        # kvalue is the C++ vector row index integer value
        # TODO(jwnimmer-tri) The per-field context.update for name, kname, doc,
        # etc. is copy-pasta'd throughout this file.  We should abbreviate it.
        context.update(kname=to_kname(field['name']))
        context.update(kvalue=kvalue)
        put(hh, INDICES_FIELD % context, 1)
    put(hh, INDICES_END % context, 2)


def generate_indices_storage(cc, caller_context, fields):
    """
    Args:
        fields is the list of fieldnames in the LCM message.
    """
    context = dict(caller_context)
    context.update(nfields=len(fields))
    context.update(kname="kNumCoordinates")
    put(cc, INDICES_FIELD_STORAGE % context, 1)
    for kvalue, field in enumerate(fields):
        # field is the LCM message field name
        # kname is the C++ kConstant name
        # kvalue is the C++ vector row index integer value
        context.update(kname=to_kname(field['name']))
        context.update(kvalue=kvalue)
        put(cc, INDICES_FIELD_STORAGE % context, 1)
    put(cc, '', 1)


# One variant of a default constructor (all zeros).  (Depending on the
# named_vector details, we will either use this variant or the subsequent one.)
DEFAULT_CTOR_ZEROS = """
  /// Default constructor.  Sets all rows to zero.
  %(camel)s() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }
"""
# A second variant of a default constructor (field-by-field setting).
DEFAULT_CTOR_CUSTOM_BEGIN_API = """
  /// Default constructor.  Sets all rows to their default value:
"""
DEFAULT_CTOR_CUSTOM_FIELD_API = """
  /// @arg @c %(field)s defaults to %(default_value)s in units of %(doc_units)s.
"""
DEFAULT_CTOR_CUSTOM_BEGIN_BODY = """
  %(camel)s() : systems::BasicVector<T>(K::kNumCoordinates) {
"""
DEFAULT_CTOR_CUSTOM_FIELD_BODY = """
    this->set_%(field)s(%(default_value)s);
"""
DEFAULT_CTOR_CUSTOM_END = """
}
"""
DEFAULT_CTOR_FIELD_DEFAULT_VALUE = '0.0'  # When not otherwise overridden.
DEFAULT_CTOR_FIELD_UNKNOWN_DOC_UNITS = 'unknown'


def generate_default_ctor(hh, caller_context, fields):
    # If all defaults are 0.0 and unit-less, then emit the simple ctor.
    if all([item['default_value'] == DEFAULT_CTOR_FIELD_DEFAULT_VALUE and
            item['doc_units'] == DEFAULT_CTOR_FIELD_UNKNOWN_DOC_UNITS
            for item in fields]):
        put(hh, DEFAULT_CTOR_ZEROS % caller_context, 2)
        return
    # Otherwise, emit a customized ctor.
    put(hh, DEFAULT_CTOR_CUSTOM_BEGIN_API % caller_context, 1)
    for field in fields:
        context = dict(caller_context)
        context.update(field=field['name'])
        context.update(default_value=field['default_value'])
        context.update(doc_units=field['doc_units'])
        put(hh, DEFAULT_CTOR_CUSTOM_FIELD_API % context, 1)
    put(hh, DEFAULT_CTOR_CUSTOM_BEGIN_BODY % caller_context, 1)
    for field in fields:
        context = dict(caller_context)
        context.update(field=field['name'])
        context.update(default_value=field['default_value'])
        put(hh, DEFAULT_CTOR_CUSTOM_FIELD_BODY % context, 1)
    put(hh, DEFAULT_CTOR_CUSTOM_END % caller_context, 2)


DO_CLONE = """
  %(camel)s<T>* DoClone() const override {
    return new %(camel)s;
  }
"""


def generate_do_clone(hh, context, _):
    put(hh, DO_CLONE % context, 2)


ACCESSOR_BEGIN = """
  /// @name Getters and Setters
  //@{
"""
ACCESSOR_FIELD_DOC = """
  /// %(doc)s
"""
ACCESSOR_FIELD_DOC_UNITS = """
  /// @note @c %(field)s is expressed in units of %(doc_units)s.
"""
ACCESSOR_FIELD_DOC_RANGE = """
  /// @note @c %(field)s has a limited domain of [%(min_doc)s, %(max_doc)s].
"""
ACCESSOR_FIELD_METHODS = """
  const T& %(field)s() const { return this->GetAtIndex(K::%(kname)s); }
  void set_%(field)s(const T& %(field)s) {
    this->SetAtIndex(K::%(kname)s, %(field)s);
  }
"""
ACCESSOR_END = """
  //@}
"""


def generate_accessors(hh, caller_context, fields):
    put(hh, ACCESSOR_BEGIN % caller_context, 1)
    for field in fields:
        context = dict(caller_context)
        context.update(field=field['name'])
        context.update(kname=to_kname(field['name']))
        context.update(doc=field['doc'])
        context.update(doc_units=field['doc_units'])
        put(hh, ACCESSOR_FIELD_DOC % context, 1)
        if context['doc_units'] != DEFAULT_CTOR_FIELD_UNKNOWN_DOC_UNITS:
            put(hh, ACCESSOR_FIELD_DOC_UNITS % context, 1)
        if field['min_value'] or field['max_value']:
            context.update(min_doc=(field['min_value'] or '-Inf'))
            context.update(max_doc=(field['max_value'] or '+Inf'))
            put(hh, ACCESSOR_FIELD_DOC_RANGE % context, 1)
        put(hh, ACCESSOR_FIELD_METHODS  % context, 1)
    put(hh, ACCESSOR_END % caller_context, 2)


IS_VALID_BEGIN = """
  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
"""
IS_VALID = """
    result = result && !isnan(%(field)s());
"""
IS_VALID_MIN_VALUE = """
    result = result && (%(field)s() >= T(%(min_value)s));
"""
IS_VALID_MAX_VALUE = """
    result = result && (%(field)s() <= T(%(max_value)s));
"""
IS_VALID_END = """
    return result;
  }
"""


def generate_is_valid(hh, caller_context, fields):
    put(hh, IS_VALID_BEGIN % caller_context, 1)
    for field in fields:
        context = dict(caller_context)
        context.update(field=field['name'])
        context.update(kname=to_kname(field['name']))
        put(hh, IS_VALID % context, 1)
        if field['min_value']:
            context.update(min_value=field['min_value'])
            put(hh, IS_VALID_MIN_VALUE % context, 1)
        if field['max_value']:
            context.update(max_value=field['max_value'])
            put(hh, IS_VALID_MAX_VALUE % context, 1)
    put(hh, IS_VALID_END % caller_context, 2)


VECTOR_HH_PREAMBLE = """
#pragma once

%(generated_code_warning)s

#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include "drake/systems/framework/basic_vector.h"

%(opening_namespace)s
"""

VECTOR_CLASS_BEGIN = """

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class %(camel)s : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef %(indices)s K;
"""

VECTOR_CLASS_END = """
};
"""

VECTOR_HH_POSTAMBLE = """
%(closing_namespace)s
"""

VECTOR_CC_PREAMBLE = """
#include "%(relative_cxx_dir)s/%(snake)s.h"

%(generated_code_warning)s

%(opening_namespace)s
"""

VECTOR_CC_POSTAMBLE = """
%(closing_namespace)s
"""

TRANSLATOR_HH_PREAMBLE = """
#pragma once

%(generated_code_warning)s

#include <memory>
#include <vector>

#include "%(relative_cxx_dir)s/%(snake)s.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"
#include "drake/lcmt_%(snake)s_t.hpp"

%(opening_namespace)s
"""

TRANSLATOR_CLASS_DECL = """
/**
 * Translates between LCM message objects and VectorBase objects for the
 * %(camel)s type.
 */
class %(camel)sTranslator
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
%(closing_namespace)s
"""

TRANSLATOR_CC_PREAMBLE = """
#include "%(relative_cxx_dir)s/%(snake)s_translator.h"

%(generated_code_warning)s

#include <stdexcept>

#include "drake/common/drake_assert.h"

%(opening_namespace)s
"""

TRANSLATOR_CC_POSTAMBLE = """
%(closing_namespace)s
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
        context.update(field=field['name'])
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
        context.update(field=field['name'])
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
    drake_dist_dir = args.workspace
    relative_cxx_dir = cxx_dir.replace(os.path.join(drake_dist_dir, ''), '')

    title_phrase = args.title.split()
    camel = ''.join([x.capitalize() for x in title_phrase])
    snake = '_'.join([x.lower() for x in title_phrase])
    screaming_snake = '_'.join([x.upper() for x in title_phrase])

    namespace = args.namespace.split("::")
    opening_namespace = "".join(["namespace " + x + "{\n" for x in namespace])
    closing_namespace = "".join(["}  // namespace " + x + "\n"
                                 for x in reversed(namespace)])

    if args.named_vector_file:
        # Load the field names and docstrings from protobuf.
        # In the future, this can be extended for nested messages.
        with open(args.named_vector_file, "r") as f:
            vec = named_vector_pb2.NamedVector()
            google.protobuf.text_format.Merge(f.read(), vec)
            fields = [{
                'name': el.name,
                'doc': el.doc,
                'default_value': el.default_value,
                'doc_units': el.doc_units,
                'min_value': el.min_value,
                'max_value': el.max_value,
                } for el in vec.element]
    else:
        # Parse the field names from the command line.
        fields = [{
            'name': x,
            'doc': x,
            'default_value': '',
            'doc_units': '',
            'min_value': '',
            'max_value': '',
        } for x in args.fields]

    # Default some field attributes if they are missing.
    for item in fields:
        if len(item['default_value']) == 0:
            item['default_value'] = DEFAULT_CTOR_FIELD_DEFAULT_VALUE
        if len(item['doc_units']) == 0:
            item['doc_units'] = DEFAULT_CTOR_FIELD_UNKNOWN_DOC_UNITS

    # The context provides string substitutions for the C++ code blocks in the
    # literal strings throughout this program.
    context = dict()
    context.update(relative_cxx_dir=relative_cxx_dir)
    context.update(camel=camel)
    context.update(indices=camel + 'Indices')
    context.update(snake=snake)
    context.update(screaming_snake=screaming_snake)
    context.update(opening_namespace=opening_namespace)
    context.update(closing_namespace=closing_namespace)

    # This is a specially-formatted code block to warn users not to edit.
    # This disclaimer text is special-cased by our review tool, reviewable.io.
    disclaimer = "// GENERATED FILE " + "DO NOT EDIT"
    context.update(generated_code_warning='\n'.join([
        disclaimer, "// See drake/tools/lcm_vector_gen.py."]))

    with open(os.path.join(cxx_dir, "%s.h" % snake), 'w') as hh:
        print "generating %s" % hh.name
        put(hh, VECTOR_HH_PREAMBLE % context, 2)
        generate_indices(hh, context, fields)
        put(hh, VECTOR_CLASS_BEGIN % context, 2)
        generate_default_ctor(hh, context, fields)
        generate_do_clone(hh, context, fields)
        generate_accessors(hh, context, fields)
        generate_is_valid(hh, context, fields)
        put(hh, VECTOR_CLASS_END % context, 2)
        put(hh, VECTOR_HH_POSTAMBLE % context, 1)

    with open(os.path.join(cxx_dir, "%s.cc" % snake), 'w') as cc:
        print "generating %s" % cc.name
        put(cc, VECTOR_CC_PREAMBLE % context, 2)
        generate_indices_storage(cc, context, fields)
        put(cc, VECTOR_CC_POSTAMBLE % context, 1)

    if args.lcmtype_dir:
        with open(os.path.join(cxx_dir, "%s_translator.h" % snake),
                  'w') as hh:
            put(hh, TRANSLATOR_HH_PREAMBLE % context, 2)
            put(hh, TRANSLATOR_CLASS_DECL % context, 2)
            put(hh, TRANSLATOR_HH_POSTAMBLE % context, 1)

        with open(os.path.join(cxx_dir, "%s_translator.cc" % snake),
                  'w') as cc:
            print "generating %s" % cc.name
            put(cc, TRANSLATOR_CC_PREAMBLE % context, 2)
            generate_allocate_output_vector(cc, context, fields)
            generate_deserialize(cc, context, fields)
            generate_serialize(cc, context, fields)
            put(cc, TRANSLATOR_CC_POSTAMBLE % context, 1)

        with open(os.path.join(lcmtype_dir, "lcmt_%s_t.lcm" % snake),
                  'w') as lcm:
            print "generating %s" % lcm.name
            put(lcm, LCMTYPE_PREAMBLE % context, 2)
            for field in fields:
                put(lcm, "  double {};  // {}".format(field['name'],
                                                      field['doc']), 1)
            put(lcm, LCMTYPE_POSTAMBLE % context, 1)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--cxx-dir', help="output directory for cxx files", default=".")
    parser.add_argument(
        '--workspace', help="Drake WORKSPACE root.", required=True)
    parser.add_argument(
        '--namespace', help="::-delimited enclosing namespace",
        default="drake")
    # By default, LCM output is disabled.
    parser.add_argument(
        '--lcmtype-dir', help="output directory for lcm file", default="")
    parser.add_argument(
        '--title', help="title phrase, from which type names will be made")
    parser.add_argument(
        '--named_vector_file',
        help="Protobuf description of vector, supersedes command-line fields")
    parser.add_argument(
        'fields', metavar='FIELD', nargs='*', help="field names for vector")
    args = parser.parse_args()
    generate_code(args)

if __name__ == "__main__":
    main()
