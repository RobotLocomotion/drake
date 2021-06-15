"""Generate c++ and LCM definitions for the LCM Vector concept.
Run this script using Bazel.
"""

import argparse
import collections
import os
import subprocess

import yaml

from drake.tools.lint.clang_format import get_clang_format_path
from drake.tools.lint.find_data import find_data


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


INDICES_NAMES_ACCESSOR_DECL = """
  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `%(indices)s::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
"""
INDICES_END = """
};
"""

INDICIES_NAMES_ACCESSOR_IMPL_START = """
const std::vector<std::string>& %(camel)sIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
"""
INDICES_NAMES_ACCESSOR_IMPL_MID = """    \"%(name)s\",  // BR"""
INDICES_NAMES_ACCESSOR_IMPL_END = """  });
  return coordinates.access();
}"""


def generate_indices_names_accessor_decl(hh, caller_context):
    context = dict(caller_context)
    put(hh, INDICES_NAMES_ACCESSOR_DECL % context, 1)
    put(hh, INDICES_END, 2)


def generate_indices_names_accessor_impl(cc, caller_context, fields):
    """
    Args:
        fields is the list of fieldnames in the LCM message.
    """
    context = dict(caller_context)
    put(cc, INDICIES_NAMES_ACCESSOR_IMPL_START % context, 1)
    for kvalue, field in enumerate(fields):
        context.update(name=field['name'])
        put(cc, INDICES_NAMES_ACCESSOR_IMPL_MID % context, 1)
    put(cc, INDICES_NAMES_ACCESSOR_IMPL_END % context, 2)


# A default constructor with field-by-field setting.
DEFAULT_CTOR_CUSTOM_BEGIN_API = """
  /// Default constructor.  Sets all rows to their default value:
"""
DEFAULT_CTOR_CUSTOM_FIELD_API = """
  /// @arg @c %(field)s defaults to %(default_value)s %(units_suffix)s.
"""
DEFAULT_CTOR_CUSTOM_BEGIN_BODY = """
  %(camel)s() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
"""
DEFAULT_CTOR_CUSTOM_FIELD_BODY = """
    this->set_%(field)s(%(default_value)s);
"""
DEFAULT_CTOR_CUSTOM_END = """
}
"""
DEFAULT_CTOR_FIELD_DUMMY_TOKEN = 'dummy'
DEFAULT_CTOR_FIELD_UNKNOWN_DOC_UNITS = 'unknown'


def generate_default_ctor(hh, caller_context, fields):
    put(hh, DEFAULT_CTOR_CUSTOM_BEGIN_API % caller_context, 1)
    for field in fields:
        context = dict(caller_context)
        context.update(field=field['name'])
        default_value = field['default_value']
        if default_value == DEFAULT_CTOR_FIELD_DUMMY_TOKEN:
            default_value = "a dummy value"
        context.update(default_value=default_value)
        if field['doc_units'] == DEFAULT_CTOR_FIELD_UNKNOWN_DOC_UNITS:
            units_suffix = "with unknown units"
        else:
            units_suffix = field['doc_units']
        context.update(units_suffix=units_suffix)
        put(hh, DEFAULT_CTOR_CUSTOM_FIELD_API % context, 1)
    put(hh, DEFAULT_CTOR_CUSTOM_BEGIN_BODY % caller_context, 1)
    for field in fields:
        context = dict(caller_context)
        context.update(field=field['name'])
        default_value = field['default_value']
        if default_value == DEFAULT_CTOR_FIELD_DUMMY_TOKEN:
            default_value = "drake::dummy_value<T>::get()"
        context.update(default_value=default_value)
        put(hh, DEFAULT_CTOR_CUSTOM_FIELD_BODY % context, 1)
    put(hh, DEFAULT_CTOR_CUSTOM_END % caller_context, 2)


# The "rule of five methods" (but only four -- default dtor is fine).
COPY_AND_ASSIGN = """
  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  %(camel)s(const %(camel)s& other)
      : drake::systems::BasicVector<T>(other.values()) {}
  %(camel)s(%(camel)s&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  %(camel)s& operator=(const %(camel)s& other) {
    this->values() = other.values();
    return *this;
  }
  %(camel)s& operator=(%(camel)s&& other) noexcept {
    this->values() = std::move(other.values());
    other.values().resize(0);
    return *this;
  }
  //@}
"""


def generate_copy_and_assign(hh, caller_context):
    # Otherwise, emit a customized ctor.
    put(hh, COPY_AND_ASSIGN % caller_context, 2)


# SetToNamedVariables (for symbolic::Expression only).
SET_TO_NAMED_VARIABLES_BEGIN = """
  /// Create a symbolic::Variable for each element with the known variable
  /// name.  This is only available for T == symbolic::Expression.
  template <typename U=T>
  typename std::enable_if_t<std::is_same_v<U, symbolic::Expression>>
  SetToNamedVariables() {
"""
SET_TO_NAMED_VARIABLES_BODY = """
    this->set_%(field)s(symbolic::Variable("%(field)s"));
"""
SET_TO_NAMED_VARIABLES_END = """
}
"""


def generate_set_to_named_variables(hh, caller_context, fields):
    put(hh, SET_TO_NAMED_VARIABLES_BEGIN % caller_context, 1)
    for field in fields:
        context = dict(caller_context)
        context.update(field=field['name'])
        put(hh, SET_TO_NAMED_VARIABLES_BODY % context, 1)
    put(hh, SET_TO_NAMED_VARIABLES_END, 2)


DO_CLONE = """
  [[nodiscard]] %(camel)s<T>* DoClone() const final {
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
  const T& %(field)s() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::%(kname)s);
  }
  /// Setter that matches %(field)s().
  void set_%(field)s(const T& %(field)s) {
    ThrowIfEmpty();
    this->SetAtIndex(K::%(kname)s, %(field)s);
  }
  /// Fluent setter that matches %(field)s().
  /// Returns a copy of `this` with %(field)s set to a new value.
  [[nodiscard]] %(camel)s<T>
  with_%(field)s(const T& %(field)s) const {
    %(camel)s<T> result(*this);
    result.set_%(field)s(%(field)s);
    return result;
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
        put(hh, ACCESSOR_FIELD_METHODS % context, 1)
    put(hh, ACCESSOR_END % caller_context, 2)


SERIALIZE_BEGIN = """
  /// Visit each field of this named vector, passing them (in order) to the
  /// given Archive.  The archive can read and/or write to the vector values.
  /// One common use of Serialize is the //common/yaml tools.
  template <typename Archive>
  void Serialize(Archive* a) {
"""
SERIALIZE_FIELD = """
    T& %(field)s_ref = this->GetAtIndex(K::%(kname)s);
    a->Visit(drake::MakeNameValue("%(field)s", &%(field)s_ref));
"""
SERIALIZE_END = """
  }
"""


def generate_serialize(hh, caller_context, fields):
    put(hh, SERIALIZE_BEGIN % caller_context, 1)
    for field in fields:
        context = dict(caller_context)
        context.update(field=field['name'])
        context.update(kname=to_kname(field['name']))
        put(hh, SERIALIZE_FIELD % context, 1)
    put(hh, SERIALIZE_END % caller_context, 2)


GET_COORDINATE_NAMES = """
    /// See %(camel)sIndices::GetCoordinateNames().
    static const std::vector<std::string>& GetCoordinateNames() {
      return %(camel)sIndices::GetCoordinateNames();
   }
"""

# TODO(russt): Resolve names differences across the codebase. The vector gen
# scripts call this IsValid, but the system and solvers Constraint classes call
# it CheckSatisfied.
IS_VALID_BEGIN = """
  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
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


GET_ELEMENT_BOUNDS_BEGIN = """
  void GetElementBounds(Eigen::VectorXd* lower,
                        Eigen::VectorXd* upper) const final {
    const double kInf = std::numeric_limits<double>::infinity();
    *lower = Eigen::Matrix<double, %(nfields)s, 1>::Constant(-kInf);
    *upper = Eigen::Matrix<double, %(nfields)s, 1>::Constant(kInf);
"""
GET_ELEMENT_BOUNDS_LOWER = """
    (*lower)(K::%(kname)s) = %(min_value)s;
"""
GET_ELEMENT_BOUNDS_UPPER = """
    (*upper)(K::%(kname)s) = %(max_value)s;
"""
GET_ELEMENT_BOUNDS_END = """
  }
"""


def generate_get_element_bounds(hh, caller_context, fields):
    is_any_element_bounded = any(
        [field['min_value'] or field['max_value'] for field in fields])
    if not is_any_element_bounded:
        return
    context = dict(caller_context)
    context.update(nfields=len(fields))
    put(hh, GET_ELEMENT_BOUNDS_BEGIN % context, 1)
    for field in fields:
        context.update(kname=to_kname(field['name']))
        if field['min_value']:
            context.update(min_value=field['min_value'])
            put(hh, GET_ELEMENT_BOUNDS_LOWER % context, 1)
        if field['max_value']:
            context.update(max_value=field['max_value'])
            put(hh, GET_ELEMENT_BOUNDS_UPPER % context, 1)
    put(hh, GET_ELEMENT_BOUNDS_END % context, 2)


VECTOR_HH_PREAMBLE = """
#pragma once

%(generated_code_warning)s

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>

#include <Eigen/Core>

#include "drake/common/drake_bool.h"
#include "drake/common/dummy_value.h"
#include "drake/common/name_value.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/symbolic.h"
#include "drake/systems/framework/basic_vector.h"

%(opening_namespace)s
"""

VECTOR_CLASS_BEGIN = """

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class %(camel)s final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef %(indices)s K;
"""

VECTOR_CLASS_END = """
 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The %(camel)s vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};
"""

VECTOR_HH_POSTAMBLE = """
%(closing_namespace)s
"""

VECTOR_CC_PREAMBLE = """
#include "%(cxx_include_path)s/%(snake)s.h"

%(generated_code_warning)s

%(opening_namespace)s
"""

VECTOR_CC_POSTAMBLE = """
%(closing_namespace)s
"""


LCMTYPE_PREAMBLE = """
%(generated_code_warning)s

package %(lcm_package)s;

struct lcmt_%(snake)s_t {
  // The timestamp in milliseconds.
  int64_t timestamp;
"""

LCMTYPE_POSTAMBLE = """
}
"""


def _schema_filename_to_snake(named_vector_filename):
    basename = os.path.basename(named_vector_filename)
    assert basename.endswith("_named_vector.yaml")
    snake = basename[:-len("_named_vector.yaml")]
    return snake


def generate_code(
        named_vector_filename,
        include_prefix=None,
        vector_hh_filename=None,
        vector_cc_filename=None,
        lcm_filename=None):

    cxx_include_path = os.path.dirname(named_vector_filename) + "/gen"
    if cxx_include_path.startswith("external/"):
        # Drake is being used from within a different workspace, so we have to
        # strip the "external/drake/" from the named_vector_filename.  (The
        # name after "external" will vary depending on what name the workspace
        # gave us, so we can't hard-code it to "drake".)
        cxx_include_path = "/".join(cxx_include_path.split("/")[2:])
    if include_prefix:
        cxx_include_path = os.path.join(include_prefix, cxx_include_path)
    snake = _schema_filename_to_snake(named_vector_filename)
    screaming_snake = snake.upper()
    camel = "".join([x.capitalize() for x in snake.split("_")])

    # Load the vector's details.
    with open(named_vector_filename, 'r') as f:
        data = yaml.safe_load(f)
    fields = [{
        'name': str(el['name']),
        'doc': str(el.get('doc', '')),
        'default_value': str(el['default_value']),
        'doc_units': str(el.get('doc_units', '')),
        'min_value': str(el.get('min_value', '')),
        'max_value': str(el.get('max_value', '')),
    } for el in data['elements']]
    namespace_list = data['namespace'].split('::')

    # Default some field attributes if they are missing.
    for item in fields:
        if len(item['doc_units']) == 0:
            item['doc_units'] = DEFAULT_CTOR_FIELD_UNKNOWN_DOC_UNITS

    # The C++ namespace open & close dance is as requested in the
    # `*.named_vector.yaml` specification.
    opening_namespace = "".join(["namespace " + x + "{\n"
                                 for x in namespace_list])
    closing_namespace = "".join(["}  // namespace " + x + "\n"
                                 for x in reversed(namespace_list)])

    # The context provides string substitutions for the C++ code blocks in the
    # literal strings throughout this program.
    context = dict()
    context.update(cxx_include_path=cxx_include_path)
    context.update(camel=camel)
    context.update(indices=camel + 'Indices')
    context.update(snake=snake)
    context.update(screaming_snake=screaming_snake)
    context.update(opening_namespace=opening_namespace)
    context.update(closing_namespace=closing_namespace)
    context.update(lcm_package="drake")

    # This is a specially-formatted code block to warn users not to edit.
    # This disclaimer text is special-cased by our review tool, reviewable.io.
    disclaimer = "// GENERATED FILE " + "DO NOT EDIT"
    context.update(generated_code_warning='\n'.join([
        disclaimer, "// See drake/tools/lcm_vector_gen.py."]))

    cxx_names = []
    if vector_hh_filename:
        with open(vector_hh_filename, 'w') as hh:
            cxx_names.append(hh.name)
            put(hh, VECTOR_HH_PREAMBLE % context, 2)
            generate_indices(hh, context, fields)
            put(hh, '', 1)
            generate_indices_names_accessor_decl(hh, context)
            put(hh, VECTOR_CLASS_BEGIN % context, 2)
            generate_default_ctor(hh, context, fields)
            generate_copy_and_assign(hh, context)
            generate_set_to_named_variables(hh, context, fields)
            generate_do_clone(hh, context, fields)
            generate_accessors(hh, context, fields)
            generate_serialize(hh, context, fields)
            put(hh, GET_COORDINATE_NAMES % context, 2)
            generate_is_valid(hh, context, fields)
            generate_get_element_bounds(hh, context, fields)
            put(hh, VECTOR_CLASS_END % context, 2)
            put(hh, VECTOR_HH_POSTAMBLE % context, 1)

    if vector_cc_filename:
        with open(vector_cc_filename, 'w') as cc:
            cxx_names.append(cc.name)
            put(cc, VECTOR_CC_PREAMBLE % context, 2)
            generate_indices_storage(cc, context, fields)
            put(cc, '', 1)
            generate_indices_names_accessor_impl(cc, context, fields)
            put(cc, VECTOR_CC_POSTAMBLE % context, 1)

    if lcm_filename:
        with open(lcm_filename, 'w') as lcm:
            put(lcm, LCMTYPE_PREAMBLE % context, 2)
            for field in fields:
                put(lcm, "  double {};  // {}".format(field['name'],
                                                      field['doc']), 1)
            put(lcm, LCMTYPE_POSTAMBLE % context, 1)

    if cxx_names:
        # Run clang-format over all C++ files.  Inserting a .clang-format
        # settings file is problematic when formatting within bazel-genfiles,
        # so instead we pass its contents on the command line.
        with open(find_data(".clang-format"), "r") as f:
            yaml_data = yaml.safe_load(f)
            style = str(yaml_data)
            # For some reason, clang-format really wants lowercase booleans.
            style = style.replace("False", "false").replace("True", "true")
        subprocess.check_call(
            [get_clang_format_path(), "--style=" + style, "-i"] + cxx_names)


def generate_all_code(args):
    # Match srcs to outs.
    src_to_args = collections.OrderedDict()
    for one_src in args.srcs:
        snake = _schema_filename_to_snake(one_src)
        basename_to_kind = {
            snake + ".h": "vector_hh_filename",
            snake + ".cc": "vector_cc_filename",
            "lcmt_" + snake + "_t.lcm": "lcm_filename",
        }
        kwargs_for_generate = dict([
            (basename_to_kind[os.path.basename(one_out)], one_out)
            for one_out in args.outs
            if os.path.basename(one_out) in basename_to_kind
        ])
        if not kwargs_for_generate:
            print("warning: no outs matched for src " + one_src)
            continue
        kwargs_for_generate["include_prefix"] = args.include_prefix
        src_to_args[one_src] = kwargs_for_generate
    # Make sure all outs will be generated.
    covered_outs = set()
    for one_kwargs in src_to_args.values():
        for one_out in one_kwargs.values():
            covered_outs.add(one_out)
    missing_outs = set(args.outs) - covered_outs
    if missing_outs:
        print("error: could not find src for some outs:")
        for one_src in sorted(src_to_args.keys()):
            print("note: have src " + one_src)
        for one_out in sorted(covered_outs):
            print("note: match out " + one_out)
        for one_out in sorted(missing_outs):
            print("error: no src for out " + one_out)
        return 1

    # Do the one, one src at a time.
    for src, kwargs_for_generate in src_to_args.items():
        generate_code(src, **kwargs_for_generate)

    # Success.
    return 0


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--src', metavar="FILE", dest='srcs', action='append', default=[],
        help="'*_named_vector.yaml' description(s) of vector(s)")
    parser.add_argument(
        '--out', metavar="FILE", dest='outs', action='append', default=[],
        help="generated filename(s) to create")
    parser.add_argument(
        '--include_prefix', metavar="STR", default="",
        help="add to the start of include statement from our cc to our h")
    args = parser.parse_args()
    return generate_all_code(args)


if __name__ == "__main__":
    main()
