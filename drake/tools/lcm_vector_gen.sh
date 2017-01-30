#!/bin/bash

# Usage: source $drake/tools/lcm_vector_gen.sh

# Definitions for auto-generating Drake BasicVectors, LCM types, and
# translators between the two.

# Requires that the variables drake, mydir, and namespace are already defined.
# $drake is the top of the drake source tree, i.e. $drake/tools contains this
#        script.
# $mydir/gen is the directory to which the generated files to be written.
# $namespace is the ::-delimited set of namespaces in which the generated
#            C++ source code should reside.

if [ -z "$drake" ] || [ -z "$mydir" ] || [ -z "$namespace" ]
then
  echo "Required input missing."
  exit 1
fi

workspace=$(dirname "$drake")

mkdir -p $drake/lcmtypes
mkdir -p $mydir/gen

CLANG_FORMAT=${CLANG_FORMAT:-clang-format}
if ! type -p $CLANG_FORMAT > /dev/null ; then
    cat <<EOF
Cannot find $CLANG_FORMAT ; see installation instructions at:
http://drake.mit.edu/code_style_tools.html
EOF
    exit 1
fi

# Call the code generator to produce an LCM message, a translator, and
# a Drake BasicVector.
#
# @param1 title -- used to create class/type names
# @param... --- used to create field names for vector entries
gen_lcm_and_vector () {
    title="$1"
    snake=$(echo "$title" | tr " " _)
    shift
    bazel run //drake/tools:lcm_vector_gen -- \
        --lcmtype-dir=$drake/lcmtypes \
        --cxx-dir=$mydir/gen \
        --namespace="$namespace" \
        --workspace="$workspace" \
        --title="$title" "$@"
    $CLANG_FORMAT --style=file -i "$mydir"/gen/$snake*.h "$mydir"/gen/$snake*.cc
}

# Call the code generator to produce an LCM message, a translator, and
# a Drake BasicVector based on specifications contained in a NamedVector proto.
#
# @param1 title -- used to create class/type names
# @param2 named_vector_file --- the file containing a text NamedVector proto.
gen_lcm_and_vector_from_proto() {
    title="$1"
    named_vector_file="$2"
    snake=$(echo "$title" | tr " " _)
    shift
    bazel run //drake/tools:lcm_vector_gen -- \
        --lcmtype-dir=$drake/lcmtypes \
        --cxx-dir=$mydir/gen \
        --namespace="$namespace" \
        --title="$title"  \
        --workspace="$workspace" \
        --named_vector_file="$named_vector_file"
    $CLANG_FORMAT --style=file -i "$mydir"/gen/$snake*.h "$mydir"/gen/$snake*.cc
}

# Call the code generator to produce just a Drake BasicVector.
#
# @param1 title -- used to create class/type names
# @param... --- used to create field names for vector entries
gen_vector () {
    title="$1"
    snake=$(echo "$title" | tr " " _)
    shift
    bazel run //drake/tools:lcm_vector_gen -- \
        --cxx-dir=$mydir/gen \
        --namespace="$namespace" \
        --workspace="$workspace" \
        --title="$title" "$@"
    $CLANG_FORMAT --style=file -i "$mydir"/gen/$snake*.h "$mydir"/gen/$snake*.cc
}

# Call the code generator to produce just a Drake BasicVector based on a
# NamedVector proto spec.
#
# @param1 title -- used to create class/type names
# @param2 named_vector_file --- the NamedVector specification of vector fields
gen_vector_proto () {
    title="$1"
    named_vector_file="$2"
    snake=$(echo "$title" | tr " " _)
    bazel run //drake/tools:lcm_vector_gen -- \
        --cxx-dir=$mydir/gen \
        --namespace="$namespace" \
        --workspace="$workspace" \
        --title="$title" \
        --named_vector_file="$named_vector_file"
    $CLANG_FORMAT --style=file -i "$mydir"/gen/$snake*.h "$mydir"/gen/$snake*.cc
}
