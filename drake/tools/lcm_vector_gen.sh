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
    $drake/tools/lcm_vector_gen.py \
        --lcmtype-dir=$drake/lcmtypes \
        --cxx-dir=$mydir/gen \
        --namespace="$namespace" \
        --title="$title" "$@"
    $CLANG_FORMAT --style=file -i "$mydir"/gen/$snake*.h "$mydir"/gen/$snake*.cc
}

# Call the code generator to produce an LCM message, a translator, and
# a Drake BasicVector based on specifications contained within a YAML file.
#
# @param1 title -- used to create class/type names
# @param2 yaml_file --- the YAML specification of vector fields
gen_lcm_and_vector_from_yaml () {
    title="$1"
    yaml_file="$2"
    snake=$(echo "$title" | tr " " _)
    shift
    $drake/tools/lcm_vector_gen.py \
        --lcmtype-dir=$drake/lcmtypes \
        --cxx-dir=$mydir/gen \
        --namespace="$namespace" \
        --title="$title"  \
        --yaml_file="$yaml_file"
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
    $drake/tools/lcm_vector_gen.py \
        --cxx-dir=$mydir/gen \
        --namespace="$namespace" \
        --title="$title" "$@"
    $CLANG_FORMAT --style=file -i "$mydir"/gen/$snake*.h "$mydir"/gen/$snake*.cc
}

# Call the code generator to produce just a Drake BasicVector based on a YAML
# spec.
#
# @param1 title -- used to create class/type names
# @param2 yaml_file --- the YAML specification of vector fields
gen_vector_yaml () {
    title="$1"
    yaml_file="$2"
    snake=$(echo "$title" | tr " " _)
    $drake/tools/lcm_vector_gen.py \
        --cxx-dir=$mydir/gen \
        --namespace="$namespace" \
        --title="$title" \
        --yaml_file="$yaml_file"
    $CLANG_FORMAT --style=file -i "$mydir"/gen/$snake*.h "$mydir"/gen/$snake*.cc
}
