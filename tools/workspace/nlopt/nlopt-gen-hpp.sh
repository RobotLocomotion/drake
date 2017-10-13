# Invoked by nlopt.BUILD to create api/nlopt.hpp.  Do not invoke directly.

# This script reimplements the CMake logic in api/CMakeLists.txt.  The goal is
# to take the enumerated integer constants from the C header nlopt.h and make
# them available to the C++ header nlopt.hpp.  The contents of nlopt.hpp are
# lines taken from nlopt.h that look like "NLOPT_FOO_BAR".  The NLOPT_ prefix
# is removed (because in C++ they are in a namespace).

set -ex

me="$0"
nlopt_in_hpp="$1"  # Template in -- copied to the output, with constants added.
nlopt_h="$2"       # Enums in -- scraped for constants to use.
nlopt_hpp="$3"     # Header out.

# Inputs should exist; outputs and intermediates should not.
[ -r "$nlopt_in_hpp" ]
[ -r "$nlopt_h" ]
[ ! -e "$nlopt_hpp" ]
[ ! -e algorithms.inc ]
[ ! -e results.inc ]
[ ! -e enums.inc ]
[ ! -e nlopt.hpp.tmp ]

# Filter api/nlopt.h ...
# ... lines matching "    NLOPT_FOO..." become "    FOO...".
# ... stop printing algorithms after "NUM_ALGORITHMS" ...
sed -n 's/^\(     *\)NLOPT_\([A-Z0-9_][A-Z0-9_]*\)/\1\2/p; /^ *NUM_ALGORITHMS/q' "$nlopt_h" > algorithms.inc

# Filter api/nlopt.h ...
# ... first match will be "FAILURE"; continue searching until EOF ...
# ... lines matching "    NLOPT_FOO..." become "    FOO...".
sed -n '/ NLOPT_FAILURE/,99999 s/^\(     *\)NLOPT_\([A-Z0-9_][A-Z0-9_]*\)/\1\2/p' "$nlopt_h" > results.inc

# Create the "GEN ENUMS HERE" content.
cat <<EOF >enums.inc
enum algorithm {
$(cat algorithms.inc)
};
enum result {
$(cat results.inc)
};
EOF

# When nlopt-in.hpp says "GEN_ENUMS_HERE", insert the enums.inc content.
sed '/GEN_ENUMS_HERE/ r enums.inc' "$nlopt_in_hpp" > nlopt.hpp.tmp
mv nlopt.hpp.tmp "$nlopt_hpp"

# Debugging output; change the 0 to 1 if you want to get a debugging dump.
head -n 120 algorithms.inc results.inc enums.inc "$nlopt_hpp"
exit 0
