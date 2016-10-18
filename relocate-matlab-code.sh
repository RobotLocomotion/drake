#!/bin/bash

# Move a file (or glob) into the drake/mablab subdirectory.
function matlabify {
    dir=$(dirname $1)
    test -d drake/matlab/$dir/
    git mv drake/$1 drake/matlab/$dir/
}

# Twain a CMakeLists.txt into the drake/mablab subdirectory.
function matlabify_cmake {
    dir=$1
    test -f drake/$dir/CMakeLists.txt
    cp -a drake/$1/CMakeLists.txt drake/matlab/$dir/CMakeLists.txt
    perl -pi -e 'BEGIN { undef $/; } s|# ==== Below this line is C.. and MATLAB joint code ===\n\n||s; s|\n*# ==== Below this line is MATLAB-specific code ===.*|\n|s;' drake/$1/CMakeLists.txt
    perl -pi -e 'BEGIN { undef $/; } s|.*# ==== Below this line is C.. and MATLAB joint code ===\n*||s; s|# ==== Below this line is MATLAB-specific code ===\n*||s;' drake/matlab/$1/CMakeLists.txt
    git add drake/$1/CMakeLists.txt drake/matlab/$1/CMakeLists.txt
}

set -ex

# Confirm drake-distro as cwd.
test -f CHANGELOG.md

# Move the relevant util files ...
mkdir drake/matlab/util
mkdir drake/matlab/util/test

# ... whole directories ...
matlabify util/geometry
matlabify util/+meshutil
matlabify util/dev
matlabify util/visualization

# ... glob files ...
matlabify util/*.java
matlabify util/*.m
matlabify util/*Mex*.cpp
matlabify util/*Mex*.h
matlabify util/*mex.cpp
matlabify util/test/*.m
matlabify util/test/*mex.cpp

# ... one-off files ...
matlabify util/.valgrind_suppressions
matlabify util/LCMScope.h
matlabify util/barycentricInterpolation.cpp
matlabify util/check_multicast_is_loopback.sh
matlabify util/drake.mdl
matlabify util/drake.mdl.r2010b
matlabify util/getKey.cpp
matlabify util/lcmLogger.cpp
matlabify util/makeFunction.h
matlabify util/mexify.h
matlabify util/publishLCMLog.cpp
matlabify util/realtime.cpp
matlabify util/rotmat2rotationType
matlabify util/setup_loopback_multicast.sh
matlabify util/swfstop.swf
matlabify util/three_dof_force.urdf

# ... duplicate the suppressions.
cp -a drake/util/CPPLINT.cfg drake/matlab/util/CPPLINT.cfg
git add drake/matlab/util/CPPLINT.cfg

# Twain the build system.
matlabify_cmake util
matlabify_cmake util/test

# Do a file-extensions sanity check.
echo "Leftover files ..."
find drake -name examples -prune -o -name thirdParty -prune -o -name matlab -prune -o -name doc -prune -o -name \*.m -print
echo "... DONE"

echo "Success!"
