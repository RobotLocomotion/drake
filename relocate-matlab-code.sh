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
    perl -pi -e 'BEGIN { undef $/; } s|\n*# ==== Below this line is MATLAB-specific code ====.*# ==== Below this line is C.. and MATLAB shared code ====\n*|\n|s;' drake/$1/CMakeLists.txt
    perl -pi -e 'BEGIN { undef $/; } s|.*# ==== Below this line is MATLAB-specific code ====\n*||s; s|# ==== Below this line is C.. and MATLAB shared code ====\n*||s;' drake/matlab/$1/CMakeLists.txt
    git add drake/$1/CMakeLists.txt drake/matlab/$1/CMakeLists.txt
}

set -ex

# Confirm drake-distro as cwd.
test -f CHANGELOG.md

# Move the relevant systems files ...
mkdir drake/matlab/systems
mkdir drake/matlab/systems/controllers
mkdir drake/matlab/systems/plants
mkdir drake/matlab/systems/plants/collision
mkdir drake/matlab/systems/plants/collision/test
mkdir drake/matlab/systems/plants/constraint
mkdir drake/matlab/systems/plants/constraint/test
mkdir drake/matlab/systems/plants/joints
mkdir drake/matlab/systems/plants/joints/test
mkdir drake/matlab/systems/plants/test
mkdir drake/matlab/systems/robotInterfaces
mkdir drake/matlab/systems/test
mkdir drake/matlab/systems/trajectories
mkdir drake/matlab/systems/trajectories/test

# ... whole directories ...
matlabify systems/@DrakeSystem
matlabify systems/@DynamicalSystem
matlabify systems/@HybridDrakeSystem
matlabify systems/@LyapunovFunction
matlabify systems/@PolynomialLyapunovFunction
matlabify systems/@PolynomialSystem
matlabify systems/@PolynomialTrajectorySystem
matlabify systems/controllers/+bipedControllers
matlabify systems/controllers/dev
matlabify systems/dev
matlabify systems/frames
matlabify systems/observers
matlabify systems/plants/@Manipulator
matlabify systems/plants/@PlanarRigidBodyManipulator
matlabify systems/plants/@RigidBodyManipulator
matlabify systems/plants/@RigidBodyWRLVisualizer
matlabify systems/plants/@RigidBodyWing
matlabify systems/plants/dev
matlabify systems/plants/test/testRootChange
matlabify systems/robotInterfaces/+footstepPlanner
matlabify systems/robotInterfaces/@Biped
matlabify systems/robotInterfaces/@LeggedRobot
matlabify systems/robotInterfaces/@QPLocomotionPlanCPPWrapper
matlabify systems/robotInterfaces/calibration
matlabify systems/robotInterfaces/test
matlabify systems/trajectories/dev
matlabify systems/trajectories/FunnelLibraries
matlabify systems/trajectories/TrajectoryLibraries
matlabify systems/visualizers

# ... glob files ...
matlabify systems/*.m
matlabify systems/controllers/*.m
matlabify systems/controllers/*mex.cpp
matlabify systems/plants/*.m
matlabify systems/plants/*mex.cpp
matlabify systems/plants/collision/*.m
matlabify systems/plants/collision/test/*.m
matlabify systems/plants/constraint/*.m
matlabify systems/plants/constraint/*mex.cpp
matlabify systems/plants/constraint/test/*.m
matlabify systems/plants/test/*.m
matlabify systems/plants/test/*mex.cpp
matlabify systems/robotInterfaces/*.m
matlabify systems/test/*.m
matlabify systems/trajectories/*.m
matlabify systems/trajectories/*mex.cpp
matlabify systems/trajectories/test/*.m

# ... one-off files ...
matlabify systems/DCSFunction.cpp
matlabify systems/controllers/constructQPDataPointerMex.cpp
matlabify systems/plants/constraint/constructPtrRigidBodyConstraint.cpp
matlabify systems/plants/constraint/constructPtrRigidBodyConstraint.h
matlabify systems/plants/constraint/test/valve_task_wall.urdf
matlabify systems/plants/joints/test/testDrakeJointsComparison.m
matlabify systems/plants/joints/test/testDrakeJointsmex.cpp
matlabify systems/plants/rigidBodyTreeMexConversions.h
matlabify systems/plants/rigidBodyTreeMexFunctions.cpp
matlabify systems/plants/rigidBodyTreeMexFunctions.h
matlabify systems/plants/test/ActuatedPendulum.urdf
matlabify systems/plants/test/Capsule.urdf
matlabify systems/plants/test/Cylinder.urdf
matlabify systems/plants/test/DoublePendWBiceptSpring.urdf
matlabify systems/plants/test/FallingBrick.urdf
matlabify systems/plants/test/FallingBrickBetterCollisionGeometry.urdf
matlabify systems/plants/test/FallingBrickContactPoints.urdf
matlabify systems/plants/test/FloatingMassSpringDamper.urdf
matlabify systems/plants/test/PointMass.urdf
matlabify systems/plants/test/ShiftedPointMass.urdf
matlabify systems/plants/test/SpringPendulum.urdf
matlabify systems/plants/test/TestWing.urdf
matlabify systems/plants/test/TorsionalSpring.urdf
matlabify systems/plants/test/ball.urdf
matlabify systems/plants/test/block_offset.urdf
matlabify systems/plants/test/brick1.urdf
matlabify systems/plants/test/brick2.urdf
matlabify systems/plants/test/brick3.urdf
matlabify systems/plants/test/brick4.urdf
matlabify systems/plants/test/brick_point_contact.urdf
matlabify systems/plants/test/fixedJointTest.urdf
matlabify systems/plants/test/ground_plane.urdf
matlabify systems/plants/test/linear_plant.mdl
matlabify systems/plants/test/regressionTestAtlasManipulatorDynamics.mat
matlabify systems/plants/test/snake.urdf
matlabify systems/plants/test/terrainTest.png
matlabify systems/plants/test/testRigidBodyBluffBody.urdf
matlabify systems/plants/test/testRigidBodyPropellor.urdf
matlabify systems/plants/test/testRigidBodyWingChangingParams.urdf
matlabify systems/plants/test/testRigidBodyWingWithControlSurface.urdf
matlabify systems/plants/test/testRigidBodyWingWithControlSurfaceOnlyControlSurface.urdf
matlabify systems/plants/test/testThrust.urdf
matlabify systems/plants/test/valve_task_wall.urdf
matlabify systems/robotInterfaces/constructQPLocomotionPlanmex.cpp
matlabify systems/robotInterfaces/footstepCollocationConstraintsMex.cpp
matlabify systems/robotInterfaces/octomapWrapper.cpp

# ... duplicate the suppressions.
cp -a drake/systems/CPPLINT.cfg drake/matlab/systems/CPPLINT.cfg
git add drake/matlab/systems/CPPLINT.cfg

# Twain the build system.
matlabify_cmake systems
matlabify_cmake systems/controllers
matlabify_cmake systems/plants
matlabify_cmake systems/plants/collision
matlabify_cmake systems/plants/collision/test
matlabify_cmake systems/plants/constraint
matlabify_cmake systems/plants/constraint/test
matlabify_cmake systems/plants/joints
matlabify_cmake systems/plants/joints/test
matlabify_cmake systems/plants/test
matlabify_cmake systems/robotInterfaces
matlabify_cmake systems/trajectories
matlabify_cmake systems/trajectories/test

# TODO(jwnimmer-tri) Patch to foopath_drake.m for built artifacts.

# Do a file-extensions sanity check.
echo "Leftover files ..."
find drake -name examples -prune -o -name thirdParty -prune -o -name matlab -prune -o -name doc -prune -o -name \*.m -print
echo "... DONE"

# TODO(jwnimmer-tri) Test with rm-rf matlab and make sure all C++ tests pass.

# TODO(jwnimmer-tri) Do we want to rework examples also?

echo "Success!"
