function make

% Builds all mex files in the directory 

%On Win64, John had to use the following lines (currently commented) and 
%comment out the lines beginning with mex to
%deal with an error in which the compiler could not find "simstruc.h"

disp('compiling mex files...');
%simulinkIncludeDir = ['-I"' matlabroot '\simulink\include"'];

load drake_config;

cd util;
mex realtime.cpp
cd(getDrakePath());

if checkDependency('eigen3_enabled')
  cd systems/plants/@RigidBodyManipulator/private;
  mex('HandCmex.cpp',['-I',conf.eigen3_incdir]);
  mex('doKinematicsmex.cpp',['-I',conf.eigen3_incdir]);
  mex('forwardKinmex.cpp',['-I',conf.eigen3_incdir]);
  cd(getDrakePath());
  
  cd systems/plants/@PlanarRigidBodyManipulator/private;
  mex('HandCpmex.cpp',['-I',conf.eigen3_incdir]);
  mex('doKinematicspmex.cpp',['-I',conf.eigen3_incdir]);
  mex('forwardKinpmex.cpp',['-I',conf.eigen3_incdir]);
  mex('forwardKinVelpmex.cpp',['-I',conf.eigen3_incdir]);
  cd(getDrakePath());
end

cd systems;
mex DCSFunction.cpp
cd(getDrakePath());

disp('done.');
