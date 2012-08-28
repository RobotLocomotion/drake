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
if checkDependency('eigen3_enabled')
%  mex('-g','HandCpmex.cpp',['-I',conf.eigen3_incdir]);
  mex('HandCpmex.cpp',['-I',conf.eigen3_incdir]);
end
cd ..

cd systems;
mex DCSFunction.cpp
cd ..

disp('done.');
