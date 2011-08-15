% Builds all mex files in the directory 

%On Win64, John had to use the following lines (currently commented) and 
%comment out the lines beginning with mex to
%deal with an error in which the compiler could not find "simstruc.h"

disp('compiling mex files...');
%simulinkIncludeDir = ['-I"' matlabroot '\simulink\include"'];

cd util;
mex realtime.cpp
%eval(['mex ' simulinkIncludeDir ' realtime.cpp']);
cd ..

cd systems;
mex -g RLCSFunction.cpp
%eval(['mex -g ' simulinkIncludeDir ' RLCSFunction.cpp']);
cd ..

disp('done.');