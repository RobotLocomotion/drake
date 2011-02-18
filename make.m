% Builds all mex files in the directory 

disp('compiling mex files...');

cd util;
mex realtime.cpp
cd ..

cd systems;
mex RLCSFunction.cpp
cd ..

disp('done.');