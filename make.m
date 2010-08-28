
disp('compiling mex files...');

cd shared;
mex realtime.cpp
cd ..

disp('done.');