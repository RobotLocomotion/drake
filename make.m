function make(varargin)

% Builds all mex files in the directory 

disp('zapping old mex files...');

flags = {'-O'};
%flags = {'-g'};

cd util;
delete(['*.',mexext]);
cd(getDrakePath());

cd systems/plants;
delete(['*.',mexext]);

cd @RigidBodyManipulator/private;
delete(['*.',mexext]);

cd ../../@PlanarRigidBodyManipulator/private;
delete(['*.',mexext]);

cd(getDrakePath());

if (nargin<1 || ~strcmp(varargin{1},'clean'))

disp('compiling mex files...');

%On Win64, John had to use the following lines (currently commented) and 
%comment out the lines beginning with mex to
%deal with an error in which the compiler could not find "simstruc.h"
%simulinkIncludeDir = ['-I"' matlabroot '\simulink\include"'];

load drake_config;

try 
  cd util;
  mex realtime.cpp
  cd(getDrakePath());

  if checkDependency('eigen3_enabled')
    eigenflags = {['-I',conf.eigen3_incdir]};
    cd systems/plants/;

    mex('-c','Model.cpp',flags{:},eigenflags{:});
    modelflags = {['Model.',objext]};
    mex('deleteModelmex.cpp',modelflags{:},flags{:},eigenflags{:});
    mex('HandCmex.cpp',modelflags{:},flags{:},eigenflags{:});
    mex('doKinematicsmex.cpp',modelflags{:},flags{:},eigenflags{:});
    mex('forwardKinmex.cpp',modelflags{:},flags{:},eigenflags{:});
    
    snoptflags = {'-lf2c','-lsnopt','-lsnopt_cpp','-lsnprint','-L/opt/local/lib/gcc48','-lgfortran','/Users/russt/mylocal/snopt7/cppsrc/snfilewrapper.o','/Users/russt/mylocal/snopt7/cppsrc/snoptProblem.o',['-I','/Users/russt/mylocal/snopt7/cppsrc'],'-I/opt/local/include','-L/opt/local/lib'};
    mex('inverseKinmex.cpp',modelflags{:},flags{:},eigenflags{:},snoptflags{:});
    % note: to make this run on my mac, i had to mv the libgfortran.3.dyld
    % in the /Applications/MATLAB_R2012a.app/sys/os/maci64 directory and
    % replace it with a symlink to the gcc48 version of the library.
    
    mex('deleteModelpmex.cpp',flags{:},eigenflags{:});
    mex('HandCpmex.cpp',flags{:},eigenflags{:});
    mex('doKinematicspmex.cpp',flags{:},eigenflags{:});
    mex('forwardKinpmex.cpp',flags{:},eigenflags{:});
    mex('forwardKinVelpmex.cpp',flags{:},eigenflags{:});

    cd @RigidBodyManipulator/private;
    modelflags = {fullfile('..','..',['Model.',objext]),eigenflags{:}};
    mex('constructModelmex.cpp',modelflags{:});
    
    cd ../../@PlanarRigidBodyManipulator/private;
    mex('constructModelpmex.cpp',eigenflags{:});
    
    cd(getDrakePath());
  end

  cd systems;
  mex DCSFunction.cpp
  cd(getDrakePath());
catch ex
  cd(getDrakePath());
  rethrow(ex);
end

end

disp('done.');
