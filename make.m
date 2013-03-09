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
  mexPrint('realtime.cpp');
  cd(getDrakePath());

  if checkDependency('eigen3_enabled')
    eigenflags = {['-I',conf.eigen3_incdir]};
    cd systems/plants/;

    mexPrint('-c','Model.cpp',flags{:},eigenflags{:});
    modelflags = {['Model.',objext]};
    mexPrint('deleteModelmex.cpp',modelflags{:},flags{:},eigenflags{:});
    mexPrint('HandCmex.cpp',modelflags{:},flags{:},eigenflags{:});
    mexPrint('doKinematicsmex.cpp',modelflags{:},flags{:},eigenflags{:});
    mexPrint('forwardKinmex.cpp',modelflags{:},flags{:},eigenflags{:});

%    SuperUsers: fix the snoptflags for your platform and uncomment.
%    will try to make compilation robust soon.
%    
%    snoptflags = {'-lf2c','-lsnopt','-lsnprint','-L/opt/local/lib/gcc48','-lgfortran','-I/Users/russt/mylocal/snopt7/cppsrc','-I/opt/local/include','-L/opt/local/lib'};
%    mex('inverseKinmex.cpp',modelflags{:},flags{:},eigenflags{:},snoptflags{:});
    % note: to make this run on my mac, i had to mv the libgfortran.3.dyld
    % in the /Applications/MATLAB_R2012a.app/sys/os/maci64 directory and
    % replace it with a symlink to the gcc48 version of the library.
    
    mexPrint('deleteModelpmex.cpp',flags{:},eigenflags{:});
    mexPrint('HandCpmex.cpp',flags{:},eigenflags{:});
    mexPrint('doKinematicspmex.cpp',flags{:},eigenflags{:});
    mexPrint('forwardKinpmex.cpp',flags{:},eigenflags{:});
    mexPrint('forwardKinVelpmex.cpp',flags{:},eigenflags{:});

    cd @RigidBodyManipulator/private;
    modelflags = {fullfile('..','..',['Model.',objext]),eigenflags{:}};
    mexPrint('constructModelmex.cpp',modelflags{:});
    
    cd ../../@PlanarRigidBodyManipulator/private;
    mexPrint('constructModelpmex.cpp',eigenflags{:});
    
    cd(getDrakePath());
  end

  cd systems;
  mexPrint DCSFunction.cpp
  cd(getDrakePath());
catch ex
  cd(getDrakePath());
  rethrow(ex);
end

end

disp('done.');


end

function mexPrint(varargin)
  disp(['mex ',sprintf('%s ',varargin{:})]);
  mex(varargin{:});
end
