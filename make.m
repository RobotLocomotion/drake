function make(varargin)

% Builds all mex files in the directory 

%flags = {'-O'};
flags = {'-g'};

if (nargin>0 && strcmpi(varargin{1},'clean'))
  disp('deleting mex files...');

  cd util;
  delete(['*.',mexext]);
  cd(getDrakePath());

  cd systems/plants;
  delete(['*.',objext]);
  delete(['*.',mexext]);

  cd @RigidBodyManipulator/private;
  delete(['*.',mexext]);

  cd ../../@PlanarRigidBodyManipulator/private;
  delete(['*.',mexext]);

  cd(getDrakePath());
else

  disp('compiling mex files...');

  %On Win64, John had to use the following lines (currently commented) and
  %comment out the lines beginning with mex to
  %deal with an error in which the compiler could not find "simstruc.h"
  %simulinkIncludeDir = ['-I"' matlabroot '\simulink\include"'];
  
  load drake_config;
  
  try
    cd util;
    mexMakeRule('realtime.cpp',flags);
    cd(getDrakePath());
    
    if checkDependency('eigen3_enabled')
      eigenflags = {['-I',conf.eigen3_incdir]};
      cd systems/plants/;
      
      makeRule(['RigidBodyManipulator.',objext],{'RigidBodyManipulator.cpp','RigidBodyManipulator.h','RigidBody.h'},['mex -c RigidBodyManipulator.cpp ',sprintf('%s ',flags{:},eigenflags{:})]);
      modelflags = {['RigidBodyManipulator.',objext]};
      mexMakeRule('deleteModelmex.cpp',horzcat(modelflags,flags,eigenflags),modelflags);
      mexMakeRule('HandCmex.cpp',horzcat(modelflags,flags,eigenflags),modelflags);
      mexMakeRule('doKinematicsmex.cpp',horzcat(modelflags,flags,eigenflags),modelflags);
      mexMakeRule('forwardKinmex.cpp',horzcat(modelflags,flags,eigenflags),modelflags);
      
      %    SuperUsers: fix the snoptflags for your platform and uncomment.
      %    will try to make compilation robust soon.
      %
      %    snoptflags = {'-lf2c','-lsnopt','-lsnprint','-L/opt/local/lib/gcc48','-lgfortran','-I/Users/russt/mylocal/snopt7/cppsrc','-I/opt/local/include','-L/opt/local/lib'};
      %    mex('inverseKinmex.cpp',modelflags{:},flags{:},eigenflags{:},snoptflags{:});
      % note: to make this run on my mac, i had to mv the libgfortran.3.dyld
      % in the /Applications/MATLAB_R2012a.app/sys/os/maci64 directory and
      % replace it with a symlink to the gcc48 version of the library.
      
      mexMakeRule('deleteModelpmex.cpp',horzcat(flags,eigenflags),modelflags);
      mexMakeRule('HandCpmex.cpp',horzcat(flags,eigenflags),modelflags);
      mexMakeRule('doKinematicspmex.cpp',horzcat(flags,eigenflags),modelflags);
      mexMakeRule('forwardKinpmex.cpp',horzcat(flags,eigenflags),modelflags);
      mexMakeRule('forwardKinVelpmex.cpp',horzcat(flags,eigenflags),modelflags);
      
      modelflags = {fullfile('..','..',['RigidBodyManipulator.',objext]),eigenflags{:}};
      cd @RigidBodyManipulator/private;
      mexMakeRule('constructModelmex.cpp',horzcat(modelflags,flags),modelflags);
      
      cd ../../@PlanarRigidBodyManipulator/private;
      mexMakeRule('constructModelpmex.cpp',horzcat(flags,eigenflags));
      cd(getDrakePath());
    end
    
    cd systems;
    mexMakeRule('DCSFunction.cpp');
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
