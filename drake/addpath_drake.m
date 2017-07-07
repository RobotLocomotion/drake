function [] = addpath_drake()
% Checks dependencies and sets up matlab path.
% Searches the machine for necessary support programs, and generates
% config.mat.  If required tools aren't found, it tries to be helpful in
% directing you to their location.
%

root = fileparts(mfilename('fullpath'));

if ~exist('drake_get_base_path','file')
  % The Drake build process produces some MATLAB files, and those files are
  % not already on the MATLAB path, so we need to find them first.
  addBuildProductsToPath(root);
end

if ~exist('drake_get_base_path','file')
  error(['The Drake build outputs are not on the MATLAB path, and could ' ...
         'not be auto-detected. Please add the build output directory ' ...
         '"matlab" to the path, then re-run addpath_drake.']);
end

if verLessThan('matlab','7.6')
  error('Drake requires MATLAB version 7.6 or above.');
  % because I rely on the new matlab classes with classdef
end

% add package directories to the matlab path
addpath(fullfile(root,'thirdParty','bsd','GetFullPath'));
addpath(fullfile(root,'thirdParty','bsd','polytopes'));
addpath(fullfile(root,'matlab','util'));
addpath(fullfile(root,'matlab','util','geometry'));
addpath(fullfile(root,'matlab','util','visualization'));


% OSX platform-specific
if (strcmp(computer('arch'),'maci64'))
  % Check if on Yosemite or after
  [OSXvers,~] = evalc('system(''sw_vers -productVersion'')');
  if ~isempty(regexp(OSXvers, '10\.1.', 'match'));
    % Check if reverted to IPv4
    ipv4_preferred = java.lang.System.getProperty('java.net.preferIPv4Stack');
    if isempty(ipv4_preferred)
      ipv4_preferred = 'false';
    end
    if ~(strcmp(ipv4_preferred,'true'))
      display('WARNING: Your JVM may crash if you do not set it to prefer IPv4 over IPv6.')
      display('This may cause any dependencies that involve the JVM (including LCM) to crash at runtime.')
      display('Please see bug report and solution here: https://github.com/RobotLocomotion/drake/issues/558.')
      display('(It just involves adding one line to your java.opts file for Matlab.)')
      display('Make sure to restart Matlab after editing your java.opts file.')
    end
  end
end

if ispc
  setenv('PATH',[getenv('PATH'),';',GetFullPath(drake_get_lib_path),';',fullfile(get_drake_binary_dir(),'lib','Release'),';',fullfile(get_drake_binary_dir(),'lib')]);
end


% Previously, we added .jar files to the classpath only when they were used.
% This introduced a number of issues because manipulating the java classpath
% clears all existing java variables. This would create problems when a user,
% for example, tried to load the LCMGL jar after creating an lcm object. Now,
% we instead just add all available .jars to the classpath at startup. See
% also https://github.com/mitdrc/drc/issues/2100
jarfiledir = fullfile(drake_get_base_path(), 'share', 'java');
if exist(jarfiledir, 'dir')
 javaaddpathIfNew(jarfiledir);
 for jarfile = dir(fullfile(jarfiledir, '*.jar'))';
   javaaddpathIfNew(fullfile(jarfiledir, jarfile.name));
 end
end

clear util/checkDependency;  % makes sure that the persistent variable in the dependency checker gets cleared
clear util/getDrakePath;

end

function javaaddpathIfNew(p)
 % Add a .jar to the dynamic java classpath only if it hasn't already been added
 if ~any(cellfun(@(x) strcmp(x, p), javaclasspath('-dynamic')))
   javaaddpathProtectGlobals(p);
 end
end

% Searches for the MATLAB outputs of the Drake build and adds them to the path.
function addBuildProductsToPath(root)
  % Common names for the installed MATLAB outputs directory.
  install_dir_names = {'build/install/matlab', ...
                       'build/matlab', ...
                       'drake-build/install/matlab'};

  % Search four directories up for an install directory.
  pfx='';
  for i=1:4
    for install_dir = install_dir_names
      full_path = fullfile(root, pfx, install_dir{1});
      if exist(full_path, 'file')
        disp(['Adding ', full_path, ' to the matlab path.']);
        addpath(full_path);
        return;
      end
    end
    pfx = fullfile('..',pfx);
  end
end
