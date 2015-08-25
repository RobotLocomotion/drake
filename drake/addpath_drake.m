function addpath_drake
% Checks dependencies and sets up matlab path.
% Searches the machine for necessary support programs, and generates
% config.mat.  If required tools aren't found, it tries to be helpful in
% directing you to their location.
%

root = pwd;

if ~exist('pods_get_base_path','file')
  % search up to 4 directories up for a build/matlab directory
  pfx='';
  for i=1:4
    if exist(fullfile(pwd,pfx,'build','matlab'),'file')
      disp(['Adding ', fullfile(pwd,pfx,'build','matlab'), ' to the matlab path']);
      addpath(fullfile(pwd,pfx,'build','matlab'));
      break;
    end
    pfx = fullfile('..',pfx);
  end
end

if ~exist('pods_get_base_path','file')
  error('You must run make first (and/or add your pod build/matlab directory to the matlab path)');
end

if verLessThan('matlab','7.6')
  error('Drake requires MATLAB version 7.6 or above.');
  % because I rely on the new matlab classes with classdef
end

% add package directories to the matlab path
addpath(fullfile(root,'systems'));
addpath(fullfile(root,'systems','plants'));
addpath(fullfile(root,'systems','plants','collision'));
addpath(fullfile(root,'systems','plants','constraint'));
addpath(fullfile(root,'systems','controllers'));
addpath(fullfile(root,'systems','observers'));
addpath(fullfile(root,'systems','trajectories'));
addpath(fullfile(root,'systems','frames'));
addpath(fullfile(root,'systems','visualizers'));
addpath(fullfile(root,'systems','robotInterfaces'));
addpath(fullfile(root,'systems','robotInterfaces','calibration'));
addpath(fullfile(root,'solvers'));
addpath(fullfile(root,'solvers','trajectoryOptimization'));
addpath(fullfile(root,'util'));
addpath(fullfile(root,'util','geometry'));
addpath(fullfile(root,'util','visualization'));
addpath(fullfile(root,'thirdParty'));
addpath(fullfile(root,'thirdParty','path'));
addpath(fullfile(root,'thirdParty','spatial'));
addpath(fullfile(root,'thirdParty','cprintf'));
addpath(fullfile(root,'thirdParty','GetFullPath'));
addpath(fullfile(root,'thirdParty','plotregion'));
addpath(fullfile(root,'thirdParty','polytopes'));
addpath(fullfile(root,'solvers','BMI'));
addpath(fullfile(root,'solvers','BMI','util'));
addpath(fullfile(root,'solvers','BMI','kinematics'));


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
  setenv('PATH',[getenv('PATH'),';',GetFullPath(pods_get_lib_path),';',fullfile(root,'pod-build','lib','Release'),';',fullfile(root,'pod-build','lib')]);
end


% Previously, we added .jar files to the classpath only when they were used.
% This introduced a number of issues because manipulating the java classpath
% clears all existing java variables. This would create problems when a user,
% for example, tried to load the LCMGL jar after creating an lcm object. Now,
% we instead just add all available .jars to the classpath at startup. See
% also https://github.com/mitdrc/drc/issues/2100
jarfiledir = fullfile(pods_get_base_path(), 'share', 'java');
if exist(jarfiledir, 'dir') 
 javaaddpathIfNew(jarfiledir);
 for jarfile = dir(fullfile(jarfiledir, '*.jar'))'; 
   javaaddpathIfNew(fullfile(jarfiledir, jarfile.name)); 
 end
end

clear util/checkDependency;  % makes sure that the persistent variable in the dependency checker gets cleared
clear util/getDrakePath;

% set up PATH LCP license
% NOTE: This license was granted exclusively for the use of PATH from the Drake matlab package.
% Any other use will be considered a violation of the license.  You can obtain a free license
% from here: http://pages.cs.wisc.edu/~ferris/path.html
setenv('PATH_LICENSE_STRING', '2096056969&Russ_Tedrake&Massachusetts_Institute_of_Technology&&USR&75042&18_4_2014&1000&PATH&GEN&0_0_0&0_0_0&5000&0_0');
end


function javaaddpathIfNew(p)
 % Add a .jar to the dynamic java classpath only if it hasn't already been added
 if ~any(cellfun(@(x) strcmp(x, p), javaclasspath('-dynamic')))
   javaaddpathProtectGlobals(p);
 end
end
