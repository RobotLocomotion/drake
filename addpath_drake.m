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
addpath(fullfile(root,'thirdParty'));
addpath(fullfile(root,'thirdParty','path'));
addpath(fullfile(root,'thirdParty','spatial'));
addpath(fullfile(root,'thirdParty','cprintf'));
addpath(fullfile(root,'thirdParty','GetFullPath'));

javaaddpath(fullfile(pods_get_base_path,'share','java','drake.jar'));

if ispc 
  setenv('PATH',[getenv('PATH'),';',GetFullPath(pods_get_lib_path),';',fullfile(root,'pod-build','lib','Release'),';',fullfile(root,'pod-build','lib')]);
end


clear util/checkDependency;  % makes sure that the persistent variable in the dependency checker gets cleared
clear util/getDrakePath;

% set up PATH LCP license
% NOTE: This license was granted exclusively for the use of PATH from the Drake matlab package.
% Any other use will be considered a violation of the license.  You can obtain a free license
% from here: http://pages.cs.wisc.edu/~ferris/path.html
setenv('PATH_LICENSE_STRING', '2096056969&Russ_Tedrake&Massachusetts_Institute_of_Technology&&USR&75042&18_4_2014&1000&PATH&GEN&0_0_0&0_0_0&5000&0_0');

% turn off autosave for simulink models (seems evil, but generating
% boatloads of autosaves is clearly worse)
if checkDependency('simulink')
  autosave_options = get_param(0,'AutoSaveOptions');
  if autosave_options.SaveOnModelUpdate
    warning('Drake:DisablingSimulinkAutosave','Disabling autosave for simulink blocks (to avoid generating a lot of *.mdl.autosave files in your directory.  If you aren''t a regular Simulink user and don''t want this disabled, comment out this section in addpath_drake.\nTo disable this warning in the future, add\n   warning(''off'',''Drake:DisablingSimulinkAutosave'')\nto your matlab startup.m');
    autosave_options.SaveOnModelUpdate = false;
    set_param(0,'AutoSaveOptions',autosave_options);
  end
end
% todo: try setting this before simulating, then resetting it after the
% simulate?

end
