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

% turn off autosave for simulink models (seems evil, but generating
% boatloads of autosaves is clearly worse)
if (com.mathworks.services.Prefs.getBooleanPref('SaveOnModelUpdate'))
  a = input('You currently have autosave enabled for simulink blocks.\nThis is fine, but will generate a lot of *.mdl.autosave files\nin your directory.  If you aren''t a regular Simulink user,\nthen I can disable that feature now.\n  Disable Simulink Autosave (y/n)? ', 's');
  if (lower(a(1))=='y')
    com.mathworks.services.Prefs.setBooleanPref('SaveOnModelUpdate',false);
  end
end
% todo: try setting this before simulating, then resetting it after the
% simulate?

% add package directories to the matlab path
addpath(fullfile(root));
addpath(fullfile(root,'systems'));
addpath(fullfile(root,'systems','plants'));
addpath(fullfile(root,'systems','plants','affordance'));
addpath(fullfile(root,'systems','plants','collision'));
addpath(fullfile(root,'systems','plants','constraint'));
addpath(fullfile(root,'systems','controllers'));
addpath(fullfile(root,'systems','observers'));
addpath(fullfile(root,'systems','trajectories'));
addpath(fullfile(root,'systems','frames'));
addpath(fullfile(root,'systems','visualizers'));
addpath(fullfile(root,'systems','robotInterfaces'));
addpath(fullfile(root,'solvers'));
addpath(fullfile(root,'solvers','trajectoryOptimization'));
addpath(fullfile(root,'util'));
addpath(fullfile(root,'util','obstacles'));
addpath(fullfile(root,'thirdParty'));
addpath(fullfile(root,'thirdParty','path'));
addpath(fullfile(root,'thirdParty','spatial'));
addpath(fullfile(root,'thirdParty','cprintf'));
addpath(fullfile(root,'thirdParty','GetFullPath'));

javaaddpath(fullfile(pods_get_base_path,'share','java','drake.jar'));

% check for all dependencies

v=ver('simulink');
if (isempty(v))
  error('Drake:SimulinkIsRequired','Drake requires simulink');
elseif verLessThan('simulink','7.3')
  warning('Drake:SimulinkVersion','Most features of Drake reguires SIMULINK version 7.3 or above.');
  % haven't actually tested with lower versions
end

clear util/checkDependency;  % makes sure that the persistent variable in the dependency checker gets cleared
clear util/getDrakePath; 

checkDependency('spotless'); % require spotless
tf = checkDependency('lcm'); % optional dependency on lcm, but load it now
tf = checkDependency('pathlcp'); % optional dependency, but load it now (since that's how it was being done before)

end
