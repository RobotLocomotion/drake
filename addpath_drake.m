function addpath_drake
% Checks dependencies and sets up matlab path.
% Searches the machine for necessary support programs, and generates
% config.mat.  If required tools aren't found, it tries to be helpful in
% directing you to their location.
%

conf.root = pwd;

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
addpath(fullfile(conf.root,'systems'));
addpath(fullfile(conf.root,'systems','plants'));
addpath(fullfile(conf.root,'systems','plants','affordance'));
addpath(fullfile(conf.root,'systems','plants','collision'));
addpath(fullfile(conf.root,'systems','plants','constraint'));
addpath(fullfile(conf.root,'systems','controllers'));
addpath(fullfile(conf.root,'systems','observers'));
addpath(fullfile(conf.root,'systems','trajectories'));
addpath(fullfile(conf.root,'systems','frames'));
addpath(fullfile(conf.root,'systems','visualizers'));
addpath(fullfile(conf.root,'systems','robotInterfaces'));
addpath(fullfile(conf.root,'solvers'));
addpath(fullfile(conf.root,'solvers','trajectoryOptimization'));
addpath(fullfile(conf.root,'util'));
addpath(fullfile(conf.root,'util','obstacles'));
addpath(fullfile(conf.root,'thirdParty'));
addpath(fullfile(conf.root,'thirdParty','path'));
addpath(fullfile(conf.root,'thirdParty','spatial'));
addpath(fullfile(conf.root,'thirdParty','cprintf'));
addpath(fullfile(conf.root,'thirdParty','GetFullPath'));

javaaddpath(fullfile(pods_get_base_path,'share','java','drake.jar'));

% check for all dependencies

v=ver('simulink');
if (isempty(v)) 
  conf.simulink_enabled = false;
elseif verLessThan('simulink','7.3')
  warning('Drake:SimulinkVersion','Most features of Drake reguires SIMULINK version 7.3 or above.');
  % haven't actually tested with lower versions
  conf.simulink_enabled = false;
else
  conf.simulink_enabled = true;
end

setenv('PATH_LICENSE_STRING', '1926793586&Courtesy&&&USR&54782&7_1_2014&1000&PATH&GEN&31_12_2015&0_0_0&5000&0_0');
  
try
  x = pathlcp(speye(500),-ones(500,1));
  valuecheck(x,ones(500,1));
  conf.pathlcp_enabled = true;
catch
  disp('The cached PATH license is out of date, and PATH will fail to solve larger problems. Please report this bug.');
  conf.pathlcp_enabled = false;
end

%conf.pathlcp_enabled = ~isempty(getenv('PATH_LICENSE_STRING'));
%if (~conf.pathlcp_enabled)
%  disp(' ');
%  disp(' The PATH LCP solver (in the thirdparty directory) needs you to get the setup the license: http://pages.cs.wisc.edu/~ferris/path.html');
%  disp(' I recommend adding a setenv(''PATH_LICENSE_STRING'',...) line to your startup.m');
%  disp(' The LCP solver will be disabled');
%  disp(' ');
%end

% save configuration options to config.mat
%conf
save([conf.root,'/util/drake_config.mat'],'conf');

%disp('To manually change any of these entries, use:')
%disp('  editDrakeConfig(param,value);');

clear util/checkDependency;  % makes sure that the persistent variable in the dependency checker gets cleared

checkDependency('spotless'); % require spotless 
tf = checkDependency('lcm'); % optional dependency on lcm, but load it now 

end



