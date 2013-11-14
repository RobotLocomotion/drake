function addpath_drake
% Checks dependencies and sets up matlab path.
% Searches the machine for necessary support programs, and generates
% config.mat.  If required tools aren't found, it tries to be helpful in
% directing you to their location.
%

try
  load drake_config.mat; 
catch
  conf=struct();
end
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
addpath(fullfile(conf.root,'solvers'));
addpath(fullfile(conf.root,'util'));
addpath(fullfile(conf.root,'util','obstacles'));
addpath(fullfile(conf.root,'thirdParty'));
addpath(fullfile(conf.root,'thirdParty','path'));
addpath(fullfile(conf.root,'thirdParty','spatial'));
addpath(fullfile(conf.root,'thirdParty','cprintf'));
addpath(fullfile(conf.root,'thirdParty','GetFullPath'));

javaaddpath(fullfile(pods_get_base_path,'share','java','drake.jar'));
javaaddpath(fullfile(pods_get_base_path,'share','java','lcmtypes_drake.jar'));

% todo: setup java classpath (not hard to do it once... but how can I set
% it up for future sessions as well?  maybe write to startup.m, or prompt
% user to do it?)

% check for all dependencies

v=ver('simulink');
if (isempty(v)) 
  conf.simulink_enabled = false;
elseif verLessThan('simulink','7.3')
  warning('Drake:SimulinkVersion','Some features of Drake reguires SIMULINK version 7.3 or above.');
  % haven't actually tested with lower versions
  conf.simulink_enabled = false;
else
  conf.simulink_enabled = true;
end

% require spotless 
if ~logical(exist('msspoly','class')) 
  pod_pkg_config('spotless');
end

conf.lcm_enabled = logical(exist('lcm.lcm.LCM','class'));
if (~conf.lcm_enabled)
  lcm_java_classpath = getCMakeParam('lcm_java_classpath');
  if ~isempty(lcm_java_classpath)
    javaaddpath(lcm_java_classpath);
    disp(' Added the lcm jar to your javaclasspath (found via cmake)');
    conf.lcm_enabled = logical(exist('lcm.lcm.LCM','class'));
  end
end

if (~conf.lcm_enabled)
  [retval,cp] = system('pkg-config --variable=classpath lcm-java');
  if (retval==0 && ~isempty(cp))
    disp(' Added the lcm jar to your javaclasspath (found via pkg-config)');
    javaaddpath(strtrim(cp));
  end

  conf.lcm_enabled = logical(exist('lcm.lcm.LCM','class'));
end

if (~conf.lcm_enabled)
  disp(' ');
  disp(' LCM not found.  LCM support will be disabled.');
  disp(' To re-enable, add lcm-###.jar to your matlab classpath');
  disp(' (e.g., by putting javaaddpath(''/usr/local/share/java/lcm-0.9.2.jar'') into your startup.m .');
  disp(' ');
end

conf.lcmgl_enabled = false;
if (conf.lcm_enabled)
  conf.lcmgl_enabled = logical(exist('bot_lcmgl.data_t','class'));
  
  if (~conf.lcmgl_enabled)
    try % try to add bot2-lcmgl.jar
      javaaddpath(fullfile(pods_get_base_path,'share','java','bot2-lcmgl.jar'));
    catch
    end
    conf.lcmgl_enabled = exist('bot_lcmgl.data_t','class');
  end
  if (~conf.lcmgl_enabled)
    disp(' ');
    disp(' LCMGL not found.  LCMGL support will be disabled.');
    disp(' To re-enable, add bot2-lcmgl.jar to your matlab classpath using javaaddpath.');
    disp(' ');
  end
end



conf.snopt_enabled = logical(exist('snopt','file'));
if (~conf.snopt_enabled) 
  conf.snopt_enabled = pod_pkg_config('snopt') && logical(exist('snopt','file'));
end

if (~conf.snopt_enabled) 
  disp(' ');
  disp(' SNOPT not found.  SNOPT support will be disabled.');
  disp(' To re-enable, add the SNOPT matlab folder to your path and rerun addpath_drake.');
  disp(' SNOPT can be obtained from <a href="https://tig.csail.mit.edu/software/">https://tig.csail.mit.edu/software/</a> .');
  disp(' ');
end

if(exist('vrinstall','file'))
  conf.vrml_enabled = logical(vrinstall('-check'));% && usejava('awt');  % usejava('awt') return 0 if running with no display
  if ismac
    [~,osx] = system('sw_vers -productVersion');
    if ~verStringLessThan(osx,'10.9') && verLessThan('matlab','8.1')
      % per my support ticket to matlab, who sent a perfunctory response
      % pointing to this: http://www.mathworks.com/support/sysreq/release2012a/macintosh.html
      conf.vrml_enabled = false;
    end
  end
else
  conf.vrml_enabled=0;
end

conf.sedumi_enabled = logical(exist('sedumi','file'));
if (~conf.sedumi_enabled)
  conf.sedumi_enabled = pod_pkg_config('sedumi') && logical(exist('sedumi','file'));
end

if (conf.sedumi_enabled)
%  sedumiA=[10,2,3,4;5,7,6,4];
%  sedumib=[4;6];
%  sedumic=[0;1;0;1];
%  sedumiT=sedumi(sedumiA,sedumib,sedumic);%,struct('f',4),struct('fid',0));
%  if(~sedumiT)
%    error('SeDuMi seems to have encountered a problem. Please verify that your SeDuMi install is working.');
%  end
else
  disp(' ');
  disp(' SeDuMi not found.  SeDuMi support will be disabled.');
  disp(' To re-enable, add SeDuMi to your matlab path and rerun addpath_drake.');
  disp(' SeDuMi can be downloaded for free from <a href="http://sedumi.ie.lehigh.edu/">http://sedumi.ie.lehigh.edu/</a> ');
  disp(' ');
end

conf.gurobi_enabled = logical(exist('gurobi','file')); %&& ~isempty(getenv('GUROBI_HOME')));
if (~conf.gurobi_enabled)
  conf.gurobi_enabled = pod_pkg_config('gurobi'); %&& ~isempty(getenv('GUROBI_HOME'));
end

if (~conf.gurobi_enabled)
  disp(' ');
  disp(' GUROBI not found. GUROBI support will be disabled.');
  disp('    To enable, install GUROBI and a free academic license from');
  disp('    <a href="http://www.gurobi.com/download/licenses/free-academic">http://www.gurobi.com/download/licenses/free-academic</a> .');
  disp(' Then, you will need to set several environment variables.');
  disp(' Please see <a href="http://drake.mit.edu/quickstart">http://drake.mit.edu/quickstart</a> for more info.');
  disp(' ');
end

conf.cplex_enabled = logical(exist('cplexlp','file'));
if (~conf.cplex_enabled)
  disp(' CPLEX not found.  CPLEX support will be disabled.  To re-enable, install CPLEX and add the matlab subdirectory to your matlab path, then rerun addpath_drake');
end

conf.yalmip_enabled = logical(exist('sdpvar','file'));
if (~conf.yalmip_enabled)
  disp(' YALMIP not found.  YALMIP support will be disabled.  To re-enable, install YALMIP and rerun addpath_drake.'); 
end

setenv('PATH_LICENSE_STRING','2069810742&Courtesy_License&&&USR&2013&14_12_2011&1000&PATH&GEN&31_12_2013&0_0_0&0&0_0');
conf.pathlcp_enabled = true;

%conf.pathlcp_enabled = ~isempty(getenv('PATH_LICENSE_STRING'));
%if (~conf.pathlcp_enabled)
%  disp(' ');
%  disp(' The PATH LCP solver (in the thirdparty directory) needs you to get the setup the license: http://pages.cs.wisc.edu/~ferris/path.html');
%  disp(' I recommend adding a setenv(''PATH_LICENSE_STRING'',...) line to your startup.m');
%  disp(' The LCP solver will be disabled');
%  disp(' ');
%end

% todo: add lcmgl_enabled (using pod_pkg_config)

conf.bullet_enabled = ~isempty(getCMakeParam('bullet'));

if ~isfield(conf,'avl') || isempty(conf.avl)
  path_to_avl = getCMakeParam('avl');
  if isempty(path_to_avl) || strcmp(path_to_avl,'avl-NOTFOUND')
    disp(' AVL support is disabled.  To enable it, install AVL from here: http://web.mit.edu/drela/Public/web/avl/, then add it to the matlab path or set the path to the avl executable explicitly using editDrakeConfig(''avl'',path_to_avl_executable) and rerun make');
    conf.avl = '';
  else
    conf.avl = path_to_avl;
  end
end
conf.avl_enabled = ~isempty(conf.avl);

if ~isfield(conf,'xfoil') || isempty(conf.xfoil)
  path_to_xfoil = getCMakeParam('xfoil');
  if isempty(path_to_xfoil) || strcmp(path_to_xfoil,'xfoil-NOTFOUND')
    disp(' XFOIL support is disabled.  To enable it, install XFOIL from here: http://web.mit.edu/drela/Public/web/xfoil/, then add it to the matlab path or set the path to the xfoil executable explicitly using editDrakeConfig(''xfoil'',path_to_avl_executable) and rerun addpath_drake');
    conf.xfoil = '';
  else
    conf.xfoil = path_to_xfoil;
  end
end
conf.xfoil_enabled = ~isempty(conf.xfoil);

if (conf.lcm_enabled)
  [retval,info] = system('util/check_multicast_is_loopback.sh');
  if (retval)
    info = strrep(info,'ERROR: ','');
    info = strrep(info,'./',[conf.root,'/util/']);
    warning('Drake:BroadcastingLCM','Currently all of your LCM traffic will be broadcast to the network, because:\n%s',info);
  end
end  


% save configuration options to config.mat
%conf
save([conf.root,'/util/drake_config.mat'],'conf');

%disp('To manually change any of these entries, use:')
%disp('  editDrakeConfig(param,value);');

clear util/checkDependency;  % makes sure that the persistent variable in the dependency checker gets cleared

end

function success=pod_pkg_config(podname)
  success=false;
  cmd = ['addpath_',podname];
  if exist(cmd,'file')
    disp([' Calling ',cmd]);
    try 
      eval(cmd);
      success=true;
    catch
      % intentionally left blank
    end
  end
    
  if ~success && nargout<1
    error(['Cannot find required pod ',podname]);
  end
end


function val = getCMakeParam(param)
% note: takes precedence over the function by the same name in util, since
% that one requires getDrakePath to be set first.

[~,val] = system(['cmake -L -N pod-build | grep ', param,' | cut -d "=" -f2']);
val = strtrim(val);

end

function tf = verStringLessThan(a,b)
  % checks if the version string a is less than the version string b

  pa = getParts(a); pb = getParts(b);
  tf = pa(1)<pb(1) || pa(2)<pb(2) || pa(3)<pb(3);
  
  function parts = getParts(V)
    parts = sscanf(V, '%d.%d.%d')';
    if length(parts) < 3
      parts(3) = 0; % zero-fills to 3 elements
    end
  end
end
