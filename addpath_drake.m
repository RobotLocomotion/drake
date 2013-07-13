function addpath_drake
% Checks dependencies and sets up matlab path.
% Searches the machine for necessary support programs, and generates
% config.mat.  If required tools aren't found, it tries to be helpful in
% directing you to their location.

try
  load drake_config.mat; 
catch
  conf=struct();
end
conf.root = pwd;

if ~exist('pods_get_base_path')
  % todo: implement the BUILD_PREFIX logic from the pod Makefiles (e.g.
  % search up to four directories higher)
  if ~exist('build/matlab')
    error('You must run make first (and/or add your pod build/matlab directory to the matlab path)');
  end
  addpath(fullfile(pwd,'build','matlab'));  
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
addpath(fullfile(conf.root,'systems','controllers'));
addpath(fullfile(conf.root,'systems','estimators'));
addpath(fullfile(conf.root,'systems','trajectories'));
addpath(fullfile(conf.root,'systems','frames'));
addpath(fullfile(conf.root,'systems','visualizers'));
addpath(fullfile(conf.root,'algorithms'));
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
  warning('Some features of Drake reguires SIMULINK version 7.3 or above.');
  % haven't actually tested with lower versions
  conf.simulink_enabled = false;
else
  conf.simulink_enabled = true;
end

% require spotless 
if ~logical(exist('msspoly')) 
  pod_pkg_config('spotless');
end

conf.lcm_enabled = logical(exist('lcm.lcm.LCM'));
if (~conf.lcm_enabled)
  [retval,cp] = system('pkg-config --variable=classpath lcm-java');
  if (retval==0 && ~isempty(cp))
    disp(' Added the lcm jar to your javaclasspath (found via pkg-config)');
    javaaddpath(strtrim(cp));
  end

  conf.lcm_enabled = logical(exist('lcm.lcm.LCM'));
  if (~conf.lcm_enabled)
    disp(' ');
    disp(' LCM not found.  LCM support will be disabled.');
    disp(' To re-enable, add lcm-###.jar to your matlab classpath');
    disp(' (e.g., by putting javaaddpath(''/usr/local/share/java/lcm-0.9.2.jar'') into your startup.m .');
    disp(' ');
  end
end

conf.snopt_enabled = logical(exist('snopt'));
if (~conf.snopt_enabled) 
  conf.snopt_enabled = pod_pkg_config('snopt') && logical(exist('snopt'));
  if (~conf.snopt_enabled) 
    disp(' ');
    disp(' SNOPT not found.  SNOPT support will be disabled.');
    disp(' To re-enable, add the SNOPT matlab folder to your path and rerun configure.');
    disp(' SNOPT can be obtained from <a href="https://tig.csail.mit.edu/software/">https://tig.csail.mit.edu/software/</a> .');
    disp(' ');
  end
end

if(exist('vrinstall'))
  conf.vrml_enabled = logical(vrinstall('-check'));
else
  conf.vrml_enabled=0;
end

conf.sedumi_enabled = logical(exist('sedumi'));
if (conf.sedumi_enabled)
  sedumiA=[10,2,3,4;5,7,6,4];
  sedumib=[4;6];
  sedumic=[0;1;0;1];
  sedumiT=sedumi(sedumiA,sedumib,sedumic,struct('f',4),struct('fid',0));
  if(~sedumiT)
    error('SeDuMi seems to have encountered a problem. Please verify that your SeDuMi install is working.');
  end
end

if (~conf.sedumi_enabled)
  disp(' ');
  disp(' SeDuMi not found.  SeDuMi support will be disabled.');
  disp(' To re-enable, add SeDuMi to your matlab path and rerun configure.');
  disp(' SeDuMi can be downloaded for free from <a href="http://sedumi.ie.lehigh.edu/">http://sedumi.ie.lehigh.edu/</a> ');
  disp(' ');
end

conf.gurobi_enabled = logical(exist('gurobi') && ~isempty(getenv('GRB_LICENSE_FILE')) && ~isempty(getenv('GUROBI_HOME')));
if (~conf.gurobi_enabled)
  conf.gurobi_enabled = pod_pkg_config('gurobi') && ~isempty(getenv('GRB_LICENSE_FILE')) && ~isempty(getenv('GUROBI_HOME'));

  if (~conf.gurobi_enabled)
    
    disp(' ');
    disp(' GUROBI not found. GUROBI support will be disabled.');
    disp('    To enable, install GUROBI and a free academic license from');
    disp('    <a href="http://www.gurobi.com/download/licenses/free-academic">http://www.gurobi.com/download/licenses/free-academic</a> .');
    disp(' Then, you will need to set several environment variables.');
    disp(' Please see <a href="http://drake.mit.edu/quickstart">http://drake.mit.edu/quickstart</a> for more info.');
    disp(' ');
  end
end

conf.cplex_enabled = logical(exist('cplexlp'));
if (~conf.cplex_enabled)
  disp(' CPLEX not found.  CPLEX support will be disabled.  To re-enable, install CPLEX and add the matlab subdirectory to your matlab path, then rerun configure');
end

conf.yalmip_enabled = logical(exist('sdpvar'));
if (~conf.yalmip_enabled)
  disp(' YALMIP not found.  YALMIP support will be disabled.  To re-enable, install YALMIP and rerun configure.'); 
end

conf.pathlcp_enabled = ~isempty(getenv('PATH_LICENSE_STRING'));
if (~conf.pathlcp_enabled)
  disp(' ');
  disp(' The PATH LCP solver (in the thirdparty directory) needs you to get the setup the license: http://pages.cs.wisc.edu/~ferris/path.html');
  disp(' I recommend adding a setenv(''PATH_LICENSE_STRING'',...) line to your startup.m');
  disp(' The LCP solver will be disabled');
  disp(' ');
end

% todo: add lcmgl_enabled (using pod_pkg_config)

if ~isfield(conf,'avl') || isempty(conf.avl)
  [retval,path_to_avl] = system('which avl');
  if (retval==0)
    conf.avl = path_to_avl;
  else
    disp(' AVL support is disabled.  To enable it, install AVL from here: http://web.mit.edu/drela/Public/web/avl/, then add it to the matlab path or set the path to the avl executable explicitly using editDrakeConfig(''avl'',path_to_avl_executable) and rerun configure');
  end
end
if ~isfield(conf,'xfoil') || isempty(conf.xfoil)
  [retval,path_to_xfoil] = system('which xfoil');
  if (retval==0)
    conf.xfoil = path_to_xfoil;
  else
    disp(' XFOIL support is disabled.  To enable it, install XFOIL from here: http://web.mit.edu/drela/Public/web/xfoil/, then add it to the matlab path or set the path to the xfoil executable explicitly using editDrakeConfig(''xfoil'',path_to_avl_executable) and rerun configure');
  end
end

if ~isfield(conf,'conf.additional_unit_test_dirs')
  conf.additional_unit_test_dirs={};
end

% save configuration options to config.mat
conf
save([conf.root,'/util/drake_config.mat'],'conf');

disp('To manually change any of these entries, use:')
disp('  editDrakeConfig(param,value);');

clear util/checkDependency;  % makes sure that the persistent variable in the dependency checker gets cleared

end

function conf = setconf(conf,field,longname,candidates)
% prompts the user to select which configuration from a set of candidates
  fprintf(1,'\n%s:\n',longname);
  bDefault = isfield(conf,field) && ~isempty(getfield(conf,field));
  if (bDefault)
    fprintf('  0) %s \t (current)\n',getfield(conf,field));
    m=0;
  else
    m=1;
  end
  if (nargin<4 || isempty(candidates)) 
    candidates=[];
    i=0; 
  else
    if (~iscell(candidates)) candidates={candidates}; end
    for i=1:length(candidates)
      fprintf('  %d) %s\n',i,candidates{i});
    end
  end
  fprintf('  %d) Manually enter a different path\n',i+1);
  fprintf('  %d) Leave blank for now\n',i+2);
  n=i+2;
  a=-1;
  while (a<m || a>n), a=input(['Enter your selection: [',num2str(m),'-',num2str(n),']: ']); end
  if (a>0 && a<=length(candidates))
    conf=setfield(conf,field,candidates{a});
  elseif a==(n-1)
    conf=setfield(conf,field,input('Enter path: ','s'));
  elseif a==n
    conf=setfield(conf,field,[]);
  end
    
end

function success=pod_pkg_config(podname)
  success=false;
  cmd = ['addpath_',podname];
  if exist(cmd)
    eval(cmd);
    success=true;
  elseif nargout<1
    error(['Cannot find pod ',podname]);
  end
end

function dir = findfile(fname)
% locate a support file on the harddrive

% todo: don't depend on locate being installed
  [a,b]=system(['locate ',fname]);
  if (a~=0) 
    warning('looks like you don''t have locate'); 
    dir = [];
    return;
  end
  if (isempty(b)) 
    dir=[];
  else
    % to do:  if there are multiple hits, allow user to select
    ind = [0,find(int32(b)==10),length(b)+1];
    dir={};
    for i=1:(length(ind)-1)
      if (ind(i+1)-ind(i)>1) 
        str = b((ind(i)+1):(ind(i+1)-1));
        j=strfind(str,fname);
        dir={dir{:},str(1:(j-2))};
      end
    end
  end
end


function [obj,lib,libprefix] = extensions
% define library extensions for different platforms
  if strcmp(computer,'PCWIN')|| strcmp(computer,'PCWIN64')
    obj = 'obj';
    lib = 'lib';
    libprefix = 'lib';
  else
    obj = 'o';
    lib = 'a';
    libprefix = 'lib';
  end

end



