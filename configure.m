function configure
% Checks dependencies and sets up matlab path.
% Searches the machine for necessary support programs, and generates
% config.mat.  If required tools aren't found, it tries to be helpful in
% directing you to their location.

try 
  load drake_config.mat; 
catch
  conf=struct();
end

if verLessThan('matlab','7.6')
  error('Drake reguires MATLAB version 7.6 or above.');
  % because I rely on the new matlab classes with classdef
end

%if (~isfield(conf,'root') || isempty(conf.root))  
  conf.root=pwd;  % always set the root
%end

[a,b,c]=extensions;
if (~isfield(conf,'objext') || isempty(conf.objext))
  conf.objext=a;
end
if (~isfield(conf,'libext') || isempty(conf.libext))
  conf.libext=b;
end
if (~isfield(conf,'libpre') || isempty(conf.libpre))
  conf.libpre=c;
end

original_path = path;

% remove any existing drake directories that are in a different root
y=strread(path,'%s','delimiter',pathsep);
ind=strfind(y,'drake');
old_drakes={}; old_drakes_delete=[];
for i=1:length(ind);  % check current paths 
  if ~isempty(ind{i}) % does it have 'drake'?
    old_d = [y{i}(1:ind{i}-1),'drake'];
    if ~strncmpi(old_d,pwd,ind{i})  % is it the current directory, or a different copy of drake?
      j = find(strcmp(old_drakes,old_d),1);  % is it already in our list of old drakes?  (only want to ask the user once)
      if isempty(j)  % then it's old and I haven't seen it before.  prompt user to see if i can delete.
        old_drakes{end+1} = old_d;
        a = input(['I found a drake install in ', old_d,' in your path.\n  This might confuse things.\n  Ok if I remove it from your path now? (y/n)? '], 's');
        old_drakes_delete = [old_drakes_delete;(lower(a(1))=='y')];
        j=length(old_drakes_delete);
      end
      % if i'm allowed to delete, then rmpath this entry
      if (old_drakes_delete(j));
        rmpath(y{i});
      end
    end
  end
end 


% add package directories to the matlab path 
addpath([conf.root,'/systems']);
addpath([conf.root,'/systems/plants']);
addpath([conf.root,'/systems/controllers']);
addpath([conf.root,'/systems/estimators']);
addpath([conf.root,'/systems/trajectories']);
addpath([conf.root,'/systems/frames']);
addpath([conf.root,'/systems/visualizers']);
addpath([conf.root,'/util']);
addpath([conf.root,'/util/obstacles']);
addpath([conf.root,'/thirdParty']);
addpath([conf.root,'/thirdParty/path']);
addpath([conf.root,'/thirdParty/spatial']);
addpath([conf.root,'/thirdParty/graphviz2mat']);
addpath([conf.root,'/thirdParty/cprintf']);

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

conf.lcm_enabled = logical(exist('lcm.lcm.LCM'));
if (~conf.lcm_enabled)
  disp(' LCM not found.  LCM support will be disabled.  To re-enable, add lcm-###.jar to your matlab classpath (e.g., by putting javaaddpath(''/usr/local/share/java/lcm-0.5.1.jar'') into your startup.m .');
elseif (~exist('drake.util.lcmt_scope_data'))
  % todo: if drake java should ever exist outside of lcm, then I can
  % change this, but i'll need to check for a class that is not
  % lcm-dependent (lcmt_scope_data fails if drake.jar is in the
  % classpath but lcm.jar is not)
  warning(['Can''t find drake files in your matlab java classpath.  Disabling LCM.  I recommend doing the following:',10,' 1) >> edit startup.m',10,' 2) add the line "javaaddpath(''',conf.root,'/drake.jar'');" to the end of startup.m',10,' 3) >> startup.m']);
  conf.lcm_enabled = false;
end

conf.snopt_enabled = logical(exist('snopt'));
if (~conf.snopt_enabled) 
  disp(' SNOPT not found.  SNOPT support will be disabled.  To re-enable, add the SNOPT matlab folder to your path and rerun configure.  SNOPT can be obtained from https://tig.csail.mit.edu/software/ .');
end

conf.vrml_enabled = logical(vrinstall('-check'));

conf.sedumi_enabled = logical(exist('sedumi'));
if (conf.sedumi_enabled)
    disp('You seem to have SeDuMi installed, so I am just going to check that it runs properly.');
    disp('-------------------------------------------------------------------------------------');
    sedumiA=[10,2,3,4;5,7,6,4];
    sedumib=[4;6];
    sedumic=[0;1;0;1];
    sedumiT=sedumi(sedumiA,sedumib,sedumic);
    if(~sedumiT)
        disp('-------------------------------------------------------------------------------------');
        error('SeDuMi seems to have encountered a problem. Please verify that your SeDuMi install is working.');
        disp('-------------------------------------------------------------------------------------');
    else 
        disp('-------------------------------------------------------------------------------------');        
        disp('SeDuMi seems to be working - that''s what all the output, above, is showing.');
        disp('-------------------------------------------------------------------------------------');
    end
    
end

if (~conf.sedumi_enabled)
  disp(' SeDuMi not found.  SeDuMi support will be disabled.  To re-enable, add SeDuMi to your matlab path and rerun configure.  SeDuMi can be downloaded for free from http://sedumi.ie.lehigh.edu/ ');
end

conf.spot_enabled = logical(exist('msspoly'));
if conf.spot_enabled
  if ~ismethod('msspoly','clean')
    disp('  Found an installation of SPOT, but it doesn''t appear to be the right version.');
    conf.spot_enabled = false;
  end
else
  disp(' SPOT not found.');
end
if ~conf.spot_enabled
  disp('  SPOT support will be disabled.  To re-enable, add SPOT to your matlab path and rerun configure.  SPOT can be found at svn co https://svn.csail.mit.edu/spot/branches/dev/ ');
end

conf.yalmip_enabled = logical(exist('sdpvar'));
if (~conf.yalmip_enabled)
  disp(' YALMIP not found.  YALMIP support will be disabled.  To re-enable, install YALMIP and rerun configure.'); 
end

conf.pathlcp_enabled = ~isempty(getenv('PATH_LICENSE_STRING'));
if (~conf.pathlcp_enabled)
  disp('The PATH LCP solver (in the thirdparty directory) needs you to get the setup the license: http://pages.cs.wisc.edu/~ferris/path.html');
  disp('I recommend adding a setenv(''PATH_LICENSE_STRING'',...) line to your startup.m');
  disp('The LCP solver will be disabled');
end

conf.simulationconstructionset_enabled = ...
    logical(exist('com.yobotics.simulationconstructionset.Robot'));
if (~conf.simulationconstructionset_enabled)
  disp(' SimulationConstructionSet not found.  SCS support will be disabled.  To re-enable, get SCS on your machine and add all of the necessary class directories and jar files to your matlab java class path');
end

conf.eigen3_enabled = isfield(conf,'eigen3_incdir') && ~isempty(conf.eigen3_incdir);
if ~conf.eigen3_enabled
  disp(' Eigen3 support is disabled; mex files that use eigen3 will be disabled.  To re-enable, set the path to eigen3 using editDrakeConfig(''eigen3_incdir'',path_to_eigen) and rerun configure');
  conf.eigen3_incdir = '';
end

% save configuration options to config.mat
conf.additional_unit_test_dirs={};
conf
save([conf.root,'/util/drake_config.mat'],'conf');

disp('To manually change any of these entries, use:')
disp('  editDrakeConfig(param,value);');

% if changes have been made to the matlab path, prompt user with option to 
% save the path for future matlab sessions.
if (~strcmp(original_path,path))
  a = input('This configure script added some necessary paths to your matlab path.\nWould you like to save your current path for future matlab sessions\nnow (y/n)? ', 's');
  if (lower(a(1))=='y')
    if (~savepath) disp('MATLAB path successfully saved.'); end
  end
end

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
