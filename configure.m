function configure
% Checks dependencies and sets up matlab path.
% Searches the machine for necessary support programs, and generates
% config.mat.  If required tools aren't found, it tries to be helpful in
% directing you to their location.

% toggle the name of the package here
packageName = ['robotlib'];

try 
  load robotlib_config.mat; 
catch
  conf=struct();
end

v=ver('matlab');
ind = find(v.Version,'.','first');
major_ver = str2num(v.Version(1:ind));
minor_ver = str2num(v.Version((ind+2):end));
if (major_ver < 7 || (major_ver==7 && minor_ver < 6)) 
  error('RobotLib reguires MATLAB version 7.6 or above.');
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

% remove any existing package directories that are in a different root
y=strread(path,'%s','delimiter',pathsep);
ind=strfind(y,'robotlib');
old_packagelibs={}; old_packagelibs_delete=[];
for i=1:length(ind);  % check current paths 
  if ~isempty(ind{i}) % does it have the name of our package?
    old_rl = [y{i}(1:ind{i}-1), packageName];
    if ~strcmpi(old_rl,pwd)  % is it the current directory, or a different copy of robotlib?
      j = find(strcmp(old_packagelibs,old_rl),1);  % is it already in our list of old robotlibs?  (only want to ask the user once)
      if isempty(j)  % then it's old and I haven't seen it before.  prompt user to see if i can delete.
        old_packagelibs{end+1} = old_rl;
        a = input(['I found another installation in ', y{i},' in your path.\n  This might confuse things.\n  Ok if I remove it from your path now? (y/n)? '], 's');
        old_packagelibs_delete = [old_packagelibs_delete;(lower(a(1))=='y')];
        j=length(old_packagelibs_delete);
      end
      % if i'm allowed to delete, then rmpath this entry
      if (old_packagelibs_delete(j));
        rmpath(y{i});
      end
    end
  end
end 


% add package directories to the matlab path 
addpath([conf.root,'/systems']);
addpath([conf.root,'/systems/plants']);
addpath([conf.root,'/systems/plants/obstacles']);
addpath([conf.root,'/systems/controllers']);
addpath([conf.root,'/systems/estimators']);
addpath([conf.root,'/systems/trajectories']);
addpath([conf.root,'/util']);
addpath([conf.root,'/thirdParty/spatial']);

% todo: setup java classpath (not hard to do it once... but how can I set
% it up for future sessions as well?  maybe write to startup.m, or prompt
% user to do it?)

% check for all dependencies

v=ver('simulink');
if (isempty(v)) conf.simulink_enabled = false;
else  
  ind = find(v.Version,'.','first');
  major_ver = str2num(v.Version(1:ind));
  minor_ver = str2num(v.Version((ind+2):end));
  if (major_ver < 7 || (major_ver==7 && minor_ver < 3)) 
    warning('Some features of ', packageName, ' requires SIMULINK version 7.3 or above.');
    % haven't actually tested with lower versions
    conf.simulink_enabled = false;
  end
conf.simulink_enabled = true;
end

conf.lcm_enabled = logical(exist('lcm.lcm.LCM'));
if (~conf.lcm_enabled)
  disp(' LCM not found.  LCM support will be disabled.  To re-enable, add lcm-###.jar to your matlab classpath (e.g., by putting javaaddpath(''/usr/local/share/java/lcm-0.5.1.jar'') into your startup.m .');
elseif (~exist('robotlib.util.lcmt_scope_data'))
  % todo: if robotlib java should ever exist outside of lcm, then I can
  % change this, but i'll need to check for a class that is not
  % lcm-dependent (lcmt_scope_data fails if robotlib.jar is in the
  % classpath but lcm.jar is not)
  error(['Can''t find ', packageName, ' files in your matlab java classpath.  I recommend doing the following:',10,' 1) >> edit startup.m',10,' 2) add the line "javaaddpath(''',conf.root,'/robotlib.jar'');" to the end of startup.m',10,' 3) >> startup.m']);
end

conf.snopt_enabled = logical(exist('snopt'));
if (~conf.snopt_enabled) 
  disp(' SNOPT not found.  SNOPT support will be disabled.  To re-enable, add the SNOPT matlab folder to your path and rerun configure.  SNOPT can be obtained from https://tig.csail.mit.edu/software/ .');
end

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
if (~conf.spot_enabled)
  disp(' SPOT not found.  SPOT support will be disabled.  To re-enable, add SPOT to your matlab path and rerun configure.  SPOT can be found at svn co https://svn.csail.mit.edu/spot/ ');
end

conf.yalmip_enabled = logical(exist('sdpvar'));
if (~conf.yalmip_enabled)
  disp(' YALMIP not found.  YALMIP support will be disabled.  To re-enable, install YALMIP and rerun configure.'); 
end

conf.simulationconstructionset_enabled = ...
    logical(exist('com.yobotics.simulationconstructionset.Robot'));
if (~conf.simulationconstructionset_enabled)
  disp(' SimulationConstructionSet not found.  SCS support will be disabled.  To re-enable, get SCS on your machine and add all of the necessary class directories and jar files to your matlab java class path');
end


% save configuration options to config.mat
conf
save([conf.root,'/util/robotlib_config.mat'],'conf');

disp('To manually change any of these entries, use:')
disp('  load robotlib_config.mat;');
disp('  conf.field = val;');
disp('  save([conf.root,''/util/robotlib_config.mat''],''conf'');');

% if changes have been made to the matlab path, prompt user with option to 
% save the path for future matlab sessions.
if (~strcmp(original_path,path))
  a = input('This configure script added some necessary paths to your matlab path.\nWould you like to save your current path for future matlab sessions\nnow (y/n)? ', 's');
  if (lower(a(1))=='y')
    if (~savepath) disp('MATLAB path successfully saved.'); end
  end
end

clear shared/checkDependency;  % makes sure that the persistent variable in the dependency checker gets cleared

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
