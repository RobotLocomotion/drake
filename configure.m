function configure

% Searches the machine for necessary support programs, and generates
% config.mat.  If required tools aren't found, it tries to be helpful in
% directing you to their location.

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

if (~isfield(conf,'root') || isempty(conf.root))
  conf.root=pwd;
end

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

% look for POT, add to matlab path 
% look for Sedumi, add to matlab path 
% look for snopt, add to matlab path 

% add robotlib directories to the matlab path 
addpath([conf.root,'/simulators']);
addpath([conf.root,'/controllers']);
addpath([conf.root,'/controllers/tools']);
addpath([conf.root,'/visualizers']);
addpath([conf.root,'/estimators']);
addpath([conf.root,'/shared']);

% todo: setup java classpath (not hard to do it once... but how can I set
% it up for future sessions as well?  maybe write to startup.m, or prompt
% user to do it?)
%  classpath should only have to be the parent directory to robotlib  (all
%  java files in here are packaged as robotlib.something) 


% save configuration options to config.mat
conf
save([conf.root,'/shared/robotlib_config.mat'],'conf');

disp('To manually change any of these entries, use:')
disp('  load robotlib_config.mat;');
disp('  conf.field = val;');
disp('  save([conf.root,''/shared/robotlib_config.mat''],''conf'');');

% if changes have been made to the matlab path, prompt user with option to 
% save the path for future matlab sessions.
if (~strcmp(original_path,path))
  a = input('This configure script added some necessary paths to your matlab path.\nWould you like to save your current path for future matlab sessions\nnow (y/n)? ', 's');
  if (lower(a(1))=='y')
    if (~savepath) disp('MATLAB path successfully saved.'); end
  end
end

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