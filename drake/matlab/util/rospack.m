function path = rospack(package)
% implements 'rospack find package' without requiring ros or rospack to be
% installed.  this is intended to (at least) help parsing ROS urdf files with
% package:// tags.


% first try to actually use rospack 
[info,path]=system(['rospack find ',package]);
if info==0  % success!
  path = regexprep(path,'\n','');
  return;
end

% otherwise, implement the algorithm myself (so it works on any system):

% todo: use matlab's hashmap instead of cell arrays

persistent packages package_paths;

if isempty(packages)  % cache packages 
  package_paths={};
  package_paths = vertcat(package_paths,searchenvvar('ROS_ROOT'));
  package_paths = vertcat(package_paths,searchenvvar('ROS_PACKAGE_PATH'));
  packages=cell(size(package_paths));
  for i=1:length(package_paths)
    [~,packages{i}]=fileparts(package_paths{i});
  end
  packages{end+1} = 'Atlas';
  package_paths{end+1} = [getDrakePath,'/examples/Atlas'];
  packages{end+1} = 'Valkyrie';
  package_paths{end+1} = [getDrakePath,'/examples/Valkyrie'];
  packages{end+1} = 'IRB140';
  package_paths{end+1} = [getDrakePath,'/examples/IRB140'];
end

if strcmp(package,'-list')
  packages
  path = '';
  return;
end

ind = find(strcmpi(package,packages),1);
if isempty(ind)
%  % check local directory for a folder with the package name
%  if isdir(package)
%    path = package;
%    return;
%  end

  % otherwise, I give up:
  error('couldn''t find ROS package %s.\n ROS_ROOT=%s\n ROS_PACKAGE_PATH=%s\n',package,getenv('ROS_ROOT'),getenv('ROS_PACKAGE_PATH'));
end
  
path = package_paths{ind};

end


function packages = searchenvvar(varname)

packages={};
path = getenv(varname);
while ~isempty(path)
  [token,path]=strtok(path,pathsep);
  % First look for package.xml
  [info,p] = system(['find -L ',token,' -iname package.xml']);
  if info==0 
    while ~isempty(p)
      [pt,p]=strtok(p);
      pt=fileparts(pt);
      p = regexprep(p,[pt,'.*\n'],'','dotexceptnewline');
      packages=vertcat(packages,pt);
    end
  else  % if find fails for some reason (windows?), then do it the hard way...
    packages = vertcat(packages,searchdir(token,'package.xml'));
  end
  % Then look for manifest.xml (deprecated)
  [info,p] = system(['find -L ',token,' -iname manifest.xml']);
  if info==0 
    while ~isempty(p)
      [pt,p]=strtok(p);
      pt=fileparts(pt);
      p = regexprep(p,[pt,'.*\n'],'','dotexceptnewline');
      packages=vertcat(packages,pt);
    end
  else  % if find fails for some reason (windows?), then do it the hard way...
    packages = vertcat(packages,searchdir(token,'manifest.xml'));
  end
end

end

function packages = searchdir(path,str)
  if ~isempty(dir(fullfile(path,str)));
    packages = {path};
    return;
  end
  
  packages={};
  d = dir(path);
  for i=find([d.isdir])
    if (d(i).name(1)=='.') continue; end
    packages = vertcat(packages,searchdir(d(i).name,str));
  end
end
