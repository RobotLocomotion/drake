function path = rospack(package)
% implements 'rospack find package' without requiring ros or rospack to be
% installed.  this is intended to (at least) help parsing ROS urdf files with
% package:// tags.

persistent packages package_paths;

if isempty(packages)  % cache packages 
  package_paths={};
  package_paths = vertcat(package_paths,searchenvvar('ROS_ROOT'));
  package_paths = vertcat(package_paths,searchenvvar('ROS_PACKAGE_PATH'));
  packages=cell(size(package_paths));
  for i=1:length(package_paths)
    [~,packages{i}]=fileparts(package_paths{i});
  end
end

ind = find(strcmpi(package,packages),1);
if isempty(ind)
  % check local directory for a folder with the package name
  if isdir(package)
    path = package;
    return;
  end

  % otherwise, I give up:
  error('couldn''t find ROS package %s.\n ROS_ROOT=%s\n ROS_PACKAGE_PATH=%s\n',package,getenv('ROS_ROOT'),getenv('ROS_PACKAGE_PATH'));
end
  
path = package_paths{ind};

% the old version:  actually uses rospack (but libraries paths were an
% issue)
%[info,path]=system(['rospack find ',package]);
%if info==0  % success!
%  path = regexprep(path,'\n','');
%else
%  error(path);
%end

end


function packages = searchenvvar(varname)

packages={};
path = getenv(varname);
while ~isempty(path)
  [token,path]=strtok(path,pathsep);
  [info,p] = system(['find -L ',token,' -iname manifest.xml']);
  if info==0 
    while ~isempty(p)
      [pt,p]=strtok(p);
      pt=fileparts(pt);
      p = regexprep(p,[pt,'.*\n'],'','dotexceptnewline');
      packages=vertcat(packages,pt);
    end
  else  % if find fails for some reason (windows?), then do it the hard way...
    packages = vertcat(packages,searchdir(token));
  end
end

end

function packages = searchdir(path)
  if ~isempty(dir(fullfile(path,'manifest.xml')));
    packages = {path};
    return;
  end
  
  packages={};
  d = dir(path);
  for i=find([d.isdir])
    if (d(i).name(1)=='.') continue; end
    packages = vertcat(packages,searchdir(d(i).name));
  end
end
