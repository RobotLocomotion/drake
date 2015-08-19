function xacro(xacro_filename,urdf_filename)

% note: can also just run it from the shell with, e.g.,:
% python xacro.py RimlessWheel.xacro > RimlessWheel.urdf

if nargin<2
  urdf_filename = xacro_filename;
  % now attempt to strip .xacro and replace it with .urdf if necessary
  [p,n,ext] = fileparts(urdf_filename);
  if strcmpi(ext,'.xacro');
    urdf_filename = fullfile(p,n);
  end
  [p,n,ext] = fileparts(urdf_filename);
  if ~strcmpi(ext,'.urdf');
    urdf_filename = fullfile(p,[n,ext,'.urdf']);
  end
end

pyfile = [mfilename('fullpath'),'.py'];
[status,result] = systemWCMakeEnv(['python ',pyfile,' ',xacro_filename,' > ',urdf_filename])

