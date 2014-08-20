function root = getDrakePath()
% Get the Drake root directory
% Useful if you want to do something like reference a file on the
% filesystem in drake
%
% @retval root path to the root of Drake


persistent myroot;

if (isempty(myroot))
  known_file = which('LinearSystem');
  if isempty(known_file)
    error('Can''t find drake root.  Did you call addpath_drake?');
  end
  myroot = fileparts(fileparts(known_file));
end
root = myroot;
