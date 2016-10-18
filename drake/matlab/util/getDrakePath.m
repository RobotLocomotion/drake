function root = getDrakePath()
% Get the Drake root directory
% Useful if you want to do something like reference a file on the
% filesystem in drake
%
% @retval root path to the root of Drake


persistent myroot;

if (isempty(myroot))
  myroot = fileparts(fileparts(which('getDrakePath')));
end
root = myroot;
