function robotlibpath = getRobotlibPath()
% Get the robotlib root directory
% Useful if you want to do something like reference a file on the
% filesystem in robotlib
%
% @retval robotlibpath path to the root of robotlib


persistent conf;

if (isempty(conf))
  try
    load robotlib_config;
  catch
    error('You must run configure once in the main robotlib directory');
  end
end
robotlibpath = conf.root;