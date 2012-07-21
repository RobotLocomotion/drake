function root = getDrakePath()
% Get the Drake root directory
% Useful if you want to do something like reference a file on the
% filesystem in drake
%
% @retval root path to the root of Drake


persistent conf;

if (isempty(conf))
  try
    load drake_config;
  catch
    error('You must run configure once in the main drake directory');
  end
end
root = conf.root;
