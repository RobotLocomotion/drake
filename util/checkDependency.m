function ok = checkDependency(conf_var)
% Robotlib code which depends on an external library or program should
% check that dependency by calling this function immediately.
%   example:
%     checkDependency('spot_enabled')
% or 
%     if (~checkDependency('spot_enabled')) error('my error'); end


persistent conf;

if (isempty(conf))
  try
    load robotlib_config;
  catch
    error('You must run configure once in the main robotlib directory');
  end
end

ok = isfield(conf,conf_var) && getfield(conf,conf_var);
if (nargout<1 && ~ok)
  error(['Cannot run this function because ', conf_var, ' was not set in the configure script']);
end
