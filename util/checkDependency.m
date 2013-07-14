function ok = checkDependency(dep)
% Drake code which depends on an external library or program should
% check that dependency by calling this function.
%   example:
%     checkDependency('snopt')
% or 
%     if (~checkDependency('snopt')) error('my error'); end


persistent conf;

conf_var = [dep,'_enabled'];

if (isempty(conf))
  try
    load drake_config;
  catch
    error('You must run configure once in the main Drake directory');
  end
end

ok = isfield(conf,conf_var) && ~isempty(getfield(conf,conf_var));
if (nargout<1 && ~ok)
  error(['Drake:MissingDependency:',dep],[conf_var, ' was not set in the addpath_drake script']);
end

if ~conf.(conf_var)
  error(['Drake:MissingDependency:',dep],['Missing dependency: ', dep, ' (you must rerun addpath_drake to resolve)']);
end
