function ok = checkDependency(dep,command)
% Drake code which depends on an external library or program should
% check that dependency by calling this function.
%
% For a required dependency, do not capture the output. The MATLAB script will
% fail and emit an error message to the console.
%
%     checkDependency('snopt');
%
% For an optional dependency, capture the output. The MATLAB script will
% continue regardless and will not emit a warning message to the console.
%
%     no_snopt = ~checkDependency('snopt');
%
% @param dep the name of the dependency to check
% @param command can be 'disable', 'enable'
%  % todo: consider supporting a minimum_version

persistent conf;

ldep = lower(dep);
conf_var = [ldep,'_enabled'];

if (nargin>1)
  if strcmp(command,'disable')
    conf.(conf_var) = false;
    return;
  elseif strcmp(command,'enable')
    conf.(conf_var) = [];
  end
end

already_checked = isfield(conf,conf_var) && ~isempty(conf.(conf_var));
if already_checked
  ok = conf.(conf_var);
else % then try to evaluate the dependency now...
  switch(ldep)
    case 'lcm'
      conf.lcm_enabled = logical(exist('lcm.lcm.LCM','class'));
      if (~conf.lcm_enabled)
        lcm_java_classpath = getCMakeParam('LCM_JAR_FILE');
        javaaddpathProtectGlobals(lcm_java_classpath);
        disp(' Added the lcm jar to your javaclasspath (found via cmake)');
        conf.lcm_enabled = logical(exist('lcm.lcm.LCM','class'));
        if (conf.lcm_enabled)
          [retval,info] = systemWCMakeEnv(fullfile(getDrakePath(),'matlab','util','check_multicast_is_loopback.sh'));
          if (retval)
            info = strrep(info,'ERROR: ','');
            info = strrep(info,'./',[getDrakePath,'/matlab/util/']);
            warning('Drake:BroadcastingLCM','Currently all of your LCM traffic will be broadcast to the network, because:\n%s',info);
          end
        elseif nargout<1
          disp(' ');
          disp(' LCM not found.  LCM support will be disabled.');
          disp(' To re-enable, add lcm.jar to your matlab classpath');
          disp(' (e.g., by putting javaaddpath(''/usr/local/share/java/lcm.jar'') into your startup.m .');
          disp(' ');
        end
      end

    case 'lcmgl'
      if nargout<1
        % If no outputs are captured, the caller wants a console error message,
        % so do not capture outputs on the recursive call either.
        checkDependency('lcm');
        lcm_enabled = true;
      else
        lcm_enabled = checkDependency('lcm');
      end
      conf.lcmgl_enabled = lcm_enabled && logical(exist('bot_lcmgl.data_t','class'));

      if (lcm_enabled && ~conf.lcmgl_enabled)
        try % try to add bot2-lcmgl.jar
          lcmgl_java_classpath = getCMakeParam('LCMGL_JAR_FILE');
          javaaddpathProtectGlobals(lcmgl_java_classpath);
          disp(' Added the lcmgl jar to your javaclasspath (found via cmake)');
        catch err
          if strcmp(err.identifier, 'Drake:CannotClearJava')
            rethrow(err);
          end
        end
        conf.lcmgl_enabled = exist('bot_lcmgl.data_t','class');
      end
      if ~conf.lcmgl_enabled && nargout<1
        disp(' ');
        disp(' LCMGL not found.  LCMGL support will be disabled.');
        disp(' To re-enable, add bot2-lcmgl.jar to your matlab classpath using javaaddpath.');
        disp(' ');
      end

    otherwise

      % todo: call ver(dep) here?
      % and/or addpath_dep?

      error(['Drake:UnknownDependency:',dep],['Don''t know how to add dependency: ', dep]);
  end

  ok = conf.(conf_var);
end

if (nargout<1 && ~ok)
  error(['Drake:MissingDependency:',dep],['Cannot find required dependency: ',dep]);
end

end
