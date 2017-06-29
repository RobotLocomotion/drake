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
    case 'simulink'
      v=ver('simulink');
      conf.simulink_enabled = ~isempty(v);
      if verLessThan('simulink','7.3')
        warning('Drake:SimulinkVersion','Most features of Drake require SIMULINK version 7.3 or above.');
        % haven't actually tested with lower versions
      end

    case 'distcomp'
      v=ver('distcomp');
      conf.distcomp_enabled = ~isempty(v);
      if ~conf.distcomp_enabled
        disp(' MATLAB Parallel Computing Toolbox was not found');
      elseif verLessThan('distcomp','6.3') && matlabpool('size')==0
        % start a matlab pool (if none exists).  this approximates the
        % now default behavior in newer versions of distcomp.
        matlabpool;
      end

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

    case 'ros'
      conf.ros_enabled = logical(exist('rosmatlab.node','class'));
      if (~conf.ros_enabled)
        if exist(fullfile(matlabroot, 'toolbox', 'psp', 'rosmatlab'), 'dir')
          addpath(fullfile(matlabroot, 'toolbox', 'psp', 'rosmatlab'));
          conf.ros_enabled = logical(exist('rosmatlab.node','class'));
        end
      end

      if ~conf.ros_enabled && nargout<1
        disp(' ');
        disp(' ROS not found.  ROS support will be disabled.');
        disp(' To re-enable, install MATLAB''s ROS support from');
        disp(' <a href="http://www.mathworks.com/ros">http://www.mathworks.com/hardware-support/ros</a>');
        disp(' ');
      end

    case 'ipopt'
      conf.ipopt_enabled = logical(exist(['ipopt.',mexext],'file'));

      if ~conf.ipopt_enabled && nargout<1
        disp(' ');
        disp(' IPOPT not found. IPOPT support will be disabled.');
        disp(' ');
      end

    case 'vrml'
      unsupported = false;
      if(exist('vrinstall','file'))
        conf.vrml_enabled = logical(vrinstall('-check','viewer'));% && usejava('awt');  % usejava('awt') return 0 if running with no display
        if ismac
          [~,osx] = system('sw_vers -productVersion');
          if ~verStringLessThan(osx,'10.9') && verLessThan('matlab','8.1')
            % per my support ticket to matlab, who sent a perfunctory response
            % pointing to this: http://www.mathworks.com/support/sysreq/release2012a/macintosh.html
            conf.vrml_enabled = false;
            disp(' ');
            disp(' Found Simulink 3D Animation Toolbox, but is not supported in this version of MATLAB.  See http://www.mathworks.com/support/sysreq/release2012a/macintosh.html ');
            disp(' ');
            unsupported = true;
          end
        end
      else
        conf.vrml_enabled=false;
      end

      if ~conf.vrml_enabled && ~unsupported && nargout<1
        disp(' ');
        disp(' Simulink 3D Animation Toolbox not found.  Have you run ''vrinstall -install viewer''?');
        disp(' ');
      end

    case 'fastqp'
      conf.fastqp_enabled = logical(exist(['fastqpmex.',mexext],'file'));

      if ~conf.fastqp_enabled && nargout<1
        disp(' ');
        disp(' fastqp not found. fastqp support will be disabled.');
      end

    case 'cplex'
      conf.cplex_enabled = logical(exist('cplexlp','file'));
      if ~conf.cplex_enabled && nargout<1
        disp(' ');
        disp(' CPLEX not found.  CPLEX support will be disabled.  To re-enable, install CPLEX and add the matlab subdirectory to your matlab path, then rerun addpath_drake');
        disp(' ');
      end

    case 'rigidbodyconstraint_mex'
      conf.rigidbodyconstraint_mex_enabled = (exist('constructPtrRigidBodyConstraintmex','file')==3);
      if ~conf.rigidbodyconstraint_mex_enabled && nargout<1
        disp(' ');
        disp(' The RigidBodyManipulatorConstraint classes were not built (because some of the dependencies where missing when cmake was run)');
        disp(' ');
      end

    case 'bullet'
      conf.bullet_enabled = ~isempty(getCMakeParam('bullet'));
      if ~conf.bullet_enabled && nargout<1
        disp(' ');
        disp(' Bullet not found.  To resolve this you will have to rerun make (from the shell)');
        disp(' ');
      end

    case 'fmincon'
      conf.fmincon_enabled = logical(exist('fmincon.m','file'));
      if(~conf.fmincon_enabled)
        if nargout<1
          disp(' ');
          disp(' fmincon support is disabled. To enable it, install MATLAB Optimization toolbox');
          disp(' ');
        end
      end

    case 'quadprog'
      conf.quadprog_enabled = logical(exist('quadprog.m','file'));
      if(~conf.quadprog_enabled)
        if nargout<1
          disp(' ');
          disp(' quadprog support is disabled. To enable it, install MATLAB Optimization toolbox');
          disp(' ');
      end
      end

    case 'lsqlin'
      conf.lsqlin_enabled = logical(exist('lsqlin.m','file'));
      if(~conf.lsqlin_enabled)
        if nargout<1
          disp(' ');
          disp(' lsqlin support is disabled. To enable it, install MATLAB Optimization toolbox');
          disp(' ');
        end
      end

    case 'cpp_bindings'
      conf.cpp_bindings_enabled = logical(exist('+rbtree/RigidBodyTree.m','file'));
      if ~conf.cpp_bindings_enabled && nargout < 1
        disp(' ');
        disp(' C++ bindings in matlab are disabled. These bindings require a version of SWIG which is compiled with matlab support. You can enable it by turning on the WITH_SWIG_MATLAB option when running `make options` in drake-distro.');
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
