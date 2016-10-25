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

    case 'spotless'
      conf.spotless_enabled = logical(exist('msspoly','class'));
      if ~conf.spotless_enabled
        if ~pod_pkg_config('spotless') && nargout<1
          disp(' ');
          disp(' SPOTLESS not found.  spotless support will be disabled.');
          disp(' To re-enable, install spotless using drake''s cmake option WITH_SPOTLESS=ON');
          disp(' Or install it directly from https://github.com/spot-toolbox/spotless and add it to your path');
          disp(' ');
        end
        conf.spotless_enabled = logical(exist('msspoly','class'));
      end

    case 'lcm'
      conf.lcm_enabled = logical(exist('lcm.lcm.LCM','class'));
      if (~conf.lcm_enabled)
        lcm_java_classpath = getCMakeParam('lcm_java_classpath');
        if ~isempty(lcm_java_classpath)
          javaaddpathProtectGlobals(lcm_java_classpath);
          disp(' Added the lcm jar to your javaclasspath (found via cmake)');
          conf.lcm_enabled = logical(exist('lcm.lcm.LCM','class'));
        end

        if (~conf.lcm_enabled)
          [retval,cp] = system(['export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:',fullfile(getCMakeParam('CMAKE_INSTALL_PREFIX'),'lib','pkgconfig'),' && pkg-config --variable=classpath lcm-java']);
          if (retval==0 && ~isempty(cp))
            disp(' Added the lcm jar to your javaclasspath (found via pkg-config)');
            javaaddpathProtectGlobals(strtrim(cp));
          end

          conf.lcm_enabled = logical(exist('lcm.lcm.LCM','class'));
        end

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
          disp(' To re-enable, add lcm-###.jar to your matlab classpath');
          disp(' (e.g., by putting javaaddpath(''/usr/local/share/java/lcm-0.9.2.jar'') into your startup.m .');
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
          lcm_java_classpath = getCMakeParam('LCMGL_JAR_FILE');
          javaaddpathProtectGlobals(lcm_java_classpath);
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

    case {'snopt','studentsnopt'}
      [conf.snopt_enabled,conf.studentsnopt_enabled] = snoptEnabled();
      if (~conf.snopt_enabled && ~conf.studentsnopt_enabled)
        % Capture the output to prevent a spurious or duplicate console warning.
        snopt_enabled = pod_pkg_config('snopt');
        [conf.snopt_enabled,conf.studentsnopt_enabled] = snoptEnabled();
      end

      if ~conf.snopt_enabled && ~conf.studentsnopt_enabled && nargout<1
        disp(' ');
        disp(' SNOPT not found.  SNOPT support will be disabled.');
        disp(' To re-enable, add the SNOPT matlab folder to your path and rerun addpath_drake.');
        disp(' SNOPT can be obtained from <a href="https://tig.csail.mit.edu/software/">https://tig.csail.mit.edu/software/</a> .');
        disp(' studentSNOPT can be obtained from <a href="http://www.cam.ucsd.edu/~peg/Software.html">http://www.cam.ucsd.edu/~peg/Software.html</a> .');
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

    case 'sedumi'
      conf.sedumi_enabled = logical(exist('sedumi','file'));
      if (~conf.sedumi_enabled)
        conf.sedumi_enabled = pod_pkg_config('sedumi') && logical(exist('sedumi','file'));
      end

      if (conf.sedumi_enabled)
        %  sedumiA=[10,2,3,4;5,7,6,4];
        %  sedumib=[4;6];
        %  sedumic=[0;1;0;1];
        %  sedumiT=sedumi(sedumiA,sedumib,sedumic);%,struct('f',4),struct('fid',0));
        %  if(~sedumiT)
        %    error('SeDuMi seems to have encountered a problem. Please verify that your SeDuMi install is working.');
        %  end
      elseif nargout<1
        disp(' ');
        disp(' SeDuMi not found.  SeDuMi support will be disabled.');
        disp(' To re-enable, add SeDuMi to your matlab path and rerun addpath_drake.');
        disp(' SeDuMi can be downloaded for free from <a href="http://sedumi.ie.lehigh.edu/">http://sedumi.ie.lehigh.edu/</a> ');
        disp(' ');
      end

    case 'mosek'
      conf.mosek_enabled = logical(exist('mosekopt','file'));
      if (~conf.mosek_enabled)
        conf.mosek_enabled = pod_pkg_config('mosek') && logical(exist('mosekopt','file'));
      end

      if (conf.mosek_enabled)
        % Check for license issues
        try
          mosekopt();
        catch ex;
          conf.mosek_enabled = false;
          disp(getReport(ex,'extended'));
        end
      end

      if ~conf.mosek_enabled && nargout<1
        disp(' ');
        disp(' Mosek not found or not working. Mosek support will be disabled.');
        disp(' Note that Mosek does provide free academic licenses')
        disp('    To enable, install Mosek and a license from');
        disp('    <a href="http://mosek.com/">http://mosek.com/</a> .');
        disp(' ');
      end

    case 'gurobi'
      conf.gurobi_enabled = logical(exist('gurobi','file')); %&& ~isempty(getenv('GUROBI_HOME')));
      if (~conf.gurobi_enabled)
        conf.gurobi_enabled = pod_pkg_config('gurobi'); %&& ~isempty(getenv('GUROBI_HOME'));
      end

      if (conf.gurobi_enabled)
        % gurobi.mex* is loaded, now test for license issues
        model.obj = 1;
        model.A  = sparse(1,1);
        model.rhs = 0;
        model.sense = '=';
        params.outputflag = false;
        try
          result = gurobi(model, params);
        catch ex;
          conf.gurobi_enabled = false;
          disp(getReport(ex,'extended'));
        end
      end

      if ~conf.gurobi_enabled && nargout<1
        disp(' ');
        disp(' GUROBI not found or not working. GUROBI support will be disabled.');
        disp(' Note that GUROBI does provide free academic licenses')
        disp('    To enable, install GUROBI and a license from');
        disp('    <a href="http://www.gurobi.com/download/licenses/free-academic">http://www.gurobi.com/download/licenses/free-academic</a> .');
        disp(' Then, you will need to set several environment variables.');
        disp(' Please see <a href="http://drake.mit.edu/quickstart">http://drake.mit.edu/quickstart</a> for more info.');
        disp(' ');
      end

    case 'gurobi_mex'
      conf.gurobi_mex_enabled = logical(exist('gurobiQPmex'));


      if ~conf.gurobi_mex_enabled && nargout<1
        disp(' ');
        disp(' GUROBI MEX not found.  GUROBI MEX support will be disabled.');
        disp('    To enable, install the GUROBI pod in your pod collection, and rerun make config; make in drake');
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

    case 'yalmip'
      conf.yalmip_enabled = logical(exist('sdpvar','file'));
      if (~conf.yalmip_enabled)
        conf.yalmip_enabled = pod_pkg_config('yalmip') && logical(exist('sdpvar','file'));
      end
      if ~conf.yalmip_enabled && nargout<1
        disp(' ');
        disp(' YALMIP not found.  To enable, install YALMIP (e.g. by cloning https://github.com/RobotLocomotion/yalmip into drake-distro and running make).');
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

    case 'avl'
      if ~isfield(conf,'avl') || isempty(conf.avl)
        path_to_avl = getCMakeParam('AVL_EXECUTABLE');
        if isempty(path_to_avl) || strcmp(path_to_avl,'AVL_EXECUTABLE-NOTFOUND')
          if nargout<1
            disp(' ');
            disp(' AVL support is disabled.  To enable it, install AVL from here: http://web.mit.edu/drela/Public/web/avl/, then add it to the matlab path or set the path to the avl executable explicitly using editDrakeConfig(''avl'',path_to_avl_executable) and rerun make');
            disp(' ');
          end
          conf.avl = '';
        else
          conf.avl = path_to_avl;
        end
      end
      conf.avl_enabled = ~isempty(conf.avl);

    case 'xfoil'
      if ~isfield(conf,'xfoil') || isempty(conf.xfoil)
        path_to_xfoil = getCMakeParam('XFOIL_EXECUTABLE');
        if isempty(path_to_xfoil) || strcmp(path_to_xfoil,'XFOIL_EXECUTABLE-NOTFOUND')
          if nargout<1
            disp(' ');
            disp(' XFOIL support is disabled.  To enable it, install XFOIL from here: http://web.mit.edu/drela/Public/web/xfoil/, then add it to the matlab path or set the path to the xfoil executable explicitly using editDrakeConfig(''xfoil'',path_to_avl_executable) and rerun addpath_drake');
            disp(' ');
          end
          conf.xfoil = '';
        else
          conf.xfoil = path_to_xfoil;
        end
      end
      conf.xfoil_enabled = ~isempty(conf.xfoil);

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

    case 'nonlinearprogramsnoptmex'
      conf.nonlinearprogramsnoptmex_enabled = logical(exist('NonlinearProgramSnoptmex','file')==3);
      if(~conf.nonlinearprogramsnoptmex_enabled)
        if(nargout<1)
          disp(' ');
          disp(' NonlinearProgramSnoptmex is disabled. To enable it, compile NonlinearProgramSnoptmex.cpp with snopt');
          disp(' ');
        end
      end

    case 'iris'
      conf.iris_enabled = logical(exist('+iris/inflate_region.m','file'));
      if (~conf.iris_enabled)
        conf.iris_enabled = pod_pkg_config('iris');
      end
      if ~conf.iris_enabled && nargout<1
        disp(' ');
        disp(' iris (Iterative Regional Inflation by SDP) is disabled. To enable it, install the IRIS matlab package from here: https://github.com/rdeits/iris-distro and re-run addpath_drake.');
        disp(' ');
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

function success=pod_pkg_config(podname)
  success=false;
  cmd = ['addpath_',podname];
  if exist(cmd,'file')
    disp([' Calling ',cmd]);
    try
      eval(cmd);
      success=true;
    catch ex
      disp(getReport(ex,'extended'))
    end
  end

  if ~success && nargout<1
    disp(['Cannot find required pod ',podname]);
  end
end

function tf = verStringLessThan(a,b)
  % checks if the version string a is less than the version string b

  pa = getParts(a); pb = getParts(b);
  tf = pa(1)<pb(1) || pa(2)<pb(2) || pa(3)<pb(3);

  function parts = getParts(V)
    parts = sscanf(V, '%d.%d.%d')';
    if length(parts) < 3
      parts(3) = 0; % zero-fills to 3 elements
    end
  end
end

function [snopt_enabled,studentsnopt_enabled] = snoptEnabled()
% check if snopt exists, if it does, check if it is student version
snopt_val = logical(exist('snopt.m','file'));
if(snopt_val)
  snopt_path = which('snopt.m');
  snopt_readme=fileread([snopt_path(1:end-7),'README']);
  if(isempty(regexp(snopt_readme,'studentVersions','match')))
    snopt_val = 1;
  else
    snopt_val = 2;
  end
else
  snopt_val = 0;
end
if(snopt_val == 0)
  snopt_enabled = false;
  studentsnopt_enabled = false;
else
  snopt_enabled = snopt_val == 1;
  studentsnopt_enabled = snopt_val == 2;
end
end
