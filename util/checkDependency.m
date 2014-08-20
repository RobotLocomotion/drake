function ok = checkDependency(dep,minimum_version)
% Drake code which depends on an external library or program should
% check that dependency by calling this function.
%   example:
%     checkDependency('snopt')
% or
%     if (~checkDependency('snopt')) error('my error'); end


persistent conf;

ldep = lower(dep);
conf_var = [ldep,'_enabled'];

ok = isfield(conf,conf_var) && ~isempty(conf.(conf_var)) && conf.(conf_var);
if ~ok
  % then try to evaluate the dependency now...

  switch(ldep)
    case 'spotless'
      % require spotless
      conf.spotless_enabled = logical(exist('msspoly','class'));
      if ~conf.spotless_enabled
        if ~pod_pkg_config('spotless')
          error('Drake:SpotlessRequired','Missing required dependency "spotless".  Please include the spotless pod in your pods collection or install spotless directly from <a href="https://github.com/mmt/spotless">https://github.com/mmt/spotless</a>');
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
          javaaddpathProtectGlobals(fullfile(pods_get_base_path,'share','java','lcmtypes_drake.jar'));
          [retval,info] = systemWCMakeEnv('util/check_multicast_is_loopback.sh');
          if (retval)
            info = strrep(info,'ERROR: ','');
            info = strrep(info,'./',[getDrakePath,'/util/']);
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
      checkDependency('lcm');
      conf.lcmgl_enabled = logical(exist('bot_lcmgl.data_t','class'));

      if (~conf.lcmgl_enabled)
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

    case 'snopt'
      conf.snopt_enabled = logical(exist('snset','file'));
      if (~conf.snopt_enabled)
        conf.snopt_enabled = pod_pkg_config('snopt') && logical(exist('snopt','file'));
      end

      if ~conf.snopt_enabled && nargout<1
        disp(' ');
        disp(' SNOPT not found.  SNOPT support will be disabled.');
        disp(' To re-enable, add the SNOPT matlab folder to your path and rerun addpath_drake.');
        disp(' SNOPT can be obtained from <a href="https://tig.csail.mit.edu/software/">https://tig.csail.mit.edu/software/</a> .');
        disp(' ');
      end

    case 'vrml'
      unsupported = false;
      if(exist('vrinstall','file'))
        conf.vrml_enabled = logical(vrinstall('-check','-viewer'));% && usejava('awt');  % usejava('awt') return 0 if running with no display
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
        disp('    To enable, install GUROBI and a free academic license from');
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

    case 'bertini'
      conf.bertini_enabled = logical(exist('bertini','file'));
      if (~conf.bertini_enabled)
        conf.bertini_enabled = pod_pkg_config('bertini');
      end

      if ~conf.bertini_enabled && nargout<1
        disp(' ');
        disp(' Bertini not found.');
        disp(' ');
      end

    case 'gloptipoly3'
      conf.gloptipoly3_enabled = logical(exist('gloptipolyversion','file'));
      if (~conf.gloptipoly3_enabled)
        conf.gloptipoly3_enabled = pod_pkg_config('gloptipoly3');
      end

      if ~conf.gloptipoly3_enabled && nargout<1
        disp(' ');
        disp(' Gloptipoly3 not found.');
        disp(' ');
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
      if ~conf.yalmip_enabled && nargout<1
        disp(' ');
        disp(' YALMIP not found.  YALMIP support will be disabled.  To re-enable, install YALMIP and rerun addpath_drake.');
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
        path_to_avl = getCMakeParam('avl');
        if isempty(path_to_avl) || strcmp(path_to_avl,'avl-NOTFOUND')
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
        path_to_xfoil = getCMakeParam('xfoil');
        if isempty(path_to_xfoil) || strcmp(path_to_xfoil,'xfoil-NOTFOUND')
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

    case 'pathlcp'
      setenv('PATH_LICENSE_STRING', '1926793586&Courtesy&&&USR&54782&7_1_2014&1000&PATH&GEN&31_12_2015&0_0_0&5000&0_0');

      try
        x = pathlcp(speye(500),-ones(500,1));
        valuecheck(x,ones(500,1));
        conf.pathlcp_enabled = true;
      catch
        disp('The cached PATH license is out of date, and PATH will fail to solve larger problems. Please report this bug.');
        conf.pathlcp_enabled = false;
      end
      
    otherwise

      % todo: call ver(dep) here?
      % and/or addpath_dep?

      error(['Drake:UnknownDependency:',dep],['Don''t know how to add dependency: ', dep]);
  end

  ok = conf.(conf_var);
  if (nargout<1 && ~ok)
    error(['Drake:MissingDependency:',dep],['Cannot find required dependency: ',dep]);
  end
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
    error(['Cannot find required pod ',podname]);
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
