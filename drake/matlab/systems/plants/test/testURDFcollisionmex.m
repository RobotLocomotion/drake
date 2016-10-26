function testURDFcollisionmex
%NOTEST
%pending fix from @avalenzu
checkDependency('bullet');

  old_ros_package_path = getenv('ROS_PACKAGE_PATH');
  setenv('ROS_PACKAGE_PATH', [old_ros_package_path, ':', ....
                              fullfile(getDrakePath(), 'examples')]);
  urdf_collision_test = fullfile(get_drake_binary_dir(), 'bin/urdfCollisionTest');
  if ispc
    urdf_collision_test = [urdf_collision_test,'.exe'];
  end

  if (~exist(urdf_collision_test,'file'))
    error('Drake:MissingDependency','testURDFmex requires that urdfCollisionTest is built (from the command line).  skipping this test');
  end

  tol = 1e-4; % low tolerance because i'm writing finite precision strings to and from the ascii terminal

  for urdf = {'../../../../examples/Atlas/urdf/atlas_convex_hull.urdf', ...
              '../../../../examples/Atlas/urdf/atlas_minimal_contact.urdf'};

    urdffile = GetFullPath(urdf{1});
    fprintf(1,'testing %s\n', urdffile);
    w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
    warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
    r = RigidBodyManipulator(urdffile,struct('floating',true));

    % URDFRigidBodyManipulator doesn't parse collision filter groups (...yet)
    for group_name = r.collision_filter_groups.keys()
      r = removeLinksFromCollisionFilterGroup(r,1:r.getNumBodies(),group_name{1},1);
    end
    r = compile(r);
    warning(w);

    q = getZeroConfiguration(r);
    kinsol = doKinematics(r,q);

    [phi,normal,xA,xB,idxA,idxB] = collisionDetect(r,kinsol);
    [retval,outstr] = systemWCMakeEnv([urdf_collision_test,' ',urdffile,sprintf(' %f',q),' 2> /dev/null']);
    valuecheck(retval,0);
    if ~isempty(outstr)
      outstr_cell = regexp(outstr, '=======', 'split');
      out = textscan(outstr_cell{2}, [repmat('%f ',1,1+3*3) ' %s %s']);
      sizecheck(out{1},size(phi));
      normal_cpp = cell2mat(out(2:4))';
      xA_cpp = cell2mat(out(5:7))';
      xB_cpp = cell2mat(out(8:10))';
      idxA_cpp = cellfun(@r.findLinkId,out{11})';
      idxB_cpp = cellfun(@r.findLinkId,out{12})';

      % The link indices are not the same between the two manipulators,
      % therefore we'll use link names and check both body A and body B when
      % testing.
      for i = 1:length(phi)
        idx = (idxA_cpp == idxA(i)) & (idxB_cpp == idxB(i));
        idx = idx | (idxA_cpp == idxB(i)) & (idxB_cpp == idxA(i));
        assert(any(abs(out{1}(idx) - phi(i)) < tol));
        assert(any(abs(normal_cpp(:,idx)'*normal(:,i)) - 1 < tol));
        assert(   (any(sqrt(sum(bsxfun(@minus,xA_cpp(:,idx),xA(:,i)).^2)) < 10*tol) ...
                    && any(sqrt(sum(bsxfun(@minus,xB_cpp(:,idx),xB(:,i)).^2)) < 10*tol)) ...
               || (any(sqrt(sum(bsxfun(@minus,xA_cpp(:,idx),xB(:,i)).^2)) < 10*tol) ...
                    && any(sqrt(sum(bsxfun(@minus,xB_cpp(:,idx),xA(:,i)).^2)) < 10*tol)) );
      end
    end
  end


end
