classdef RigidBodyManipulator < Manipulator
  % This class wraps the spatial vector library (v1)
  % provided by Roy Featherstone on his website:
  %   http://users.cecs.anu.edu.au/~roy/spatial/documentation.html

  properties (SetAccess=protected)
    name={};        % names of the objects in the rigid body system
    urdf={};        % names of the urdf files loaded
    body=[];        % array of RigidBody objects
    actuator = [];  % array of RigidBodyActuator objects
    param_db = {};  % cell of structures (one per robot) containing parsed parameter info
    loop=[];        % array of RigidBodyLoop objects
    sensor={};      % cell array of RigidBodySensor objects
    force={};       % cell array of RigidBodyForceElement objects
      % note: need {} for arrays that will have multiple types (e.g.
      % a variety of derived classes), but can get away with [] for arrays
      % with elements that are all exactly the same type
    gravity=[0;0;-9.81];
    dim=3;
    terrain;
    num_contact_pairs;
    contact_options; % struct containing options for contact/collision handling
    contact_constraint_id=[];

    
    % struct containing the output of 'obj.getTerrainContactPoints()'.
    % That output does not change between compilations, and is requested
    % at every simulation dt, so storing it can speed things up.
    cached_terrain_contact_points_struct=[];
    
    quat_norm_constraint_id=[];  % indices of state constraints asserting that the quaternion coordinates have norm=1
    
    % default kinematics caches; temporary way to make it easy to avoid creating
    % and destroying DrakeMexPointers to KinematicsCache every time
    % doKinematics is called
    default_kinematics_cache_ptr_no_gradients = [];
    default_kinematics_cache_ptr_with_gradients = [];
                                            
    frame = [];     % array of RigidBodyFrame objects

    robot_state_frames;
    robot_position_frames;
    robot_velocity_frames;
  end

  properties (Access=public)  % i think these should be private, but probably needed to access them from mex? - Russ
    B = [];
    mex_model_ptr = nullPointer();
    dirty = true;
    collision_filter_groups;  % map of CollisionFilterGroup objects
  end

  methods
    function obj = RigidBodyManipulator(filename,options)
      % Construct a new rigid body manipulator object with a single (empty)
      % RigidBody (called 'world'), and optionally load a first robot from
      % urdf (see documentation for addRobotFromURDF for details on the
      % inputs).

      if (nargin<2), options = struct(); end
      if ~isfield(options,'terrain'), options.terrain = []; end;

      obj = obj@Manipulator(0,0);
      obj.body = RigidBody();
      obj.body.linkname = 'world';
      obj = setTerrain(obj,options.terrain);
      obj.contact_options = obj.parseContactOptions(options);

      obj.collision_filter_groups = PassByValueMap('KeyType','char','ValueType','any');
      obj.collision_filter_groups('no_collision') = CollisionFilterGroup();

      if (nargin>0 && ~isempty(filename))
        [path,name,ext]=fileparts(filename);
        if strcmpi(ext,'.urdf')
          obj = addRobotFromURDF(obj,filename,zeros(3,1),zeros(3,1),options);
        elseif strcmpi(ext,'.sdf') || strcmpi(ext,'.world')
          obj = addRobotFromSDF(obj,filename,zeros(3,1),zeros(3,1),options);
        else
          error('Drake:RigidBodyManipulator:UnsupportedFileExtension','Unrecognized file extension: %s for file %s',ext,name);
        end
      end
    end
  end

  methods (Static)
    function obj = loadobj(obj)
      obj.mex_model_ptr = nullPointer();
      obj = compile(obj);
      % NOTEST
    end
    function contact_options = parseContactOptions(options)
      % contact_options = parseContactOptions(options) returns a struct
      % containing settings for contact/collision detection. If no value
      % is given for a particular setting, a default value is used.
      %
      % @param options - Structure that may have fields specifying
      %                  values for contact options.
      %
      % @retval contact_options - Struct with the following fields:
      %     * ignore_self_collisions          @default false
      %     * replace_cylinders_with_capsules @default true
      %     * use_bullet                      @default checkDependency('bullet')
      %   If a corresponding field exists in `options`, its value will
      %   be used.
      contact_options = struct();
      if isfield(options,'ignore_self_collisions')
        typecheck(options.ignore_self_collisions,'logical');
        contact_options.ignore_self_collisions = options.ignore_self_collisions;
      else
        contact_options.ignore_self_collisions = false;
      end
      if isfield(options,'replace_cylinders_with_capsules')
        typecheck(options.replace_cylinders_with_capsules,'logical');
        contact_options.replace_cylinders_with_capsules = ...
          options.replace_cylinders_with_capsules;
      else
        contact_options.replace_cylinders_with_capsules = true;
      end
      if isfield(options,'use_bullet')
        typecheck(options.use_bullet,'logical');
        if options.use_bullet
          checkDependency('bullet')
        end
        contact_options.use_bullet = options.use_bullet;
      else
        contact_options.use_bullet = checkDependency('bullet');
      end
      % NOTEST
    end
  end

  methods
    function [Vq, dVq] = qdotToV(obj, kinsol)
      if obj.mex_model_ptr ~= 0 && kinsol.mex
        Vq = velocityToPositionDotMappingmex(kinsol.mex_ptr);
        if kinsol.has_gradients
          [Vq, dVq] = eval(Vq);
          nq = length(kinsol.q);
          if isempty(dVq)
            dVq = zeros(numel(Vq), nq);
          else
            dVq = dVq(:, 1 : nq);
          end
        end
      else % fall back to the Matlab version:
        compute_gradient = nargout > 1;
        
        bodies = obj.body;
        nb = length(bodies);
        nv = obj.num_velocities;
        nq = obj.num_positions;
        Vq = zeros(nv, nq) * q(1); % to make TaylorVar work better
        
        if compute_gradient
          dVq = zeros(numel(Vq), nq) * q(1);
        end
        for i = 2 : nb
          bodyI = bodies(i);
          q_body = q(bodyI.position_num);
          if compute_gradient
            [VqJoint, dVqJoint] = jointQdot2v(bodyI, q_body);
            dVq = setSubMatrixGradient(dVq, dVqJoint, bodyI.velocity_num, bodyI.position_num, size(Vq), bodyI.position_num);
          else
            VqJoint = jointQdot2v(bodyI, q_body);
          end
          Vq(bodyI.velocity_num, bodyI.position_num) = VqJoint;
        end
      end
    end

    function [VqInv, dVqInv] = vToqdot(obj, kinsol)
      if obj.mex_model_ptr ~= 0 && kinsol.mex
        VqInv = velocityToPositionDotMappingmex(kinsol.mex_ptr);
        if kinsol.has_gradients
          [VqInv, dVqInv] = eval(VqInv);
          nq = length(kinsol.q);
          if isempty(dVqInv)
            dVqInv = zeros(numel(VqInv), nq);
          else
            dVqInv = dVqInv(:, 1 : nq);
          end
        end
      else % fall back to the Matlab version:
        compute_gradient = nargout > 1;
        bodies = obj.body;
        nb = length(bodies);
        nv = obj.num_velocities;
        nq = obj.num_positions;
        VqInv = zeros(nq, nv) * kinsol.q(1); % to make TaylorVar work better
        
        if compute_gradient
          dVqInv = zeros(numel(VqInv), nq) * kinsol.q(1);
        end
        for i = 2 : nb
          bodyI = bodies(i);
          q_body = kinsol.q(bodyI.position_num);
          if compute_gradient
            [VqInvJoint, dVqInvJoint] = jointV2qdot(bodyI, q_body);
            dVqInv = setSubMatrixGradient(dVqInv, dVqInvJoint, bodyI.position_num, bodyI.velocity_num, size(VqInv), bodyI.position_num);
          else
            VqInvJoint = jointV2qdot(bodyI, q_body);
          end
          VqInv(bodyI.position_num, bodyI.velocity_num) = VqInvJoint;
        end
      end
    end

    function y = output(obj,t,x,u)
      % The outputs of this dynamical system are concatenated from each of
      % the sensors.  If no sensor has been added to the system, then the
      % output defaults to the full robot state.

      checkDirty(obj);

      if isempty(obj.sensor)
        y = x;
      else
        if ~isDirectFeedthrough(obj)
          u=[];
        end
        y = [];
        for i=1:length(obj.sensor)
          y = [y; obj.sensor{i}.output(obj,t,x,u)];
        end
      end
    end

    function obj = setTerrain(obj,terrain)
      % Set the ground height profile which the contact points on the robot
      % may not penetrate.
      %
      % @param terrain a RigidBodyTerrain object

      if ~isempty(terrain)
        typecheck(terrain,'RigidBodyTerrain');
      end
      obj = removeTerrainGeometry(obj);
      obj.terrain = terrain;
      obj = addTerrainGeometry(obj);
    end

    function obj = addTerrainGeometries(varargin)
      errorDeprecatedFunction('addTerrainGeometry')
    end

    function obj = addTerrainGeometry(obj)
      if ~isempty(obj.terrain)
        geom = obj.terrain.getCollisionGeometry();
        if ~isempty(geom)
          if ~iscell(geom), geom={geom}; end
          for i=1:numel(geom)
            if ~any(cellfun(@(geometry) isequal(geom{i},geometry),obj.body(1).collision_geometry))
              obj = obj.addCollisionGeometryToBody(1,geom{i},'terrain');
              obj.dirty = true;
            end
          end
        end
        geom = obj.terrain.getVisualGeometry();
        if ~isempty(geom)
          if ~iscell(geom), geom={geom}; end
          for i=1:numel(geom)
            if ~any(cellfun(@(geometry) isequal(geom{i},geometry),obj.body(1).visual_geometry))
              obj.body(1).visual_geometry{end+1} = geom{i};
              obj.dirty = true;
            end
          end
        end
      end
    end

    function obj = removeTerrainGeometries(varargin)
      errorDeprecatedFunction('removeTerrainGeometry');
    end

    function obj = removeTerrainGeometry(obj)
      if ~isempty(obj.terrain)
        geom = obj.terrain.getCollisionGeometry();
        geom_contact_idx = cellfun(@(geometry) isequal(geom,geometry),obj.body(1).collision_geometry);
        obj.body(1).collision_geometry(geom_contact_idx) = [];

        geom = obj.terrain.getVisualGeometry();
        geom_visual_idx = cellfun(@(geometry) isequal(geom,geometry),obj.body(1).visual_geometry);
        obj.body(1).visual_geometry(geom_visual_idx) = [];

        obj.dirty = true;
      end
    end

    function [z,normal] = getTerrainHeight(obj,contact_pos)
      % Access method for querying the terrain height from the
      % RigidBodyTerrain object assigned to this manipulator
      %
      % @param contact_pos a 2xN or 3xN list of positions in the world
      % frame below which to compute the height of the terrain.
      %
      % @retval z a 1xN list of terrain heights in world coordinates
      % @retval normal a 3xN list of surface normals in world cooridinates

      [z,normal] = getHeight(obj.terrain,contact_pos(1:2,:));
    end

    function B = getB(obj,q,qd)
      % Note:  getB(obj) is ok iff there are no direct feedthrough force
      % elements.
      checkDirty(obj);

      B = obj.B;
      if length(obj.force)>0
        for i=1:length(obj.force)
          if (obj.force{i}.direct_feedthrough_flag)
            [~,B_force] = computeSpatialForce(obj.force{i},obj,q,qd);
            B = B+B_force;
          end
        end
      end

    end
    function n = getNumBodies(obj)
      % @ingroup Kinematic Tree
      n = length(obj.body);
    end

    function str = getLinkName(obj,body_ind)
      % str = getLinkName(obj,body_ind) returns a string containing the
      % link name the specified body if body_ind is a scalar. If body
      % ind is a vector, it returns a cell array containing the names of
      % the bodies specified by the elements of body_ind.
      %
      % @param obj - RigidBodyManipulator object
      % @param body_ind - Body index or vector of body indices
      %
      % @retval str - String (cell array of strings) containing the
      % requested linkname(s)
      %
      % @ingroup Kinematic Tree

      if numel(body_ind) > 1
        str = {obj.body(body_ind).linkname};
      else
        str = obj.body(body_ind).linkname;
      end
    end

    function obj = setGravity(obj,grav)
      sizecheck(grav,size(obj.gravity));
      obj.gravity = grav;
      obj.dirty = true;
    end
    
    function q0 = getRandomConfiguration(obj)
      q0 = zeros(getNumPositions(obj),1);
      for i=1:getNumBodies(obj)
        if any(obj.body(i).position_num>0)
          q0(obj.body(i).position_num) = getRandomConfiguration(obj.body(i));
        end
      end
    end
    
    function q = getZeroConfiguration(obj)
      q = zeros(getNumPositions(obj),1);
      for i=1:getNumBodies(obj)
        if any(obj.body(i).position_num>0)
          q(obj.body(i).position_num) = getZeroConfiguration(obj.body(i));
        end
      end
    end
    
    function x0 = getInitialState(obj)
      if ~isempty(obj.initial_state)
        x0 = obj.initial_state;
        return;
      end
      
      x0 = [ getRandomConfiguration(obj); .01*randn(getNumVelocities(obj),1)];
      if ~isempty(obj.state_constraints)
        attempts=0;
        success=false;
        while (~success)
          attempts=attempts+1;
          try
            [x0,success] = resolveConstraints(obj,x0);
          catch ex
            if strcmp(ex.identifier,'Drake:DrakeSystem:FailedToResolveConstraints');
              success=false;
            else
              rethrow(ex);
            end
          end
          if (~success)
            x0 = randn(obj.num_xd+obj.num_xc,1);
            if (attempts>=10)
              error('Drake:DrakeSystem:FailedToResolveConstraints','Failed to resolve state constraints on initial conditions after 10 tries');
            end
          end
        end
      end
      x0 = double(x0);      
    end

    function obj = setJointLimits(obj,jl_min,jl_max)
      obj = setJointLimits@Manipulator(obj,jl_min,jl_max);
      obj.dirty = true;
      for i = 1:getNumBodies(obj)
        pos_num = obj.body(i).position_num;
        if(any(pos_num~= 0)) % not the world body
          obj.body(i).joint_limit_min = jl_min(pos_num)';
          obj.body(i).joint_limit_max = jl_max(pos_num)';
        end
      end
    end

    function g = getGravity(obj,grav)
      g = obj.gravity;
    end

    function [f_friction, df_frictiondv] = computeFrictionForce(model,v)
      % Note: gradient is with respect to v, not q!
      compute_gradient = nargout > 1;

      nv = model.getNumVelocities();
      % Note: this will fail if damping is a trigpoly (we could handle this case, but need to do it carefully to not hurt performance)
      damping = zeros(nv, 1);
      % Note: this will fail if friction is a trigpoly (as it should)
      coulomb_friction = zeros(nv, 1);
      static_friction = zeros(nv, 1);
      coulomb_window = zeros(nv, 1);

      for i = 2 : model.getNumBodies()
        b = model.body(i);
        damping(b.velocity_num) = b.damping;
        coulomb_friction(b.velocity_num) = b.coulomb_friction;
        static_friction(b.velocity_num) = b.static_friction;
        coulomb_window(b.velocity_num) = b.coulomb_window;
      end

      f_friction = damping .* v;
      if compute_gradient
        df_frictiondv = diag(damping);
      end

      if any(coulomb_friction)
        f_friction = f_friction + min(1,max(-1,v./coulomb_window)).*coulomb_friction;
        if compute_gradient
          ind = find(abs(v)<coulomb_window');
          dind = coulomb_friction(ind)' ./coulomb_window(ind)';
          fc_drv = zeros(model.getNumVelocities(),1);
          fc_drv(ind) = dind;
          df_frictiondv = df_frictiondv + diag(fc_drv);
        end
      end
    end

    function ptr = getMexModelPtr(obj)
      % Please do note use this.
      % This access should not be allowed (the method should be private or not exist).
      % But it is currently used by QPControlBlock to pass to QPControllermex in drc/software/control
      ptr = obj.mex_model_ptr;
    end

    function [f,dfdq,dfdforce] = cartesianForceToSpatialForce(obj,kinsol,body_ind,point,force)
      % @param body_ind is an index of the body
      % @param point is a point on the rigid body (in body coords)
      % @param force is a cartesion force (in world coords)

      % convert force to body coordinates
      if (nargout>1)
        [ftmp,ftmpP,ftmpJ]=bodyKin(obj,kinsol,body_ind,[force,zeros(3,1)]);
      else
        ftmp=bodyKin(obj,kinsol,body_ind,[force,zeros(3,1)]);
      end

      force = ftmp(:,1)-ftmp(:,2);
      f = [ cross(point,force,1); force ];  % spatial force in body coordinates

      if (nargout>1)
        dforcedq = ftmpJ(1:3,:)-ftmpJ(4:6,:);
        dforcedforce = ftmpP(1:3,1:size(force,1))-ftmpP(4:6,1:size(force,1));
        dfdq = [ cross(repmat(point,1,size(dforcedq,2)),dforcedq); dforcedq ];
        dfdforce = [ cross(repmat(point,1,size(dforcedforce,2)),dforcedforce); dforcedforce];
      end
    end

    function [model,id]=addLink(model,b)
      % @ingroup Kinematic Tree
      typecheck(b,'RigidBody');
      sizecheck(b,1);
      model.body = [model.body,b];
      id = length(model.body);
      model.dirty = true;
    end

    function model=addJoint(model,name,type,parent_ind,child_ind,xyz,rpy,axis,damping,coulomb_friction,static_friction,coulomb_window,limits)
      % @ingroup Kinematic Tree
      typecheck(parent_ind,'double');
      typecheck(child_ind,'double');
      if (nargin<6) xyz=zeros(3,1); end
      if (nargin<7) rpy=zeros(3,1); end
      if (nargin<8) axis=[1;0;0]; end
      if (nargin<9) damping=0; end
      if (nargin<10 || isempty(coulomb_friction)) coulomb_friction=0; end
      if (nargin<11 || isempty(static_friction)) static_friction=0; end
      if (nargin<12 || isempty(coulomb_window)) coulomb_window=eps; end
      if (nargin<13)
        limits = struct();
        limits.joint_limit_min = -Inf;
        limits.joint_limit_max = Inf;
        limits.effort_min = -Inf;
        limits.effort_max = Inf;
        limits.velocity_limit = Inf;
      end

      child = model.body(child_ind);

      jointname = regexprep(name, '\.', '_', 'preservecase');
      if ~isempty(jointname) && ismember(lower(jointname),lower({model.body([model.body.robotnum]==child.robotnum).jointname}))
        model.warning_manager.warnOnce('Drake:RigidBodyManipulator:DuplicateJointName',['You already have a joint named ', jointname, ' on this robot.  This can cause problems later if you try to access elements of the state vector by name']);
      end
      child.jointname = jointname;
      rangecheck(parent_ind,1,getNumBodies(model));
      child.parent = parent_ind;

      child.Ttree = [rpy2rotmat(rpy), xyz; 0,0,0,1];

      child.floating = 0;
      switch lower(type)
        case {'revolute','continuous'}
          child.pitch = 0;
          child.damping = damping;
          child.coulomb_friction = coulomb_friction;
          child.static_friction = static_friction;
          child.coulomb_window = coulomb_window;

        case 'prismatic'
          child.pitch = inf;
          child.damping = damping;
          child.coulomb_friction = coulomb_friction;
          child.static_friction = static_friction;
          child.coulomb_window = coulomb_window;

        case 'fixed'
          child.pitch = nan;

        case 'floating_rpy'
          child.pitch = 0;
          child.floating = 1;
          limits.joint_limit_min = repmat(limits.joint_limit_min,1,6);
          limits.joint_limit_max = repmat(limits.joint_limit_max,1,6);
          limits.effort_min = repmat(limits.effort_min,1,6);
          limits.effort_max = repmat(limits.effort_max,1,6);
          limits.velocity_limit = repmat(limits.velocity_limit,1,6);

        case 'floating_quat'
          child.pitch = 0;
          child.floating = 2;
          limits.joint_limit_min = repmat(limits.joint_limit_min,1,7);
          limits.joint_limit_max = repmat(limits.joint_limit_max,1,7);
          limits.effort_min = repmat(limits.effort_min,1,7);
          limits.effort_max = repmat(limits.effort_max,1,7);
          limits.velocity_limit = repmat(limits.velocity_limit,1,7);

        otherwise
          error(['joint type ',type,' not supported (yet?)']);
      end
      child.joint_axis = axis;
      child.joint_limit_min = limits.joint_limit_min;
      child.joint_limit_max = limits.joint_limit_max;
      child.effort_min = limits.effort_min;
      child.effort_max = limits.effort_max;
      child.velocity_limit = limits.velocity_limit;

      model.body(child_ind) = child;
      model.dirty = true;
    end

    function model = addFloatingBase(model,parent,rootlink,xyz,rpy,joint_type)
      % note that the case matters.
      % use lower case for extrinsic (absolute) rotations, and upper case
      % for intrinsic (relative) rotations
      % @ingroup Kinematic Tree

      typecheck(parent,'double');      % these should be body indices
      typecheck(rootlink,'double');
      robotnum = model.body(rootlink).robotnum;

      if (nargin<6) joint_type = 'rpy'; end

      switch (joint_type)
        case 'rpy'  % extrinsic coordinates
          model = addJoint(model,'base','floating_rpy',parent,rootlink,xyz,rpy);

        case 'quat'
          model = addJoint(model,'base','floating_quat',parent,rootlink,xyz,rpy);

        case 'RPY'  % instrinsic coordinates
          body1 = RigidBody();
          body1.linkname = 'base_x';
          body1.robotnum=robotnum;
          model.body = [model.body,body1];
          body1_ind = length(model.body);
          model = addJoint(model,body1.linkname,'prismatic',parent,body1_ind,xyz,rpy,[1;0;0],0);

          body2=RigidBody();
          body2.linkname = 'base_y';
          body2.robotnum=robotnum;
          model.body = [model.body,body2];
          body2_ind = length(model.body);
          model = addJoint(model,body2.linkname,'prismatic',body1_ind,body2_ind,zeros(3,1),zeros(3,1),[0;1;0],0);

          body3=RigidBody();
          body3.linkname = 'base_z';
          body3.robotnum=robotnum;
          model.body = [model.body,body3];
          body3_ind = length(model.body);
          model = addJoint(model,body3.linkname,'prismatic',body2_ind,body3_ind,zeros(3,1),zeros(3,1),[0;0;1],0);

          body4=RigidBody();
          body4.linkname = 'base_relative_roll';
          body4.robotnum=robotnum;
          model.body = [model.body,body4];
          body4_ind = length(model.body);
          model = addJoint(model,body4.linkname,'revolute',body3_ind,body4_ind,zeros(3,1),zeros(3,1),[1;0;0],0);

          body5=RigidBody();
          body5.linkname = 'base_relative_pitch';
          body5.robotnum=robotnum;
          model.body = [model.body,body5];
          body5_ind = length(model.body);
          model = addJoint(model,body5.linkname,'revolute',body4_ind,body5_ind,zeros(3,1),zeros(3,1),[0;1;0],0);

          model = addJoint(model,'base_relative_yaw','revolute',body5_ind,rootlink,zeros(3,1),zeros(3,1),[0;0;1],0);

        case 'YPR' % intrinsic

          body1 = RigidBody();
          body1.linkname = 'base_x';
          body1.robotnum=robotnum;
          model.body = [model.body,body1];
          body1_ind = length(model.body);
          model = addJoint(model,body1.linkname,'prismatic',parent,body1_ind,xyz,rpy,[1;0;0],0);

          body2 = RigidBody();
          body2.linkname = 'base_y';
          body2.robotnum=robotnum;
          model.body = [model.body,body2];
          body2_ind = length(model.body);
          model = addJoint(model,body2.linkname,'prismatic',body1_ind,body2_ind,zeros(3,1),zeros(3,1),[0;1;0],0);

          body3= RigidBody();
          body3.linkname = 'base_z';
          body3.robotnum=robotnum;
          model.body = [model.body,body3];
          body3_ind = length(model.body);
          model = addJoint(model,body3.linkname,'prismatic',body2_ind,body3_ind,zeros(3,1),zeros(3,1),[0;0;1],0);

          body4=RigidBody();
          body4.linkname = ['base_relative_yaw'];
          body4.robotnum=robotnum;
          model.body = [model.body,body4];
          body4_ind = length(model.body);
          model = addJoint(model,body4.linkname,'revolute',body3_ind,body4_ind,zeros(3,1),zeros(3,1),[0;0;1],0);

          body5=RigidBody();
          body5.linkname = 'base_relative_pitch';
          body5.robotnum=robotnum;
          model.body = [model.body,body5];
          body5_ind = length(model.body);
          model = addJoint(model,body5.linkname,'revolute',body4_ind,body5_ind,zeros(3,1),zeros(3,1),[0;1;0],0);

          model = addJoint(model,'base_relative_roll','revolute',body5_ind,rootlink,zeros(3,1),zeros(3,1),[1;0;0],0);

        otherwise
          error('unknown floating base type');
      end
    end

    function model = addSensor(model,sensor)
      % Adds a sensor to the RigidBodyManipulator.  This modifies the
      % model.sensor parameter and marks the model as dirty.
      %
      % @param model existing RigidBodyManipulator the sensor should be
      %   added to
      % @param sensor sensor to add
      %
      % @retval new RigidBodyManipulator with the sensor added.

      typecheck(sensor,'RigidBodySensor');
      model.sensor{end+1}=sensor;
      model.dirty = true;
    end

    function model = compile(model)
      % After parsing, compute some relevant data structures that will be
      % accessed in the dynamics and visualization

      % reorder body list to make sure that parents before children in the
      % list (otherwise simple loops over bodies might not compute
      % kinematics/dynamics correctly)
      i=1;
      while(i<=length(model.body))
        if model.body(i).parent>0
          ind = model.body(i).parent;
          if (ind>i)
            model = updateBodyIndices(model,[1:i-1,ind,i:ind-1,ind+1:length(model.body)]);
            i=i-1;
          end
        end
        i=i+1;
      end

      model = removeFixedJoints(model);

      % Clear cached contact points
      model.cached_terrain_contact_points_struct = [];

      %% update RigidBodyElements
      % todo: use applyToAllRigidBodyElements (but will have to generalize
      % it to take multiple outputs, or make sure onCompile doesn't have to
      % return the second argument)
      for i=1:length(model.force)
        [new_element, model] = model.force{i}.onCompile(model);
        model.force{i} = new_element;
      end

      for i=1:length(model.sensor)
        [new_element, model] = model.sensor{i}.onCompile(model);
        model.sensor{i} = new_element;
      end

      for i=1:length(model.actuator)
        [new_element, model] = model.actuator(i).onCompile(model);
        model.actuator(i) = new_element;
      end

      % set position and velocity vector indices
      num_q=0;num_v=0;
      for i=1:length(model.body)
        if model.body(i).parent>0
          if (model.body(i).floating==1)
            model.body(i).position_num=num_q+(1:6)';
            num_q=num_q+6;
            model.body(i).velocity_num=num_v+(1:6)';
            num_v=num_v+6;
          elseif (model.body(i).floating==2)
            model.body(i).position_num=num_q+(1:7)';
            num_q=num_q+7;
            model.body(i).velocity_num=num_v+(1:6)';
            num_v=num_v+6;
          else
            num_q=num_q+1;
            model.body(i).position_num=num_q;
            num_v=num_v+1;
            model.body(i).velocity_num=num_v;
          end
        else
          model.body(i).position_num=0;
          model.body(i).velocity_num=0;
        end
      end

      u_limit = repmat(inf,length(model.actuator),1);
      u_limit = [-u_limit u_limit]; % lower/upper limits

      %% extract B matrix
      B = sparse(num_v,0);
      for i=1:length(model.actuator)
        joint = model.body(model.actuator(i).joint);
        B(joint.velocity_num,i) = model.actuator(i).reduction;
        u_limit(i,1) = joint.effort_min;
        u_limit(i,2) = joint.effort_max;
      end

      for i=1:length(model.force)
        if model.force{i}.direct_feedthrough_flag
          input_num = size(B,2)+1;
          B(1,size(B,2)+1) = 0; %Add another column to B
          model.force{i} = model.force{i}.setInputNum(input_num);
          u_limit(size(u_limit,1)+1,:) = model.force{i}.input_limits;
        end
      end
      model.B = full(B);

      model = setNumInputs(model,size(model.B,2));
      model = setNumPositions(model,num_q);
      model = setNumVelocities(model,num_v);
      model = setNumOutputs(model,num_q+num_v);

      [model,paramframe,~,pmin,pmax] = constructParamFrame(model);
      if ~isequal_modulo_transforms(paramframe,getParamFrame(model)) % let the previous handle stay valid if possible
        model = setParamFrame(model,paramframe);
      end
      model = setParamLimits(model,pmin,pmax);

      if getNumInputs(model)>0
        inputframe = constructInputFrame(model);
        if ~isequal_modulo_transforms(inputframe,getInputFrame(model)) % let the previous handle stay valid if possible
          model = setInputFrame(model,inputframe);
        end
      end

      if getNumStates(model)>0
        model = constructStateFrame(model);
      end

      if any([model.body.has_position_sensor])
        % add a rigid body joint sensor if necessary
        robotnums = unique([model.body([model.body.has_position_sensor]).robotnum]);
        for i=1:numel(robotnums)
          already_has_sensor = false;
          for j=1:length(model.sensor)
            if isa(model.sensor{j},'RigidBodyJointSensor') && model.sensor{j}.robotnum==i
              already_has_sensor=true;
              break;
      end
          end
          if ~already_has_sensor
            model = addSensor(model,RigidBodyJointSensor(model,i));
          end
        end
      end

      if length(model.sensor)>0
        feedthrough = false;
        for i=1:length(model.sensor)
          model.sensor{i} = model.sensor{i}.compile(model);
          outframe{i} = model.sensor{i}.coordinate_frame;
          feedthrough = feedthrough || model.sensor{i}.isDirectFeedthrough;
        end
        fr = MultiCoordinateFrame.constructFrame(outframe);
        if ~isequal_modulo_transforms(fr,getOutputFrame(model)) % let the previous handle stay valid if possible
          model = setNumOutputs(model,fr.dim);
          model = setOutputFrame(model,fr);
        end
        model = setDirectFeedthrough(model,feedthrough);
      else
        model = setOutputFrame(model,getStateFrame(model));  % output = state
        model = setDirectFeedthrough(model,false);
      end

      if getNumPositions(model)>0
        model = model.setJointLimits([model.body.joint_limit_min]',[model.body.joint_limit_max]');
      end

      model = model.setInputLimits(u_limit(:,1),u_limit(:,2));

      %% check basic assumption from kinematics:
      for i=1:length(model.body)
        valuecheck(model.body(i).Ttree(end,1:end-1),0);
        valuecheck(model.body(i).Ttree(end,end),1);
      end

      model = adjustCollisionGeometry(model);
      model = setupCollisionFiltering(model);

      model.dirty = false;

      model = createMexPointer(model);

      % collisionDetect may require the mex version of the manipulator,
      % so it should go after createMexPointer
      [phi,~,~,~,idxA,idxB] = model.collisionDetect(getZeroConfiguration(model));
      model.num_contact_pairs = length(phi);

      % cache the full set of terrain contact points
      model.cached_terrain_contact_points_struct = model.getTerrainContactPoints();

      % can't really add the full complementarity constraints here,
      % since the state constraints only take x as the input.  so
      % just adding the non-penetration constraints
      function [phi,dphi,ddphi] = nonpenetrationConstraint(q)
        kinsol = doKinematics(model,q);
        if nargout>2
          [phi,~,~,~,~,~,~,~,dphi,~,ddphi] = contactConstraints(model,kinsol,false,model.contact_options);
        elseif nargout>1
          [phi,~,~,~,~,~,~,~,dphi] = contactConstraints(model,kinsol,false,model.contact_options);
        else
          phi = contactConstraints(model,kinsol,false,model.contact_options);
        end
      end

      if (model.num_contact_pairs>0)
        nonpenetration_constraint = FunctionHandleConstraint(zeros(model.num_contact_pairs,1),inf(model.num_contact_pairs,1),model.getNumPositions,@nonpenetrationConstraint,2);
        nonpenetration_constraint = nonpenetration_constraint.setName(cellstr(num2str([idxA;idxB]','non-penetration: body %d <-> body %d')));
        if isempty(model.contact_constraint_id)
          [model,id] = addStateConstraint(model,nonpenetration_constraint,1:model.getNumPositions);
          model.contact_constraint_id = id;
        else
          model = updateStateConstraint(model,model.contact_constraint_id,nonpenetration_constraint,1:model.getNumPositions);
        end
      elseif ~isempty(model.contact_constraint_id)
        model = updateStateConstraint(model,model.contact_constraint_id,NullConstraint(model.getNumPositions),1:model.getNumPositions);
      end
      
      quat_inds = find([model.body.floating]==2);
      for j=1:length(quat_inds)
        bind = quat_inds(j);
        quat_norm_constraint = QuadraticConstraint(1,1,2*eye(4),zeros(4,1));
        quat_norm_constraint = quat_norm_constraint.setName({[model.body(bind).jointname,' quat norm = 1']});
        if length(model.quat_norm_constraint_id)<j
          % note: these could be PositionEqualityConstraints, but then they
          % would be evaluated in the lcp solution.  But we currently 
          % special case the normalization in the lcp. 
          [model,id] = addStateConstraint(model,quat_norm_constraint,model.body(bind).position_num(4:7));
          model.quat_norm_constraint_id(j) = id;
        else
          model = updateStateConstraint(model,model.quat_norm_constraint_id(j),quat_norm_constraint,model.body(bind).position_num(4:7));
        end
      end
      for j=(length(quat_inds)+1):length(model.quat_norm_constraint_id)
        model = updateStateConstraint(model,model.quat_norm_constraint_id(j),NullConstraint(0),1);
      end

      for j=1:length(model.loop)
        [loop,model] = updateConstraints(model.loop(j),model);
        model.loop(j) = loop;
      end

      for j=1:length(model.position_constraints)
        % todo: generalize this by moving the updateConstraint logic above into
        % drakeFunction.RBM
        if isa(model.position_constraints{j},'DrakeFunctionConstraint') && isa(model.position_constraints{j}.fcn,'drakeFunction.kinematic.CableLength')
          cable_length_function = setRigidBodyManipulator(model.position_constraints{j}.fcn,model);
          constraint = DrakeFunctionConstraint(model.position_constraints{j}.lb,model.position_constraints{j}.ub,cable_length_function);
          constraint = setName(constraint,cable_length_function.name);
          constraint.grad_level = 2; %declare that the second derivative is provided
          constraint.grad_method = 'user';
          model = updatePositionEqualityConstraint(model,j,constraint); 
        end
      end

      if (model.num_contact_pairs>0)
        warnOnce(model.warning_manager,'Drake:RigidBodyManipulator:UnsupportedContactPoints','Contact is not supported by the dynamics methods of this class.  Consider using TimeSteppingRigidBodyManipulator or HybridPlanarRigidBodyManipulator');
      end

%      H = manipulatorDynamics(model,zeros(model.num_positions,1),zeros(model.num_positions,1));
%      if cond(H)>1e3
%        warning('Drake:RigidBodyManipulator:SingularH','H appears to be singular (cond(H)=%f).  Are you sure you have a well-defined model?',cond(H));
%      end
    end

    function indices = findJointIndices(model, str)
      model.warning_manager.warnOnce('Drake:RigidBodyManipulator:findJointIndicesDeprecated','findJointIndices has been replaced with findPositionIndices.  please update your code');
      indiced = findPositionIndices(model,str);
    end

    function indices = findPositionIndices(model, str)
      %findPositionIndices Returns position indices in the state vector for joints whose
      % name contains a specified string.
      %   @param str (sub)string to be searched for
      %   @retvall indices array of indices into state vector
      coordinates = model.getStateFrame().getCoordinateNames();
      indices = find(~cellfun('isempty',strfind(coordinates(1:getNumPositions(model)),str)));
    end

    function indices = findVelocityIndices(model, str)
      %findJointVelocityIndices Returns velocity indices in the state vector for joints whose
      % name contains a specified string.
      %   @param str (sub)string to be searched for
      %   @retvall indices array of indices into state vector
      coordinates = model.getStateFrame().getCoordinateNames();
      indices = find(~cellfun('isempty',strfind(coordinates((getNumPositions(model)+1):end),str)));
    end


    function body_ind = findLinkInd(model,varargin)
      model.warning_manager.warnOnce('Drake:RigidBodyManipulator:finkLinkIndDeprecated','findLinkInd has been replaced with findLinkId.  please update your code');
      body_ind = findLinkId(model,varargin{:});
    end

    function body_id = findLinkId(model,linkname,robot,error_level)
      % @param robot can be the robot number or the name of a robot
      % robot<0 means look at all robots
      % @param error_level >0 for throw error, 0 for throw warning, <0 for do nothing. @default throw error
      % @ingroup Kinematic Tree
      if nargin<3 || isempty(robot), robot=-1; end
      linkname = lower(linkname);
      linkname=regexprep(linkname, '[\[\]\\\/\.]+', '_', 'preservecase');

      if ischar(robot)
        robot = strmatch(lower(robot),lower(model.name));
      end
      items = strfind(lower({model.body.linkname}),linkname);
      ind = find(~cellfun(@isempty,items));
      if (robot>=0), ind = ind(ismember([model.body(ind).robotnum],robot)); end
      if (length(ind)>0) % then handle removed fixed joints
        i=1;
        while i<=length(ind)
          if ~strcmp(lower(model.body(ind(i)).linkname),linkname)
            %          sublinks=strsplit(lower(model.body(i).linkname),'+');  % for >R2013a
            sublinks = strread(lower(model.body(ind(i)).linkname),'%s','delimiter','+');  % for older versions
            subind = strmatch(linkname,sublinks,'exact');
            if isempty(subind),
              ind(i)=[]; % not actually a match
              i=i-1;
            elseif subind>1
              if nargin>3 && error_level>0
                error('Drake:RigidBodyManipulator:WeldedLinkInd',['found ', linkname,' but it has been welded to it''s parent link (and the link''s coordinate frame may have changed).']);
              else
                warning('Drake:RigidBodyManipulator:WeldedLinkInd',['found ', linkname,' but it has been welded to it''s parent link (and the link''s coordinate frame may have changed).']);
              end
            end
          end
          i=i+1;
        end
      end
      if (length(ind)~=1)
        if (nargin<4 || error_level>0)
          if robot < 0
            error('Drake:RigidBodyManipulator:UniqueLinkNotFound', ...
              'couldn''t find unique link %s.',linkname);
          else
            error('Drake:RigidBodyManipulator:UniqueLinkNotFound', ...
              'couldn''t find unique link %s on robot number %d.', ...
              linkname,robot);
          end
        else
          body_id=ind;
          if (error_level==0)
            warning(['couldn''t find unique link ' ,linkname]);
          end
        end
      else
        body_id = ind;
      end
    end

    function body = findLink(model,linkname,varargin)
      % @ingroup Deprecated
      error('the finkLink method has been deprecated.  if you really must get a *copy* of the body, then use finkLinkId followed by getBody');
    end

    function is_valid = isValidLinkIndex(obj,idx)
      model.warning_manager.warnOnce('Drake:RigidBodyManipulator:isValidLinkIndexDeprecated','isValidLinkIndex has been replaced with isValidLinkId.  please update your code');
    end

    function is_valid = isValidLinkId(obj,id)
      % @ingroup Kinematic Tree
      if ~isnumeric(id)
        is_valid=false(size(id));
      else
        is_valid = id >= 1 & id <= obj.getNumBodies() & mod(id,1) == 0;
      end
    end

    function frame_id = findFrameId(model,name,robotnum)
      % @param name is the string name to search for
      % @param robotnum if specified restricts the search to a particular
      % robot
      if nargin<3, robotnum=-1; end
      if ~isempty(model.frame)
        items = strfind(lower({model.frame.name}),lower(name));
        ind = find(~cellfun(@isempty,items));
        if (robotnum~=-1), ind = ind([model.body(model.frame(ind).body_ind).robotnum]==robotnum); end
      else
        ind = [];
      end
      if numel(ind)~=1
        error('Drake:RigidBodyManipulator:UniqueFrameNotFound',['Cannot find unique frame named ', name, ' on robot number ',num2str(robotnum)]);
      end
      frame_id = -ind;  % keep frame_ind distinct from body_ind
    end

    function body = getBody(model,body_ind)
      % @ingroup Kinematic Tree
      body = model.body(body_ind);
    end

    function model = setBody(model,body_ind,body)
      % @ingroup Kinematic Tree
      typecheck(body_ind,'numeric');
      typecheck(body,'RigidBody');
      model.body(body_ind) = body;
      model.dirty = true;
    end

    function model = setForce(model,force_ind,force)
      typecheck(force_ind,'numeric');
      typecheck(force,'RigidBodyForceElement');
      model.force{force_ind} = force;
      model.dirty = true;
    end

    function [model,frame_id] = addFrame(model,frame)
      % @ingroup Kinematic Tree
      typecheck(frame,'RigidBodyFrame');
      model.frame = vertcat(model.frame,frame);
      frame_id = -(numel(model.frame));
      model.dirty = true;
    end

    function frame = getFrame(model,frame_id)
      % @ingroup Kinematic Tree
      frame = model.frame(-frame_id);
    end

    function model = setFrame(model,frame_id,frame)
      % @ingroup Kinematic Tree
      typecheck(frame_id,'numeric');
      typecheck(frame,'RigidBodyFrame');
      model.frame(-frame_id) = frame;
      model.dirty = true;
    end


    function str = getBodyOrFrameName(model,body_or_frame_id)
      if (body_or_frame_id>0)
        str = model.body(body_or_frame_id).linkname;
      else
        str = model.frame(-body_or_frame_id).name;
      end
    end

    function model = setParams(model,p)
      fr = getParamFrame(model);
      if isa(p,'Point')
        p = double(inFrame(p,fr));
      else
        sizecheck(p,fr.dim);
      end
      k=1;
      for i=1:min(numel(model.name),numel(model.param_db))
        pn = fieldnames(model.param_db{i});
        for j=1:numel(pn)
          model.param_db{i}.(pn{j}).value = p(k);
          k=k+1;
        end
      end

      model = applyToAllRigidBodyElements(model,'updateParams',fr.getPoly,p);

      model = compile(model);
    end

    function p = getParams(model)
      p = [];
      for i=1:min(numel(model.name),numel(model.param_db))
        if ~isempty(model.param_db{i})
          pn = fieldnames(model.param_db{i});
          for j=1:numel(pn)
            p = vertcat(p,model.param_db{i}.(pn{j}).value);
          end
        end
      end
      p = Point(getParamFrame(model),p);
    end

    function model = weldJoint(model,body_ind_or_joint_name,robot)
      % @ingroup Kinematic Tree
      if ischar(body_ind_or_joint_name)
        if nargin>2
          body_ind_or_joint_name = findJointId(model,body_ind_or_joint_name,robot);
        else
          body_ind_or_joint_name = findJointId(model,body_ind_or_joint_name);
        end
      end

      typecheck(body_ind_or_joint_name,'numeric');
      model.body(body_ind_or_joint_name).pitch = nan;
      model.dirty = true;
    end

    function body_ind = findJointInd(model,varargin)
      model.warning_manager.warnOnce('Drake:RigidBodyManipulator:finkJointIndDeprecated','findJointInd has been replaced with findJointId.  please update your code');
      body_ind = findJointId(model,varargin{:});
    end

    function body_id = findJointId(model,jointname,robot_num,error_level)
      % @param robot_num can be the robot number or the name of a robot
      % robot_num<0 means look at all robots
      % @ingroup Kinematic Tree
      if nargin<3 || isempty(robot_num), robot_num=-1; end
      jointname = lower(jointname);
      if ischar(robot_num) robot_num = strmatch(lower(robot_num),lower({model.name})); end
      ind = find(strcmp(jointname,lower({model.body.jointname})));
      if (robot_num>=0), ind = ind([model.body(ind).robotnum]==robot_num); end
      if (length(ind)~=1)
        if (nargin<4 || error_level>0)
          error('Drake:RigidBodyManipulator:UniqueJointNotFound',['couldn''t find unique joint ' ,jointname]);
        else
          body_id=0;
          if (error_level==0)
            warning('Drake:RigidBodyManipulator:UniqueJointNotFound',['couldn''t find unique joint ' ,jointname]);
          end
        end
      else
        body_id = ind;
      end
    end

    function body = findJoint(model,jointname,robot)
      % @ingroup Deprecated
      error('the finkJoint method has been deprecated.  if you really must get a *copy* of the body associated with this joint, then use finkJointInd followed by getBody');
    end

    function b=leastCommonAncestor(model,body1,body2)
      % recursively searches for the lowest body in the tree that is an
      % ancestor to both body1 and body2
      % @ingroup Kinematic Tree

      typecheck(body1,'double');  % takes in body indices
      typecheck(body2,'double');

      b=body2;
      if (body1==body2) return; end

      % check if body1 is an ancestor to body2
      while b.parent>0
        b=b.parent;
        if (body1==b) return; end
      end

      % body1 is not an ancestor to body2.  check body1's parent (and
      % recurse)
      b = leastCommonAncestor(model,body1.parent,body2);
    end

    function terrain_contact_point_struct = ...
        getTerrainContactPoints(obj,body_idx,contact_groups)
      % terrain_contact_point_struct = getTerrainContactPoints(obj)
      % returns a structure array containing the terrain contact points
      % on all bodies of this manipulator.
      %
      % terrain_contact_point_struct = getTerrainContactPoints(obj,body_idx)
      % returns a structure array containing the terrain contact points
      % on the bodies specified by body_idx.
      %
      % For a general description of terrain contact points see
      % <a href="matlab:help RigidBodyGeometry/getTerrainContactPoints">RigidBodyGeometry/getTerrainContactPoints</a>
      %
      % @param obj - RigidBodyManipulator object
      % @param body_idx - vector of body-indices indicating the bodies
      %                   for which terrain contact points should be
      %                   found @default All bodies except the world
      % @param contact_groups - (optional) cell array of cell arrays
      %   containing contact group names for each body
      % @retval terrain_contact_point_struct - nx1 structure array,
      %   where n is the number of bodies with terrain contact points.
      %   Each element has the following fields
      %     * idx - Index of a body in the RigidBodyManipulator
      %     * pts - 3xm array containing points on the body specified by
      %             idx (in body frame) that can collide with arbitrary
      %             terrain.
      %
      % See also RigidBodyGeometry/getTerrainContactPoints,
      % RigidBodyManipulator/terrainContactPositions
      checkDirty(obj);
      if nargin == 1 && ~isempty(obj.cached_terrain_contact_points_struct)
        terrain_contact_point_struct = obj.cached_terrain_contact_points_struct;
      else
        if nargin < 2
          body_idx = 2:obj.getNumBodies(); % World-fixed objects can't collide
          % with the terrain
        end
        if nargin >= 3
          if all(cellfun(@ischar,contact_groups))
            contact_groups = {contact_groups};
          end
          if numel(contact_groups) == 1
            contact_groups = repmat(contact_groups,size(body_idx));
          else
            sizecheck(contact_groups,size(body_idx));
          end
        end
        terrain_contact_point_struct = struct('pts',{},'idx',{});
        for i = 1:length(body_idx)
          bi=body_idx(i);
          if bi ~= 1
            if nargin < 3
              pts = getTerrainContactPoints(obj.body(bi));
            else
              pts = getTerrainContactPoints(obj.body(bi),contact_groups{i});
            end
            if ~isempty(pts)
              terrain_contact_point_struct(end+1) = struct('pts',pts,'idx',bi);
            end
          end
        end
      end
    end

    function varargout = getContactShapeGroupNames(varargin)
      errorDeprecatedFunction('getCollisionGeometryGroupNames');
    end

    function groups = getCollisionGeometryGroupNames(model)
      groups = {};
      for i=1:length(model.body)
        groups = horzcat(groups,model.body(i).collision_geometry_group_names);
      end
      groups = unique(groups);
    end

    function model = removeCollisionGroups(model,contact_groups,robotnum)
      % model = removeCollisionGroups(model,contact_groups,robotnum) returns
      % the model with the specified contact groups removed
      %
      % @param model          -- RigidBodyManipulator object
      % @param contact_groups -- String or cell array of strings specifying the
      %                          contact groups to be removed
      % @param robotnum       -- Vector of robot indices to which operation
      %                          will be restricted. Optional.
      %                          @default 1:numel(model.name)
      if nargin < 3,          robotnum = 1:numel(model.name); end
      if all(robotnum == -1),  robotnum = 0:numel(model.name); end
      for i=1:length(model.body)
        if ismember(model.body(i).robotnum,robotnum)
          model.body(i) = removeCollisionGroups(model.body(i),contact_groups);
        end
      end
      model.dirty = true;
    end

    function model = removeCollisionGroupsExcept(model,contact_groups,robotnum,body_ids)
      % model = removeCollisionGroups(model,contact_groups,robotnum) returns
      % the model with all contact groups removed except for those specified
      %
      % @param model          -- RigidBodyManipulator object
      % @param contact_groups -- String or cell array of strings specifying the
      %                          contact groups to be preserved.
      % @param robotnum       -- Vector of robot indices to which operation
      %                          will be restricted. Optional.
      %                          @default 1:numel(model.name)
      % @param body_ids       -- Vector of body indices to which operation
      %                          will be restricted. Optional.
      %                          @default 1:numel(model.body)
      if nargin < 4,          body_ids = 1:numel(model.body); end
      if nargin < 3,          robotnum = 1:numel(model.name); end
      if all(robotnum == -1),  robotnum = 0:numel(model.name); end
      for i=body_ids
        if ismember(model.body(i).robotnum,robotnum)
          model.body(i) = removeCollisionGroupsExcept(model.body(i),contact_groups);
        end
      end
      model.dirty = true;
    end

    function body_idx_or_frame_id = parseBodyOrFrameID(obj,body_or_frame,robotnum)
      % body_idx = parseBodyOrFrameID(obj,body_or_frame) returns the body index or frame
      % id associated with the input.
      % @param obj            -- RigidBodyManipulator object
      % @param body_or_frame  -- Can be either:
      %                           * Numeric body index or frame id
      %                           * String containing body or frame name
      % @param robotnum       -- Scalar restricting the search to a particular
      %                          robot. Optional. @default -1 (all robots)
      %
      % @retval body_idx_or_frame_id  -- Numeric body index or frame id
      typecheck(body_or_frame,{'numeric','char'});
      if nargin < 3, robotnum = -1; end
      if isnumeric(body_or_frame)
        sizecheck(body_or_frame,[1,1]);
        body_idx_or_frame_id = body_or_frame;
      else % then it's a string
        try
          body_idx_or_frame_id = findLinkId(obj,body_or_frame,robotnum);
        catch ex
          if strcmp(ex.identifier,'Drake:RigidBodyManipulator:UniqueLinkNotFound')
            try
              body_idx_or_frame_id = findFrameId(obj,body_or_frame,robotnum);
            catch ex2
              if strcmp(ex.identifier,'Drake:RigidBodyManipulator:UniqueLinkNotFound')
                if robotnum == -1
                  error('Drake:RigidBodyManipulator:UniqueFrameOrLinkNotFound', ...
                    'Cannot find unique link or frame named %s',body_or_frame);
                else
                  error('Drake:RigidBodyManipulator:UniqueFrameOrLinkNotFound', ...
                    'Cannot find unique link or frame named %s on robot %d', ...
                    body_or_frame, robotnum);
                end
              else
                rethrow(ex2);
              end
            end
          else
            rethrow(ex);
          end
        end
      end
    end

    function obj = addContactShapeToBody(varargin)
      errorDeprecatedFunction('addCollisionGeometryToBody');
    end

    function obj = addCollisionGeometryToBody(obj,body_id,geometry,varargin)
      % obj = addCollisionGeometryToBody(obj,body_id,geometry,group_name)
      %
      % obj must be re-compiled after calling this method
      %
      % @param obj - RigidBodyManipulator object
      % @param body_id - Body index or body name
      % @param geometry - RigidBodyGeometry (or child class) object
      % @param group_name - String containing the name of the collision group
      %   (optional) @default 'default'

      body_idx = obj.parseBodyOrFrameID(body_id);
      obj.body(body_idx) = obj.body(body_idx).addCollisionGeometry(geometry, varargin{:});
      obj.dirty = true;
    end

    function varargout = addVisualShapeToBody(varargin)
      errorDeprecatedFunction('addVisualGeometryToBody');
    end

    function obj = addVisualGeometryToBody(obj,body_id,geometry)
      % obj = addCollisionGeometryToBody(obj,body_id,geometry)
      %
      % @param obj - RigidBodyManipulator object
      % @param body_id - Body index or body name
      % @param geometry - RigidBodyGeometry (or child class) object
      body_idx = obj.parseBodyOrFrameID(body_id);
      obj.body(body_idx).visual_geometry{end+1} = geometry;
    end

    function varargout = addShapeToBody(varargin)
      errorDeprecatedFunction('addGeometryToBody');
    end

    function obj = addGeometryToBody(obj,body_id,geometry,varargin)
      % obj = addGeometryToBody(obj,body_id,geometry)
      %
      % @param obj - RigidBodyManipulator object
      % @param body_id - Body index or body name
      % @param geometry - RigidBodyGeometry (or child class) object
      % @param group_name - String containing the name of the collision group
      %   (optional) @default 'default'
      obj = obj.addVisualGeometryToBody(body_id,geometry);
      obj = obj.addCollisionGeometryToBody(body_id,geometry,varargin{:});
    end

    function varargout = removeShapeFromBody(varargin)
      errorDeprecatedFunction('removeVisualGeometryFromBody');
    end

    function obj = removeVisualGeometryFromBody(obj, body_id, geometry_name)
      % Removes all geometry from a given body that match a name
      %
      % @param body_id body to remove geometry from
      % @param geometry_name name to match (will remove the first match)
      %
      % @retval obj updated object

      body_idx = obj.parseBodyOrFrameID(body_id);

      removed_count = 0;

      for i = 1:length(obj.body(body_idx).visual_geometry)

        index = i - removed_count;

        if strcmp(obj.body(body_idx).visual_geometry{index}.name, geometry_name) == true

          % remove this geometry
          obj.body(body_idx).visual_geometry(index) = [];
          removed_count = removed_count + 1;

        end
      end
    end

    function varargout = replaceContactShapesWithCHull(varargin)
      errorDeprecatedFunction('replaceCollisionGeometryWithConvexHull');
    end

    function model = replaceCollisionGeometryWithConvexHull(model,body_indices,varargin)
      if any(body_indices==1)
        model = removeTerrainGeometry(model);
      end
      for body_idx = reshape(body_indices,1,[])
        model.body(body_idx) = replaceCollisionGeometryWithConvexHull(model.body(body_idx),varargin{:});
      end
      if any(body_indices==1)
        model = addTerrainGeometry(model);
      end
      model.dirty = true;
    end

    function drawKinematicTree(model)
      % depends on having graphviz installed
      % todo: make that a dependency in configure?
      % @ingroup Kinematic Tree

      A = cell(length(model.body));
      for i=1:length(model.body)
        if model.body(i).parent>0
          rpy = rotmat2rpy(model.body(i).Ttree(1:3,1:3));
          A{model.body(i).parent,i} = [model.body(i).jointname,'\npos:', num2str(model.body(i).Ttree(1:3,4)'),'\nrpy: ',num2str(rpy','%.2f  ')];
        end
      end
      for i=1:length(model.loop)
        bodyA = model.frame(-model.loop(i).frameA).body_ind;
        bodyB = model.frame(-model.loop(i).frameB).body_ind;
        A{bodyA,bodyB} = ['loop',num2str(i),':',model.loop(i).name];%,'\npt1:', num2str(model.loop(i).pt1'),'\npt2:', num2str(model.loop(i).pt2'),'\naxis: ',num2str(model.loop(i).axis')];
      end
      node_names = {model.body.linkname};
%      node_names = regexprep({model.body.linkname},'+(.)*','');
      drawGraph(A,node_names);
    end

    function drawLCMGLAxes(model,lcmgl,q,body_indices)
      if nargin < 4
        body_indices = 1:model.getNumBodies();
      end
      kinsol = doKinematics(model,q,false,false);
      for i=body_indices
        applyTransform(lcmgl,kinsol.T{i});
        lcmgl.glDrawAxes();
        applyTransform(lcmgl,inv(kinsol.T{i}));
      end
      function applyTransform(lcmgl,T)
        a = rotmat2axis(T(1:3,1:3));
        lcmgl.glTranslated(T(1,4),T(2,4),T(3,4));
        lcmgl.glRotated(a(4)*180/pi,a(1),a(2),a(3));
      end
    end

    function drawLCMGLClosestPoints(model,lcmgl,kinsol,varargin)
      if ~isstruct(kinsol)
        kinsol = model.doKinematics(kinsol);
      end
      [~,~,xA,xB,idxA,idxB] = model.closestPoints(kinsol,varargin{:});
      for i = 1:length(idxA)
        xA_in_world = forwardKin(model,kinsol,idxA(i),xA(:,i));
        xB_in_world = forwardKin(model,kinsol,idxB(i),xB(:,i));

        lcmgl.glColor3f(0,0,0); % black

        lcmgl.glBegin( lcmgl.LCMGL_LINES);
        lcmgl.glVertex3f(xA_in_world(1), xA_in_world(2), xA_in_world(3));
        lcmgl.glVertex3f(xB_in_world(1), xB_in_world(2), xB_in_world(3));
        lcmgl.glEnd();

        lcmgl.glColor3f(1,0,0); % red

        lcmgl.sphere(xA_in_world,.01,20,20);
        lcmgl.sphere(xB_in_world,.01,20,20);
      end
      lcmgl.glColor3f(.7,.7,.7); % gray
    end

    function drawLCMGLGravity(model,q,gravity_visual_magnitude)
        % draws a vector centered at the robot's center
        % of mass having the direction of the gravitational
        % force on the robot.
        %
        % @param q the position of the robot
        % @param gravity_visual_magnitude specifies the visual length of
        % the vector representing the gravitational force.

        if (nargin<3), gravity_visual_magnitude=0.25; end
        gravity_force = getMass(model)*model.gravity;
        vector_scale = gravity_visual_magnitude/norm(gravity_force,2);
        lcmgl = drake.matlab.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton,'Gravity');
        lcmgl.glColor3f(1,0,0);
        lcmgl.drawVector3d(getCOM(model,q),vector_scale*gravity_force);
        lcmgl.switchBuffers();
    end

    function drawLCMGLForces(model,q,qd,gravity_visual_magnitude)
        % draws the forces and torques on the robot. They are
        % spatial vectors (wrenches) and are drawn at the body's
        % origin on which they act. Forces are in green and torques
        % in purple. The magnitude of each vector is scaled by
        % the same amount that the vector corresponding to the
        % gravitational force on the robot would be scaled in order to
        % make it have a norm equal to gravity_visual_magnitude. Use
        % drawLCMGLGravity to draw the gravity.
        %
        % @param q the position of the robot
        % @param qd the velocities of the robot
        % @param gravity_visual_magnitude specifies the (would-be) visual
        % length of the vector representing the gravitational force.

        if (nargin<4), gravity_visual_magnitude=0.25; end
        gravity_force = getMass(model)*model.gravity;
        vector_scale = gravity_visual_magnitude/norm(gravity_force,2);

        kinsol = doKinematics(model,q);
        force_vectors = {};
        for i=1:length(model.force)
            if ~model.force{i}.direct_feedthrough_flag
                force_element = model.force{i};
                force_type = class(force_element);
                if isprop(force_element,'child_body')
                    body_ind = force_element.child_body;
                else
                    body_frame = getFrame(model,force_element.kinframe);
                    body_ind = body_frame.body_ind;
                end
                f_ext = computeSpatialForce(force_element,model,q,qd);
                body_wrench = f_ext(:,body_ind);
                pos = forwardKin(model,kinsol,body_ind,[zeros(3,1),body_wrench(1:3),body_wrench(4:6)]);
                point = pos(:,1);
                torque_ext = pos(:,2)-point;
                force_ext = pos(:,3)-point;
                if ~isfield(force_vectors,force_type), force_vectors.(force_type) = []; end
                force_vectors.(force_type) = [force_vectors.(force_type),[point;torque_ext;force_ext]];
            end
        end
        force_types = fields(force_vectors);
        for i=1:length(force_types);
            force_type = force_types(i); force_type = force_type{1};
            vectors = force_vectors.(force_type);
            lcmgl = drake.matlab.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton,force_type);
            for j=1:size(vectors,2)
                point = vectors(1:3,j);
                torque_ext = vectors(4:6,j);
                force_ext = vectors(7:9,j);
                lcmgl.glColor3f(.4,.2,.4);
                lcmgl.drawVector3d(point,vector_scale*torque_ext);
                lcmgl.glColor3f(.2,.4,.2);
                lcmgl.drawVector3d(point,vector_scale*force_ext);
            end
            lcmgl.switchBuffers();
        end
    end

    function m = getMass(model, robotnum)
      % todo: write total_mass to class and simply return it instead of
      % looping every time (since the result is a constant between
      % compiles)
      if nargin < 2
        robotnum = -1; % robot num of -1 means all robots
      end

      m = 0;
      for i=1:length(model.body)
        if isBodyPartOfRobot(model, model.body(i), robotnum)
          bm = model.body(i).mass;
          m = m + bm;
        end
      end
    end

    function [T,U] = energy(model, x)
      % @param x the state vector
      % @retval T the total kinetic energy
      % @retval U the total potential energy
      % todo: add support for an optional robotnum argument?
      q = x(1:getNumPositions(model));
      v = x(getNumPositions(model)+1:end);
      H = manipulatorDynamics(model,q,v);
      T = .5*v'*H*v;

      if nargout>1
        U = 0;
        kinsol = doKinematics(model,q);
        for i=1:length(model.body)
          if model.body(i).robotnum<1, continue; end % don't include the world
          mass = model.body(i).mass;
          if (mass>0)
            com = forwardKin(model,kinsol,i,model.body(i).com);
            U = U - mass*model.gravity'*com;
          end
        end

        for i=1:length(model.force)
          [thisT,thisU] = energy(model.force{i},model,q,v);
          T = T+thisT;
          U = U+thisU;
        end
      end
    end

    function body = newBody(model)
      % @ingroup Kinematic Tree
      errorDeprecatedFunction('RigidBody()');  % since it doesn't actually serve any purpose now that planar rigid bodies are rigid bodies.
      body = RigidBody();
    end

    function fr = constructCOMFrame(model)
      fr = CoordinateFrame([model.name,'COM'],3,'m',{'com_x','com_y','com_z'});

      return;

      % in order to re-enable this, I have to figure out how we should be
      % distinguishing between reference and actual frames.  e.g., if i
      % make this, then i create a controller that takes a desired COM +
      % the actual state x as input, then things will be confusing when I
      % feedbackControl combine them.

      % construct a transform from the state vector to the COM
      checkDirty(model);
      tf = FunctionHandleCoordinateTransform(0,0,model.getStateFrame(),fr,true,true,[],[], ...
        @(obj,~,~,x) getCOM(model,x(1:model.num_positions)));

      model.getStateFrame().addTransform(tf);
    end

    function v = constructVisualizer(obj,options)
      checkDirty(obj);
      if nargin<2, options=struct(); end
      if ~isfield(options,'use_collision_geometry'), options.use_collision_geometry = false; end;
      if ~isfield(options,'viewer'), options.viewer = {'BotVisualizer','RigidBodyWRLVisualizer','NullVisualizer'};
      elseif ~iscell(options.viewer), options.viewer = {options.viewer}; end

      v=[]; i=1;
      while isempty(v)
        type = options.viewer{i};

        switch (type)
          case 'NullVisualizer'
            arg = {getPositionFrame(obj)};
          case 'BotVisualizer'
            arg = {obj,options.use_collision_geometry};
          otherwise
            arg = {obj,options};
        end

        if (i==length(options.viewer))  % then it's the last one
          v = feval(type,arg{:});
        else
          try
            v = feval(type,arg{:});
          catch ex
            if ~strncmp(ex.identifier,'Drake:MissingDependency',23)
              rethrow(ex);
            end
          end
        end
        i = i+1;
      end

    end

    function index = getActuatedJoints(model)
      % @ingroup Kinematic Tree
      if isempty(model.actuator)
        index=[];
      else
        joint = [model.actuator.joint];
        index = [model.body(joint).position_num]';
      end
    end

    function varargout = pdcontrol(sys,Kp,Kd,index)
      % creates new blocks to implement a PD controller, roughly
      % illustrated by
      %   q_d --->[ Kp ]-->(+)----->[ sys ]----------> yout
      %                     | -                 |
      %                     -------[ Kp,Kd ]<----
      %
      % when invoked with a single output argument:
      %   newsys = pdcontrol(sys,...)
      % then it returns a new system which contains the new closed loop
      % system containing the PD controller and the plant.
      %
      % when invoked with two output arguments:
      %   [pdff,pdfb] = pdcontrol(sys,...)
      % then it return the systems which define the feed-forward path and
      % feedback-path of the PD controller (but not the closed loop
      % system).
      %
      % @param Kp a num_u x num_u matrix with the position gains
      % @param Kd a num_u x num_u matrix with the velocity gains
      % @param index a num_u dimensional vector specifying the mapping from q to u.
      % index(i) = j indicates that u(i) actuates q(j). @default: 1:num_u
      %
      % For example, the a 2D floating base (with adds 3 passive joints in
      % positions 1:3)model with four actuated joints in a serial chain might have
      %      Kp = diag([10,10,10,10])
      %      Kd = diag([1, 1, 1, 1])
      %      and the default index would automatically be index = 4:7

      checkDirty(sys);

      if nargin<4 || isempty(index)  % this is too slow on the atlas model
        B = sys.B;  % note: unlike the general Manipulator version this exploits the fact that B is a constant
        [I,J] = find(B);
        if length(unique(J))~=length(J)
          error('Drake:RigidBodyManipulator:PDControlComplexB','The B matrix for this system is nontrivial, so you must manually specify the index for the PD controller');
        end
        index(J)=I;

        % try to alert if it looks like there are any obvious sign errors
        if all(diag(diag(Kp))==Kp)
          d = diag(Kp);
          if any(sign(B(sub2ind(size(B),I,J)))~=sign(d(J)))
            warning('Drake:RigidBodyManipulator:PDControlSignWarning','You might have a sign flipped?  The sign of Kp does not match the sign of the associated B');
          end
        end
        if all(diag(diag(Kd))==Kd)
          d = diag(Kd);
          if any(sign(B(sub2ind(size(B),I,J)))~=sign(d(J)))
            warning('Drake:RigidBodyManipulator:PDControlSignWarning','You might have a sign flipped?  The sign of Kd does not match the sign of the associated B');
          end
        end
      end

      varargout=cell(1,nargout);
      [varargout{:}] = pdcontrol@Manipulator(sys,Kp,Kd,index);
    end

    function [xstar,ustar,success] = findFixedPoint(obj,x0,u0,options)
      if (nargin<2 || isempty(x0))
        x0 = Point(obj.getStateFrame());
      elseif ~isa(x0,'Point')
        x0 = Point(obj.getStateFrame(),x0);
      end
      x0 = x0.inFrame(obj.getStateFrame);
      x0 = resolveConstraints(obj,x0);

      if (nargin<3 || isempty(u0))
        u0 = zeros(obj.getNumInputs(),1);
      elseif isa(u0,'Point')
        u0 = double(u0.inFrame(obj.getInputFrame));
      end
      if (nargin<4) options=struct(); end

      if ~isfield(options,'visualize'), options.visualize=false; end

      nq = getNumPositions(obj);
      nv = getNumVelocities(obj);
      nu = getNumInputs(obj);

      active_collision_options = struct();
      if isfield(options,'active_collision_groups')
        active_collision_options.collision_groups = options.active_collision_groups;
      end

      if isfield(options,'active_collision_bodies')
        active_collision_options.body_idx = options.active_collision_bodies;
      end

      % Compute number of contacts by evaluating at x0
      [phi0,~,~,~,~,~,~,~,~,D0] = obj.contactConstraints(x0(1:nq),false,active_collision_options);

      % total number of contact forces (normal + frictional)
      nz = length(phi0) + size(cell2mat(D0'),1);

      z0 = zeros(nz,1);
      q0 = x0(1:nq);

      problem.x0 = [q0;u0;z0];
      problem.objective = @(quz) 0; % feasibility problem
      problem.nonlcon = @(quz) mycon(quz);
      problem.solver = 'fmincon';

      if options.visualize
        v = obj.constructVisualizer;
        %problem.options=optimset('DerivativeCheck','on','GradConstr','on','Algorithm','interior-point','Display','iter','OutputFcn',@drawme,'TolX',1e-14,'MaxFunEvals',5000);
        problem.options=optimset('GradConstr','on','Algorithm','interior-point','Display','iter','OutputFcn',@drawme,'TolX',1e-14,'TolCon',1e-8,'MaxFunEvals',5000);
      else
        problem.options=optimset('GradConstr','on','Algorithm','interior-point','TolX',1e-14,'TolCon',1e-8,'MaxFunEvals',5000);
      end

      lb_z = -1e6*ones(nz,1);
      ub_z = 1e6*ones(nz,1);
      lb_z(1:length(phi0)) = 0; % normal forces must be positive

      [jl_min,jl_max] = obj.getJointLimits();
      % force search to be close to starting position
      problem.lb = [jl_min; obj.umin; lb_z];
      problem.ub = [jl_max; obj.umax; ub_z];

      [quz_sol,~,exitflag] = fmincon(problem);
      success=(exitflag==1);
      xstar = Point(obj.getStateFrame(), [quz_sol(1:nq); zeros(nq,1)]);
      ustar = Point(obj.getInputFrame(), quz_sol(nq+(1:nu)));
      zstar = quz_sol(nq+nu+(1:nz));
      if (~success)
        error('failed to find fixed point');
      end

      function stop=drawme(quz,optimValues,state)
        stop=false;
        v.draw(0,[quz(1:nq); zeros(nv,1)]);
      end

      function [c,ceq,GC,GCeq] = mycon(quz)
        q=quz(1:nq);
        u=quz(nq+(1:nu));
        z=quz(nq+nu+(1:nz));
        c = [];
        GC = [];
        [~,C,B,~,dC,~] = obj.manipulatorDynamics(q,zeros(nv,1));

        if obj.getNumContactPairs > 0,
          [phiC,normal,d,xA,xB,idxA,idxB,mu,n,D,dn,dD] = obj.contactConstraints(q,false,active_collision_options);

          % construct J such that J'*z is the contact force vector in joint
          J = [n;cell2mat(D')];
          % similarly, construct dJz
          dJ = zeros(numel(J),nq);
          J_idx = reshape(1:numel(J),size(J,1),size(J,2));
          Jn_idx = J_idx(1:size(n,1),:);
          JD_idx = J_idx(size(n,1)+1:end,:);
          dJ(Jn_idx(:),:) = dn;
          dJ(JD_idx(:),:) = cell2mat(dD');
          dJz = matGradMult(dJ,z,true);

          ceq = [C-B*u-J'*z; phiC];
          GCeq = [[dC(1:nq,1:nq)-dJz,-B,-J']',[n'; zeros(nu+nz,length(phiC))]];
        else
          ceq = [C-B*u];
          GCeq = [dC(1:nq,1:nq),-B]';
        end
        if getNumStateConstraints(obj)>0
          [phi,dphi] = obj.stateConstraints([q;zeros(nv,1)]);
          ceq = [ceq; phi];
          GCeq = [GCeq, [dphi(:,1:nq),zeros(numel(phi),nu+nz)]'];
        end
      end
    end

    function model = addLinksToCollisionFilterGroup(model,linknames,collision_fg_name,robotnums)
      % Adds links to the specified collision filter group
      % @param linknames        Names of the links to be added to the
      %   collision filter group. Can be a string containing the name of a
      %   link, a cell-array containing the names of multiple links, or a
      %   vector of link indices
      % @param collision_fg_name  String containing the name of the collision
      %   filter group to which the links should be added
      % @param robotnums          Robot number of the links to be added to the
      %   collision filter group. Can be a vector or a 1-d cell-array. If
      %   non-scalar, it must have the same number of elements as linknames.

      [linknames,robotnums] = model.processCFGroupArgs(linknames,robotnums);
      model.collision_filter_groups(collision_fg_name) = ...
        model.collision_filter_groups(collision_fg_name).addMembers(linknames,robotnums);
      model.dirty = true;
    end

    function model = removeLinksFromCollisionFilterGroup(model,linknames,collision_fg_name,robotnums)
      % Removes links from the specified collision filter group
      % @param linknames        Names of the links to be removed from the
      %   collision filter group. Can be a string containing the name of a
      %   link, a cell-array containing the names of multiple links, or a
      %   vector of link indices
      % @param collision_fg_name  String containing the name of the collision
      %   filter group from which the links should be removed
      % @param robotnums           Robot number of the links to be removed from
      %   the collision filter group. Can be a vector or a 1-d cell-array. If
      %   non-scalar, it must have the same number of elements as linknames.

      [linknames,robotnums] = model.processCFGroupArgs(linknames,robotnums);
      model.collision_filter_groups(collision_fg_name) = ...
        model.collision_filter_groups(collision_fg_name).removeMembers(linknames,robotnums);
      model.dirty = true;
    end

    function model = addToIgnoredListOfCollisionFilterGroup(model,ignored_collision_fgs,collision_fg_name)
      % Adds the specified collision filter groups to the "ignored" list of
      % another collision filter group
      % @param ignored_collision_fgs  Cell array of strings containing the
      %   names of the collision filter groups to be ignored.
      % @param collision_fg_name      String giving the name of the collision
      %   filter group that should ignore the groups specified by
      %   ignored_collision_fgs

      typecheck(ignored_collision_fgs,{'cell','char'});
      typecheck(collision_fg_name,'char');

      collision_fg = model.collision_filter_groups(collision_fg_name);
      collision_fg.ignored_collision_fgs = union(collision_fg.ignored_collision_fgs,ignored_collision_fgs);
      model.collision_filter_groups(collision_fg_name) = collision_fg;
      model.dirty = true;
    end

    function model = removeFromIgnoredListOfCollisionFilterGroup(model,ignored_collision_fgs,collision_fg_name)
      % Removes the specified collision filter groups from the "ignored" list of
      % another collision filter group
      % @param ignored_collision_fgs  Cell array of strings containing the
      %   names of the collision filter groups that should no longer be ignored
      % @param collision_fg_name      String giving the name of the collision
      %   filter group that should no longer ignore the groups specified by
      %   ignored_collision_fgs

      typecheck(ignored_collision_fgs,{'cell','char'});
      typecheck(collision_fg_name,'char');

      collision_fg = model.collision_filter_groups(collision_fg_name);
      collision_fg.ignored_collision_fgs = setdiff(collision_fg.ignored_collision_fgs,ignored_collision_fgs);
      model.collision_filter_groups(collision_fg_name) = collision_fg;
      model.dirty = true;
    end

    function [ val, default_value ] = parseParamString(model,robotnum,str)
      % Parses parameter strings from URDFs and returns either the value or
      % a msspoly expression for later use with
      % RigidBodyElement.bindParams().
      %
      % @ingroup URDF Parsing
      %
      % @param model model we are building from a URDF
      % @param robotnum robot number
      % @param str string to parse
      %
      % @retval val parameter value (possibly with msspolys inside) to put
      % in the property of the object
      %
      % @retval default_val value of the parameter when default values of
      % params are used

      if any(str=='$') % then it has some parameters
        fr = getParamFrame(model); p=fr.getPoly;
        pstr = regexprep(str,'\$(\w+)','p(model.param_db{robotnum}.(''$1'').index)');
        val = eval(['[',pstr,']']);
      else
        val = eval(['[',str,']']);
      end

      if nargout > 1
        % get the default value
        pstr2 = regexprep(str,'\$(\w+)','model.param_db{robotnum}.(''$1'').value');
        default_value = eval(['[',pstr2,']']);

      end
    end

    function [linknames,robotnums] = processCFGroupArgs(model,linknames,robotnums)
      % @param linknames          Names of the links to be added to the
      %   collision filter group. Can be a string containing the name of a
      %   link, a cell-array containing the names of multiple links, or a
      %   vector of link indices
      % @param robotnum           Robot number of the links to be added to the
      %   collision filter group. Can be a vector or a 1-d cell-array.
      if iscell(robotnums)
        % nothing needs to be done
      elseif isnumeric(robotnums)
        robotnums = num2cell(robotnums);
      else
        error('robotnums must be a cell array or a numeric array');
      end
      assert(isscalar(robotnums) || numel(robotnums) == numel(linknames),...
        ['If ''robotnums'' is non-scalar, it must have the same number ' ...
        'of elements as ''linknames'''])
      robotnums = robotnums(:);

      if iscell(linknames)
        % nothing needs to be done
      elseif ischar(linknames)
        linknames = {linknames};
      elseif isnumeric(linknames)
        linknames = arrayfun(@(idx)getLinkName(model,idx),linknames, ...
                               'UniformOutput',false);
      else
        error('linknames must be a string, cell array or numeric array');
      end
      linknames = linknames(:);
    end

    function d=surfaceTangents(obj,normal)
      %% compute tangent vectors, according to the description in the last paragraph of Stewart96, p.2678
      assert(~isempty(normal), ...
        'Drake:RigidBodyManipulator:surfaceTangents:emptyNormals', ...
        '''normal'' must be a non-empty array')

      if obj.mex_model_ptr ~= 0
        d = surfaceTangentsmex(obj.mex_model_ptr, normal);
        return
      end

      t1=normal; % initialize size

      % handle the normal = [0;0;1] case
      ind=(1-normal(3,:))<10e-8;  % since it's a unit normal, i can just check the z component
      t1(:,ind) = [ones(1,sum(ind)); zeros(2,sum(ind))];

      % handle the normal = [0;0;-1] case
      indneg=(1+normal(3,:))<10e-8;  % since it's a unit normal, i can just check the z component
      t1(:,indneg) = [-ones(1,sum(indneg)); zeros(2,sum(indneg))];

      ind=~(ind | indneg);

      % now the general case
      t1(:,ind) = [normal(2,ind);-normal(1,ind);zeros(1,sum(ind))]; % cross(normal,[0;0;1]) normalized
      t1(:,ind) = bsxfun(@rdivide,t1(:,ind),sqrt(sum(t1(1:2,ind).^2,1))); % normalize

      t2 = cross(t1,normal);

      m = 2;  % half of the the number of direct vectors
      theta = (0:(m-1))*pi/m;

      for k=1:m
        d{k}=cos(theta(k))*t1 + sin(theta(k))*t2;
      end
    end

    function n=getNumContactPairs(obj)
      n = obj.num_contact_pairs;
    end

    function [phi,dphi] = unilateralConstraints(obj,x)
      q = x(1:obj.getNumPositions);
      if nargout<2
        phi = contactConstraints(obj,q);
      else
        [phi,~,~,~,~,~,~,~,dphi] = contactConstraints(obj,q);
        dphi = [dphi,zeros(size(phi,1),obj.getNumVelocities)];
      end
    end

    function n = getNumUnilateralConstraints(obj)
      % Returns the number of unilateral constraints, currently only
      % contains the contact pairs
      n = obj.getNumContactPairs();
    end

    function fr = getPositionFrame(obj,robotnum)
      % if robotnum is not specified, then it returns a position frame
      % including all position variables (for all robots)
      if getNumPositions(obj)<1,
        fr = CoordinateFrame('JointPositions',0);
        return;
      end

      if nargin<2 || robotnum<0,
        fr = MultiCoordinateFrame.constructFrame(obj.robot_position_frames,[],true);
      else
        fr = obj.robot_position_frames{robotnum};
      end
    end

    function fr = getVelocityFrame(obj,robotnum)
      % if robotnum is not specified, then it returns a velocity frame
      % including all velocity variables (for all robots)
      if getNumVelocities(obj)<1,
        fr = CoordinateFrame('JointVelocities',0);
        return;
      end

      if nargin<2 || robotnum<0,
        fr = MultiCoordinateFrame.constructFrame(obj.robot_velocity_frames,[],true);
      else
        fr = obj.robot_velocity_frames{robotnum};
      end
    end

    function fr = getStateFrame(obj,robotnum)
      % if robotnum is not specified, then it returns a state frame
      % including all state variables (for all robots)
      if nargin<2 || robotnum<0,
        fr = getStateFrame@DrakeSystem(obj);
      else
        fr = obj.robot_state_frames{robotnum};
      end
    end

  end

  methods (Static)
    [c,options] = parseMaterial(node,options);
  end

  methods (Access=protected)

    function checkDirty(obj)
      if (obj.dirty)
        error('You''ve changed something about this model and need to manually compile it.  Use obj=compile(obj).');
      end
    end

    function obj = createMexPointer(obj)
      if (exist('constructModelmex')==3 && isnumeric(getParams(obj))) % note that this getParams call could be somewhat expensive to be putting here, but it seems like it does need to be there for symbolic parameters (which are not supported in the c++ version yet)
%        obj.mex_model_ptr = debugMexEval('constructModelmex',obj);
        obj.mex_model_ptr = constructModelmex(obj);
        obj.default_kinematics_cache_ptr_no_gradients = createKinematicsCachemex(obj.mex_model_ptr);
        obj.default_kinematics_cache_ptr_with_gradients = createKinematicsCacheAutoDiffmex(obj.mex_model_ptr, obj.getNumPositions() + obj.getNumVelocities());
      end
    end

    function model = constructStateFrame(model)
      frame_dims=[];
      for i=1:length(model.name)
        positions={};
        velocities={};
        for j=1:length(model.body)
          b = model.body(j);
          if b.parent>0 && b.robotnum==i
            if (b.floating==1)
              newpos = {[b.jointname,'_x'];[b.jointname,'_y'];[b.jointname,'_z'];[b.jointname,'_roll'];[b.jointname,'_pitch'];[b.jointname,'_yaw']};
              positions = vertcat(positions,newpos);
              velocities = vertcat(velocities,cellfun(@(a) [a,'dot'],newpos,'UniformOutput',false));
            elseif (b.floating==2)
              positions = vertcat(positions,{[b.jointname,'_x'];[b.jointname,'_y'];[b.jointname,'_z'];[b.jointname,'_qw'];[b.jointname,'_qx'];[b.jointname,'_qy'];[b.jointname,'_qz']});
              velocities = vertcat(velocities,{[b.jointname,'_wx'];[b.jointname,'_wy'];[b.jointname,'_wz'];[b.jointname,'_vx'];[b.jointname,'_vy'];[b.jointname,'_vz']});
            else
              positions = vertcat(positions,{b.jointname});
              velocities = vertcat(velocities,{[b.jointname,'dot']});
            end
            frame_dims(b.position_num)=i;
            frame_dims(model.getNumPositions() + b.velocity_num)=i;
          end
        end

        fr = CoordinateFrame([model.name{i},'Position'],length(positions),'q',positions);
        if numel(model.robot_position_frames)<i || ~isequal_modulo_transforms(fr,model.robot_position_frames{i}) % let the previous handle stay valid if possible
          model.robot_position_frames{i} = fr;
        end
        fr = CoordinateFrame([model.name{i},'Velocity'],length(velocities),'v',velocities);
        if numel(model.robot_velocity_frames)<i || ~isequal_modulo_transforms(fr,model.robot_velocity_frames{i}) % let the previous handle stay valid if possible
          model.robot_velocity_frames{i} = fr;
        end
        fr = MultiCoordinateFrame.constructFrame({model.robot_position_frames{i},model.robot_velocity_frames{i}},[],true);
        if numel(model.robot_state_frames)<i || ~isequal_modulo_transforms(fr,model.robot_state_frames{i}) % let the previous handle stay valid if possible
          model.robot_state_frames{i} = fr;
        end
      end
      fr = MultiCoordinateFrame.constructFrame(model.robot_state_frames,frame_dims,true);
      if ~isequal_modulo_transforms(fr,getStateFrame(model)) % let the previous handle stay valid if possible
        model = setStateFrame(model,fr);
      end
    end

    function [model,fr,pval,pmin,pmax] = constructParamFrame(model)
      frames = {};
      pval=[]; pmin=[]; pmax=[];
      index=1;
      for i=1:min(numel(model.name),numel(model.param_db))
        pn = fieldnames(struct(model.param_db{i}));
        frames{i} = CoordinateFrame([model.name{i},'Params'],numel(pn),'p',pn);
        for j=1:numel(pn)
          pval=vertcat(pval,model.param_db{i}.(pn{j}).value);
          pmin=vertcat(pmin,model.param_db{i}.(pn{j}).lb);
          pmax=vertcat(pmax,model.param_db{i}.(pn{j}).ub);
          model.param_db{i}.(pn{j}).index = index;
          index = index+1;
        end
      end
      fr = MultiCoordinateFrame.constructFrame(frames);
    end

    function fr = constructInputFrame(model)
      %inputparents is an array of the parent RigidBodies of each joint.
      inputparents = [];
      inputnames = {};
      if ~isempty(model.actuator)
        inputparents = [model.body([model.actuator.joint])];
        inputnames = {model.actuator.name};
      end
      for i = 1:length(model.force)
        if isa(model.force{i},'RigidBodyThrust') || isa(model.force{i}, 'RigidBodyPropellor')
          frame = model.frame(-model.force{i}.kinframe);
          inputparents = [inputparents model.body(frame.body_ind)];
          inputnames{end+1} = model.force{i}.name;
        elseif model.force{i}.direct_feedthrough_flag
          frame = model.frame(-model.force{i}.kinframe);
          inputparents = [inputparents model.body(frame.body_ind)];
          inputnames{end+1} = model.force{i}.name;
        end

      end
      for i=1:length(model.name)
        robot_inputs = [inputparents.robotnum]==i;
        coordinates = {inputnames{robot_inputs}}';
        fr{i}=CoordinateFrame([model.name{i},'Input'],sum(robot_inputs),'u',coordinates);
      end
      frame_dims = [inputparents.robotnum];
      fr = MultiCoordinateFrame.constructFrame(fr,frame_dims,true);
    end

    function model=removeFixedJoints(model)
      % takes any fixed joints out of the tree, adding their visuals and
      % inertia to their parents, and connecting their children directly to
      % their parents
      % @ingroup Kinematic Tree

      fixedind = find(isnan([model.body.pitch]) | ... % actual fixed joint
        cellfun(@(a) (isnumeric(a) && ~any(any(a))),{model.body.I}));    % body has no inertia (yes, it happens in pr2.urdf)

      for i=fixedind(end:-1:1)  % go backwards, since it is presumably more efficient to start from the bottom of the tree
        body = model.body(i);
        if body.parent<1
          % if it happens to be the root joint, then don't remove this one.
          continue;
        end
        parent = model.body(body.parent);

        if ~isnan(body.pitch)
          if any([model.body.parent] == i) || any(any(body.I))
            % link has inertial importance from child links
            % pr link has inertia now (from a fixed link coming from a
            % descendant).  abort removal.
            continue;
          end
          if ~isempty(model.loop)   % check if it's in a loop joint
            frames_on_this_body = find([model.frame.body_ind]==i);
            if ~isempty(intersect(frames_on_this_body,-[model.loop.frameA,model.loop.frameB]))
              % then it's part of a loop joint
              continue;
            end
          end
          warning('Drake:RigidBodyManipulator:BodyHasZeroInertia',['Link ',body.linkname,' has zero inertia (even though gravity is on and it''s not a fixed joint) and will be removed']);
        end

        parent.linkname=[parent.linkname,'+',body.linkname];

        % add inertia into parent
        if (any(any(body.I)))
            %Check before running setInertial() that the body doesn't have added-mass
            %coefficients (I haven't written up the welding support for that yet - JSI)
          if ~valuecheck(body.Iaddedmass,zeros(6,6));
              error('Adding inertia to parent with added-mass coefficients is not supported yet');
          end

          % same as the composite inertia calculation in HandC.m
          T_joint_predecessor_frame_to_parent_body = body.Ttree;
          Ad_T_parent_body_to_joint_predecessor_frame = transformAdjoint(homogTransInv(T_joint_predecessor_frame_to_parent_body));
          I_body_in_parent = Ad_T_parent_body_to_joint_predecessor_frame' * body.I * Ad_T_parent_body_to_joint_predecessor_frame;
          parent = setInertial(parent,parent.I + I_body_in_parent);
        end

        for j=1:length(body.visual_geometry)
          body.visual_geometry{j}.T = body.Ttree*body.visual_geometry{j}.T;
        end
        parent.visual_geometry = horzcat(parent.visual_geometry,body.visual_geometry);

        if (~isempty(body.collision_geometry))
          for j=1:length(body.collision_geometry)
            body.collision_geometry{j}.T = body.Ttree*body.collision_geometry{j}.T;
          end
          ngeometry = length(parent.collision_geometry);
          parent.collision_geometry = {parent.collision_geometry{:},body.collision_geometry{:}};

          if ~isempty(body.collision_geometry_group_names)
            ngroups=length(parent.collision_geometry_group_names);
            [parent.collision_geometry_group_names,ia,ic]=unique(horzcat(parent.collision_geometry_group_names,body.collision_geometry_group_names),'stable');
            % note: passing 'stable' to unique (above) ensures that
            if length(parent.collision_geometry_group_indices)<length(parent.collision_geometry_group_names)
              parent.collision_geometry_group_indices{length(parent.collision_geometry_group_names)}=[];
            end
            for j=1:length(body.collision_geometry_group_indices)
              parent.collision_geometry_group_indices{ic(ngroups+j)} = [parent.collision_geometry_group_indices{ic(ngroups+j)},ngeometry+body.collision_geometry_group_indices{j}];
            end
          end
        end

        model = applyToAllRigidBodyElements(model,'updateForRemovedLink',model,i);
        for key = model.collision_filter_groups.keys
          model.collision_filter_groups(key{1}) = updateForRemovedLink(model.collision_filter_groups(key{1}),model,i,parent.linkname,key{1});
        end

        % remove actuators
        if (~isempty(model.actuator) && any([model.actuator.joint] == i))
          model.actuator(find([model.actuator.joint]==i))=[];
          % actuators could be attached to fixed joints, because I
          % occasionally weld joints together (e.g. in planar processing of
          % a 3D robot)
        end

        % connect children to parents
%        children = find([model.body.parent] == body);
        for j=1:length(model.body),
          if model.body(j).parent == i
            if (body.gravity_off && ~model.body(j).gravity_off)
              error([model.body(j).linkname,' has gravity on, but it''s parent, ', body.linkname,' has gravity off']);
            end
            model.body(j).parent = body.parent;
            model.body(j).Ttree = body.Ttree*model.body(j).Ttree;
          end
        end
        model.body(body.parent) = parent;
        model = updateBodyIndices(model,[1:i-1,i+1:length(model.body)]);
      end
    end

    function obj = adjustContactShapes(varargin)
      errorDeprecatedFunction('adjustCollisionGeometry');
    end

    function model = adjustCollisionGeometry(model)
      % model = adjustCollisionGeometry(model) returns the model with
      % adjusted collision geometry, according to the setings in
      % model.contact_options. These are
      %   * Replace cylinders with capsules
      %
      % @param model - RigidBodyManipulator object
      %
      % @retval model - RigidBodyManipulator object

      if model.contact_options.replace_cylinders_with_capsules
        message_id = 'Drake:RigidBodyManipulator:ReplacedCylinder';
        body_changed = false(model.getNumBodies(),1);
        for i = 1:model.getNumBodies()
          [model.body(i),body_changed(i)] = replaceCylindersWithCapsules(model.body(i));
        end
        if any(body_changed)
          changed_body_idx_and_names =  ...
            [num2cell(find(body_changed))';getLinkName(model,body_changed)];
          warning(message_id, ...
            ['The bodies listed below each contained at least one ' ...
            'RigidBodyCylinder as a collision geometry:' ...
            '\n\n' ...
            '\tBody Idx\tBody Name\n' ...
            repmat('\t%d:\t\t%s\n',1,sum(body_changed)) ...
            '\n\n' ...
            'These collision geometries were replaced by ' ...
            'RigidBodyCapsule objects, as the cylinder contact geometry ' ...
            'is less robust.\n\nTo prevent this replacement, ' ...
            'construct your manipulator with: ' ...
            '\n\n' ...
            '    >> options.replace_cylinders_with_capsules = false;\n' ...
            '    >> r = %s(...,options);' ...
            '\n\n' ...
            'To silence this warning, construct your manipulator ' ...
            'with: ' ...
            '\n\n' ...
            '    >> w = warning(''off'',''%s'');\n' ...
            '    >> r = %s(...);\n' ...
            '    >> warning(w)' ...
            '\n\n'],changed_body_idx_and_names{:},class(model), ...
            message_id,class(model));
        end
      end
    end

    function id = findCollisionFilterGroupID(model,collision_fg_name)
        id = find(strcmp(model.collision_filter_groups.keys(),collision_fg_name));
        if isempty(id)
          error('RigidBodyManipulator:findCollisionFilterGroupID', ...
                'Unable to find collision filter group, %s',collision_fg_name);
        end
    end

    function model = setupCollisionFiltering(model)
      % Transfers collision filtering information from the collision_filter_groups map
      % to the links themselves. Must be run BEFORE createMexPointer.

      % In case no robots were loaded from urdf, initialize
      % collision_filter_groups here.
      if isempty(model.collision_filter_groups)
        model.collision_filter_groups = PassByValueMap('KeyType','char','ValueType','any');
        model.collision_filter_groups('no_collision') = CollisionFilterGroup();
      end
      if model.contact_options.ignore_self_collisions
        body_indices = 1:model.getNumBodies();

        % Loop over the robots in the RigidBodyManipulator
        for i = 1:length(model.name)
          % Get the indices of the bodies belonging to this robot
          robot_i_body_indices = body_indices([model.body.robotnum] == i);

          % Remove the bodies in this robot from all collision filter
          % groups
          for collision_fg_name = model.collision_filter_groups.keys()
            model = model.removeLinksFromCollisionFilterGroup(robot_i_body_indices,collision_fg_name{1},i);
          end

          % Create a collision filter group with the name of the robot
          model.collision_filter_groups(model.name{i}) = CollisionFilterGroup();

          % Add all links in the robot to the new collision filter group
          model = model.addLinksToCollisionFilterGroup(robot_i_body_indices,model.name{i},i);

          % Set the new collision filter group to ignore itself
          model = model.addToIgnoredListOfCollisionFilterGroup(model.name{i},model.name{i});
        end
      end
      % The DEFAULT_COLLISION_FILTER_GROUP is reserved for bodies that don't belong to any
      % other collision collision_filter_groups.
      model.collision_filter_groups('default') = CollisionFilterGroup();
      model = addLinksToCollisionFilterGroup(model, {model.body.linkname}, 'default',{model.body.robotnum});
      for collision_fg_name = model.collision_filter_groups.keys()
        if ~strcmp(collision_fg_name,'default')
          [linknames,robotnums] = model.collision_filter_groups(cell2mat(collision_fg_name)).getMembers();
          model = removeLinksFromCollisionFilterGroup(model,linknames,'default',robotnums);
        end
        model = addToIgnoredListOfCollisionFilterGroup(model,cell2mat(collision_fg_name),'no_collision');
      end

      % Reset the collision filtering properties for all links
      for i = 1:model.getNumBodies()
        model.body(i) = model.body(i).makeBelongToNoCollisionFilterGroups();
        model.body(i) = model.body(i).makeIgnoreNoCollisionFilterGroups();
      end

      % Set the collision filtering properties of each body to match those
      % specified by the collision filter groups.
      for collision_fg_name = model.collision_filter_groups.keys()
        collision_fg_id = model.findCollisionFilterGroupID(cell2mat(collision_fg_name));
        member_indices = findCollisionFilterGroupMemberIndices(model,cell2mat(collision_fg_name));
        model = makeLinksBelongToCollisionFilterGroup(model,member_indices,collision_fg_id);
        for ignored_collision_fg_name = getIgnoredCollisionFilterGroups(model.collision_filter_groups(cell2mat(collision_fg_name)))
          ignored_collision_fg_id = model.findCollisionFilterGroupID(cell2mat(ignored_collision_fg_name));
          model = makeLinksIgnoreCollisionFilterGroup(model,member_indices,ignored_collision_fg_id);
        end
      end
    end

    function link_indices = findCollisionFilterGroupMemberIndices(model,collision_fg_name)
      [linknames,robotnums] = model.collision_filter_groups(collision_fg_name).getMembers();
      link_indices = cellfun(@(name,num) findLinkId(model,name,num), ...
                             linknames,robotnums);
    end

    function model = makeLinksBelongToCollisionFilterGroup(model,link_indices,collision_fg_id)
      for idx = reshape(link_indices,1,[])
        model.body(idx) = makeBelongToCollisionFilterGroup(model.body(idx),collision_fg_id);
      end
    end

    function model = makeLinksBelongToNoCollisionFilterGroups(model,link_indices)
      for idx = reshape(link_indices,1,[])
        model.body(idx) = makeBelongToNoCollisionFilterGroups(model.body(idx));
      end
    end

    function model = makeLinksIgnoreCollisionFilterGroup(model,link_indices,collision_fg_id)
      for idx = reshape(link_indices,1,[])
        model.body(idx) = makeIgnoreCollisionFilterGroup(model.body(idx),collision_fg_id);
      end
    end

    function model = updateBodyIndices(model,map_from_new_to_old)
      % @ingroup Kinematic Tree
      nold = length(model.body);
      model.body = model.body(map_from_new_to_old);

      % build a simple function to map from old indices to new indices:
      map = zeros(1,nold);
      map(map_from_new_to_old) = 1:length(model.body);
      map = [0,map];
      mapfun = @(i) map(i+1);

      model = applyToAllRigidBodyElements(model,'updateBodyIndices',mapfun);
    end

    function model = applyToAllRigidBodyElements(model,fcn,varargin)
      for i=1:length(model.body)
        model.body(i) = feval(fcn,model.body(i),varargin{:});
      end
      for i=1:length(model.actuator)
        model.actuator(i) = feval(fcn,model.actuator(i),varargin{:});
      end
      for i=1:length(model.loop)
        model.loop(i) = feval(fcn,model.loop(i),varargin{:});
      end
      for i=1:length(model.sensor)
        model.sensor{i} = feval(fcn,model.sensor{i},varargin{:});
      end
      for i=1:length(model.force)
        model.force{i} = feval(fcn,model.force{i},varargin{:});
      end
      for i=1:length(model.frame)
        model.frame(i) = feval(fcn,model.frame(i),varargin{:});
      end
      for j=1:length(model.position_constraints)
        % todo: generalize this by moving the updateConstraint logic above into
        % drakeFunction.RBM
        if isa(model.position_constraints{j},'DrakeFunctionConstraint') && isa(model.position_constraints{j}.fcn,'drakeFunction.kinematic.CableLength')
          cable_length_function = feval(fcn,model.position_constraints{j}.fcn,varargin{:});
          constraint = DrakeFunctionConstraint(model.position_constraints{j}.lb,model.position_constraints{j}.ub,cable_length_function);
          constraint = setName(constraint,cable_length_function.name);
          constraint.grad_level = 2; %declare that the second derivative is provided
          constraint.grad_method = 'user';
          model = updatePositionEqualityConstraint(model,j,constraint);            
        end
      end
    end

  end

end
