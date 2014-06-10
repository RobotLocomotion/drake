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
    frame = [];     % array of RigidBodyFrame objects
  end
  
  properties (Access=public)  % i think these should be private, but probably needed to access them from mex? - Russ
    featherstone = [];  
    B = [];
    mex_model_ptr = 0;
    dirty = true;
    %collision_filter_groups=containers.Map('KeyType','char','ValueType','any');     
    collision_filter_groups;     
      % map of CollisionFilterGroup objects
  end
    
  methods
    function obj = RigidBodyManipulator(urdf_filename,options)
      % Construct a new rigid body manipulator object with a single (empty)
      % RigidBody (called 'world'), and optionally load a first robot from
      % urdf (see documentation for addRobotFromURDF for details on the
      % inputs).
      
      if (nargin<2), options = struct(); end
      if ~isfield(options,'terrain'), options.terrain = []; end;

      obj = obj@Manipulator(0,0);
      obj.body = newBody(obj);
      obj.body.linkname = 'world';
      obj = setTerrain(obj,options.terrain);
      obj.contact_options = obj.parseContactOptions(options);
      
      if (nargin>0 && ~isempty(urdf_filename))
        obj = addRobotFromURDF(obj,urdf_filename,zeros(3,1),zeros(3,1),options);
      end
    end
  end
  
  methods (Static)
    function obj = loadobj(obj)
      obj.mex_model_ptr = 0;
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
      % NOTEST
    end
  end
  
  methods
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
      obj = removeTerrainGeometries(obj);
      obj.terrain = terrain;
      obj = addTerrainGeometries(obj);
    end

    function obj = addTerrainGeometries(obj)
      if ~isempty(obj.terrain)
        geom = obj.terrain.getRigidBodyGeometry();
        if ~isempty(geom)
          if ~any(cellfun(@(shape) isequal(geom,shape),obj.body(1).contact_shapes))
            obj.body(1).contact_shapes{end+1} = geom;
          end
          if ~any(cellfun(@(shape) isequal(geom,shape),obj.body(1).visual_shapes))
            obj.body(1).visual_shapes{end+1} = geom;
          end
          obj.dirty = true;
        end
      end
    end

    function obj = removeTerrainGeometries(obj)
      if ~isempty(obj.terrain)
        geom = obj.terrain.getRigidBodyGeometry();
        geom_contact_idx = cellfun(@(shape) isequal(geom,shape),obj.body(1).contact_shapes);
        obj.body(1).contact_shapes(geom_contact_idx) = [];
        geom_visual_idx = cellfun(@(shape) isequal(geom,shape),obj.body(1).visual_shapes);
        obj.body(1).visual_shapes(geom_visual_idx) = [];
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
        f_ext = sparse(6,m.NB);
        for i=1:length(obj.force)
          if (obj.force{i}.direct_feedthrough_flag)
            [~,B_force] = computeSpatialForce(obj.force{i},obj,q,qd);
            B = B+B_force;
          end
        end
      end
      
    end
    
    function n = getNumPositions(obj)
      n = obj.num_q; %placeholder waiting for Russ' changes
    end
    
    function n = getNumVelocities(obj)
      n = obj.num_q; %placeholder waiting for Russ' changes
    end
    
    function n = getNumDOF(obj)
      n = obj.num_q;
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
    
    function g = getGravity(obj,grav)
      g = obj.gravity;
    end
    
    function f_friction = computeFrictionForce(model,qd)
      m = model.featherstone;
      f_friction = m.damping'.*qd;
      if (m.coulomb_friction)
        f_friction = f_friction + min(1,max(-1,qd./m.coulomb_window')).*m.coulomb_friction';
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
        [ftmp,ftmpJ,ftmpP]=bodyKin(obj,kinsol,body_ind,[force,zeros(3,1)]);                                                                                                   
      else
        ftmp=bodyKin(obj,kinsol,body_ind,[force,zeros(3,1)]);
      end
      
      % try to do it the Xtree way
      force = ftmp(:,1)-ftmp(:,2);
      f_body = [ cross(point,force,1); force ];  % spatial force in body coordinates
      
      % convert to joint frame (featherstone dynamics algorithm never reasons in body coordinates)
      f = obj.body(body_ind).X_joint_to_body'*f_body;
      
      if (nargout>1)
        dforcedq = ftmpJ(1:3,:)-ftmpJ(4:6,:);                                                                                                                                 
        dforcedforce = ftmpP(1:3,1:size(force,1))-ftmpP(4:6,1:size(force,1));
        df_bodydq = [ cross(repmat(point,1,size(dforcedq,2)),dforcedq); dforcedq ];                                                                                           
        df_bodydforce = [ cross(repmat(point,1,size(dforcedforce,2)),dforcedforce); dforcedforce];
        dfdq = obj.body(body_ind).X_joint_to_body'*df_bodydq;                                                                                                                 
        dfdforce = obj.body(body_ind).X_joint_to_body'*df_bodydforce;
      end
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
      
      if child.parent>0
        error(['there is already a joint connecting this child (' child.linkname ') to a parent (' model.body(parent_ind).linkname ') on joint ' name ]);
      end
      
      jointname = regexprep(name, '\.', '_', 'preservecase');
      if ismember(lower(jointname),lower({model.body.jointname}))
        num = 2;
        while ismember(lower([jointname,num2str(num)]),lower({model.body.jointname}))
          num = num+1;
        end
        jointname = [jointname,num2str(num)];
      end
      child.jointname = jointname;
      rangecheck(parent_ind,1,getNumBodies(model));
      child.parent = parent_ind;
      
%      axis = quat2rotmat(rpy2quat(rpy))*axis;  % axis is specified in joint frame

      wrl_joint_origin='';
      if any(xyz)
        wrl_joint_origin=[wrl_joint_origin,sprintf('\ttranslation %f %f %f\n',xyz(1),xyz(2),xyz(3))];
      end
      if (any(rpy))
        wrl_joint_origin=[wrl_joint_origin,sprintf('\trotation %f %f %f %f\n',rpy2axis(rpy))];
      end
      if ~isempty(wrl_joint_origin)
        child.wrljoint = wrl_joint_origin;
      end      

      child.Xtree = Xrotx(rpy(1))*Xroty(rpy(2))*Xrotz(rpy(3))*Xtrans(xyz);
      child.Ttree = [rotz(rpy(3))*roty(rpy(2))*rotx(rpy(1)),xyz; 0,0,0,1];  % equivalent to rpy2rotmat

      % note that I only now finally understand that my Ttree*[x;1] is
      % *ALMOST* (up to translation?? need to resolve this!) the same as inv(Xtree)*[x;zeros(3,1)].  sigh.
%      valuecheck([eye(3),zeros(3,1)]*child.Ttree*ones(4,1),[eye(3),zeros(3)]*inv(child.Xtree)*[ones(3,1);zeros(3,1)]);

      if ~any(strcmp(lower(type),{'fixed','floating_rpy','floating_quat'})) && dot(axis,[0;0;1])<1-1e-4
        % featherstone dynamics treats all joints as operating around the
        % z-axis.  so I have to add a transform from the origin of this
        % link to align the joint axis with the z-axis, update the spatial
        % inertia of this joint, and then rotate back to keep the child
        % frames intact.  this happens in extractFeatherstone
        axis_angle = [cross(axis,[0;0;1]); acos(dot(axis,[0;0;1]))]; % both are already normalized
        if all(abs(axis_angle(1:3))<1e-4)
          % then it's a scaling of the z axis.  
          valuecheck(sin(axis_angle(4)),0,1e-4);  
          axis_angle(1:3)=[0;1;0];  
        end
        jointrpy = quat2rpy(axis2quat(axis_angle));
        child.X_joint_to_body=Xrotx(jointrpy(1))*Xroty(jointrpy(2))*Xrotz(jointrpy(3));
        child.T_body_to_joint=[rotz(jointrpy(3))*roty(jointrpy(2))*rotx(jointrpy(1)),zeros(3,1); 0,0,0,1];

        valuecheck(inv(child.X_joint_to_body)*[axis;zeros(3,1)],[0;0;1;zeros(3,1)],1e-6);
        valuecheck(child.T_body_to_joint*[axis;1],[0;0;1;1],1e-6);
      end

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
          model = addJoint(model,'base','floating_rpy',parent,rootlink,zeros(3,1),zeros(3,1));

        case 'quat'
          model = addJoint(model,'base','floating_quat',parent,rootlink,zeros(3,1),zeros(3,1));
        
        case 'RPY'  % instrinsic coordinates
          body1 = newBody(model);
          body1.linkname = 'base_x';
          body1.robotnum=robotnum;
          model.body = [model.body,body1];
          body1_ind = length(model.body);
          model = addJoint(model,body1.linkname,'prismatic',parent,body1_ind,xyz,rpy,[1;0;0],0);
          
          body2=newBody(model);
          body2.linkname = 'base_y';
          body2.robotnum=robotnum;
          model.body = [model.body,body2];
          body2_ind = length(model.body);
          model = addJoint(model,body2.linkname,'prismatic',body1_ind,body2_ind,zeros(3,1),zeros(3,1),[0;1;0],0);
          
          body3=newBody(model);
          body3.linkname = 'base_z';
          body3.robotnum=robotnum;
          model.body = [model.body,body3];
          body3_ind = length(model.body);
          model = addJoint(model,body3.linkname,'prismatic',body2_ind,body3_ind,zeros(3,1),zeros(3,1),[0;0;1],0);
          
          body4=newBody(model);
          body4.linkname = 'base_relative_roll';
          body4.robotnum=robotnum;
          model.body = [model.body,body4];
          body4_ind = length(model.body);
          model = addJoint(model,body4.linkname,'revolute',body3_ind,body4_ind,zeros(3,1),zeros(3,1),[1;0;0],0);
          
          body5=newBody(model);
          body5.linkname = 'base_relative_pitch';
          body5.robotnum=robotnum;
          model.body = [model.body,body5];
          body5_ind = length(model.body);
          model = addJoint(model,body5.linkname,'revolute',body4_ind,body5_ind,zeros(3,1),zeros(3,1),[0;1;0],0);
          
          model = addJoint(model,'base_relative_yaw','revolute',body5_ind,rootlink,zeros(3,1),zeros(3,1),[0;0;1],0);
          
        case 'YPR' % intrinsic
        
          body1 = newBody(model);
          body1.linkname = 'base_x';
          body1.robotnum=robotnum;
          model.body = [model.body,body1];
          body1_ind = length(model.body);
          model = addJoint(model,body1.linkname,'prismatic',parent,body1_ind,xyz,rpy,[1;0;0],0);
          
          body2=newBody(model);
          body2.linkname = 'base_y';
          body2.robotnum=robotnum;
          model.body = [model.body,body2];
          body2_ind = length(model.body);
          model = addJoint(model,body2.linkname,'prismatic',body1_ind,body2_ind,zeros(3,1),zeros(3,1),[0;1;0],0);
          
          body3=newBody(model);
          body3.linkname = 'base_z';
          body3.robotnum=robotnum;
          model.body = [model.body,body3];
          body3_ind = length(model.body);
          model = addJoint(model,body3.linkname,'prismatic',body2_ind,body3_ind,zeros(3,1),zeros(3,1),[0;0;1],0);
          
          body4=newBody(model);
          body4.linkname = ['base_relative_yaw'];
          body4.robotnum=robotnum;
          model.body = [model.body,body4];
          body4_ind = length(model.body);
          model = addJoint(model,body4.linkname,'revolute',body3_ind,body4_ind,zeros(3,1),zeros(3,1),[0;0;1],0);
          
          body5=newBody(model);
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
      typecheck(sensor,'RigidBodySensor');
      model.sensor{end+1}=sensor;
      model.dirty = true;
    end
    
    function model = compile(model)
      % After parsing, compute some relevant data structures that will be
      % accessed in the dynamics and visualization
      if isempty(model.name) 
        return;  % nothing to compile
      end

      model = removeFixedJoints(model);
      
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
            
      %% extract featherstone model structure
      [model,num_dof] = extractFeatherstone(model);
      
      if (num_dof<1) error('This model has no DOF!'); end

      u_limit = repmat(inf,length(model.actuator),1);
      u_limit = [-u_limit u_limit]; % lower/upper limits

      %% extract B matrix
      B = sparse(num_dof,0);
      for i=1:length(model.actuator)
        joint = model.body(model.actuator(i).joint);
        B(joint.dofnum,i) = model.actuator(i).reduction;
        u_limit(i,1) = joint.effort_min;
        u_limit(i,2) = joint.effort_max;
      end
      for i=1:length(model.force)
        if model.force{i}.direct_feedthrough_flag
          input_num = size(B,2)+1;
          B(1,size(B,2)+1) = 0; %Add another column to B
          model.force{i}.input_num = input_num;
          u_limit(size(u_limit,1)+1,:) = model.force{i}.input_limits;
        end
      end
      model.B = full(B);

      model = setNumInputs(model,size(model.B,2));
      model = setNumDOF(model,num_dof);
      model = setNumOutputs(model,2*num_dof);

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
        stateframe = constructStateFrame(model);
        if ~isequal_modulo_transforms(stateframe,getStateFrame(model)) % let the previous handle stay valid if possible
          model = setStateFrame(model,stateframe);
        end
      end
      
      if length(model.sensor)>0
        feedthrough = false;
        for i=1:length(model.sensor)
          model.sensor{i} = model.sensor{i}.compile(model);
          outframe{i} = model.sensor{i}.constructFrame(model);
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
      
      if (length(model.loop)>0)
        model = model.setNumPositionConstraints(3*length(model.loop));  % should be 5? for continous joints once they enforce the joint axis constraint.
      else 
        model = model.setNumPositionConstraints(0);
      end

      model.joint_limit_min = [model.body.joint_limit_min]';
      model.joint_limit_max = [model.body.joint_limit_max]';
     
      if (any(model.joint_limit_min~=-inf) || any(model.joint_limit_max~=inf))
        warnOnce(model.warning_manager,'Drake:RigidBodyManipulator:UnsupportedJointLimits','Joint limits are not supported by the dynamics methods of this class.  Consider using HybridPlanarRigidBodyManipulator');
      end
      
      model = model.setInputLimits(u_limit(:,1),u_limit(:,2));
      
      %% check basic assumption from kinematics:
      for i=1:length(model.body)
        valuecheck(model.body(i).Ttree(end,1:end-1),0);
        valuecheck(model.body(i).Ttree(end,end),1);
        valuecheck(model.body(i).T_body_to_joint(end,1:end-1),0);
        valuecheck(model.body(i).T_body_to_joint(end,end),1);
      end

      model = adjustContactShapes(model);
      model = setupCollisionFiltering(model);      
            
      model.dirty = false;
      
      model = createMexPointer(model);

      % collisionDetect may require the mex version of the manipulator,
      % so it should go after createMexPointer
      phi = model.collisionDetect(zeros(model.getNumPositions,1));
      model.num_contact_pairs = length(phi);
      
      if (model.num_contact_pairs>0)
        warning('Drake:RigidBodyManipulator:UnsupportedContactPoints','Contact is not supported by the dynamics methods of this class.  Consider using TimeSteppingRigidBodyManipulator or HybridPlanarRigidBodyManipulator');
      end
      
%      H = manipulatorDynamics(model,zeros(model.num_q,1),zeros(model.num_q,1));
%      if cond(H)>1e3
%        warning('Drake:RigidBodyManipulator:SingularH','H appears to be singular (cond(H)=%f).  Are you sure you have a well-defined model?',cond(H));
%      end
    end
    
    function body_ind = findLinkInd(model,linkname,robot,error_level)
      % @param robot can be the robot number or the name of a robot
      % robot=0 means look at all robots
      % @param error_level >0 for throw error, 0 for throw warning, <0 for do nothing. @default throw error 
      % @ingroup Kinematic Tree
      if nargin<3 || isempty(robot), robot=0; end
      linkname = lower(linkname);
      linkname=regexprep(linkname, '[\[\]\\\/\.]+', '_', 'preservecase');
      
      if ischar(robot) robot = strmatch(lower(robot),lower({model.name})); end
      items = strfind(lower({model.body.linkname}),linkname);
      ind = find(~cellfun(@isempty,items));
      if (robot~=0), ind = ind([model.body(ind).robotnum]==robot); end
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
              warning('Drake:RigidBodyManipulator:WeldedLinkInd',['found ', linkname,' but it has been welded to it''s parent link (and the link''s coordinate frame may have changed).']);
            end
          end
          i=i+1;
        end
      end
      if (length(ind)~=1)
        if (nargin<4 || error_level>0)
          error(['couldn''t find unique link ' ,linkname]);
        else 
          body_ind=0;
          if (error_level==0)
            warning(['couldn''t find unique link ' ,linkname]);
          end
        end
      else
        body_ind = ind;
      end
    end

    function body = findLink(model,linkname,varargin)
      % @ingroup Deprecated
      error('the finkLink method has been deprecated.  if you really must get a *copy* of the body, then use finkLinkInd followed by getBody');
    end
    
    function is_valid = isValidLinkIndex(obj,idx)
      % @ingroup Kinematic Tree
      if ~isnumeric(idx) 
        is_valid=false(size(idx));
      else
        is_valid = idx >= 1 & idx <= obj.getNumBodies() & mod(idx,1) == 0;
      end
    end

    function frame_id = findFrameId(model,name,robotnum)
      % @param name is the string name to search for
      % @param robotnum if specified restricts the search to a particular
      % robot
      if nargin<3, robotnum=0; end
      if ~isempty(model.frame)
        items = strfind(lower({model.frame.name}),lower(name));
        ind = find(~cellfun(@isempty,items));
        if (robotnum~=0), ind = ind([model.body(model.frame(ind).body_ind).robotnum]==robotnum); end
      else
        ind = [];
      end
      if numel(ind)~=1, error('Drake:RigidBodyManipulator:UniqueFrameNotFound',['Cannot find unique frame named ', name, ' on robot number ',num2str(robotnum)]); end
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
      
      for i=1:length(model.body)
        model.body(i) = updateParams(model.body(i),fr.poly,p);
      end
      
      model = compile(model);
    end
    
    function p = getParams(model)
      p = [];
      for i=1:min(numel(model.name),numel(model.param_db))
        pn = fieldnames(model.param_db{i});
        for j=1:numel(pn)
          p = vertcat(p,model.param_db{i}.(pn{j}).value);
        end
      end
      p = Point(getParamFrame(model),p);
    end    
    
    function model = weldJoint(model,body_ind_or_joint_name,robot)
      % @ingroup Kinematic Tree
      if ischar(body_ind_or_joint_name)
        if nargin>2
          body_ind_or_joint_name = findJointInd(model,body_ind_or_joint_name,robot);
        else
          body_ind_or_joint_name = findJointInd(model,body_ind_or_joint_name);
        end
      end
      
      typecheck(body_ind_or_joint_name,'numeric');
      model.body(body_ind_or_joint_name).pitch = nan;
      model.dirty = true;
    end
        
    function body_ind = findJointInd(model,jointname,robot)
      % @param robot can be the robot number or the name of a robot
      % robot=0 means look at all robots
      % @ingroup Kinematic Tree
      if nargin<3 || isempty(robot), robot=0; end
      jointname = lower(jointname);
      if ischar(robot) robot = strmatch(lower(robot),lower({model.name})); end
      items = strfind(lower({model.body.jointname}),jointname);
      ind = find(~cellfun(@isempty,items));
      if (robot~=0), ind = ind([model.body(ind).robotnum]==robot); end
      if (length(ind)~=1)
        if (nargin<4 || throw_error)
          error(['couldn''t find unique joint ' ,jointname]);
        else 
          warning(['couldn''t find unique joint ' ,jointname]);
          body_ind=0;
        end
      else
        body_ind = ind;
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
        getTerrainContactPoints(obj,body_idx)
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
      if nargin < 2
        body_idx = 2:obj.getNumBodies(); % World-fixed objects can't collide
                                         % with the terrain
      end
      terrain_contact_point_struct = struct('pts',{},'idx',{});
      for i = body_idx
        if i ~= 1
          pts = getTerrainContactPoints(obj.body(i));
          if ~isempty(pts)
            terrain_contact_point_struct(end+1) = struct('pts',pts,'idx',i);
          end
        end
      end
    end
    
    function groups = getContactGroups(model)
      groups = {};
      for i=1:length(model.body)
        groups = horzcat(groups,model.body(i).collision_group_name);
      end
      groups = unique(groups);
    end
    
    function model = removeCollisionGroups(model,contact_groups)
      for i=1:length(model.body)
        model.body(i) = removeCollisionGroups(model.body(i),contact_groups);
      end
      model.dirty = true;
    end
    
    function model = removeCollisionGroupsExcept(model,contact_groups)
      for i=1:length(model.body)
        model.body(i) = removeCollisionGroupsExcept(model.body(i),contact_groups);
      end
      model.dirty = true;
    end
    
    function body_idx = parseBodyID(obj,body_id)
      % body_idx = parseBodyID(obj,body_id)
      % @param obj - RigidBodyManipulator object
      % @param body_id - Body index or body name
      %
      % @retval body_idx - Body index
      typecheck(body_id,{'numeric','char'});
      if isnumeric(body_id)
        body_idx = body_id;
      else % then it's a string
        body_idx = findLinkInd(obj,body_id);
      end
    end

    function obj = addContactShapeToBody(obj,body_id,shape)
      % obj = addContactShapeToBody(obj,body_id,shape)
      %
      % obj must be re-compiled after calling this method
      %
      % @param obj - RigidBodyManipulator object
      % @param body_id - Body index or body name
      % @param shape - RigidBodyGeometry (or child class) object 
      body_idx = obj.parseBodyID(body_id);
      obj.body(body_idx).contact_shapes{end+1} = shape;
      obj.dirty = true;
    end

    function obj = addVisualShapeToBody(obj,body_id,shape)
      % obj = addContactShapeToBody(obj,body_id,shape)
      %
      % @param obj - RigidBodyManipulator object
      % @param body_id - Body index or body name
      % @param shape - RigidBodyGeometry (or child class) object 
      body_idx = obj.parseBodyID(body_id);
      obj.body(body_idx).visual_shapes{end+1} = shape;
    end

    function obj = addShapeToBody(obj,body_id,shape)
      % obj = addShapeToBody(obj,body_id,shape)
      %
      % @param obj - RigidBodyManipulator object
      % @param body_id - Body index or body name
      % @param shape - RigidBodyGeometry (or child class) object 
      obj = obj.addVisualShapeToBody(body_id,shape);
      obj = obj.addContactShapeToBody(body_id,shape);
    end

    function model = replaceContactShapesWithCHull(model,body_indices,varargin)
      if any(body_indices==1)
        model = removeTerrainGeometries(model);
      end
      for body_idx = reshape(body_indices,1,[])
        model.body(body_idx) = replaceContactShapesWithCHull(model.body(body_idx),varargin{:});
      end
      if any(body_indices==1)
        model = addTerrainGeometries(model);
      end
      model.dirty = true;
    end
    
    function drawKinematicTree(model)
      % depends on having graphviz2mat installed (from matlabcentral)
      % todo: make that a dependency in configure?
      % @ingroup Kinematic Tree
      
      A = cell(length(model.body));
      for i=1:length(model.body)
        if model.body(i).parent>0
          A{model.body(i).parent,i} = model.body(i).jointname;
        end
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
        
    function m = getMass(model)
      % todo: write total_mass to class and simply return it instead of
      % looping every time (since the result is a constant between
      % compiles)
      m = 0;
      for i=1:length(model.body)
        bm = model.body(i).mass;
        m = m + bm;
      end
    end

    function body = newBody(model)
      % @ingroup Kinematic Tree
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
        @(obj,~,~,x) getCOM(model,x(1:model.featherstone.NB))); 
      
      model.getStateFrame().addTransform(tf);
    end
    
    function v = constructVisualizer(obj,options)
      checkDirty(obj);
      if nargin<2, options=struct(); end 
      if ~isfield(options,'use_contact_shapes'), options.use_contact_shapes = false; end;
      if ~isfield(options,'viewer'), options.viewer = {'BotVisualizer','RigidBodyWRLVisualizer','NullVisualizer'};
      elseif ~iscell(options.viewer), options.viewer = {options.viewer}; end
      
      v=[]; i=1;
      while isempty(v)
        type = options.viewer{i};
        
        switch (type)
          case 'NullVisualizer'
            arg = {getOutputFrame(obj)};
          case 'BotVisualizer'
            arg = {obj,options.use_contact_shapes};
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
      joint = [model.actuator.joint];
      index = [model.body(joint).dofnum];
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
      % note: intentionally jump straight to second order system (skipping manipulator)... even though it's bad form
      [varargout{:}] = pdcontrol@SecondOrderSystem(sys,Kp,Kd,index);
    end    
    
    function [phi,dphi,ddphi] = positionConstraints(obj,q)
      checkDirty(obj);
      % so far, only loop constraints are implemented
      [phi,dphi,ddphi]=loopConstraints(obj,q);
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
      
      nq = obj.getNumPositions();
      nu = obj.getNumInputs();
      
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
        v.draw(0,[quz(1:nq); zeros(nq,1)]);
      end

      function [c,ceq,GC,GCeq] = mycon(quz)
        q=quz(1:nq);
        u=quz(nq+(1:nu));
        z=quz(nq+nu+(1:nz));
        c = [];
        GC = [];
        [~,C,B,~,dC,~] = obj.manipulatorDynamics(q,zeros(nq,1));
        
        if obj.getNumContactPairs > 0,
          [phiC,normal,d,xA,xB,idxA,idxB,mu,n,D,dn,dD] = obj.contactConstraints(q,false,active_collision_options);
          
          % construct J such that J'*z is the contact force vector in joint
          J = [n;cell2mat(D')];
          % similarly, construct dJz
          dJz = matGradMult([dn;cell2mat(dD')],z,true);
          
          ceq = [C-B*u-J'*z; phiC];
          GCeq = [[dC(1:nq,1:nq)-dJz,-B,-J']',[n'; zeros(nu+nz,length(phiC))]];
        else
          ceq = [C-B*u];
          GCeq = [dC(1:nq,1:nq),-B]';
        end
        if (obj.num_xcon>0)
          [phi,dphi] = geval(@obj.stateConstraints,[q;0*q]);
          ceq = [ceq; phi];
          GCeq = [GCeq, [dphi(:,1:nq),zeros(obj.num_xcon,nu+nz)]'];
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
    
    function val = parseParamString(model,robotnum,str)
      % @ingroup URDF Parsing
      
      fr = getParamFrame(model); p=fr.poly;
      pstr = regexprep(str,'\$(\w+)','p(model.param_db{robotnum}.(''$1'').index)');
%      if strcmp(pstr,str)  % then it didn't have any parameters in it
        val = eval(['[',pstr,']']);
%      else
%        val = RigidBodyParameterizedValue(['[',pstr,']'],model,robotnum);
%      end
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
      [phi,~,~,~,~,~,~,dphi] = obj.contactConstraints(q);
    end
    
    function n = getNumUnilateralConstraints(obj)
      % Returns the number of unilateral constraints, currently only
      % contains the contact pairs
      n = obj.getNumContactPairs();
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
      if (exist('constructModelmex')==3)
%        obj.mex_model_ptr = debugMexEval('constructModelmex',obj);
        obj.mex_model_ptr = constructModelmex(obj);
      end
    end
        
    function fr = constructStateFrame(model)
      frame_dims=[];
      for i=1:length(model.name)
        joints={};
        for j=1:length(model.body)
          b = model.body(j);
          if b.parent>0 && b.robotnum==i
            if (b.floating==1)
              joints = vertcat(joints,{[b.jointname,'_x'];[b.jointname,'_y'];[b.jointname,'_z'];[b.jointname,'_roll'];[b.jointname,'_pitch'];[b.jointname,'_yaw']});
            elseif (b.floating==2)
              joints = vertcat(joints,{[b.jointname,'_x'];[b.jointname,'_y'];[b.jointname,'_z'];[b.jointname,'_qw'];[b.jointname,'_qx'];[b.jointname,'_qy'];[b.jointname,'_qz']});
            else
              joints = vertcat(joints,{b.jointname});
            end
            frame_dims(b.dofnum)=i;
          end
        end
        coordinates = vertcat(joints,cellfun(@(a) [a,'dot'],joints,'UniformOutput',false));
        fr{i} = CoordinateFrame([model.name{i},'State'],length(coordinates),'x',coordinates);
      end
%      frame_dims=[model.body(arrayfun(@(a) ~isempty(a.parent),model.body)).robotnum];
      frame_dims=[frame_dims,frame_dims];
      fr = MultiCoordinateFrame.constructFrame(fr,frame_dims,true);
    end

    function [model,fr,pval,pmin,pmax] = constructParamFrame(model)
      frames = {};
      pval=[]; pmin=[]; pmax=[];
      index=1;
      for i=1:min(numel(model.name),numel(model.param_db))
        pn = fieldnames(model.param_db{i});
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
        if isa(model.force{i},'RigidBodyThrust')
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
        
    function [model,dof] = extractFeatherstone(model)
      % @ingroup Kinematic Tree
      
      %      m=struct('NB',{},'parent',{},'jcode',{},'Xtree',{},'I',{});
      dof=0;inds=[];
      for i=1:length(model.body)
        if model.body(i).parent>0
          if (model.body(i).floating==1)
            model.body(i).dofnum=dof+(1:6)';
            dof=dof+6;
            inds = [inds,i];
          elseif (model.body(i).floating==2)
            model.body(i).dofnum=dof+(1:7)';
            dof=dof+7;
            inds = [inds,i];
          else
            dof=dof+1;
            model.body(i).dofnum=dof;
            inds = [inds,i];
          end
        else
          model.body(i).dofnum=0;
        end
      end
      m.NB= dof;  
      n=1;
      m.f_ext_map_from = inds;  % size is length(model.body) output is index into NB, or zero
      m.f_ext_map_to = [];

      for i=1:length(inds) % number of links with parents
        b=model.body(inds(i));
        if (b.floating==1)   % implement relative ypr, but with dofnums as rpy
          % todo:  remove this and handle the floating joint directly in
          % HandC.  this is really just a short term hack.
          m.dofnum(n+(0:5)) = b.dofnum([1;2;3;6;5;4]);
          m.pitch(n+(0:2)) = inf;  % prismatic
          m.pitch(n+(3:5)) = 0;    % revolute
          m.damping(n+(0:5)) = 0;
          m.coulomb_friction(n+(0:5)) = 0;
          m.static_friction(n+(0:5)) = 0;
          m.coulomb_window(n+(0:5)) = eps;
          m.parent(n+(0:5)) = [model.body(b.parent).dofnum,n+(0:4)];  % rel ypr
          m.Xtree{n} = Xroty(pi/2);   % x
          m.Xtree{n+1} = Xrotx(-pi/2)*Xroty(-pi/2); % y (note these are relative changes, x was up, now I'm rotating so y will be up)
          m.Xtree{n+2} = Xrotx(pi/2); % z
          m.Xtree{n+3} = eye(6);       % yaw
          m.Xtree{n+4} = Xrotx(-pi/2);  % pitch
          m.Xtree{n+5} = Xroty(pi/2)*Xrotx(pi/2);  % roll

%          valuecheck(b.X_joint_to_body,eye(6))); % if this isn't true, then I probably need to handle it better on the line below
          % but I can't leave the check in because it is also very ugly because this method gets run potentially
          % multiple times, and will update b each time! (so the test will
          % fail on the second pass)
          
          b.X_joint_to_body = Xroty(-pi/2); 
          % note: this is a strange and ugly case where I have to let the
          % X_joint_to_body get out of sync with the T_body_to_joint, since
          % the kinematics believes one thing and the featherstone dynamics
          % believes another.
          
          for j=0:4, m.I{n+j} = zeros(6); end
          m.I{n+5} = b.X_joint_to_body'*b.I*b.X_joint_to_body;
          m.f_ext_map_to = [m.f_ext_map_to,n+5];
          n=n+6;
        elseif (b.floating==2)
          error('dynamics for quaternion floating base not implemented yet');
        else
          m.parent(n) = max(model.body(b.parent).dofnum);
          m.dofnum(n) = b.dofnum;  % note: only need this for my floating hack above (remove it when gone)
          m.pitch(n) = b.pitch;
          m.Xtree{n} = inv(b.X_joint_to_body)*b.Xtree*model.body(b.parent).X_joint_to_body;
          m.I{n} = b.X_joint_to_body'*b.I*b.X_joint_to_body;
          m.damping(n) = b.damping;  % add damping so that it's faster to look up in the dynamics functions.
          m.coulomb_friction(n) = b.coulomb_friction;
          m.static_friction(n) = b.static_friction;
          m.coulomb_window(n) = b.coulomb_window;
          m.f_ext_map_to = [m.f_ext_map_to,n];
          n=n+1;
        end
        model.body(inds(i)) = b;  % b isn't a handle anymore, so store any changes
      end
      model.featherstone = m;
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
          else
            warning('Drake:RigidBodyManipulator:BodyHasZeroInertia',['Link ',body.linkname,' has zero inertia (even though gravity is on and it''s not a fixed joint) and will be removed']);
          end
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
          parent = setInertial(parent,parent.I + body.Xtree' * body.I * body.Xtree);
        end
        
        for j=1:length(body.visual_shapes)
          body.visual_shapes{j}.T = body.Ttree*body.visual_shapes{j}.T;
        end
        parent.visual_shapes = horzcat(parent.visual_shapes,body.visual_shapes);
        
        if (~isempty(body.contact_shapes))
          for j=1:length(body.contact_shapes)
            body.contact_shapes{j}.T = body.Ttree*body.contact_shapes{j}.T;
          end
          nshapes = length(parent.contact_shapes);
          parent.contact_shapes = {parent.contact_shapes{:},body.contact_shapes{:}};

          if ~isempty(body.collision_group_name)
            ngroups=length(parent.collision_group_name);
            [parent.collision_group_name,ia,ic]=unique(horzcat(parent.collision_group_name,body.collision_group_name),'stable');
            % note: passing 'stable' to unique (above) ensures that
            if length(parent.contact_shape_group)<length(parent.collision_group_name)
              parent.contact_shape_group{length(parent.collision_group_name)}=[];
            end
            for j=1:length(body.contact_shape_group)
              parent.contact_shape_group{ic(ngroups+j)} = [parent.contact_shape_group{ic(ngroups+j)},nshapes+body.contact_shape_group{j}];
            end
          end
        end
                
        for j=1:length(model.loop)
          model.loop(j) = updateForRemovedLink(model.loop(j),model,i);
        end
        for j=1:length(model.sensor)
          model.sensor{j} = updateForRemovedLink(model.sensor{j},model,i);
        end
        for j=1:length(model.force)
          model.force{j} = updateForRemovedLink(model.force{j},model,i);
        end
        for j=1:length(model.frame)
          model.frame(j) = updateForRemovedLink(model.frame(j),model,i);
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
            model.body(j).Xtree = model.body(j).Xtree*body.Xtree;
            if (body.wrljoint)
              model.body(j).wrljoint = [ body.wrljoint, sprintf('\n\tchildren [ Transform {\n'),model.body(j).wrljoint, sprintf('\n')];
            end
          end
        end
        model.body(body.parent) = parent;
        model = updateBodyIndices(model,[1:i-1,i+1:length(model.body)]);
      end
    end
    
    function [phi,dphi,ddphi] = loopConstraints(obj,q)
      % handle kinematic loops

      phi=[];dphi=[];ddphi=[];

      kinsol = doKinematics(obj,q,nargout>2);
      
      for i=1:length(obj.loop)
        % for each loop, add the constraints that the pt1 on body1 is in
        % the same location as pt2 on body2
        
        if (nargout>2)
          [pt1,J1,dJ1] = obj.forwardKin(kinsol,obj.loop(i).body1,obj.loop(i).pt1);
          [pt2,J2,dJ2] = obj.forwardKin(kinsol,obj.loop(i).body2,obj.loop(i).pt2);
          ddphi = [ddphi; dJ1-dJ2];
          dphi = [dphi; J1-J2];
        elseif nargout>1
          [pt1,J1] = obj.forwardKin(kinsol,obj.loop(i).body1,obj.loop(i).pt1);
          [pt2,J2] = obj.forwardKin(kinsol,obj.loop(i).body2,obj.loop(i).pt2);
          dphi = [dphi; J1-J2];
        else
          pt1 = obj.forwardKin(kinsol,obj.loop(i).body1,obj.loop(i).pt1);
          pt2 = obj.forwardKin(kinsol,obj.loop(i).body2,obj.loop(i).pt2);
        end
        phi = [phi; pt1-pt2];
      end
    end      
    
    function model=parseLink(model,robotnum,node,options)
      
      ignore = char(node.getAttribute('drakeIgnore'));
      if strcmp(lower(ignore),'true')
        return;
      end
      
      body = newBody(model);
      body.robotnum = robotnum;
      
      body.linkname=char(node.getAttribute('name'));
      body.linkname=regexprep(body.linkname, '[\[\]\\\/\.]+', '_', 'preservecase');
      
      if (options.inertial && node.getElementsByTagName('inertial').getLength()>0)
        body = parseInertial(body,node.getElementsByTagName('inertial').item(0),model,options);
      end
      
      if (options.visual && node.getElementsByTagName('visual').getLength()>0)
        visualItem = 0;
        while(~isempty(node.getElementsByTagName('visual').item(visualItem)))
          body = parseVisual(body,node.getElementsByTagName('visual').item(visualItem),model,options);
          visualItem = visualItem+1;
        end
      end
      
      if options.collision && node.getElementsByTagName('collision').getLength()>0
        collisionItem = 0;
        while(~isempty(node.getElementsByTagName('collision').item(collisionItem)))
          body = parseCollision(body,node.getElementsByTagName('collision').item(collisionItem),model,options);
          collisionItem = collisionItem+1;
        end
      end
      
      if options.sensors && node.getElementsByTagName('sensor').getLength()>0
        sensorItem = 0;
        while(~isempty(node.getElementsByTagName('sensor').item(sensorItem)))
          model = parseSensor(model,robotnum,node.getElementsByTagName('sensor').item(sensorItem),numel(model.body)+1,options);
          sensorItem = sensorItem+1;
        end
      end          
      
      model.body=[model.body,body];
    end

    function model = parseSensor(model,robotnum,node,body_ind,options)
      switch char(node.getAttribute('type'))
        case 'imu'
          model = addSensor(model,RigidBodyInertialMeasurementUnit.parseURDFNode(model,robotnum,node,body_ind,options));
        otherwise
          error(['sensor element type ',type,' not supported (yet?)']);
      end
    end

    function model = adjustContactShapes(model)
      % model = adjustContactShapes(model) returns the model with
      % adjusted contact geometries, according to the setings in
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
            'RigidBodyCylinder as a contact shape:' ...
            '\n\n' ...
            '\tBody Idx\tBody Name\n' ...
            repmat('\t%d:\t\t%s\n',1,sum(body_changed)) ...
            '\n\n' ...
            'These contact shapes were replaced by ' ...
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
    
    function model = parseCollisionFilterGroup(model,robotnum,node,options)
      ignore = char(node.getAttribute('drakeIgnore'));
      if strcmpi(ignore,'true')
        return;
      end
      collision_fg_name = char(node.getAttribute('name'));
      if isKey(model.collision_filter_groups,collision_fg_name)
        error('RigidBodyManipulator:parseCollisionFilterGroup:repeated_collision_fg_name', ...
              ['A collision filter group with the collision_fg_name %s already exists in this '...
               'RigidBodyManipulator'], collision_fg_name); 
      end

      model.collision_filter_groups(collision_fg_name) = CollisionFilterGroup();

      members = node.getElementsByTagName('member');
      if members.getLength()>0
        members_cell = cell(1,members.getLength());
        for i=0:(members.getLength()-1)
          members_cell{i+1} = char(members.item(i).getAttribute('link'));
        end
        model = addLinksToCollisionFilterGroup(model,members_cell,collision_fg_name,robotnum);
      end

      ignored_collision_fgs = node.getElementsByTagName('ignored_collision_filter_group');
      if ignored_collision_fgs.getLength()>0
        ignored_collision_fgs_cell = cell(1,ignored_collision_fgs.getLength());
        for i=0:(ignored_collision_fgs.getLength()-1)
          ignored_collision_fgs_cell{i+1} = char(ignored_collision_fgs.item(i).getAttribute('collision_filter_group'));
        end
        model = addToIgnoredListOfCollisionFilterGroup(model,ignored_collision_fgs_cell,collision_fg_name);
      end
    end

    function id = findCollisionFilterGroupID(model,collision_fg_name)
        id = uint16(find(~cellfun(@isempty,strfind(model.collision_filter_groups.keys(),collision_fg_name))));
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
        model.collision_filter_groups=containers.Map('KeyType','char','ValueType','any');     
        model.collision_filter_groups('no_collision') = CollisionFilterGroup();
      end
      if model.contact_options.ignore_self_collisions
        body_indices = 1:model.getNumBodies();
        for i = 1:length(model.name)
          robot_i_body_indices = body_indices([model.body.robotnum] == i);
          model.collision_filter_groups(model.name{i}) = CollisionFilterGroup();
          model = model.addLinksToCollisionFilterGroup(robot_i_body_indices,model.name{i},i);
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
      link_indices = cellfun(@(name,num) findLinkInd(model,name,num), ...
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

    function model=parseJoint(model,robotnum,node,options)
      
      ignore = char(node.getAttribute('drakeIgnore'));
      if strcmp(lower(ignore),'true')
        return;
      end
      
      parentNode = node.getElementsByTagName('parent').item(0);
      if isempty(parentNode) % then it's not the main joint element.  for instance, the transmission element has a joint element, too
        return
      end
      parent = findLinkInd(model,char(parentNode.getAttribute('link')),robotnum);
      
      childNode = node.getElementsByTagName('child').item(0);
      child = findLinkInd(model,char(childNode.getAttribute('link')),robotnum);
      
      name = char(node.getAttribute('name'));
      type = char(node.getAttribute('type'));
      xyz=zeros(3,1); rpy=zeros(3,1);
      origin = node.getElementsByTagName('origin').item(0);  % seems to be ok, even if origin tag doesn't exist
      if ~isempty(origin)
        if origin.hasAttribute('xyz')
          xyz = reshape(parseParamString(model,robotnum,char(origin.getAttribute('xyz'))),3,1);
        end
        if origin.hasAttribute('rpy')
          rpy = reshape(parseParamString(model,robotnum,char(origin.getAttribute('rpy'))),3,1);
        end
      end
      axis=[1;0;0];  % default according to URDF documentation
      axisnode = node.getElementsByTagName('axis').item(0);
      if ~isempty(axisnode)
        if axisnode.hasAttribute('xyz')
          axis = reshape(parseParamString(model,robotnum,char(axisnode.getAttribute('xyz'))),3,1);
          axis = axis/(norm(axis)+eps); % normalize
        end
      end
      damping=0;
      coulomb_friction=0;
      static_friction=0;
      coulomb_window=eps;
      dynamics = node.getElementsByTagName('dynamics').item(0);
      if ~isempty(dynamics)
        if dynamics.hasAttribute('damping')
          damping = parseParamString(model,robotnum,char(dynamics.getAttribute('damping')));
        end
        if ~options.ignore_friction && dynamics.hasAttribute('friction')
          coulomb_friction = parseParamString(model,robotnum,char(dynamics.getAttribute('friction')));
          if coulomb_friction < 0
            error('RigidBodyManipulator: coulomb_friction must be >= 0');
          end
        end
        if ~options.ignore_friction && dynamics.hasAttribute('stiction')
          warning('RigidBodyManipulator:  stiction is not supported yet.');
          static_friction = parseParamString(model,robotnum,char(dynamics.getAttribute('stiction')));
          if static_friction < 0
            error('RigidBodyManipulator: static_friction must be >= 0');
          end
        end
        if ~options.ignore_friction && dynamics.hasAttribute('coulomb_window')
          coulomb_window = parseParamString(model,robotnum,char(dynamics.getAttribute('coulomb_window')));
          if coulomb_window <= 0
            error('RigidBodyManipulator: coulomb_window must be > 0');
          end
        end
      end
      
      % add noise to damping
      if ~isnumeric(damping) && options.damping_error
        warning('damping error not supported for parameterized values (yet)');
      end
      if isnumeric(damping)
        damping = max(0,(1+options.damping_error*randn())*damping);
      end

      joint_limit_min=-inf;
      joint_limit_max=inf;
      effort_min=-inf;
      effort_max=inf;
      velocity_limit=inf;
      limits = node.getElementsByTagName('limit').item(0);
      if ~isempty(limits)
        if limits.hasAttribute('lower')
          joint_limit_min = parseParamString(model,robotnum,char(limits.getAttribute('lower')));
        end
        if limits.hasAttribute('upper');
          joint_limit_max = parseParamString(model,robotnum,char(limits.getAttribute('upper')));
        end
        if limits.hasAttribute('effort');
          effort = parseParamString(model,robotnum,char(limits.getAttribute('effort')));
          effort_min = min(-effort,effort); % just in case someone puts the min effort in the URDF
          effort_max = max(-effort,effort);
        end
        if limits.hasAttribute('effort_min');
          effort_min = parseParamString(model,robotnum,char(limits.getAttribute('effort_min')));
        end
        if limits.hasAttribute('effort_max');
          effort_max = parseParamString(model,robotnum,char(limits.getAttribute('effort_max')));
        end
        if limits.hasAttribute('velocity');
          warnOnce(model.warning_manager,'Drake:RigidBodyManipulator:UnsupportedVelocityLimits','RigidBodyManipulator: velocity limits are not supported yet');
          velocity_limit = parseParamString(model,robotnum,char(limits.getAttribute('velocity')));
        end
      end
      
      limits = struct();
      limits.joint_limit_min = joint_limit_min;
      limits.joint_limit_max = joint_limit_max;
      limits.effort_min = effort_min;
      limits.effort_max = effort_max;
      limits.velocity_limit = velocity_limit;
      
      name=regexprep(name, '\.', '_', 'preservecase');
      model = addJoint(model,name,type,parent,child,xyz,rpy,axis,damping,coulomb_friction,static_friction,coulomb_window,limits);
      
      if node.hasAttribute('has_position_sensor')
        model.body(child).has_position_sensor = str2num(char(node.getAttribute('has_position_sensor')));
      else
        model.body(child).has_position_sensor = true;
      end          
    end
    
    
    function model = parseLoopJoint(model,robotnum,node,options)
      loop = RigidBodyLoop();
      loop.name = char(node.getAttribute('name'));
      loop.name = regexprep(loop.name, '\.', '_', 'preservecase');

      link1Node = node.getElementsByTagName('link1').item(0);
      link1 = findLinkInd(model,char(link1Node.getAttribute('link')),robotnum);
      loop.body1 = link1;
      if link1Node.hasAttribute('xyz')
        loop.pt1 = reshape(str2num(char(link1Node.getAttribute('xyz'))),3,1);
      end
      
      link2Node = node.getElementsByTagName('link2').item(0);
      link2 = findLinkInd(model,char(link2Node.getAttribute('link')),robotnum);
      loop.body2 = link2;
      if link2Node.hasAttribute('xyz')
        loop.pt2 = reshape(str2num(char(link2Node.getAttribute('xyz'))),3,1);
      end
      
      axis=[1;0;0];  % default according to URDF documentation
      axisnode = node.getElementsByTagName('axis').item(0);
      if ~isempty(axisnode)
        if axisnode.hasAttribute('xyz')
          axis = reshape(parseParamString(model,robotnum,char(axisnode.getAttribute('xyz'))),3,1);
          axis = axis/(norm(axis)+eps); % normalize
        end
      end
      loop.axis = axis;
      
      type = char(node.getAttribute('type'));
      switch (lower(type))
        case {'continuous'}
          warning('Drake:RigidBodyManipulator:ThreeDLoopJoints','3D loop joints do not properly enforce the joint axis constraint.  (they perform more like a ball joint).  See bug 1389');
        otherwise
          error(['joint type ',type,' not supported (yet?)']);
      end
      
      model.loop=[model.loop,loop];
    end
    
    function model = parseForceElement(model,robotnum,node,options)
      fe = [];
      childNodes = node.getChildNodes();
      elnode = node.getElementsByTagName('linear_spring_damper').item(0);
      if ~isempty(elnode)
        [model,fe] = RigidBodySpringDamper.parseURDFNode(model,robotnum,elnode,options);
      end
      
      elnode = node.getElementsByTagName('wing').item(0);
      if ~isempty(elnode)
        [model,fe] = RigidBodyWing.parseURDFNode(model,robotnum,elnode,options);
      end
      
      elnode = node.getElementsByTagName('thrust').item(0);
      if ~isempty(elnode)
        [model,fe] = RigidBodyThrust.parseURDFNode(model,robotnum,elnode,options);
      end
      
      elnode = node.getElementsByTagName('added_mass').item(0);
      if ~isempty(elnode)
        [model,fe] = RigidBodyAddedMass.parseURDFNode(model,robotnum,elnode,options);
      end
      
      elnode = node.getElementsByTagName('buoyancy').item(0);
      if ~isempty(elnode)
        [model,fe] = RigidBodyBuoyant.parseURDFNode(model,robotnum,elnode,options);
      end
      
      if ~isempty(fe)
        model.force{end+1} = fe;
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
      
      for i=1:length(model.body)  
        model.body(i) = updateBodyIndices(model.body(i),mapfun);
      end
      for i=1:length(model.actuator)
        model.actuator(i) = updateBodyIndices(model.actuator(i),mapfun);
      end
      for i=1:length(model.loop)
        model.loop(i) = updateBodyIndices(model.loop(i),mapfun);
      end
      for i=1:length(model.sensor)
        model.sensor{i} = updateBodyIndices(model.sensor{i},mapfun);
      end
      for i=1:length(model.force)
        model.force{i} = updateBodyIndices(model.force{i},mapfun);
      end
      for i=1:length(model.frame)
        model.frame(i) = updateBodyIndices(model.frame(i),mapfun);
      end
    end
    
  end

end

