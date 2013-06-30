classdef RigidBodyManipulator < Manipulator
  % This class wraps the spatial vector library (v1) 
  % provided by Roy Featherstone on his website: 
  %   http://users.cecs.anu.edu.au/~roy/spatial/documentation.html
    
  properties (SetAccess=protected)
    name={};        % names of the objects in the rigid body system
    urdf={};        % names of the urdf files loaded
    body=[];        % array of RigidBody objects
    actuator = [];  % array of RigidBodyActuator objects
    loop=[];        % array of RigidBodyLoop objects
    sensor={};      % cell array of RigidBodySensor objects
    force={};       % cell array of RigidBodyForceElement objects 
      % note: need {} for arrays that will have multiple types (e.g.
      % a variety of derived classes), but can get away with [] for arrays
      % with elements that are all exactly the same type
    gravity=[0;0;-9.81];
    terrain;
  end
  
  properties (Access=public)
    featherstone = [];
  end
  
  properties (Access=public)
    B = [];
    mex_model_ptr = 0;
    dirty = true;
  end
  
  methods
    function obj = RigidBodyManipulator(urdf_filename,options)
      obj = obj@Manipulator(0,0);
      obj.body = newBody(obj);
      obj.body.linkname = 'world';
      
      if (nargin>0 && ~isempty(urdf_filename))
        if (nargin<2) options = struct(); end
        obj = addRobotFromURDF(obj,urdf_filename,zeros(3,1),zeros(3,1),options);
      end
      obj.terrain = RigidBodyTerrain;
    end
    
    function checkDirty(obj)
      if (obj.dirty)
        error('You''ve changed something about this model and need to manually compile it.  Use obj=compile(obj).');
      end
    end
    
    function y = output(obj,t,x,u)
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
      typecheck(terrain,'RigidBodyTerrain');
      obj.terrain = terrain;
    end

    function [z,normal] = getTerrainHeight(obj,contact_pos)
      [z,normal] = getHeight(obj.terrain,contact_pos(1:2,:));
    end
  
    function B = getB(obj)
      checkDirty(obj);
      B = obj.B;
    end
    
    function n = getNumDOF(obj)
      n = obj.num_q;
    end
    
    function n = getNumBodies(obj)
      n = length(obj.body);
    end
    
    function str = getLinkName(obj,body_ind)
      str = obj.body(body_ind).linkname;
    end
    
    function obj = setGravity(obj,grav)
      sizecheck(grav,size(obj.gravity));
      obj.gravity = grav;
      obj.dirty = true;
    end

    function ptr = getMexModelPtr(obj)
      ptr = obj.mex_model_ptr;
    end
    
    function f = cartesianForceToSpatialForce(obj,kinsol,body,point,force)  
      % @param body is a rigid body element
      % @param point is a point on the rigid body (in body coords)
      % @param force is a cartesion force (in world coords)
      
      % convert force to body coordinates
      ftmp=bodyKin(obj,kinsol,body,[force,zeros(3,1)]);
      
      % convert to joint frame (featherstone dynamics algorithm never reasons in body coordinates)
      point = body.T_body_to_joint(1:end-1,:)*[point;1];
      ftmp = body.T_body_to_joint(1:end-1,:)*[ftmp; 1,1];

      force = ftmp(:,1)-ftmp(:,2);  
      
      % compute spatial force (from Fpt in featherstone v2)
      f = [ cross(point,force,1); force ];
    end
    
    function model=addJoint(model,name,type,parent,child,xyz,rpy,axis,damping,limits)
      if (nargin<6) xyz=zeros(3,1); end
      if (nargin<7) rpy=zeros(3,1); end
      if (nargin<8) axis=[1;0;0]; end
      if (nargin<9) damping=0; end
      if (nargin<10)
        limits = struct();
        limits.joint_limit_min = -Inf;
        limits.joint_limit_max = Inf;
        limits.effort_limit = Inf;
        limits.velocity_limit = Inf;
      end
        
      if ~isempty(child.parent)
        error('there is already a joint connecting this child to a parent');
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
      child.parent = parent;
      
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
          
        case 'prismatic'
          child.pitch = inf;
          child.damping = damping;
          
        case 'fixed'
          child.pitch = nan;
          
        case 'floating_rpy'
          child.pitch = 0;
          child.floating = 1;
          limits.joint_limit_min = repmat(limits.joint_limit_min,1,6);
          limits.joint_limit_max = repmat(limits.joint_limit_max,1,6);
          limits.effort_limit = repmat(limits.effort_limit,1,6);
          limits.velocity_limit = repmat(limits.velocity_limit,1,6);
          
        case 'floating_quat'
          child.pitch = 0;
          child.floating = 2;
          limits.joint_limit_min = repmat(limits.joint_limit_min,1,7);
          limits.joint_limit_max = repmat(limits.joint_limit_max,1,7);
          limits.effort_limit = repmat(limits.effort_limit,1,7);
          limits.velocity_limit = repmat(limits.velocity_limit,1,7);
          
        otherwise
          error(['joint type ',type,' not supported (yet?)']);
      end
      child.joint_axis = axis;
      child.joint_limit_min = limits.joint_limit_min;
      child.joint_limit_max = limits.joint_limit_max;
      child.effort_limit = limits.effort_limit;
      child.velocity_limit = limits.velocity_limit;
      
      model.dirty = true;
    end
    
    function model = addFloatingBase(model,parent,rootlink,xyz,rpy,joint_type)
      % note that the case matters.
      % use lower case for extrinsic (absolute) rotations, and upper case
      % for intrinsic (relative) rotations
      
      if (nargin<6) joint_type = 'rpy'; end

      switch (joint_type)
        case 'rpy'  % extrinsic coordinates 
          model = addJoint(model,'base','floating_rpy',parent,rootlink,zeros(3,1),zeros(3,1));

        case 'quat'
          model = addJoint(model,'base','floating_quat',parent,rootlink,zeros(3,1),zeros(3,1));
        
        case 'RPY'  % instrinsic coordinates
          body1 = newBody(model);
          body1.linkname = 'base_x';
          body1.robotnum=rootlink.robotnum;
          model.body = [model.body,body1];
          model = addJoint(model,body1.linkname,'prismatic',parent,body1,xyz,rpy,[1;0;0],0);
          
          body2=newBody(model);
          body2.linkname = 'base_y';
          body2.robotnum=rootlink.robotnum;
          model.body = [model.body,body2];
          model = addJoint(model,body2.linkname,'prismatic',body1,body2,zeros(3,1),zeros(3,1),[0;1;0],0);
          
          body3=newBody(model);
          body3.linkname = 'base_z';
          body3.robotnum=rootlink.robotnum;
          model.body = [model.body,body3];
          model = addJoint(model,body3.linkname,'prismatic',body2,body3,zeros(3,1),zeros(3,1),[0;0;1],0);
          
          body4=newBody(model);
          body4.linkname = 'base_relative_roll';
          body4.robotnum=rootlink.robotnum;
          model.body = [model.body,body4];
          model = addJoint(model,body4.linkname,'revolute',body3,body4,zeros(3,1),zeros(3,1),[1;0;0],0);
          
          body5=newBody(model);
          body5.linkname = 'base_relative_pitch';
          body5.robotnum=rootlink.robotnum;
          model.body = [model.body,body5];
          model = addJoint(model,body5.linkname,'revolute',body4,body5,zeros(3,1),zeros(3,1),[0;1;0],0);
          
          model = addJoint(model,'base_relative_yaw','revolute',body5,rootlink,zeros(3,1),zeros(3,1),[0;0;1],0);
          
        case 'YPR' % intrinsic
        
          body1 = newBody(model);
          body1.linkname = 'base_x';
          body1.robotnum=rootlink.robotnum;
          model.body = [model.body,body1];
          model = addJoint(model,body1.linkname,'prismatic',parent,body1,xyz,rpy,[1;0;0],0);
          
          body2=newBody(model);
          body2.linkname = 'base_y';
          body2.robotnum=rootlink.robotnum;
          model.body = [model.body,body2];
          model = addJoint(model,body2.linkname,'prismatic',body1,body2,zeros(3,1),zeros(3,1),[0;1;0],0);
          
          body3=newBody(model);
          body3.linkname = 'base_z';
          body3.robotnum=rootlink.robotnum;
          model.body = [model.body,body3];
          model = addJoint(model,body3.linkname,'prismatic',body2,body3,zeros(3,1),zeros(3,1),[0;0;1],0);
          
          body4=newBody(model);
          body4.linkname = ['base_relative_yaw'];
          body4.robotnum=rootlink.robotnum;
          model.body = [model.body,body4];
          model = addJoint(model,body4.linkname,'revolute',body3,body4,zeros(3,1),zeros(3,1),[0;0;1],0);
          
          body5=newBody(model);
          body5.linkname = 'base_relative_pitch';
          body5.robotnum=rootlink.robotnum;
          model.body = [model.body,body5];
          model = addJoint(model,body5.linkname,'revolute',body4,body5,zeros(3,1),zeros(3,1),[0;1;0],0);
          
          model = addJoint(model,'base_relative_roll','revolute',body5,rootlink,zeros(3,1),zeros(3,1),[1;0;0],0);

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
        if (~isempty(model.body(i).parent))
          ind = find([model.body] == model.body(i).parent);
          if (ind>i)
            model.body = [model.body(1:i-1),model.body(ind),model.body(i:ind-1),model.body(ind+1:end)];
            i=i-1;
          end
        end
        i=i+1;
      end
            
      %% extract featherstone model structure
      [model,num_dof] = extractFeatherstone(model);
      
      if (num_dof<1) error('This model has no DOF!'); end

      u_limit = repmat(inf,length(model.actuator),1);

      %% extract B matrix
      B = sparse(num_dof,0);
      for i=1:length(model.actuator)
        B(model.actuator(i).joint.dofnum,i) = model.actuator(i).reduction;
        if ~isinf(model.actuator(i).joint.effort_limit)
          u_limit(i) = abs(model.actuator(i).joint.effort_limit/model.actuator(i).reduction);
          if sum(B(model.actuator(i).joint.dofnum,:)~=0)>1
            warning('Drake:RigidBodyManipulator:UnsupportedJointEffortLimit','The specified joint effort limit cannot be expressed as simple input limits; the offending limits will be ignored');
            u_limit(B(model.actuator(i).joint.dofnum,:)~=0)=inf;
          end
        end
      end
      model.B = full(B);

      model = model.setNumInputs(size(model.B,2));
      model = model.setNumDOF(num_dof);
      model = model.setNumOutputs(2*num_dof);

      if getNumInputs(model)>0
        model = setInputFrame(model,constructInputFrame(model));
      end

      if getNumStates(model)>0
        stateframe = constructStateFrame(model);
        model = setStateFrame(model,stateframe);
      else
        stateframe = getStateFrame(model);  % needed for output frame logic below
      end
      
      if length(model.sensor)>0
        feedthrough = false;
        for i=1:length(model.sensor)
          model.sensor{i} = model.sensor{i}.compile(model);
          outframe{i} = model.sensor{i}.getFrame(model);
          feedthrough = feedthrough || model.sensor{i}.isDirectFeedthrough;
        end
        fr = MultiCoordinateFrame.constructFrame(outframe);
        model = setNumOutputs(model,fr.dim);
        model = setOutputFrame(model,fr);
        model = setDirectFeedthrough(model,feedthrough);
      else
        model = setOutputFrame(model,stateframe);  % output = state
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
        warning('Drake:RigidBodyManipulator:UnsupportedJointLimits','Joint limits are not supported by this class.  Consider using HybridPlanarRigidBodyManipulator');
      end
      model.num_contacts = size([model.body.contact_pts],2);
      if (model.num_contacts>0)
        warning('Drake:RigidBodyManipulator:UnsupportedContactPoints','Contact is not supported by this class.  Consider using HybridPlanarRigidBodyManipulator');
      end
      
      model = model.setInputLimits(-u_limit,u_limit);
      
      %% initialize kinematics caching
      for i=1:length(model.body)
        if ~isempty(model.body(i).parent)
          if (model.body(i).floating==1)
            model.body(i).cached_q = nan(1,6);
            model.body(i).cached_qd = nan(1,6);
          elseif (model.body(i).floating==2)
            model.body(i).cached_q = nan(1,7);
            model.body(i).cached_qd = nan(1,7);
          else
            model.body(i).cached_q = nan;
            model.body(i).cached_qd = nan;
          end
        end
        % check basic assumption from kinematics:
        valuecheck(model.body(i).Ttree(end,1:end-1),0);
        valuecheck(model.body(i).Ttree(end,end),1);
        valuecheck(model.body(i).T_body_to_joint(end,1:end-1),0);
        valuecheck(model.body(i).T_body_to_joint(end,end),1);
      end
      
      model = createMexPointer(model);
      model.dirty = false;
      
      H = manipulatorDynamics(model,zeros(model.num_q,1),zeros(model.num_q,1));
      if cond(H)>1e3
        warning('Drake:RigidBodyManipulator:SingularH','H appears to be singular (cond(H)=%f).  Are you sure you have a well-defined model?',cond(H));
      end
    end
    
    function body_ind = findLinkInd(model,linkname,robot,throw_error)
      % @param robot can be the robot number or the name of a robot
      % robot=0 means look at all robots
      if nargin<3 || isempty(robot), robot=0; end
      linkname = lower(linkname);
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
              warning(['found ', linkname,' but it has been welded to it''s parent link (and the link''s coordinate frame may have changed).']);
            end
          end
          i=i+1;
        end
      end
      if (length(ind)~=1)
        if (nargin<4 || throw_error)
          error(['couldn''t find unique link ' ,linkname]);
        else 
          warning(['couldn''t find unique link ' ,linkname]);
          body_ind=[];
        end
      else
        body_ind = ind;
      end
    end

    function body = findLink(model,linkname,varargin)
      % @param robot can be the robot number or the name of a robot
      % robot=0 means look at all robots
      ind = findLinkInd(model,linkname,varargin{:});
      body = model.body(ind);
    end
    
    function body = getLink(model,body_ind)
      body = model.body(body_ind);
    end
        
    function body = findJoint(model,jointname,robot)
      % @param robot can be the robot number or the name of a robot
      % robot=0 means look at all robots
      if nargin<3 || isempty(robot), robot=0; end
      if ischar(robot) robot = strmatch(lower(robot),lower({model.name})); end
      ind = strmatch(lower(jointname),lower({model.body.jointname}),'exact');
      if (robot~=0), ind = ind([model.body(ind).robotnum]==robot); end
      if (length(ind)~=1)
        error(['couldn''t find unique joint ' ,jointname]);
      else
        body = model.body(ind);
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
        removeCollisionGroups(model.body(i),contact_groups);
      end
      model.dirty = true;
    end
    
    function model = removeCollisionGroupsExcept(model,contact_groups)
      for i=1:length(model.body)
        removeCollisionGroupsExcept(model.body(i),contact_groups);
      end
      model.dirty = true;
    end
    
    function drawKinematicTree(model)
      % depends on having graphviz2mat installed (from matlabcentral)
      % todo: make that a dependency in configure?
      
      A = cell(length(model.body));
      for i=1:length(model.body)
        if ~isempty(model.body(i).parent)
          A{find(model.body(i).parent==[model.body]),i} = model.body(i).jointname;
        end
      end
      node_names = {model.body.linkname};
%      node_names = regexprep({model.body.linkname},'+(.)*','');  
      drawGraph(A,node_names);
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
      body = RigidBody();
    end
    
    function newmodel = copy(model)
      % Makes a deep copy of the manipulator
      % Since RigidBody's are handles, they must be copied more carefully.
      
      newmodel = model;
      for i=1:length(model.body)
        newmodel.body(i) = copy(model.body(i));
        a=model.body(i); b=newmodel.body(i);
        
        % now crawl through data structures and update all pointers
        for j=1:length(newmodel.body)
          if (newmodel.body(j).parent == a), newmodel.body(j).parent = b; end
        end
        
        for j=1:length(newmodel.actuator)
          if (newmodel.actuator(j).body == a), newmodel.actuator(j).body = b; end
        end
        
        for j=1:length(newmodel.loop)
          if (newmodel.loop(j).body1 == a), newmodel.loop(j).body1 = b; end
          if (newmodel.loop(j).body2 == a), newmodel.loop(j).body2 = b; end
        end
        
      end
      
    end

    function fr = constructCOMFrame(model)
      fr = SingletonCoordinateFrame([model.name,'COM'],3,'m',{'com_x','com_y','com_z'});
      
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
%      try 
%        v = BotVisualizer(obj);
%      catch ex
%        warning(ex.identifier,ex.message);
        v = RigidBodyWRLVisualizer(obj);
%      end      
    end
    
    function index = getActuatedJoints(model)
      joint = [model.actuator.joint];
      index = [joint.dofnum];
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
      
      nq = obj.getNumDOF();
      nu = obj.getNumInputs();

      if isfield(options,'active_collision_groups') 
        nz=0; npts=0; active_contacts=[];
        if isfield(options,'active_collision_bodies')
          bodies=obj.body(active_collision_bodies);
        else
          bodies=obj.body;
        end
        for i=1:length(bodies)
          b=bodies(i);
          for j=1:length(b.collision_group_name)
            if any(strcmpi(b.collision_group_name{j},options.active_collision_groups))
              nz=nz+length(b.collision_group{j})*3;
              active_contacts=[active_contacts,npts+b.collision_group{j}];
            end
          end
          npts=npts+size(b.contact_pts,2);
        end
      else
        active_contacts = 1:getNumContacts(obj);
        nz = getNumContacts(obj)*3;
      end      

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
      lb_z(3:3:end) = 0; % normal forces must be >=0
      ub_z = 1e6*ones(nz,1);
    
      [jl_min,jl_max] = obj.getJointLimits();
      % force search to be close to starting position
      problem.lb = [jl_min; obj.umin; lb_z];
      problem.ub = [jl_max; obj.umax; ub_z];

      [quz_sol,~,exitflag] = fmincon(problem);
      success=(exitflag==1);
      xstar = [quz_sol(1:nq); zeros(nq,1)];
      ustar = quz_sol(nq+(1:nu));
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

        [~,C,B,~,dC,~] = obj.manipulatorDynamics(q,zeros(nq,1));
        [phiC,JC] = obj.contactConstraints(q);
        [~,J,dJ] = obj.contactPositions(q);
        
        % note: it would be more elegant to handle collision groups in
        % contactConstraints and contactPositions
        phiC=phiC(active_contacts);
        JC = JC(active_contacts,:);
        ind = [active_contacts*3 - 2; active_contacts*3 - 1; active_contacts*3];
        ind = ind(:);
        J = J(ind,:);
        dJ = dJ(ind,:);
        
        % ignore friction constraints for now
        c = 0;
        GC = zeros(nq+nu+nz,1); 
        
        dJz = zeros(nq,nq);
        for i=1:nq
            dJz(:,i) = dJ(:,(i-1)*nq+1:i*nq)'*z;
        end
        
        ceq = [C-B*u-J'*z; phiC];
        GCeq = [[dC(1:nq,1:nq)-dJz,-B,-J']',[JC'; zeros(nu+nz,length(phiC))]]; 
        
        if (obj.num_xcon>0)
          [phi,dphi] = geval(@obj.stateConstraints,[q;0*q]);
          ceq = [ceq; phi];
          GCeq = [GCeq, [dphi(:,1:nq),zeros(obj.num_xcon,nu+nz)]'];
        end
      end      
    end
  end
  
  methods (Static)
    function d=surfaceTangents(normal)
      %% compute tangent vectors, according to the description in the last paragraph of Stewart96, p.2678
      t1=normal; % initialize size
      
      % handle the normal = [0;0;1] case
      ind=(1-normal(3,:))<10e-8;  % since it's a unit normal, i can just check the z component
      t1(:,ind) = [ones(1,sum(ind)); zeros(2,sum(ind))];
      ind=~ind;
      
      % now the general case
      t1(:,ind) = [normal(2,ind);-normal(1,ind);zeros(1,sum(ind))]; % cross(normal,[0;0;1]) normalized
      t1(:,ind) = bsxfun(@rdivide,t1(:,ind),sqrt(sum(t1(1:2,ind).^2,1))); % normalize
      
      t2 = cross(t1,normal);
      
      m = 2;  % half of the the number of direct vectors
      theta = (0:(m-1))*pi/m;
      
      for k=1:m
        d{k}=cos(theta(k))*t1 + sin(theta(k))*t2;
      end      
      % NOTEST
    end
    
    [c,options] = parseMaterial(node,options);
  end
  
  methods (Access=protected)
    
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
          if ~isempty(b.parent) && b.robotnum==i
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
        fr{i} = SingletonCoordinateFrame([model.name{i},'State'],length(coordinates),'x',coordinates);
      end
%      frame_dims=[model.body(arrayfun(@(a) ~isempty(a.parent),model.body)).robotnum];
      frame_dims=[frame_dims,frame_dims];
      fr = MultiCoordinateFrame.constructFrame(fr,frame_dims,true);
    end
    
    function fr = constructInputFrame(model)
      for i=1:length(model.name)
        robot_inputs = cellfun(@(a) a.robotnum==i,{model.actuator.joint});
        coordinates = {model.actuator(robot_inputs).name}';
        fr{i}=SingletonCoordinateFrame([model.name{i},'Input'],sum(robot_inputs),'u',coordinates);
      end
      actjoints = [model.actuator.joint];
      frame_dims = [actjoints.robotnum];
      fr = MultiCoordinateFrame.constructFrame(fr,frame_dims,true);
    end
    
    function [model,dof] = extractFeatherstone(model)
%      m=struct('NB',{},'parent',{},'jcode',{},'Xtree',{},'I',{});
      dof=0;inds=[];
      for i=1:length(model.body)
        if (~isempty(model.body(i).parent))
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
        end
      end
      m.NB= dof;  
      n=1;
      for i=1:length(inds) % number of links with parents
        b=model.body(inds(i));
        if (b.floating==1)   % implement relative ypr, but with dofnums as rpy
          % todo:  remove this and handle the floating joint directly in
          % HandC.  this is really just a short term hack.
          m.dofnum(n+(0:5)) = b.dofnum([1;2;3;6;5;4]);
          m.pitch(n+(0:2)) = inf;  % prismatic
          m.pitch(n+(3:5)) = 0;    % revolute
          m.damping(n+(0:5)) = 0;
          m.parent(n+(0:5)) = [b.parent.dofnum,n+(0:4)];  % rel ypr
          m.Xtree{n} = Xroty(pi/2);   % x
          m.Xtree{n+1} = Xrotx(-pi/2)*Xroty(-pi/2); % y (note these are relative changes, x was up, now I'm rotating so y will be up)
          m.Xtree{n+2} = Xrotx(pi/2); % z
          m.Xtree{n+3} = eye(6);       % yaw
          m.Xtree{n+4} = Xrotx(-pi/2);  % pitch
          m.Xtree{n+5} = Xroty(pi/2)*Xrotx(pi/2);  % roll
          b.X_joint_to_body = Xroty(-pi/2);
          for j=0:4, m.I{n+j} = zeros(6); end
          m.I{n+5} = b.X_joint_to_body'*b.I*b.X_joint_to_body;
          n=n+6;
        elseif (b.floating==2)
          error('dynamics for quaternion floating base not implemented yet');
        else
          m.parent(n) = max(b.parent.dofnum);
          m.dofnum(n) = b.dofnum;  % note: only need this for my floating hack above (remove it when gone)
          m.pitch(n) = b.pitch;
          m.Xtree{n} = inv(b.X_joint_to_body)*b.Xtree*b.parent.X_joint_to_body;
          m.I{n} = b.X_joint_to_body'*b.I*b.X_joint_to_body;
          m.damping(n) = b.damping;  % add damping so that it's faster to look up in the dynamics functions.
          n=n+1;
        end
      end
      model.featherstone = m;
    end
      
    function model=removeFixedJoints(model)
      % takes any fixed joints out of the tree, adding their visuals and
      % inertia to their parents, and connecting their children directly to
      % their parents

      fixedind = find(isnan([model.body.pitch]) | ... % actual fixed joint
        cellfun(@(a)~any(any(a)),{model.body.I}));    % body has no inertia (yes, it happens in pr2.urdf)
      
      for i=fixedind(end:-1:1)  % go backwards, since it is presumably more efficient to start from the bottom of the tree
        body = model.body(i);
        parent = body.parent;
        if isempty(parent)
          % if it happens to be the root joint, then don't remove this one.
          continue;
        end
          
        if ~isnan(body.pitch)
          if any([model.body.parent] == body) || any(any(body.I))
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
          % same as the composite inertia calculation in HandC.m
          setInertial(parent,parent.I + body.Xtree' * body.I * body.Xtree);
        end
        
        % add wrl geometry into parent
        if ~isempty(body.wrlgeometry)
          if isempty(body.wrljoint)
            parent.wrlgeometry = [ parent.wrlgeometry, '\n', body.wrlgeometry ];
          else
            parent.wrlgeometry = [ parent.wrlgeometry, '\nTransform {\n', body.wrljoint, '\n children [\n', body.wrlgeometry, '\n]\n}\n'];
          end
        end
        
        if (~isempty(body.contact_pts))
          npts = size(parent.contact_pts,2);
          parent.contact_pts = [parent.contact_pts, body.Ttree(1:end-1,:)*[body.contact_pts;ones(1,size(body.contact_pts,2))]];
          % todo: finish this!
          for j=1:length(body.contact_shapes)
            body.contact_shapes(j).T = body.Ttree*body.contact_shapes(j).T;
          end
          nshapes = length(parent.contact_shapes);
          parent.contact_shapes = horzcat(parent.contact_shapes,body.contact_shapes);
          if ~isempty(body.collision_group_name)
            ngroups=length(parent.collision_group_name);
            [parent.collision_group_name,ia,ic]=unique(horzcat(parent.collision_group_name,body.collision_group_name),'stable');
            % note: passing 'stable' to unique (above) ensures that
            % parent.collision_group is still valid, so just add to it here
            if length(parent.collision_group)<length(parent.collision_group_name)
              parent.collision_group{length(parent.collision_group_name)}=[];
            end
            if length(parent.contact_shape_group)<length(parent.collision_group_name)
              parent.contact_shape_group{length(parent.collision_group_name)}=[];
            end
            for j=1:length(body.collision_group)
              parent.collision_group{ic(ngroups+j)} = [parent.collision_group{ic(ngroups+j)},npts+body.collision_group{j}];
              parent.contact_shape_group{ic(ngroups+j)} = [parent.contact_shape_group{ic(ngroups+j)},nshapes+body.contact_shape_group{j}];
            end
          end
        end
        
        for j=1:length(model.loop)
          if (model.loop(j).body1 == body)
            model.loop(j).pt1 = body.Ttree(1:end-1,:)*[model.loop(j).pt1;1];
            model.loop(j).body1 = parent;
          end
          if (model.loop(j).body2 == body)
            model.loop(j).pt2 = body.Ttree(1:end-1,:)*[model.loop(j).pt2;1];
            model.loop(j).body2 = parent;
          end
        end
        
        for j=1:length(model.force)
          if isa(model.force{j},'RigidBodySpringDamper')
            if (model.force{j}.body1 == body)
              model.force{j}.pos1 = body.Ttree(1:end-1,:)*[model.force{j}.pos1;1];
              model.force{j}.body1 = parent;
            end
            if (model.force{j}.body2 == body)
              model.force{j}.pos2 = body.Ttree(1:end-1,:)*[model.force{j}.pos2;1];
              model.force{j}.body2 = parent;
            end
          end
        end
          
        % error on actuators
        if (~isempty(model.actuator) && any([model.actuator.joint] == body))
          model.actuator(find([model.actuator.joint]==body))=[];
          % actuators could be attached to fixed joints, because I
          % occasionally weld joints together (e.g. in planar processing of
          % a 3D robot)
        end
        
        % connect children to parents
%        children = find([model.body.parent] == body);
        for j=1:length(model.body),
          if model.body(j).parent == body
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
        model.body(i)=[];
        delete(body);
      end
    end
    
    function [phi,dphi,ddphi] = loopConstraints(obj,q)
      % handle kinematic loops
      phi=[];dphi=[];ddphi=[];

      kinsol = doKinematics(obj,q,true);
      
      for i=1:length(obj.loop)
        % for each loop, add the constraints that the pt1 on body1 is in
        % the same location as pt2 on body2
        
        [pt1,J1,dJ1] = obj.forwardKin(kinsol,obj.loop(i).body1,obj.loop(i).pt1);
        [pt2,J2,dJ2] = obj.forwardKin(kinsol,obj.loop(i).body2,obj.loop(i).pt2);
        
        phi = [phi; pt1-pt2];
        dphi = [dphi; J1-J2];
        ddphi = [ddphi; dJ1-dJ2];
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
      body.linkname=regexprep(body.linkname, '\.', '_', 'preservecase');
      
      if (options.inertial && node.getElementsByTagName('inertial').getLength()>0)
        body = parseInertial(body,node.getElementsByTagName('inertial').item(0),options);
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
          body = parseCollision(body,node.getElementsByTagName('collision').item(collisionItem),options);
          collisionItem = collisionItem+1;
        end
      end
      
      model.body=[model.body,body];
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
      parent = findLink(model,char(parentNode.getAttribute('link')),robotnum);
      
      childNode = node.getElementsByTagName('child').item(0);
      child = findLink(model,char(childNode.getAttribute('link')),robotnum);
      
      name = char(node.getAttribute('name'));
      type = char(node.getAttribute('type'));
      xyz=zeros(3,1); rpy=zeros(3,1);
      origin = node.getElementsByTagName('origin').item(0);  % seems to be ok, even if origin tag doesn't exist
      if ~isempty(origin)
        if origin.hasAttribute('xyz')
          xyz = reshape(str2num(char(origin.getAttribute('xyz'))),3,1);
        end
        if origin.hasAttribute('rpy')
          rpy = reshape(str2num(char(origin.getAttribute('rpy'))),3,1);
        end
      end
      axis=[1;0;0];  % default according to URDF documentation
      axisnode = node.getElementsByTagName('axis').item(0);
      if ~isempty(axisnode)
        if axisnode.hasAttribute('xyz')
          axis = reshape(str2num(char(axisnode.getAttribute('xyz'))),3,1);
          axis = axis/(norm(axis)+eps); % normalize
        end
      end
      damping=0;
      dynamics = node.getElementsByTagName('dynamics').item(0);
      if ~isempty(dynamics)
        if dynamics.hasAttribute('damping')
          damping = str2num(char(dynamics.getAttribute('damping')));
        end
      end
      
      joint_limit_min=-inf;
      joint_limit_max=inf;
      effort_limit=inf;
      velocity_limit=inf;
      limits = node.getElementsByTagName('limit').item(0);
      if ~isempty(limits)
        if limits.hasAttribute('lower')
          joint_limit_min = str2num(char(limits.getAttribute('lower')));
        end
        if limits.hasAttribute('upper');
          joint_limit_max = str2num(char(limits.getAttribute('upper')));
        end
        if limits.hasAttribute('effort');
          effort_limit = str2num(char(limits.getAttribute('effort')));
        end
        if limits.hasAttribute('velocity');
          velocity_limit = str2num(char(limits.getAttribute('velocity')));
        end
      end
      
      limits = struct();
      limits.joint_limit_min = joint_limit_min;
      limits.joint_limit_max = joint_limit_max;
      limits.effort_limit = effort_limit;
      limits.velocity_limit = velocity_limit;
      
      name=regexprep(name, '\.', '_', 'preservecase');
      model = model.addJoint(name,type,parent,child,xyz,rpy,axis,damping,limits);
    end
    
    
    function model = parseLoopJoint(model,robotnum,node,options)
      loop = RigidBodyLoop();
      loop.name = char(node.getAttribute('name'));
      loop.name = regexprep(loop.name, '\.', '_', 'preservecase');

      link1Node = node.getElementsByTagName('link1').item(0);
      link1 = findLink(model,char(link1Node.getAttribute('link')),robotnum);
      loop.body1 = link1;
      loop.pt1 = loop.parseLink(link1Node,options);
      
      link2Node = node.getElementsByTagName('link2').item(0);
      link2 = findLink(model,char(link2Node.getAttribute('link')),robotnum);
      loop.body2 = link2;
      loop.pt2 = loop.parseLink(link2Node,options);
      
      axis=[1;0;0];  % default according to URDF documentation
      axisnode = node.getElementsByTagName('axis').item(0);
      if ~isempty(axisnode)
        if axisnode.hasAttribute('xyz')
          axis = reshape(str2num(char(axisnode.getAttribute('xyz'))),3,1);
          axis = axis/(norm(axis)+eps); % normalize
        end
      end
      
      type = char(node.getAttribute('type'));
      switch (lower(type))
        case {'continuous'}
          warning('3D loop joints do not properly enforce the joint axis constraint.  (they perform more like a ball joint).  See bug 1389');
        otherwise
          error(['joint type ',type,' not supported (yet?)']);
      end
      
      model.loop=[model.loop,loop];
    end
    
    function model = parseForceElement(model,robotnum,node,options)
      name = char(node.getAttribute('name'));
      name = regexprep(name, '\.', '_', 'preservecase');
      
      type = char(node.getAttribute('type'));
      switch (lower(type))
        case 'linearspringdamper'
          fe = RigidBodySpringDamper();
          fe.name = name;
          linkNode = node.getElementsByTagName('link1').item(0);
          fe.body1 = findLink(model,char(linkNode.getAttribute('link')),robotnum);
          if linkNode.hasAttribute('xyz')
            fe.pos1 = reshape(str2num(char(linkNode.getAttribute('xyz'))),3,1);
          end
          linkNode = node.getElementsByTagName('link2').item(0);
          fe.body2 = findLink(model,char(linkNode.getAttribute('link')),robotnum);
          if linkNode.hasAttribute('xyz')
            fe.pos2 = reshape(str2num(char(linkNode.getAttribute('xyz'))),3,1);
          end
          
          elnode = node.getElementsByTagName('rest_length').item(0);
          if ~isempty(elnode) && elnode.hasAttribute('value')
            fe.rest_length = str2num(char(elnode.getAttribute('value')));
          end
          elnode = node.getElementsByTagName('stiffness').item(0);
          if ~isempty(elnode) && elnode.hasAttribute('value')
            fe.k = str2num(char(elnode.getAttribute('value')));
          end
          elnode = node.getElementsByTagName('damping').item(0);
          if ~isempty(elnode) && elnode.hasAttribute('value')
            fe.b = str2num(char(elnode.getAttribute('value')));
          end
          
          model.force{end+1} = fe;

        otherwise
          error(['force element type ',type,' not supported (yet?)']);
      end
    end
    
  end
  
end

