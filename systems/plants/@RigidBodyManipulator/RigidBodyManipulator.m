classdef RigidBodyManipulator < Manipulator
  % This class wraps the spatial vector library (v1) 
  % provided by Roy Featherstone on his website: 
  %   http://users.cecs.anu.edu.au/~roy/spatial/documentation.html
    
  properties
    name=[];        % name of the rigid body system
    body=[];        % cell array of RigidBody objects
    actuator = [];  % cell array of RigidBodyActuator objects
    loop=[];        % cell array RigidBodyLoop objects

    gravity=[0;0;-9.81];
    
    B = [];
    featherstone = [];
    
    material=[];

    mex_model_ptr = 0;
%    cached_kinematics = struct('b_second_derivatives',false,'use_mex',true,'q',[],'qd',[]);
  end
  
  methods
    function obj = RigidBodyManipulator(urdf_filename,options)
      obj = obj@Manipulator(0,0);

      if (nargin>0 && ~isempty(urdf_filename))
        if (nargin<2) options = struct(); end
        obj = parseURDF(obj,urdf_filename,options);
      end
    end
    
    function obj = createMexPointer(obj)
      if (obj.mex_model_ptr) deleteMexPointer(obj); end
      obj.mex_model_ptr = HandCmex(struct(obj),obj.gravity);
    end
    
    function obj = deleteMexPointer(obj)
      HandCpmex(obj.mex_model_ptr);
      obj.mex_model_ptr = 0;
    end
    
    function [x,J,dJ] = kinTest(m,q)
      % test for kinematic gradients
      doKinematics(m,q,nargout>2,false);
      
      count=0;
      for i=1:length(m.body)
        body = m.body(i);
        for j=1:length(body.geometry)
          s = size(body.geometry{j}.x); n=prod(s);
          pts = [reshape(body.geometry{j}.x,1,n); reshape(body.geometry{j}.y,1,n)];
          if (nargout>1)
            [x(:,count+(1:n)),J(2*count+(1:2*n),:),dJ(2*count+(1:2*n),:)] = forwardKin(m,i,pts);
          else
            if ~exist('x') % extra step to help taylorvar
              x = forwardKin(m,i,pts);
            else
              xn = forwardKin(m,i,pts);
              x=[x,xn];
            end
          end
          count = count + n;
        end
      end
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
      
      child.jointname = regexprep(name, '\.', '_', 'preservecase');
      child.parent = parent;
      
      axis = quat2rotmat(rpy2quat(rpy))*axis;  % axis is specified in joint frame

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
      child.Ttree = [rotz(rpy(3))*roty(rpy(2))*rotx(rpy(1)),xyz; 0,0,0,1];

      % note that I only now finally understand that my Ttree*[x;1] is
      % *ALMOST* (up to translation?? need to resolve this!) the same as inv(Xtree)*[x;zeros(3,1)].  sigh.
%      valuecheck([eye(3),zeros(3,1)]*child.Ttree*ones(4,1),[eye(3),zeros(3)]*inv(child.Xtree)*[ones(3,1);zeros(3,1)]);

      if ~strcmp(lower(type),'fixed') && dot(axis,[0;0;1])<1-1e-4
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

	% these fail for atlas (so are commented out):
%        valuecheck(inv(child.X_joint_to_body)*[axis;zeros(3,1)],[0;0;1;zeros(3,1)],1e-6);
%        valuecheck(child.T_body_to_joint*[axis;1],[0;0;1;1],1e-6);
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
          
        otherwise
          error(['joint type ',type,' not supported (yet?)']);
      end
      child.joint_axis = axis;
      child.joint_limit_min = limits.joint_limit_min;
      child.joint_limit_max = limits.joint_limit_max;
      child.effort_limit = limits.effort_limit;
      child.velocity_limit = limits.velocity_limit;
    end
    
    function model = addFloatingBase(model)
      % for now, just adds x,y,z,roll,pitch,yaw (in euler angles).
      % todo: consider using quaternions)
      
      rootlink = find(cellfun(@isempty,{model.body.parent}));
      if (length(rootlink)>1)
        warning('multiple root links');
      end
      
      if strcmpi('world',{model.body.linkname})
        error('world link already exists.  cannot add floating base.');
      end
      world = newBody(model);
      world.linkname = 'world';
      world.parent = [];
      model.body = [model.body,world];
      
      for i=1:length(rootlink)
        child = model.body(i);

        body1=newBody(model);
        name = [child.linkname,'_x'];
        if strcmpi(name,horzcat({model.body.linkname},{model.body.jointname}))
          error('floating name already exists.  cannot add floating base.');
        end
        body1.linkname = name;
        model.body = [model.body,body1];
        model = addJoint(model,name,'prismatic',world,body1,zeros(3,1),zeros(3,1),[1;0;0],0);

        body2=newBody(model);
        name = [child.linkname,'_y'];
        if strcmpi(name,horzcat({model.body.linkname},{model.body.jointname}))
          error('floating name already exists.  cannot add floating base.');
        end
        body2.linkname = name;
        model.body = [model.body,body2];
        model = addJoint(model,name,'prismatic',body1,body2,zeros(3,1),zeros(3,1),[0;1;0],0);
        
        body3=newBody(model);
        name = [child.linkname,'_z'];
        if strcmpi(name,horzcat({model.body.linkname},{model.body.jointname}))
          error('floating name already exists.  cannot add floating base.');
        end
        body3.linkname = name;
        body3.jointname = name;
        model.body = [model.body,body3];
        model = addJoint(model,name,'prismatic',body2,body3,zeros(3,1),zeros(3,1),[0;0;1],0);
        
        body4=newBody(model);
        name = [child.linkname,'_roll'];
        if strcmpi(name,horzcat({model.body.linkname},{model.body.jointname}))
          error('floating name already exists.  cannot add floating base.');
        end
        body4.linkname = name;
        model.body = [model.body,body4];
        model = addJoint(model,name,'revolute',body3,body4,zeros(3,1),zeros(3,1),[1;0;0],0);
        
        body5=newBody(model);
        name = [child.linkname,'_pitch'];
        if strcmpi(name,horzcat({model.body.linkname},{model.body.jointname}))
          error('floating name already exists.  cannot add floating base.');
        end
        body5.linkname = name;
        model.body = [model.body,body5];
        model = addJoint(model,name,'revolute',body4,body5,zeros(3,1),zeros(3,1),[0;1;0],0);
        model = addJoint(model,[child.linkname,'_yaw'],'revolute',body5,child,zeros(3,1),zeros(3,1),[0;0;1],0);

      end
    end
        
    
    function model = compile(model)
      % After parsing, compute some relevant data structures that will be
      % accessed in the dynamics and visualization

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
      model = extractFeatherstone(model);
      %sanity check
      if (model.featherstone.NB + 1 ~= length(model.body))
        error('Expected there to be only one body without a parent (i.e. world)')
      end
      
      %% extract B matrix
      B = sparse(model.featherstone.NB,0);
      for i=1:length(model.actuator)
        B(model.actuator(i).joint.dofnum,i) = model.actuator(i).reduction;
        if ~isinf(model.actuator(i).joint.effort_limit)
          model.u_limit(i) = abs(model.actuator(i).joint.effort_limit/model.actuator(i).reduction);
          if sum(B(model.actuator(i).joint.dofnum,:)~=0)>1
            warning('Drake:RigidBodyModel:UnsupportedJointEffortLimit','The specified joint effort limit cannot be expressed as simple input limits; the offending limits will be ignored');
            model.u_limit(B(model.actuator(i).joint.dofnum,:)~=0)=inf;
          end
        end
      end
      model.B = full(B);

      model = model.setNumInputs(size(model.B,2));
      model = model.setNumDOF(model.featherstone.NB);
      model = model.setNumOutputs(2*model.featherstone.NB);

      if getNumInputs(model)>0
        model = setInputFrame(model,constructInputFrame(model));
      end

      if getNumStates(model)>0
        stateframe = constructStateFrame(model);
        model = setStateFrame(model,stateframe);
        model = setOutputFrame(model,stateframe);  % output = state
      end
      
      if (length(model.loop)>0)
        error('haven''t reimplemented position and velocity constraints yet'); 
      end
%      obj = obj.setNumPositionConstraints(2*length(obj.model.loop)+size([obj.model.body.ground_contact],2));
%      obj = obj.setNumVelocityConstraints(0);%size([obj.model.body.ground_contact],2));

      model.joint_limit_min = [model.body.joint_limit_min]';
      model.joint_limit_max = [model.body.joint_limit_max]';
     
      if (any(model.joint_limit_min~=-inf) || any(model.joint_limit_max~=inf))
        warning('Drake:RigidBodyManipulator:UnsupportedJointLimits','Joint limits are not supported by this class.  Consider using HybridPlanarRigidBodyManipulator');
      end
      model.num_contacts = size([model.body.contact_pts],2);
      if (model.num_contacts>0)
        warning('Drake:RigidBodyManipulator:UnsupportedContactPoints','Contact is not supported by this class.  Consider using HybridPlanarRigidBodyManipulator');
      end
      
      u_limit = repmat(inf,length(model.actuator),1);
      model = model.setInputLimits(-u_limit,u_limit);

      
      %% initialize kinematics caching
      for i=1:length(model.body)
        if ~isempty(model.body(i).parent)
          model.body(i).cached_q = nan;
          model.body(i).cached_qd = nan;
        end
      end
      
      if (checkDependency('eigen3_enabled'))
        model = createMexPointer(model);
      end
    end
    
    function body = findLink(model,linkname,throw_error)
      ind = strmatch(lower(linkname),lower({model.body.linkname}),'exact');
      if (length(ind)~=1)
        if (nargin<3 || throw_error)
          error(['couldn''t find unique link ' ,linkname]);
        else 
          body=[];
        end
      else
        body = model.body(ind);
      end
    end
    
    function body = findJoint(model,jointname)
      ind = strmatch(lower(jointname),lower({model.body.jointname}),'exact');
      if (length(ind)~=1)
        error(['couldn''t find unique joint ' ,jointname]);
      else
        body = model.body(ind);
      end
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
      m = 0;
      for i=1:length(model.body)
        bm = model.body(i).getInertial();
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

        
    function fr = constructStateFrame(model)
      joints = {model.body(~cellfun(@isempty,{model.body.parent})).jointname}';
      coordinates = vertcat(joints,cellfun(@(a) [a,'dot'],joints,'UniformOutput',false));
      fr = CoordinateFrame([model.name,'State'],2*model.featherstone.NB,'x',coordinates);
    end
    
    function fr = constructInputFrame(model)
      if size(model.B,2)>0
        coordinates = {model.actuator.name}';
      else
        coordinates={};
      end
       
      fr = CoordinateFrame([model.name,'Input'],size(model.B,2),'u',coordinates);
    end
    
    function v = constructVisualizer(obj)
      v = RigidBodyWRLVisualizer(obj);
    end
   
  end
  
  methods (Access=protected)
    

    function model = extractFeatherstone(model)
%      m=struct('NB',{},'parent',{},'jcode',{},'Xtree',{},'I',{});
      dof=0;inds=[];
      for i=1:length(model.body)
        if (~isempty(model.body(i).parent))
          dof=dof+1;
          model.body(i).dofnum=dof;
          inds = [inds,i];
        end
      end
      m.NB=length(inds);
      for i=1:m.NB
        b=model.body(inds(i));
        m.parent(i) = b.parent.dofnum;
        m.pitch(i) = b.pitch;
        m.Xtree{i} = inv(b.X_joint_to_body)*b.Xtree*b.parent.X_joint_to_body;
        m.I{i} = b.X_joint_to_body'*b.I*b.X_joint_to_body;
%        disp(m.I{i})
%        if isequal(m.I{i},zeros(6))
%          error(['Body ',model.body.linkname,' has zero inertia.  that''s bad']);
%        end
        m.damping(i) = b.damping;  % add damping so that it's faster to look up in the dynamics functions.
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
          % if it happens to be a root joint, then don't remove this one.
          continue;
        end
          
        if ~isnan(body.pitch)
          if any([model.body.parent] == body) || any(any(body.I))
            % link has inertial importance from child links
            % pr link has inertia now (from a fixed link coming from a
            % descendant).  abort removal.
            continue;
          else
            warning('Drake:RigidBodyModel:BodyHasZeroInertia',['Link ',body.linkname,' has zero inertia (even though gravity is on and it''s not a fixed joint) and will be removed']);
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
          parent.contact_pts = [parent.contact_pts, body.Ttree(1:end-1,:)*[body.contact_pts;ones(1,size(body.contact_pts,2))]];
        end
        
        % todo: handle loops
        if (~isempty(model.loop) && any(model.loop.body1 == body || model.loop.body2 == body))
          error('loop_joints connected to fixed links not implemented yet');
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
    
    function model=parseJoint(model,node,options)
      
      ignore = char(node.getAttribute('drakeIgnore'));
      if strcmp(lower(ignore),'true')
        return;
      end
      
      parentNode = node.getElementsByTagName('parent').item(0);
      if isempty(parentNode) % then it's not the main joint element.  for instance, the transmission element has a joint element, too
        return
      end
      parent = findLink(model,char(parentNode.getAttribute('link')));
      
      childNode = node.getElementsByTagName('child').item(0);
      child = findLink(model,char(childNode.getAttribute('link')));
      
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
    
    
    function model = parseLoopJoint(model,node,options)
      error('not implemented yet for 3D');
    end
    
    
  end
  
end

