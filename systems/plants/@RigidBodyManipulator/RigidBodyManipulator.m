classdef RigidBodyManipulator < Manipulator
  % This class wraps the spatial vector library (v1) 
  % provided by Roy Featherstone on his website: 
  %   http://users.cecs.anu.edu.au/~roy/spatial/documentation.html
    
  properties
    name=[];        % name of the rigid body system
    body=[];        % array of RigidBody objects
    actuator = [];  % array of RigidBodyActuator objects
    loop=[];        % array of RigidBodyLoop objects
    sensor={};      % cell array of RigidBodySensor objects
      % note: need {} for arrays that will have multiple types (e.g.
      % a variety of derived classes), but can get away with [] for arrays
      % with elements that are all exactly the same type

    gravity=[0;0;-9.81];
    
    B = [];
    featherstone = [];
    
    material=[];

    mex_model_ptr = 0;
  end
  
  methods
    function obj = RigidBodyManipulator(urdf_filename,options)
      obj = obj@Manipulator(0,0);

      if (nargin>0 && ~isempty(urdf_filename))
        if (nargin<2) options = struct(); end
        obj = parseURDF(obj,urdf_filename,options);
      end
    end
    
    function y = output(obj,t,x,u)
      if isempty(obj.sensor)
        y = x;
      else
        if ~isDirectFeedthrough(obj)
          u=[];
        end
        y = [];
        for i=1:length(obj.sensor)
          y = [y; obj.sensor{i}.output(t,x,u)];
        end
      end
    end
    
    function obj = createMexPointer(obj)
      if (obj.mex_model_ptr) deleteMexPointer(obj); end
      obj.mex_model_ptr = HandCmex(obj);
    end
    
    function obj = deleteMexPointer(obj)
      HandCmex(obj,obj.mex_model_ptr);
      obj.mex_model_ptr = 0;
    end
    
    function [x,J,dJ] = kinTest(m,q)
      % test for kinematic gradients
      kinsol=doKinematics(m,q,nargout>2,false);
      
      count=0;
      for i=1:length(m.body)
        body = m.body(i);
        for j=1:length(body.geometry)
          s = size(body.geometry{j}.x); n=prod(s);
          pts = [reshape(body.geometry{j}.x,1,n); reshape(body.geometry{j}.y,1,n)];
          if (nargout>2)
            [x(:,count+(1:n)),J(2*count+(1:2*n),:),dJ(2*count+(1:2*n),:)] = forwardKin(m,kinsol,i,pts);
          elseif (nargout>1)
            [x(:,count+(1:n)),J(2*count+(1:2*n),:)] = forwardKin(m,kinsol,i,pts);
          else
            if ~exist('x') % extra step to help taylorvar
              x = forwardKin(m,kinsol,i,pts);
            else
              xn = forwardKin(m,kinsol,i,pts);
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
        child = model.body(rootlink(i));

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

      u_limit = repmat(inf,length(model.actuator),1);

      %% extract B matrix
      B = sparse(model.featherstone.NB,0);
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
      model = model.setNumDOF(model.featherstone.NB);
      model = model.setNumOutputs(2*model.featherstone.NB);

      if getNumInputs(model)>0
        model = setInputFrame(model,constructInputFrame(model));
      end

      if getNumStates(model)>0
        stateframe = constructStateFrame(model);
        model = setStateFrame(model,stateframe);
      end
      
      if length(model.sensor)>0
        for i=1:length(model.sensor)
          model.sensor{i} = model.sensor{i}.compile();
          outframe{i} = model.sensor{i}.getFrame();
        end
        if (length(outframe)>1)
          fr = MultiCoordinateFrame(outframe);
        else
          fr = outframe{1};
        end
        model = setNumOutputs(model,fr.dim);
        model = setOutputFrame(model,fr);
      else
        model = setOutputFrame(model,stateframe);  % output = state
      end
      
      if (length(model.loop)>0)
        model = model.setNumPositionConstraints(3*length(model.loop));  % should be 5? for continous joints once they enforce the joint axis constraint.
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

    function fr = constructStateFrame(model)
      joints = {model.body(~cellfun(@isempty,{model.body.parent})).jointname}';
      coordinates = vertcat(joints,cellfun(@(a) [a,'dot'],joints,'UniformOutput',false));
      fr = SingletonCoordinateFrame([model.name,'State'],2*model.featherstone.NB,'x',coordinates);
    end
    
    function fr = constructInputFrame(model)
      if size(model.B,2)>0
        coordinates = {model.actuator.name}';
      else
        coordinates={};
      end
       
      fr = SingletonCoordinateFrame([model.name,'Input'],size(model.B,2),'u',coordinates);
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
      tf = FunctionHandleCoordinateTransform(0,0,model.getStateFrame(),fr,true,true,[],[], ...
        @(obj,~,~,x) getCOM(model,x(1:model.featherstone.NB))); 
      
      model.getStateFrame().addTransform(tf);
    end
    
    function v = constructVisualizer(obj,options)
      v = RigidBodyWRLVisualizer(obj);
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
      % so far, only loop constraints are implemented
      [phi,dphi,ddphi]=loopConstraints(obj,q);
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
    
  end
  
  methods (Static)
    function d=surfaceTangents(normal)
      %% compute tangent vectors, according to the description in the last paragraph of Stewart96, p.2678
      t1=normal; % initialize size
      % handle the normal = [0;0;1] case
      ind=(1-normal(3,:))<eps;  % since it's a unit normal, i can just check the z component
      t1(:,ind) = repmat([1;0;0],1,sum(ind));
      ind=~ind;
      % now the general case
      t1(:,ind) = cross(normal(:,ind),repmat([0;0;1],1,sum(ind)));
      t1 = t1./repmat(sqrt(sum(t1.^2,1)),3,1); % normalize
      
      t2 = cross(t1,normal);
      
      m = 4;  % must be an even number
      theta = (0:(m-1))*2*pi/m;
      
      for k=1:m
        d{k}=cos(theta(k))*t1 + sin(theta(k))*t2;
      end      
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
          parent.contact_pts = [parent.contact_pts, body.Ttree(1:end-1,:)*[body.contact_pts;ones(1,size(body.contact_pts,2))]];
        end
        
        % todo: handle loops
        if (~isempty(model.loop) && (any([model.loop.body1] == body) || any([model.loop.body2] == body)))
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
      loop = RigidBodyLoop();
      loop.name = char(node.getAttribute('name'));
      loop.name = regexprep(loop.name, '\.', '_', 'preservecase');

      link1Node = node.getElementsByTagName('link1').item(0);
      link1 = findLink(model,char(link1Node.getAttribute('link')));
      loop.body1 = link1;
      loop.pt1 = loop.parseLink(link1Node,options);
      
      link2Node = node.getElementsByTagName('link2').item(0);
      link2 = findLink(model,char(link2Node.getAttribute('link')));
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
    
  end
  
end

