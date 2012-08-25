classdef RigidBodyModel
  
  properties
    name=[];        % name of the rigid body system
    body=[];        % cell array of RigidBody objects
    actuator = [];  % cell array of RigidBodyActuator objects
    loop=[];        % cell array RigidBodyLoop objects

    gravity=[0;0;-9.81];
    
    B = [];         
    featherstone = [];
    
    material=[];
  end

  methods 
    
    function model = RigidBodyModel(urdf_filename,options)
      % Parses URDF 
      %
      % @param urdf_filename filename of file to parse
      %
      % @options floating boolean where true means that a floating joint is
      % automatically added to the root. @default false
      % @options inertial boolean where true means parse dynamics parameters,
      % false means skip them.  @default true
      % @options visual boolean where true means parse graphics parameters, false
      % means skip them.  @default true
      %
      % @retval model Structure compatible with the Featherstone Spatial Vector
      % and Dynamics library, with additional robotlib tags added.
      
      if (nargin>0 && ~isempty(urdf_filename))
        if (nargin<2) options = struct(); end
        if (~isfield(options,'floating')) options.floating = false; end
        if (~isfield(options,'inertial')) options.inertial = true; end
        if (~isfield(options,'visual')) options.visual = true; end
        
        %disp(['Parsing ', urdf_filename]);
        urdf = xmlread(urdf_filename);
        
        robot = urdf.getElementsByTagName('robot').item(0);
        if isempty(robot)
          error('there are no robots in this urdf file');
        end
        
        model = parseRobot(model,robot,options);
        
        model=compile(model,options);
      end
    end
    
    function model=parseRobot(model,node,options)
      % Constructs a robot from a URDF XML node 
      
      %disp(['Parsing robot ', char(node.getAttribute('name')), ' from URDF file...']);
      model.name = char(node.getAttribute('name'));

      materials = node.getElementsByTagName('material');
      for i=0:(materials.getLength()-1)
        [~,model] = parseMaterial(model,materials.item(i),options);
      end
      
      links = node.getElementsByTagName('link');
      for i=0:(links.getLength()-1)
        model = parseLink(model,links.item(i),options);
      end
      
      joints = node.getElementsByTagName('joint');
      for i=0:(joints.getLength()-1)
        model = parseJoint(model,joints.item(i),options);
      end

      loopjoints = node.getElementsByTagName('loop_joint');
      for i=0:(loopjoints.getLength()-1)
        model = parseLoopJoint(model,loopjoints.item(i),options);
      end
      
      transmissions = node.getElementsByTagName('transmission');
      for i=0:(transmissions.getLength()-1)
        model = parseTransmission(model,transmissions.item(i),options);
      end

      gazebos = node.getElementsByTagName('gazebo');
      for i=0:(gazebos.getLength()-1)
        model = parseGazebo(model,gazebos.item(i),options);
      end
      
      if (options.floating)
        % then add a floating joint here
        model = addFloatingBase(model,options);
      end
      
    end
    
    function newmodel = copy(model)
      % Makes a deep copy of the model
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
    
    function model = compile(model,options)
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
      model = extractFeatherstone(model,options);
      
      %% extract B matrix
      B = sparse(model.featherstone.NB,0);
      for i=1:length(model.actuator)
        B(model.actuator(i).joint.dofnum,i) = model.actuator(i).reduction;
      end
      model.B = full(B);
      
      %% initialize kinematics caching
      for i=1:length(model.body)
        if ~isempty(model.body(i).parent)
          model.body(i).cached_q = nan;
        end
      end
    end
    
    function [c,model] = parseMaterial(model,node,options)
      
      name=char(node.getAttribute('name'));
      
      c = .7*[1 1 1];
      
      % look up material
      if length(model.material)>0
        ind = strmatch(lower(name),lower({model.material.name}),'exact');
        if (~isempty(ind))
          c=model.material(ind).c;
        end
      end
      
      colornode = node.getElementsByTagName('color').item(0);
      if ~isempty(colornode) && colornode.hasAttribute('rgba')
        c = str2num(char(colornode.getAttribute('rgba')));
        c = c(1:3);
      end

      if ~isempty(name)
        model.material = [model.material,struct('name',name,'c',c)];
      end
    end

    function model = parseGazebo(model,node,options)
      ref = char(node.getAttribute('reference'));
      if ~isempty(ref)
        body = findLink(model,ref,false);
        if ~isempty(body)
          grav = node.getElementsByTagName('turnGravityOff').item(0);
          if ~isempty(grav)
            val='';
            if grav.hasAttribute('value')
              val = grav.getAttribute('value');
            elseif grav.hasChildNodes
              val = grav.getChildNodes.item(0).getNodeValue();
            end
            if strcmpi(val,'true')
              body.gravity_off = true;
            end
          end
        end
%        joint = findJoint(model,ref,false)
%        if ~isempty(joint)
%          todo: parse initial conditions here?
%        end
      end
    end

    function doKinematics(model,q)
      if 0 %any(abs(q-reshape([model.body.cached_q],1,[])')<1e-6)  % todo: make this tolerance a parameter
        % then my kinematics are up to date, don't recompute
        return
      end
      for i=1:length(model.body)
        body = model.body(i);
        if (isempty(body.parent))
          body.T = body.Ttree;
          for j=1:model.featherstone.NB
            body.dTdq{j} = zeros(4);
          end
        else
          qi = q(body.dofnum);
          
          TJ = Tjcalc(body.pitch,qi);
          body.T=body.parent.T*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint;

          % todo: consider pulling this out into a
          % "doKinematicsAndVelocities" version?  but I'd have to be
          % careful with caching.
          for j=1:model.featherstone.NB
            body.dTdq{j} = body.parent.dTdq{j}*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint;
          end
          body.dTdq{body.dofnum} = body.dTdq{body.dofnum} + body.parent.T*body.Ttree*inv(body.T_body_to_joint)*dTjcalc(body.pitch,qi)*body.T_body_to_joint;
          
          body.cached_q = q(body.dofnum);
        end
      end
    end    
  end
  
  methods  % make these private?
    function model = extractFeatherstone(model,options)
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
        m.Xtree{i} = b.X_body_to_joint*b.Xtree*inv(b.parent.X_body_to_joint);
        m.I{i} = inv(b.X_body_to_joint)'*b.I*inv(b.X_body_to_joint);
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
          parent.I = parent.I + body.Xtree' * body.I * body.Xtree;
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
      
      A = zeros(length(model.body));
      for i=1:length(model.body)
        if ~isempty(model.body(i).parent)
          A(i,find(model.body(i).parent==[model.body.parent])) = 1;
        end
      end
      graph_draw(A,'node_labels',{model.body.linkname});
      % todo: add joint names, etc on edges (like the ros urdf display).  might have to use drawDot
      %   http://www.mathworks.com/matlabcentral/fileexchange/24652
      % and write my own .dot file instead of relying on graphviz2mat
    end
    
    function body = newBody(model)
      body = RigidBody();
    end
    
    function model=parseLink(model,node,options)
      body = newBody(model);
      
      body.linkname=char(node.getAttribute('name'));
      
      if (options.inertial && node.getElementsByTagName('inertial').getLength()>0)
        body = parseInertial(body,node.getElementsByTagName('inertial').item(0),options);
      end
      
      if (options.visual && node.getElementsByTagName('visual').getLength()>0)
        body = parseVisual(body,node.getElementsByTagName('visual').item(0),model,options);
      end
      
      if node.getElementsByTagName('collision').getLength()>0
        body = parseCollision(body,node.getElementsByTagName('collision').item(0),options);
      end
      
      model.body=[model.body,body];
    end
    
    function model=parseJoint(model,node,options)

      parentNode = node.getElementsByTagName('parent').item(0);
      if isempty(parentNode) % then it's not the main joint element.  for instance, the transmission element has a joint element, too
          return
      end
      parent = findLink(model,char(parentNode.getAttribute('link')));
      
      childNode = node.getElementsByTagName('child').item(0);
      child = findLink(model,char(childNode.getAttribute('link')));
      
      if (child.parent>=0)
        error('there is already a joint connecting this child to a parent');
      end
      
      child.jointname = char(node.getAttribute('name'));
      child.parent = parent;
      
      type = char(node.getAttribute('type'));
      xyz=zeros(3,1); rpy=zeros(3,1);
      wrl_joint_origin='';
      origin = node.getElementsByTagName('origin').item(0);  % seems to be ok, even if origin tag doesn't exist
      if ~isempty(origin)
        if origin.hasAttribute('xyz')
          xyz = reshape(str2num(char(origin.getAttribute('xyz'))),3,1);
          if any(xyz)
            wrl_joint_origin=[wrl_joint_origin,sprintf('\ttranslation %f %f %f\n',xyz(1),xyz(2),xyz(3))];
          end
        end
        if origin.hasAttribute('rpy')
          rpy = reshape(str2num(char(origin.getAttribute('rpy'))),3,1);
          if (any(rpy))
            wrl_joint_origin=[wrl_joint_origin,sprintf('\trotation %f %f %f %f\n',rpy2axis(rpy))];
          end
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
      
      child.Xtree = Xrotz(rpy(3))*Xroty(rpy(2))*Xrotx(rpy(1))*Xtrans(xyz);
      child.Ttree = [rotz(rpy(3))*roty(rpy(2))*rotx(rpy(1)),xyz; 0,0,0,1];
      
      if ~strcmp(lower(type),'fixed') && dot(axis,[0;0;1])<1-1e-6
        [child.X_body_to_joint,child.T_body_to_joint] = RigidBodyModel.bodyToJoint(axis);
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
      
      if ~isempty(wrl_joint_origin)
        child.wrljoint = wrl_joint_origin;
      end
      
    end
    
    function model = addFloatingBase(model,options)
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
        body1.jointname = name;
        body1.pitch=inf;
        body1.joint_axis = [1;0;0];
        [body1.X_body_to_joint,body1.T_body_to_joint] = RigidBodyModel.bodyToJoint(body1.joint_axis);
        body1.joint_limit_min = -inf;
        body1.joint_limit_max = inf;
        body1.parent=world; 

        body2=newBody(model);
        name = [child.linkname,'_y'];
        if strcmpi(name,horzcat({model.body.linkname},{model.body.jointname}))
          error('floating name already exists.  cannot add floating base.');
        end
        body2.linkname = name;
        body2.jointname = name;
        body2.pitch=inf;
        body2.joint_axis = [0;1;0];
        [body2.X_body_to_joint,body2.T_body_to_joint] = RigidBodyModel.bodyToJoint(body2.joint_axis);
        body2.joint_limit_min = -inf;
        body2.joint_limit_max = inf;
        body2.parent = body1;
        
        body3=newBody(model);
        name = [child.linkname,'_z'];
        if strcmpi(name,horzcat({model.body.linkname},{model.body.jointname}))
          error('floating name already exists.  cannot add floating base.');
        end
        body3.linkname = name;
        body3.jointname = name;
        body3.pitch=inf;
        body3.joint_axis = [0;0;1];
        [body3.X_body_to_joint,body3.T_body_to_joint] = RigidBodyModel.bodyToJoint(body3.joint_axis);
        body3.joint_limit_min = -inf;
        body3.joint_limit_max = inf;
        body3.parent = body2;
        
        body4=newBody(model);
        name = [child.linkname,'_roll'];
        if strcmpi(name,horzcat({model.body.linkname},{model.body.jointname}))
          error('floating name already exists.  cannot add floating base.');
        end
        body4.linkname = name;
        body4.jointname = name;
        body4.pitch=0;
        body4.joint_axis = [1;0;0];
        [body4.X_body_to_joint,body4.T_body_to_joint] = RigidBodyModel.bodyToJoint(body4.joint_axis);
        body4.joint_limit_min = -inf;
        body4.joint_limit_max = inf;
        body4.parent = body3;
        
        body5=newBody(model);
        name = [child.linkname,'_pitch'];
        if strcmpi(name,horzcat({model.body.linkname},{model.body.jointname}))
          error('floating name already exists.  cannot add floating base.');
        end
        body5.linkname = name;
        body5.jointname = name;
        body5.pitch=0;
        body5.joint_axis = [0;1;0];
        [body5.X_body_to_joint,body5.T_body_to_joint] = RigidBodyModel.bodyToJoint(body5.joint_axis);
        body5.joint_limit_min = -inf;
        body5.joint_limit_max = inf;
        body5.parent = body4;
        
        child.pitch=0;
        child.jointname = [child.linkname,'_yaw'];
        child.joint_axis=[0;0;1];
        [child.X_body_to_joint,child.T_body_to_joint] = RigidBodyModel.bodyToJoint(child.joint_axis);
        child.joint_limit_min = -inf;
        child.joint_limit_max = inf;
        child.parent = body5;
        
        model.body=[model.body,body1,body2,body3,body4,body5];
      end
    end
    
    function model = parseLoopJoint(model,node,options)
      error('not implemented yet for 3D');
    end

    function model=parseTransmission(model,node,options)
        
      if isempty(strfind(char(node.getAttribute('type')),'SimpleTransmission'))
          return; % only parse SimpleTransmissions so far');
      end
        
      actuator = RigidBodyActuator();

      childNodes = node.getChildNodes();
      for i=1:childNodes.getLength()
        thisNode = childNodes.item(i-1);
        switch (lower(char(thisNode.getNodeName())))
          case 'actuator'
            actuator.name = char(thisNode.getAttribute('name'));
          case 'joint'
            actuator.joint = findJoint(model,char(thisNode.getAttribute('name')));
          case 'mechanicalreduction'
            actuator.reduction = str2num(char(thisNode.getFirstChild().getNodeValue()));
          case {'#text','#comment'}
            % intentionally blank
          otherwise
            warning([char(thisNode.getNodeName()),' is not a supported element of robot/transmission.']);
        end
      end
      
      if (isempty(actuator.joint)) error('transmission elements must specify a joint name'); end

      model.actuator=[model.actuator,actuator];
    end
    
    function writeWRL(model,wrlfile,options)
      if nargin<2 || isempty(wrlfile)
        [filename,pathname]=uiputfile('*.wrl','Save WRL File',[model.name,'.wrl']);
        wrlfile=[pathname,filename];
      else
        typecheck(wrlfile,'char');
        [path,name,ext]=fileparts(wrlfile);
        if isempty(ext)
          ext='.wrl';
        elseif ~strcmpi(ext,'.wrl');
          error('second argument should point to a wrl file (with extension .wrl)');
        end
      end
      
      if (nargin<3) options=struct(); end
      if ~isfield(options,'ground') options.ground = false; end
      
      fp = fopen(wrlfile,'w');
      
      % write header
      fprintf(fp,'#VRML V2.0 utf8\n\n');
      fprintf(fp,'## ------------------------------------------------- ##\n');
      fprintf(fp,'## This file was automatically generated using Drake ##\n');
      fprintf(fp,'##   EDITING THIS FILE BY HAND IS NOT RECOMMENDED    ##\n');
      fprintf(fp,'## ------------------------------------------------- ##\n\n');
      
      % write default background color  % todo: get this from urdf?
      fprintf(fp,'Background {\n\tskyColor 1 1 1\n}\n\n');

      if (0) %options.ground)  % not working yet.
        m=10; n=10;
        fprintf(fp,'Shape { geometry ElevationGrid {\n');
%        fprintf(fp,'  solid "false"\n');
        fprintf(fp,'  xDimension %d\n',m);
        fprintf(fp,'  zDimension %d\n',n);
%        fprintf(fp,'  color 1 0 0\n');
        fprintf(fp,'  height [');
        fprintf(fp,' %d', zeros(m,n));
        fprintf(fp,' ]\n');
        fprintf(fp,'} appearance Appearance { material Material { diffuseColor 1 0 0 } }}\n\n');
      end
      
      % write default viewpoints
      fprintf(fp,'Viewpoint {\n\tdescription "right"\n\tposition 0 -4 0\n\torientation 1 0 0 1.5708\n}\n\n');
      fprintf(fp,'Transform {\n\trotation 0 1 0 1.5708\n\tchildren Viewpoint {\n\tdescription "front"\n\tposition 0 0 4\n\torientation 0 0 1 1.5708\n}\n}\n\n');
      fprintf(fp,'Viewpoint {\n\tdescription "top"\n\tposition 0 0 4\n}\n\n');
      
      % loop through bodies
      for i=1:length(model.body)
        if isempty(model.body(i).parent)
          writeWRLBodyAndChildren(model,model.body(i),fp);
        end
      end
    end
    
  end
  
  methods (Access=protected)    
    function writeWRLBodyAndChildren(model,body,fp,td)
      if (nargin<4) td=0; end % tab depth
      function tabprintf(varargin), for i=1:td, fprintf(fp,'\t'); end, fprintf(fp,varargin{:}); end
      
      if ~isempty(body.wrljoint)
        fprintf(fp,'Transform {\n%s\n\tchildren [\n',body.wrljoint);
      end
      
      % if there is a a joint between the parent and the body, add it here
      if ~isempty(body.parent)
        writeWRLJoint(body,fp);
        tabprintf('children [\n'); td=td+1;
      end
      td = writeWRLBody(body,fp,td);
      for i=1:length(model.body)
        if (model.body(i).parent == body)
          writeWRLBodyAndChildren(model,model.body(i),fp,td);
        end
      end
      if ~isempty(body.parent)
        td=td-1; tabprintf(']\n');
        td=td-1; tabprintf('}\n'); % end Transform {
      end
      
      if ~isempty(body.wrljoint)
        % close brackets that were added during removal of fixed joints
        brac=body.wrljoint(body.wrljoint=='{'|body.wrljoint=='[');
        brac=regexprep(brac(end:-1:1),'[',']');
        brac=regexprep(brac,'{','}');
        
        fprintf(fp,']\n%s\n}\n',brac); % end wrljoint transform
      end
    end
  end
  
  methods (Static)
    function [X,T] = bodyToJoint(axis)
      % featherstone dynamics treats all joints as operating around the
      % z-axis.  so I have to add a transform from the origin of this
      % link to align the joint axis with the z-axis, update the spatial
      % inertia of this joint, and then rotate back to keep the child
      % frames intact.  this happens in extractFeatherstone
      axis_angle = [cross([0;0;1],axis); atan2(norm(cross([0;0;1],axis)),dot([0;0;1],axis))]; % both are already normalized
      jointrpy = quat2rpy(axis2quat(axis_angle));
      X=Xrotz(jointrpy(3))*Xroty(jointrpy(2))*Xrotx(jointrpy(1));
      T=[rotz(jointrpy(3))*roty(jointrpy(2))*rotx(jointrpy(1)),zeros(3,1); 0,0,0,1];
    end
  end
end
