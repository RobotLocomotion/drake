classdef RigidBodyModel
  
  properties
    name=[];        % name of the rigid body system
    body=[];        % cell array of RigidBody objects
    actuator = [];  % cell array of RigidBodyActuator objects
    loop=[];        % cell array RigidBodyLoop objects

    gravity=[0;-9.81];
    
    B = [];         
    featherstone = [];
    
    material=[];
  end

  methods (Static=true)
    
    function model = parseURDF(urdf_filename,options)
      % Parse URDF 
      %
      % @param urdf_filename filename of file to parse
      %
      % @option twoD boolean where true means parse to 2D model, false means parse to 3D model. @default true
      % @options inertial boolean where true means parse dynamics parameters,
      % false means skip them.  @default true
      % @options visual boolean where true means parse graphics parameters, false
      % means skip them.  @default true
      %
      % @retval model Structure compatible with the Featherstone Spatial Vector
      % and Dynamics library, with additional robotlib tags added.
      
      if (nargin<2) options = struct(); end
      if (~isfield(options,'twoD')) options.twoD = false; end
      if (~isfield(options,'inertial')) options.inertial = true; end
      if (~isfield(options,'visual')) options.visual = true; end
      
      %disp(['Parsing ', urdf_filename]);
      urdf = xmlread(urdf_filename);
      
      if (~urdf.hasChildNodes())
        error('No nodes in URDF file');
      end
      
      bHaveRobot=false;
      childNodes = urdf.getChildNodes();
      for i=1:childNodes.getLength()
        node = childNodes.item(i-1);
        switch (lower(char(node.getNodeName())))
          case 'robot'
            if (bHaveRobot)
              warning('There are multiple robots in this urdf file.  Only loading the first one');
            else
              bHaveRobot=true;
              model = RigidBodyModel.parseRobot(node,options);
            end
          case {'#comment','#text'}
            % intentionally left blank
          otherwise
            warning(['node ',char(node.getNodeName()),' not supported']);
        end
      end
      
      if (~bHaveRobot)
        error('didn''t find a robot in the urdf file!');
      end
      
    end
    
    function model=parseRobot(node,options)
      % Constructs a robot from a URDF XML node 
      
      %disp(['Parsing robot ', char(node.getAttribute('name')), ' from URDF file...']);
      model = RigidBodyModel();
      model.name = char(node.getAttribute('name'));
      
      childNodes = node.getChildNodes();
      for i=1:childNodes.getLength()
        thisNode = childNodes.item(i-1);
        switch (lower(char(thisNode.getNodeName())))
          case 'link'
            model=parseLink(model,thisNode,options);
          case 'joint'
            model=parseJoint(model,thisNode,options);
          case 'loop_joint'
            model=parseLoopJoint(model,thisNode,options);
          case 'transmission'
            model=parseTransmission(model,thisNode,options);
          case 'material'
            [c,model]=parseMaterial(model,thisNode,options);
          case {'sensor','gazebo','#text','#comment'}
            % intentionally empty. ok to skip these quietly.
          otherwise
            warning([char(thisNode.getNodeName()),' is not a supported element of robot.']);
        end
      end
      
      model=compile(model);
    end
    
  end
  
  methods 
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
      
      %% extract B matrix
      B = sparse(model.featherstone.NB,0);
      for i=1:length(model.actuator)
        B(model.actuator(i).body.dofnum,i) = model.actuator(i).reduction;
      end
      model.B = full(B);
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
      
      childNodes = node.getChildNodes();
      for i=1:childNodes.getLength()
        thisNode = childNodes.item(i-1);
        switch (lower(char(thisNode.getNodeName())))
          case 'color'
            c = str2num(char(thisNode.getAttribute('rgba')));
            c = c(1:3);
          case {'texture','#text','#comment'}
            % intentionally blank
          otherwise
            warning([char(thisNode.getNodeName()),' is not a supported element of robot/link/visual/material.']);
        end
      end
      
      model.material = [model.material,struct('name',name,'c',c)];
    end
    
    function model = doKinematics(model,q,qd)
      for i=1:length(model.body)
        body = model.body(i);
        if (isempty(body.parent))
          body.T = eye(3);
%          body.v = zeros(3,1);
        else
          TJ = Tjcalcp(body.jcode,q(body.dofnum));
          [Xj,S] = jcalcp(body.jcode,q(body.dofnum));
          body.T=body.parent.T*body.Ttree*TJ;
%          body.v=body.parent.v + S*qd(body.dofnum) + [0; body.parent.v(1)*body.T(1:2,3)];
        end
      end
    end
    
    function model = doVelocities(model,q,qd)
      error('don''t use this method.  it''s not finished yet');
      for i=1:length(model.body)
        body = model.body(i);
        if (body.dofnum)
          [ XJ, S ] = jcalcp( body.jcode, q(body.dofnum) );
          X = XJ * body.Xtree;
          body.v = X*body.parent.v + S*qd(body.dofnum);
          body.X = X*body.parent.X;
        else
          body.v = zeros(3,1);
          body.X = body.Xtree;
        end
      end
    end
  end
  
  methods  % make these private?
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
        m.jcode(i) = b.jcode;
        m.Xtree{i} = b.Xtree;
        m.I{i} = b.I;
        m.damping(i) = b.damping;  % add damping so that it's faster to look up in the dynamics functions.
      end
      model.featherstone = m;
    end
      
    function model=removeFixedJoints(model)
      % takes any fixed joints out of the tree, adding their visuals and
      % inertia to their parents, and connecting their children directly to
      % their parents

      fixedind = find([model.body.jcode]==10);
      
      for i=fixedind(end:-1:1)  % go backwards, since it is presumably more efficient to start from the bottom of the tree
        body = model.body(i);
        parent = body.parent;
        
        parent.linkname=[parent.linkname,'+',body.linkname];
        
        % add inertia into parent
        if (any(any(body.I))) 
          % same as the composite inertia calculation in HandC.m
          parent.I = parent.I + body.Xtree' * body.I * body.Xtree;
        end
        
        % add geometry into parent
        if (~isempty(body.geometry))
          for j=1:length(body.geometry)
            for k=1:length(body.geometry{j}.x)
              pt0 = [body.geometry{j}.x(k); body.geometry{j}.z(k); 1];
              pt1 = body.Ttree * pt0;  %rotation might be backwards
              body.geometry{j}.x(k) = pt1(1);
              body.geometry{j}.z(k) = pt1(2);
            end
            parent.geometry = {parent.geometry{:},body.geometry{j}};
            
            parent.wrlgeometry = [ parent.wrlgeometry, sprintf('\n'), body.wrlgeometry ];
          end
        end
        
        if (~isempty(body.ground_contact))
          parent.ground_contact = [parent.ground_contact, body.Ttree(1:2,1:2)*body.ground_contact + body.Ttree(1:2,3)];
        end
        
        % todo: handle loops
        if (~isempty(model.loop) && (~isempty(model.loop.body1 == body || model.loop.body2 == body)))
          error('loop_joints connected to fixed links not implemented yet');
        end
          
        % error on actuators
        if (~isempty(model.actuator) && isempty([model.actuator.body] == body))
          error('actuators shouldn''t be attached to fixed joints');
        end
        
        % connect children to parents
        children = find([model.body.parent] == body);
        for j=1:length(model.body),
          if model.body(j).parent == body
            model.body(j).parent = body.parent;
          end
        end
        model.body(i)=[];
        delete(body);
      end
    end

    function body = findLink(model,linkname)
      ind = strmatch(lower(linkname),lower({model.body.linkname}),'exact');
      if (length(ind)~=1)
        error(['couldn''t find unique link ' ,linkname]);
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
    
    function model=parseLink(model,node,options)
      body = RigidBody();
      body.linkname=char(node.getAttribute('name'));
      
      if (options.twoD)
        body.I = zeros(3);
        body.Xtree = eye(3);
        body.Ttree = eye(3);
      else
        body.I = zeros(6); % equivalent to mcI(0,zeros(3,1),zeros(3));
        body.Xtree = eye(6);
        body.Ttree = eye(4);
      end

      childNodes = node.getChildNodes();
      for i=1:childNodes.getLength()
        thisNode = childNodes.item(i-1);
        switch (lower(char(thisNode.getNodeName())))
          case 'inertial'
            if (options.inertial)
              body=parseInertial(body,thisNode,options);
            end
          case 'visual'
            if (options.visual)
              body=parseVisual(body,thisNode,model,options);
            end
          case 'collision'
            body = parseCollision(body,thisNode,options);
          case {'#text','#comment'}
            % intentionally empty. ok to skip these quietly.
          otherwise
            warning([char(thisNode.getNodeName()),' is not a supported element of robot.']);
        end
      end
      
      model.body=[model.body,body];
    end
    
    function model=parseJoint(model,node,options)

      parentNode = node.getElementsByTagName('parent').item(0);
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
          wrl_joint_origin=[wrl_joint_origin,sprintf('\ttranslation %f %f %f\n',xyz(1),xyz(2),xyz(3))];
        end
        if origin.hasAttribute('rpy')
          rpy = reshape(str2num(char(origin.getAttribute('rpy'))),3,1);
          wrl_joint_origin=[wrl_joint_origin,sprintf('\trotation %f %f %f %f\n',rpy2axis(rpy))];
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
      
      if (options.twoD)
        switch (lower(type))
          case {'revolute','continuous'}
            child.jcode=1;

          case 'prismatic'
            if dot(axis,[1;0;0])>(1-1e-6)
              child.jcode=2;
            elseif dot(axis,[0;0;1])>(1-1e-6)
              child.jcode=3;
            else
              error('Currently only prismatic joints with their axis in the x-axis or z-axis are supported right now (twoD assumes x-z plane)');
            end              
            
          case 'planar'
            % create two links with sliders, then finish this function with
            % the first of these joints (which need to catch the kinematics)
            body1=RigidBody();
            body1.linkname=[child.jointname,'_x'];
            body1.jointname = body1.linkname;
            body1.jcode=2;
            body1.I = zeros(3);
            body1.Xtree = eye(3);
            body1.Ttree = eye(3);
            body1.damping=damping;
            body1.parent=parent;
            body2=RigidBody();
            body2.linkname=[child.jointname,'_z'];
            body2.jointname = body2.linkname;
            body2.jcode=3;
            body2.I = zeros(3);
            body2.Xtree = eye(3);
            body2.Ttree = eye(3);
            body2.damping=damping;
            body2.parent = body1;
            child.jcode=1;
            child.pitch=0;
            child.parent = body2;
            model.body=[model.body,body1,body2];
            
            child=body1;

          case 'fixed'
            child.jcode=10;
            
          otherwise
            error(['joint type ',type,' not supported in twoD mode']);
        end
        
        if abs(rpy(1))>1e-4 || abs(rpy(3)>1e-4) 
          error('joints out of the x-z plane are not supported yet');
          % note: i should eventually support PI*n, and prismatic joints on
          % the y axis if rpy rotates it into the z axis, etc.  but I should get pretty far with this.
        end
        
        child.Xtree = Xpln(rpy(2),xyz([1 3]));
        child.damping = damping;
        child.Ttree = [rotmat(rpy(2)),xyz([1 3]); 0,0,1];  
      else % 3D
        child.Xtree = Xrotz(rpy(3))*Xroty(rpy(2))*Xrotx(rpy(1))*Xtrans(xyz);
        child.Ttree = [rotz(rpy(3))*roty(rpy(2))*rotz(rpy(1)),xyz; 0,0,0,1];
        
        if dot(axis,[0;0;1])<1-1e-6
          % featherstone dynamics treats all joints as operating around the
          % z-axis.  so I have to add a transform from the origin of this
          % link to align the joint axis with the z-axis, update the spatial
          % inertia of this joint, and then rotate back to keep the child
          % frames intact.
          warning('non-z-axis joints not implemented yet');  
        end
        
        switch lower(type)
          case {'revolute','continuous'}
            child.pitch = 0;
            
          case 'prismatic'
            child.pitch = inf;
            
          case 'fixed'
            child.jcode = 10;
            
          otherwise
            error(['joint type ',type,' not supported (yet?)']);
        end
      end
      
      if ~isempty(wrl_joint_origin)
        child.wrljoint = wrl_joint_origin;
      end
      
    end
    
    function model = parseLoopJoint(model,node,options)
      
      loop = RigidBodyLoop();
      loop.name = char(node.getAttribute('name'));
      
      link1Node = node.getElementsByTagName('link1').item(0);
      link1 = findLink(model,char(link1Node.getAttribute('link')));
      loop.body1 = link1;
      loop.T1 = loop.parseLink(link1Node,options);
      
      link2Node = node.getElementsByTagName('link2').item(0);
      link2 = findLink(model,char(link2Node.getAttribute('link')));
      loop.body2 = link2;
      loop.T2 = loop.parseLink(link2Node,options);
      
      %% find the lowest common ancestor
      loop.least_common_ancestor = leastCommonAncestor(loop.body1,loop.body2);
      
      type = char(node.getAttribute('type'));
      switch (lower(type))
        case {'revolute','continuous'}
          loop.jcode=1;
        case 'prismatic'
          loop.jcode=2;
        otherwise
          error(['joint type ',type,' not supported (yet?)']);
      end
      
      childNodes = node.getChildNodes();
      for i=1:childNodes.getLength()
        thisNode = childNodes.item(i-1);
        switch (lower(char(thisNode.getNodeName())))
          case 'axis'
            ax = reshape(str2num(char(thisNode.getAttribute('xyz'))),3,1);
            switch (loop.jcode)
              case 1
                if (abs((ax'*[0; 1; 0]) / (ax'*ax) - 1)>1e-6) error('for 2D processing, revolute and continuous joints must be aligned with [0 1 0]'); end
              case 2
                if (abs((ax'*[1; 0; 0]) / (ax'*ax) - 1)>1e-6)
                  if (abs((ax'*[0; 0; 1]) / (ax'*ax) - 1)>1e-6)
                    error('Currently prismatic joints must have their axis in the x-axis or z-axis are supported right now');
                  else
                    loop.jcode = 3;
                  end
                end
              otherwise
                error('shouldn''t get here');
            end
        end
      end
      
      model.loop=[model.loop,loop];
    end

    function model=parseTransmission(model,node,options)
      actuator = RigidBodyActuator();

      childNodes = node.getChildNodes();
      for i=1:childNodes.getLength()
        thisNode = childNodes.item(i-1);
        switch (lower(char(thisNode.getNodeName())))
          case 'actuator'
            actuator.name = char(thisNode.getAttribute('name'));
          case 'joint'
            actuator.body = findJoint(model,char(thisNode.getAttribute('name')));
          case 'mechanicalreduction'
            actuator.reduction = str2num(char(thisNode.getFirstChild().getNodeValue()));
          case {'#text','#comment'}
            % intentionally blank
          otherwise
            warning([char(thisNode.getNodeName()),' is not a supported element of robot/transmission.']);
        end
      end
      
      if (isempty(actuator.body)) error('transmission elements must specify a joint name'); end

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
      
      fp = fopen(wrlfile,'w');
      
      % write header
      fprintf(fp,'#VRML V2.0 utf8\n\n');
      fprintf(fp,'## ------------------------------------------------- ##\n');
      fprintf(fp,'## This file was automatically generated using Drake ##\n');
      fprintf(fp,'##   EDITING THIS FILE BY HAND IS NOT RECOMMENDED    ##\n');
      fprintf(fp,'## ------------------------------------------------- ##\n\n');
      
      % write default background color  % todo: get this from urdf?
      fprintf(fp,'Background {\n\tskyColor 1 1 1\n}\n\n');
      
      % write default viewpoint % todo: get this from urdf?
      fprintf(fp,'Transform {\n\tscale 1 -1 1\n\tchildren DEF side Viewpoint {\n\t\tdescription "front"\n\t\tposition 0 -2 0\n\t\torientation 1 0 0 1.5708\n\t}\n}\n\n');
      
      % loop through bodies
      for i=1:length(model.body)
        if isempty(model.body(i).parent)
          writeWRLBodyAndChildren(model,model.body(i),fp);
        end
      end
    end
    
  end
  
  methods (Access=private)
    function writeWRLBodyAndChildren(model,body,fp,td)
      if (nargin<4) td=0; end % tab depth
      function tabprintf(varargin), for i=1:td, fprintf(fp,'\t'); end, fprintf(fp,varargin{:}); end
      
      if ~isempty(body.wrljoint)
        fprintf(fp,'Transform {\n%s\n\tchildren [\n',body.wrljoint);
      end
      
      % if there is a a joint between the parent and the body, add it here
      if ~isempty(body.parent)
        writeWRLJoint(body,fp);
      else
        tabprintf('Transform {\n'); td=td+1;
      end
      tabprintf('children [\n'); td=td+1;
      td = writeWRLBody(body,fp,td);
      for i=1:length(model.body)
        if (model.body(i).parent == body)
          writeWRLBodyAndChildren(model,model.body(i),fp,td);
        end
      end
      td=td-1; tabprintf(']\n');
      td=td-1; tabprintf('}\n'); % end Transform {
      
      if ~isempty(body.wrljoint)
        fprintf(fp,'\t]\n}\n'); % end wrljoint transform
      end
    end
  end
end
