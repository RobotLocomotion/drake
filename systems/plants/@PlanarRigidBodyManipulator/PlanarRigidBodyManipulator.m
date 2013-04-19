classdef PlanarRigidBodyManipulator < RigidBodyManipulator
  % This class wraps the planar pieces of the spatial vector library (v1)
  % provided by Roy Featherstone on his website:
  %   http://users.cecs.anu.edu.au/~roy/spatial/documentation.html
  
  properties (SetAccess=protected)
    x_axis_label;
    y_axis_label;
    x_axis;
    y_axis;
    view_axis;
  end
  
  methods
    function obj = PlanarRigidBodyManipulator(urdf_filename,options)
      % Constructs a PlanarRigidBodyManipulator
      % 
      % @param urdf_filename string path+filename for a .urdf file to parse
      % @option view is a string which must be one of 'right','front', or
      % 'top'.  
      %
      % see also the options described in RigidBodyManipulator/parseURDF 
      %  (the options for this function call are passed through to
      %  parseURDF)
      
      
      obj = obj@RigidBodyManipulator();

      if (nargin<1) urdf_filename=''; end
      if (nargin<2) options = struct(); end
      if (~isfield(options,'view')) 
        options.view = 'right';
      else
        options.view = lower(options.view);
        if ~any(strcmp(options.view,{'front','right','top'}))
          error('supported view options are front,back,top,bottom,right,or left');
        end
      end
            
      switch options.view % joint_axis = view_axis => counter-clockwise
        case 'front'
          obj.x_axis = [0;1;0];
          obj.y_axis = [0;0;1];
          obj.view_axis = [1;0;0];
          obj.x_axis_label='y';
          obj.y_axis_label='z';
          %          obj.name = [obj.name,'Front'];
          obj.gravity = [0;-9.81];
        case 'right'
          obj.x_axis = [1;0;0];
          obj.y_axis = [0;0;1];
          obj.view_axis = [0;1;0];
          obj.x_axis_label='x';
          obj.y_axis_label='z';
          obj.gravity = [0;-9.81];
          %          obj.name = [obj.name,'Right'];
        case 'top'
          obj.x_axis = [1;0;0];
          obj.y_axis = [0;1;0];
          obj.view_axis = [0;0;1];
          obj.x_axis_label='x';
          obj.y_axis_label='y';
          obj.gravity = [0;0];
          %          obj.name = [obj.name,'Top'];
      end
      
      if ~isempty(urdf_filename)
        if ~isfield(options,'namesuffix') options.namesuffix=''; end
        options.namesuffix = [upper([obj.x_axis_label,obj.y_axis_label]),options.namesuffix];
        obj = addRobotFromURDF(obj,urdf_filename,zeros(2,1),0,options);
      end
    end
        
    function f = cartesianForceToSpatialForce(obj,kinsol,body,point,force)  
      % @param body is a rigid body element
      % @param point is a point on the rigid body (in body coords)
      % @param force is a cartesion force (in world coords)
      
      % convert force to body coordinates
      ftmp=bodyKin(obj,kinsol,body,[force,zeros(2,1)]);
      force = ftmp(:,1)-ftmp(:,2);  
      
      % compute spatial force (from Fpt in featherstone v2)
      f = [ point(1,:).*force(2,:) - point(2,:).*force(1,:); force ];
    end
    
    function model=addJoint(model,name,type,parent,child,xyz,rpy,axis,damping,limits,options)
      if (nargin<6) xyz=zeros(3,1); end
      if (nargin<7) rpy=zeros(3,1); end
      if (nargin<9) damping=0; end
      if (nargin<10 || isempty(limits))
        limits = struct();
        limits.joint_limit_min = -Inf;
        limits.joint_limit_max = Inf;
        limits.effort_limit = Inf;
        limits.velocity_limit = Inf;
      end
      
      % support xyz in view coordinates
      if length(xyz)==2
        xyz = xyz(1)*model.x_axis + xyz(2)*model.y_axis;
      end
      if length(rpy)==1
        rpy = rpy(1)*model.view_axis; % seems to simple, but i think it's right
      end
      
      switch (lower(type))
        case {'revolute','continuous','planar'}
          if abs(dot(axis,model.view_axis))<(1-1e-6)
            warning('Drake:PlanarRigidBodyManipulator:RemovedJoint',['Welded revolute joint ', child.jointname,' because it did not align with the viewing axis']);
            model = addJoint(model,name,'fixed',parent,child,xyz,rpy,axis,damping,limits,options);
            return;
          end
        case 'prismatic'
          if abs(dot(axis,model.view_axis))>1e-6
            warning('Drake:PlanarRigidBodyManipulator:RemovedJoint',['Welded prismatic joint ', child.jointname,' because it did not align with the viewing axis']);
            model = addJoint(model,name,'fixed',parent,child,xyz,rpy,axis,damping,limits,options);
            return;
          end
      end
      
      if ~isempty(child.parent)
        error('there is already a joint connecting this child to a parent');
      end
      
      switch (lower(type))
        case {'revolute','continuous'}
          child.pitch=0;
          child.joint_axis=axis;
          child.jsign = sign(dot(axis,model.view_axis));
          if dot(model.view_axis,[0;1;0])>0  % flip rotational kinematics view='right' to be consistent with vehicle coordinates
            child.jsign = -child.jsign;
          end
          child.jcode=1;
          
        case 'prismatic'
          child.pitch=inf;
          child.joint_axis=axis;
          if abs(dot(axis,model.x_axis))>(1-1e-6)
            child.jcode=2;
            child.jsign = sign(dot(axis,model.x_axis));
          elseif dot(axis,model.y_axis)>(1-1e-6)
            child.jcode=3;
            child.jsign = sign(dot(axis,model.y_axis));
          else
            error('Currently only prismatic joints with their axis in the x-axis or z-axis are supported right now (twoD assumes x-z plane)');
          end
          
        case 'planar'
          % create two links with sliders, then finish this function with
          % the first of these joints (which need to catch the kinematics)
          if (limits.joint_limit_min~=-inf || limits.joint_limit_max~=inf)
            error('joint limits not defined for planar joints');
          end
          jsign = sign(dot(axis,model.view_axis));
          
          body1=newBody(model);
          body1.linkname=[name,'_',model.x_axis_label];
          body1.robotnum = child.robotnum;
          model.body = [model.body,body1];
          model = addJoint(model,body1.linkname,'prismatic',parent,body1,xyz,rpy,jsign*model.x_axis,damping);
          
          body2=newBody(model);
          body2.linkname=[name,'_',model.y_axis_label];
          body2.robotnum = child.robotnum;
          model.body = [model.body,body2];
          model = addJoint(model,body2.linkname,'prismatic',body1,body2,zeros(3,1),zeros(3,1),jsign*model.y_axis,damping);
          
          model = addJoint(model,[name,'_p'],'revolute',body2,child,zeros(3,1),zeros(3,1),axis,damping);
          return;
          
        case 'fixed'
          child.pitch=nan;
          
        otherwise
          error(['joint type ',type,' not supported in planar models']);
      end
      
      child.jointname = name;
      child.parent = parent;
      
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
      
      xy = [model.x_axis'; model.y_axis']*xyz;
      if any(rpy)
        rpya=rpy2axis(rpy); p=rpya(4); rpyaxis=rpya(1:3);
        if abs(dot(rpyaxis,model.view_axis))<(1-1e-6)
          warning(['joint ',child.jointname,': joint axes out of the plane are not supported.  the dependent link ',child.linkname,' (and all of it''s decendants) will be zapped']);
          ind = find([model.body]==child);
          model.body(ind)=[];
          return;
          % note that if they were, it would change the way that I have to
          % parse geometries, inertias, etc, for all of the children.
        elseif dot(rpyaxis,model.view_axis)<0
          p=-p;
        end
        if dot(model.view_axis,[0;1;0])>0  % flip axis for vehicle coordinates
          p=-p;
        end
      else
        p=0;
      end
      
      child.Xtree = Xpln(p,xy);
      child.Ttree = [rotmat(p),xy; 0,0,1];
      child.T = parent.T*child.Ttree;
      child.damping = damping;
      child.joint_limit_min = limits.joint_limit_min;
      child.joint_limit_max = limits.joint_limit_max;
      child.effort_limit = limits.effort_limit;
      child.velocity_limit = limits.velocity_limit;
      model.dirty = true;
    end
    
    function model = addFloatingBase(model,parent,rootlink,xyz,rpy)
      model = addJoint(model,'base','planar',parent,rootlink,xyz,rpy,model.view_axis);
    end
    
    function model = compile(model)
      S = warning('off','Drake:RigidBodyManipulator:UnsupportedLoopJoint');
      model = compile@RigidBodyManipulator(model);
      warning(S);
      model = model.setNumPositionConstraints(2*length(model.loop));
    end
    
    function body = newBody(model)
      body = PlanarRigidBody();
    end
    
    function v=constructVisualizer(obj)
      checkDirty(obj);
      v = PlanarRigidBodyVisualizer(obj);
    end
    
  end
  
  methods (Static)
    function t=surfaceTangents(normal)
      %% compute a tangent vector, t
      % for each n, it looks like:
      % if (abs(normal(2))>abs(normal(1))) t = [1,-n(1)/n(2)];
      % else t = [-n(2)/n(1),1]; end
      % and the vectorized form is:
      t=normal; % initialize size
      ind=abs(normal(2,:))>abs(normal(1,:));
      t(:,ind) = [ones(1,sum(ind));-normal(1,ind)./normal(2,ind)];
      ind=~ind;
      t(:,ind) = [-normal(2,ind)./normal(1,ind); ones(1,sum(ind))];
      t = {t./repmat(sqrt(sum(t.^2,1)),2,1)}; % normalize
      
      % NOTEST
    end
    
  end
  
  methods (Access=protected)
    
    function obj = createMexPointer(obj)
      if (exist('constructModelpmex')==3)
        obj.mex_model_ptr = SharedDataHandle(constructModelpmex(obj.featherstone,obj.body,obj.gravity),@deleteModelpmex);
      end
    end
    
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
        m.jsign(i) = b.jsign;
        m.Xtree{i} = b.Xtree;
        m.I{i} = b.I;
        m.damping(i) = b.damping;  % add damping so that it's faster to look up in the dynamics functions.
      end
      model.featherstone = m;
    end    
  
    function model=removeFixedJoints(model)
      fixedind = find(isnan([model.body.pitch]));
      
      for i=fixedind(end:-1:1)  % go backwards, since it is presumably more efficient to start from the bottom of the tree
        body = model.body(i);
        parent = body.parent;
        
        % add geometry into parent
        if (~isempty(body.geometry))
          for j=1:length(body.geometry)
            pts = body.Ttree * [reshape(body.geometry{j}.x,1,[]); reshape(body.geometry{j}.y,1,[]); ones(1,numel(body.geometry{j}.x))];
            body.geometry{j}.x = reshape(pts(1,:),size(body.geometry{j}.x));
            body.geometry{j}.y = reshape(pts(2,:),size(body.geometry{j}.y));
            parent.geometry = {parent.geometry{:},body.geometry{j}};
          end
        end
      end
      
      model = removeFixedJoints@RigidBodyManipulator(model);
    end
    
    
    function model=parseLink(model,robotnum,node,options)
      options.x_axis = model.x_axis;
      options.y_axis = model.y_axis;
      options.view_axis = model.view_axis;
      model = parseLink@RigidBodyManipulator(model,robotnum,node,options);
    end
    
    function model=parseJoint(model,robotnum,node,options)
      ignore = char(node.getAttribute('drakeIgnore'));
      if strcmp(lower(ignore),'true')
        return;
      end
      
      name = char(node.getAttribute('name'));
      name = regexprep(name, '\.', '_', 'preservecase');

      childNode = node.getElementsByTagName('child').item(0);
      if isempty(childNode) % then it's not the main joint element
        return;
      end
      child = findLink(model,char(childNode.getAttribute('link')),robotnum);
      
      parentNode = node.getElementsByTagName('parent').item(0);
      parent = findLink(model,char(parentNode.getAttribute('link')),robotnum,false);
      if (isempty(parent))
        % could have been zapped
        warning(['joint ',name,' parent link is missing or was deleted.  deleting the child link:', child.linkname,'(too)']);
        ind = find([model.body]==child);
        model.body(ind)=[];
        return;
      end
      
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
          warning('Drake:PlanarRigidBodyManipulator:UnsupportedVelocityLimits','Velocity limits are not supported yet');
        end          
      end

      limits = struct();
      limits.joint_limit_min = joint_limit_min;
      limits.joint_limit_max = joint_limit_max;
      limits.effort_limit = effort_limit;
      limits.velocity_limit = velocity_limit;
      
      model = addJoint(model,name,type,parent,child,xyz,rpy,axis,damping,limits,options);
    end
    
    function model = parseLoopJoint(model,robotnum,node,options)
      loop = PlanarRigidBodyLoop();
      loop.name = char(node.getAttribute('name'));
      loop.name = regexprep(loop.name, '\.', '_', 'preservecase');

      options.x_axis = model.x_axis;
      options.y_axis = model.y_axis;
      options.view_axis = model.view_axis;

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
        case 'continuous'
          loop.jcode=1;          
          if abs(dot(axis,model.view_axis))<(1-1e-6)
            axis
            model.view_axis
            error('i do not yet support continuous joints that do not align with the viewing axis');
            % note: i could potentially support this (and the fixed joint 
            % type) by creating a fixed joint here, then finding the least 
            % common ancestor (or actually any non-fixed ancestor) and 
            % turning that joint into a loop joint. 
          end

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
            fe.pos1 = [model.x_axis'; model.y_axis']*reshape(str2num(char(linkNode.getAttribute('xyz'))),3,1);
          end
          linkNode = node.getElementsByTagName('link2').item(0);
          fe.body2 = findLink(model,char(linkNode.getAttribute('link')),robotnum);
          if linkNode.hasAttribute('xyz')
            fe.pos2 = [model.x_axis'; model.y_axis']*reshape(str2num(char(linkNode.getAttribute('xyz'))),3,1);
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
          
        case 'wing'
          elNode = node.getElementsByTagName('parent').item(0);
          parent = findLink(model,char(elNode.getAttribute('link')),robotnum);
          
          xyz=zeros(3,1); rpy=zeros(3,1);
          elnode = node.getElementsByTagName('origin').item(0);
          if ~isempty(elnode)
            if elnode.hasAttribute('xyz')
              xyz = str2num(char(elnode.getAttribute('xyz')));
            end
            if elnode.hasAttribute('rpy')
              rpy = str2num(char(elnode.getAttribute('rpy')));
            end
          end
          
          elNode = node.getElementsByTagName('profile').item(0);
          profile = char(elNode.getAttribute('value'));

          elnode = node.getElementsByTagName('chord').item(0);
          chord = str2num(char(elnode.getAttribute('value')));
          
          elnode = node.getElementsByTagName('span').item(0);
          span = str2num(char(elnode.getAttribute('value')));
          
          elnode = node.getElementsByTagName('stall_angle').item(0);
          stall_angle = str2num(char(elnode.getAttribute('value')));

          elnode = node.getElementsByTagName('nominal_speed').item(0);
          nominal_speed = str2num(char(elnode.getAttribute('value')));
          
          fe = PlanarRigidBodyWing();
          fe.name = name;
          model.force{end+1} = fe;
        otherwise
          error(['force element type ',type,' not supported (yet?)']);
      end
    end
  end
  
end

