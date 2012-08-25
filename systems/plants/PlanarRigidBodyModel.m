classdef PlanarRigidBodyModel < RigidBodyModel
  
  properties
    x_axis_label;
    y_axis_label;
    x_axis;
    y_axis;
    view_axis;
  end
  
  methods 
      
    function model = PlanarRigidBodyModel(urdf_filename,options)
      % @options view when options.twoD = true, then this defines the axes.
      %  Use 'front' for Y-Z axes, 'right' for X-Z axes, or 'top' for X-Y 
      %  axes.  @default right
      
      if (nargin<1) urdf_filename=''; end
      if (nargin<2) options = struct(); end
      if (~isfield(options,'view')) options.view = 'right';
      else
        options.view = lower(options.view);
        if ~any(strcmp(options.view,{'front','right','top'}))
          error('supported view options are front,back,top,bottom,right,or left');
        end
      end
      
      model = model@RigidBodyModel();

      switch options.view % joint_axis = view_axis => counter-clockwise
        case 'front'
          model.x_axis = [0;1;0];
          model.y_axis = [0;0;1];
          model.view_axis = [1;0;0];  
        case 'right'
          model.x_axis = [1;0;0];
          model.y_axis = [0;0;1];
          model.view_axis = [0;1;0];
        case 'top'
          model.x_axis = [1;0;0];
          model.y_axis = [0;1;0];
          model.view_axis = [0;0;1];
      end
        
      if ~isempty(urdf_filename)
        options.x_axis = model.x_axis;
        options.y_axis = model.y_axis;
        options.view_axis = model.view_axis;
        model = parseURDF(model,urdf_filename,options);
      end
      
      switch options.view
        case 'front'
          model.x_axis_label='y';
          model.y_axis_label='z';
          model.gravity = [0;-9.81];

        case 'right'
          model.x_axis_label='x';
          model.y_axis_label='z';
          model.gravity = [0;-9.81];
          if ~isempty(model.body)
            % 'flip' rotational kinematics (since y-axis goes into the page)
            rotind = find([model.body.jcode]==1);
            for i=rotind
              model.body(i).jsign = -model.body(i).jsign;
            end
          end           
        case 'top'
          model.x_axis_label='x';
          model.y_axis_label='y';
          model.gravity = [0;0];
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
            body.dTdq{j} = zeros(3);
          end
        else
          qi = body.jsign*q(body.dofnum);
          
          TJ = Tjcalcp(body.jcode,qi);
          body.T=body.parent.T*body.Ttree*TJ;

          % todo: consider pulling this out into a
          % "doKinematicsAndVelocities" version?  but I'd have to be
          % careful with caching.
          for j=1:model.featherstone.NB
            body.dTdq{j} = body.parent.dTdq{j}*body.Ttree*TJ;
          end
          body.dTdq{body.dofnum} = body.dTdq{body.dofnum} + body.parent.T*body.Ttree*dTjcalcp(body.jcode,qi)*body.jsign;
          
          body.cached_q = q(body.dofnum);
        end
      end
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
      
      model = removeFixedJoints@RigidBodyModel(model);
    end

    function body = newBody(model)
      body = PlanarRigidBody();
    end
    
    function model=addJoint(model,name,type,parent,child,xyz,rpy,axis,damping,joint_lim_min,joint_lim_max,options)
      if (nargin<6) xy=zeros(2,1); end
      if (nargin<7) p=0; end
      if (nargin<9) damping=0; end
      if (nargin<10) joint_lim_min=-inf; end
      if (nargin<11) joint_lim_max=inf; end
      if (nargin<12) options=struct(); end % no options yet
      
      switch (lower(type))
        case {'revolute','continuous','planar'}
          if abs(dot(axis,model.view_axis))<(1-1e-6)
            warning('DRC:PlanarRigidBodyModel:RemovedJoint',['Welded revolute joint ', child.jointname,' because it did not align with the viewing axis']);
            model = addJoint(model,name,'fixed',parent,child,xyz,rpy,axis,damping,joint_lim_min,joint_lim_max,options);
            return;
          end
        case 'prismatic'
          if abs(dot(axis,model.view_axis))>1e-6
            warning('DRC:PlanarRigidBodyModel:RemovedJoint',['Welded prismatic joint ', child.jointname,' because it did not align with the viewing axis']);
            model = addJoint(model,name,'fixed',parent,child,xyz,rpy,axis,damping,joint_lim_min,joint_lim_max,options);
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
          if (joint_lim_min~=-inf || joint_lim_max~=inf)
            error('joint limits not defined for planar joints');
          end
          jsign = sign(dot(axis,model.view_axis));

          body1=newBody(model);
          body1.linkname=[child.jointname,'_x'];
          model.body = [model.body,body1];
          model = addJoint(model,body1.linkname,'prismatic',parent,body1,xyz,rpy,jsign*model.x_axis,damping);

          body2=newBody(model);
          body2.linkname=[child.jointname,'_z'];
          model.body = [model.body,body2];
          model = addJoint(model,body1.linkname,'prismatic',body1,body2,zeros(3,1),zeros(3,1),jsign*model.y_axis,damping);
          
          model = addJoint(model,name,'revolute',body2,child,zeros(3,1),zeros(3,1),axis,damping);
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
          error('joints out of the plane are not supported, and will be removed (along with their descendants)');
          % note that if they were, it would change the way that I have to 
          % parse geometries, inertias, etc, for all of the children.
        elseif dot(rpyaxis,model.view_axis)<0
          p=-p;
        end
      else
        p=0;
      end
      
      child.Xtree = Xpln(p,xy);
      child.Ttree = [rotmat(p),xy; 0,0,1];
      child.T = parent.T*child.Ttree;
      child.damping = damping;
      child.joint_limit_min = joint_lim_min;
      child.joint_limit_max = joint_lim_max;
    end

    
    function model=parseJoint(model,node,options)

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
      limits = node.getElementsByTagName('limit').item(0);
      if ~isempty(limits)
        if limits.hasAttribute('lower')
          joint_limit_min = str2num(char(limits.getAttribute('lower')));
        end
        if limits.hasAttribute('upper');
          joint_limit_max = str2num(char(limits.getAttribute('upper')));
        end          
      end
      
      model = addJoint(model,name,type,parent,child,xyz,rpy,axis,damping,joint_limit_min,joint_limit_max,options);
    end
    
    function model = addFloatingBase(model,options)
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
        model = addJoint(model,name,'prismatic',world,body1,zeros(3,1),zeros(3,1),model.x_axis,0,-inf,inf,options);

        body2=newBody(model);
        name = [child.linkname,'_z'];
        if strcmpi(name,horzcat({model.body.linkname},{model.body.jointname}))
          error('floating name already exists.  cannot add floating base.');
        end
        body2.linkname = name;
        model.body = [model.body,body2];
        model = addJoint(model,name,'prismatic',body1,body2,zeros(3,1),zeros(3,1),model.y_axis,0,-inf,inf,options);
        
        model = addJoint(model,[child.linkname,'_p'],'revolute',body2,child,zeros(3,1),zeros(3,1),model.view_axis,0,-inf,inf,options);
      end
    end
    
    function model = parseLoopJoint(model,node,options)
      loop = PlanarRigidBodyLoop();
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
        case {'revolute','continuous'}
          loop.jcode=1;          
          if dot(axis,model.view_axis)<(1-1e-6)
            axis
            model.view_axis
            error('revolute joints must align with the viewing axis');
            % note: i don't support negative angles here yet (via jsign),
            % but could
          end

        case 'prismatic'
          if dot(axis,model.x_axis)>(1-1e-6)
            loop.jcode=2;
          elseif dot(axis,model.y_axis)>(1-1e-6)
            loop.jcode=3;
          else
            error('axis must be aligned with x or z');
            % note: i don't support negative angles here yet (via jsign),
            % but could
          end
        otherwise
          error(['joint type ',type,' not supported (yet?)']);
      end
      
      model.loop=[model.loop,loop];
    end
    
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
        m.jcode(i) = b.jcode;
        m.Xtree{i} = b.Xtree;
        m.I{i} = b.I;
        m.damping(i) = b.damping;  % add damping so that it's faster to look up in the dynamics functions.
      end
      model.featherstone = m;
    end    
  end

end