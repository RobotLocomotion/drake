classdef PlanarRigidBodyModel < RigidBodyModel
  
  properties
    x_axis_label;
    y_axis_label;
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
      
      switch options.view % joint_axis = view_axis => counter-clockwise
        case 'front'
          options.x_axis = [0;1;0];
          options.y_axis = [0;0;1];
          options.view_axis = [1;0;0];  
        case 'right'
          options.x_axis = [1;0;0];
          options.y_axis = [0;0;1];
          options.view_axis = [0;1;0];
        case 'top'
          options.x_axis = [1;0;0];
          options.y_axis = [0;1;0];
          options.view_axis = [0;0;1];
      end
        
      model = model@RigidBodyModel(urdf_filename,options);
  
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
    
    function doKinematics(model,q,qd)
      if 0 %any(abs([q;qd]-reshape([model.body.cached_q_qd],1,[])')<1e-6)  % todo: make this tolerance a parameter
        % then my kinematics are up to date, don't recompute
        return
      end
      for i=1:length(model.body)
        body = model.body(i);
        if (isempty(body.parent))
          body.T = body.Ttree;
          body.v = zeros(3,1);
        else
          qi = body.jsign*q(body.dofnum);
          qdi = body.jsign*qd(body.dofnum);
          
          TJ = Tjcalcp(body.jcode,qi);
          [~,S] = jcalcp(body.jcode,qi);
          body.T=body.parent.T*body.Ttree*TJ;
          body.v=body.parent.v + S*qdi + [0; body.parent.v(1)*body.T(1:2,3)];
          body.cached_q_qd = [q(body.dofnum);qd(body.dofnum)];
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
    
    function model=parseJoint(model,node,options)

      parentNode = node.getElementsByTagName('parent').item(0);
      if isempty(parentNode) % then it's not the main joint element.  for instance, the transmission element has a joint element, too
          return
      end
      parent = findLink(model,char(parentNode.getAttribute('link')));
      
      childNode = node.getElementsByTagName('child').item(0);
      child = findLink(model,char(childNode.getAttribute('link')));
      child.jointname = char(node.getAttribute('name'));

      if (child.parent>=0)
        error('there is already a joint connecting this child to a parent');
      end
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

      
      if any(rpy)
        rpya=rpy2axis(rpy); rpyangle=rpya(4); rpyaxis=rpya(1:3);
        if abs(dot(rpyaxis,options.view_axis))<(1-1e-6)
%          error('joints out of the plane are not supported, and will be removed (along with their descendants)');
          % note that if they were, it would change the way that I have to 
          % parse geometries, inertias, etc, for all of the children.
        elseif dot(rpyaxis,options.view_axis)<0
          rpyangle=-rpyangle;
        end
      else
        rpyangle=0;
      end
      
      switch (lower(type))
        case {'revolute','continuous'}
          child.pitch=0;
          if abs(dot(axis,options.view_axis))<(1-1e-6)
            warning('DRC:PlanarRigidBodyModel:RemovedJoint',['Welded revolute joint ', child.jointname,' because it did not align with the viewing axis']);
            node.setAttribute('type','fixed');
            model = parseJoint(model,node,options);
            return;
          end
          child.joint_axis=axis;
          child.jsign = sign(dot(axis,options.view_axis));
          child.jcode=1;
          
        case 'prismatic'
          child.pitch=inf;
          child.joint_axis=axis;
          if abs(dot(axis,options.view_axis))>1e-6
            warning('DRC:PlanarRigidBodyModel:RemovedJoint',['Welded prismatic joint ', child.jointname,' because it did not align with the viewing axis']);
            node.setAttribute('type','fixed');
            model = parseJoint(model,node,options);
            return;
          end
          if abs(dot(axis,options.x_axis))>(1-1e-6)
            child.jcode=2;
            child.jsign = sign(dot(axis,options.x_axis));
          elseif dot(axis,options.y_axis)>(1-1e-6)
            child.jcode=3;
            child.jsign = sign(dot(axis,options.y_axis));
          else
            error('Currently only prismatic joints with their axis in the x-axis or z-axis are supported right now (twoD assumes x-z plane)');
          end
          
        case 'planar'
          % create two links with sliders, then finish this function with
          % the first of these joints (which need to catch the kinematics)
          if abs(dot(axis,options.view_axis))<(1-1e-6)
            error('planar joints only supported in the viewing axis');
          end
          jsign = sign(dot(axis,options.view_axis));
          body1=newBody(model);
          body1.linkname=[child.jointname,'_x'];
          body1.jointname = body1.linkname;
          body1.pitch=inf;
          body1.joint_axis = [1;0;0];
          body1.jcode=2;
          body1.jsign=jsign;
          body1.damping=damping;
          body1.joint_limit_min = -inf;
          body1.joint_limit_max = inf;
          body1.parent=parent;
          body2=newBody(model);
          body2.linkname=[child.jointname,'_z'];
          body2.jointname = body2.linkname;
          body2.pitch=inf;
          body1.joint_axis = [0;0;1];
          body2.jcode=3;
          body2.jsign=jsign;
          body2.damping=damping;
          body2.joint_limit_min = -inf;
          body2.joint_limit_max = inf;
          body2.parent = body1;
          child.pitch=0;
          child.joint_axis=axis;
          child.jcode=1;
          child.jsign=jsign;
          child.parent = body2;
          model.body=[model.body,body1,body2];
          
          child=body1;
          
        case 'fixed'
          child.pitch=nan;
          
        otherwise
          error(['joint type ',type,' not supported in planar models']);
      end
      
      child.joint_limit_min=-inf;
      child.joint_limit_max=inf;
      limits = node.getElementsByTagName('limit').item(0);
      if ~isempty(limits)
        if limits.hasAttribute('lower')
          child.joint_limit_min = str2num(char(limits.getAttribute('lower')));
        end
        if limits.hasAttribute('upper');
          child.joint_limit_max = str2num(char(limits.getAttribute('upper')));
        end          
      end
      
      xy = [options.x_axis'; options.y_axis']*xyz;
      child.Xtree = Xpln(rpyangle,xy);
      child.Ttree = [rotmat(rpyangle),xy; 0,0,1];
      child.T = parent.T*child.Ttree;
      child.damping = damping;
      
      if ~isempty(wrl_joint_origin)
        child.wrljoint = wrl_joint_origin;
      end
      
    end
    
    function model = addFloatingBase(model,options)
      rootlink = find(cellfun(@isempty,{model.body.parent}));
      if (length(rootlink)>1)
        warning('multiple root links');
      end
      
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
        body1.jcode=2;
        body1.joint_limit_min = -inf;
        body1.joint_limit_max = inf;
        body1.parent=[];  % this is the new root link
        body2=newBody(model);
        name = [child.linkname,'_z'];
        if strcmpi(name,horzcat({model.body.linkname},{model.body.jointname}))
          error('floating name already exists.  cannot add floating base.');
        end
        body2.linkname = name;
        body2.jointname = name;
        body2.pitch=inf;
        body1.joint_axis = [0;0;1];
        body2.jcode=3;
        body2.joint_limit_min = -inf;
        body2.joint_limit_max = inf;
        body2.parent = body1;
        child.pitch=0;
        child.joint_axis=axis;
        child.jcode=1;
        child.parent = body2;
        model.body=[model.body,body1,body2];
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
          if dot(axis,options.view_axis)<(1-1e-6)
            axis
            options.view_axis
            error('revolute joints must align with the viewing axis');
            % note: i don't support negative angles here yet (via jsign),
            % but could
          end

        case 'prismatic'
          if dot(axis,options.x_axis)>(1-1e-6)
            loop.jcode=2;
          elseif dot(axis,options.y_axis)>(1-1e-6)
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