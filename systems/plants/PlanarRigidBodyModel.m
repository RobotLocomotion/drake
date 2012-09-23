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
          model.x_axis_label='y';
          model.y_axis_label='z';
%          model.name = [model.name,'Front'];
        case 'right'
          model.x_axis = [1;0;0];
          model.y_axis = [0;0;1];
          model.view_axis = [0;1;0];
          model.x_axis_label='x';
          model.y_axis_label='z';
%          model.name = [model.name,'Right'];
        case 'top'
          model.x_axis = [1;0;0];
          model.y_axis = [0;1;0];
          model.view_axis = [0;0;1];
          model.x_axis_label='x';
          model.y_axis_label='y';
%          model.name = [model.name,'Top'];
      end
        
      if ~isempty(urdf_filename)
        options.x_axis = model.x_axis;
        options.y_axis = model.y_axis;
        options.view_axis = model.view_axis;
        model = parseURDF(model,urdf_filename,options);
      end
      
      switch options.view
        case 'front'
          model.gravity = [0;-9.81];

        case 'right'
          model.gravity = [0;-9.81];
          if ~isempty(model.body)
            % 'flip' rotational kinematics (since y-axis goes into the page)
            rotind = find([model.body.jcode]==1);
            for i=rotind
              model.body(i).jsign = -model.body(i).jsign;
            end
          end           
        case 'top'
          model.gravity = [0;0];
      end
    end    
    
    
    function doKinematics(model,q)
      if isnumeric(q) && all(abs(q-[model.body.cached_q]')<1e-8)  % todo: make this tolerance a parameter
        % then my kinematics are up to date, don't recompute
        % the "isnumeric" check is for the sake of taylorvars
        return
      end
      nq = model.featherstone.NB;
      for i=1:length(model.body)
        body = model.body(i);
        if (isempty(body.parent))
          body.T = body.Ttree;
          body.dTdq = zeros(3*nq,3);
          body.ddTdqdq = zeros(3*nq*nq,3);
        else
          qi = body.jsign*q(body.dofnum);
          
          TJ = Tjcalcp(body.jcode,qi);
          dTJ = dTjcalcp(body.jcode,qi)*body.jsign;

          body.T=body.parent.T*body.Ttree*TJ;

          % todo: consider pulling this out into a
          % "doKinematicsAndVelocities" version?  but I'd have to be
          % careful with caching.
          
          % note the unusual format of dTdq (chosen for efficiently calculating jacobians from many pts)
          % dTdq = [dT(1,:)dq1; dT(1,:)dq2; ...; dT(1,:)dqN; dT(2,dq1) ...]
          body.dTdq = body.parent.dTdq*body.Ttree*TJ;
          this_dof_ind = body.dofnum+0:nq:3*nq;
          body.dTdq(this_dof_ind,:) = body.dTdq(this_dof_ind,:) + body.parent.T*body.Ttree*dTJ;

          % ddTdqdq = [d(dTdq)dq1; d(dTdq)dq2; ...]
          body.ddTdqdq = body.parent.ddTdqdq*body.Ttree*TJ;

          ind = 3*nq*(body.dofnum-1) + (1:3*nq);  %ddTdqdqi
          body.ddTdqdq(ind,:) = body.ddTdqdq(ind,:) + body.parent.dTdq*body.Ttree*dTJ;

          ind = reshape(reshape(body.dofnum+0:nq:3*nq*nq,3,[])',[],1); % ddTdqidq
          body.ddTdqdq(ind,:) = body.ddTdqdq(ind,:) + body.parent.dTdq*body.Ttree*dTJ;
          
          ind = 3*nq*(body.dofnum-1) + this_dof_ind;  % ddTdqidqi
          body.ddTdqdq(ind,:) = body.ddTdqdq(ind,:) + body.parent.T*body.Ttree*ddTjcalcp(body.jcode,qi);  % body.jsign^2 is there, but unnecessary (since it's always 1)
          
          body.cached_q = q(body.dofnum);
        end
      end
    end
    
    function [x,J,dJ] = kinTest(m,q)
      % test for kinematic gradients
      doKinematics(m,q);
        
      count=0;
      for i=1:length(m.body)
        body = m.body(i);
        for j=1:length(body.geometry)
          s = size(body.geometry{j}.x); n=prod(s);
          pts = [reshape(body.geometry{j}.x,1,n); reshape(body.geometry{j}.y,1,n)];
          if (nargout>1)
            [x(:,count+(1:n)),J(2*count+(1:2*n),:),dJ(2*count+(1:2*n),:)] = forwardKin(body,pts);
          else
          if ~exist('x') % extra step to help taylorvar
            x = forwardKin(body,pts);
          else
            xn = forwardKin(body,pts);
            x=[x,xn];
          end
          end
          count = count + n;
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
    
    function fr = getOutputFrameWContactForces(model)
      stateframe = getStateFrame(model);
      coordinates = stateframe.coordinates;
      for b=model.body
        for j=1:size(b.contact_pts,2)
          coordinates = vertcat(coordinates,sprintf('%sContact%d_x',b.linkname,j),sprintf('%sContact%d_y',b.linkname,j));
        end
      end      
      fr = SingletonCoordinateFrame([model.name,'Output'],length(coordinates),'x',coordinates);
      if isempty(findTransform(fr,stateframe)) 
        % then create the transform which drops the contact forces and
        % returns just the states
        addTransform(fr,AffineTransform(fr,stateframe,[eye(stateframe.dim),zeros(stateframe.dim,length(coordinates)-stateframe.dim)],zeros(stateframe.dim,1)));
      end
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
            warning('Drake:PlanarRigidBodyModel:RemovedJoint',['Welded revolute joint ', child.jointname,' because it did not align with the viewing axis']);
            model = addJoint(model,name,'fixed',parent,child,xyz,rpy,axis,damping,joint_lim_min,joint_lim_max,options);
            return;
          end
        case 'prismatic'
          if abs(dot(axis,model.view_axis))>1e-6
            warning('Drake:PlanarRigidBodyModel:RemovedJoint',['Welded prismatic joint ', child.jointname,' because it did not align with the viewing axis']);
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
          body1.linkname=[name,'_',model.x_axis_label];
          model.body = [model.body,body1];
          model = addJoint(model,body1.linkname,'prismatic',parent,body1,xyz,rpy,jsign*model.x_axis,damping);

          body2=newBody(model);
          body2.linkname=[name,'_',model.y_axis_label];
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
          warning('joint axes out of the plane are not supported.  the dependent link (and all of it''s decendants) will be zapped');
          ind = find([model.body]==child);
          model.body(ind)=[];
          return;
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

      name = char(node.getAttribute('name'));

      childNode = node.getElementsByTagName('child').item(0);
      if isempty(childNode) % then it's not the main joint element
        return;
      end
      child = findLink(model,char(childNode.getAttribute('link')));
      
      parentNode = node.getElementsByTagName('parent').item(0);
      parent = findLink(model,char(parentNode.getAttribute('link')),false);
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
        model = addJoint(model,model.body(i).linkname,'planar',world,model.body(i),zeros(3,1),zeros(3,1),model.view_axis,0,-inf,inf,options);
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