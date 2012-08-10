classdef PlanarRigidBodyModel < RigidBodyModel
  
  methods 
      
    function model = PlanarRigidBodyModel(urdf_filename,options)
      % @options view when options.twoD = true, then this defines the axes.
      %  Use 'front' for Y-Z axes, 'top' for X-Y axes,
      %  or 'right' for X-Z axes.  @default right
      
      if (nargin<1) urdf_filename=''; end
      if (nargin<2) options = struct(); end
      if (~isfield(options,'view')) options.view = 'right';
      else
        options.view = lower(options.view);
        if ~any(strcmp(options.view,{'front','back','top','bottom','right','left'}))
          error('supported view options are front,back,top,bottom,right,or left');
        end
      end
      model = model@RigidBodyModel(urdf_filename,options);
        
      if any(strcmp(options.view,{'top','bottom'}))
        model.gravity = [0;0];
      else
        model.gravity = [0;-9.81];
      end
    end    
    
    function model = doKinematics(model,q,qd)
      for i=1:length(model.body)
        body = model.body(i);
        if (isempty(body.parent))
          body.T = eye(3);
          body.v = zeros(3,1);
        else
          TJ = Tjcalcp(body.jcode,q(body.dofnum));
          [Xj,S] = jcalcp(body.jcode,q(body.dofnum));
          body.T=body.parent.T*body.Ttree*TJ;
          body.v=body.parent.v + S*qd(body.dofnum) + [0; body.parent.v(1)*body.T(1:2,3)];
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
            for k=1:length(body.geometry{j}.x)
              pt0 = [body.geometry{j}.x(k); body.geometry{j}.z(k); 1];
              pt1 = body.Ttree * pt0;  %rotation might be backwards
              body.geometry{j}.x(k) = pt1(1);
              body.geometry{j}.z(k) = pt1(2);
            end
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
      
      switch (lower(type))
        case {'revolute','continuous'}
          child.pitch=0;
          child.joint_axis=axis;
          child.jcode=1;
          
        case 'prismatic'
          child.pitch=inf;
          child.joint_axis=axis;
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
          if dot(axis,[0;1;0])<(1-1e-6)
            error('planar joints only supported in the y axis');
          end
          body1=newBody(model);
          body1.linkname=[child.jointname,'_x'];
          body1.jointname = body1.linkname;
          body1.pitch=inf;
          body1.joint_axis = [1;0;0];
          body1.jcode=2;
          body1.damping=damping;
          body1.parent=parent;
          body2=newBody(model);
          body2.linkname=[child.jointname,'_z'];
          body2.jointname = body2.linkname;
          body2.pitch=inf;
          body1.joint_axis = [0;0;1];
          body2.jcode=3;
          body2.damping=damping;
          body2.parent = body1;
          child.pitch=0;
          child.joint_axis=axis;
          child.jcode=1;
          child.parent = body2;
          model.body=[model.body,body1,body2];
          
          child=body1;
          
        case 'fixed'
          child.pitch=nan;
          
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
      
      if ~isempty(wrl_joint_origin)
        child.wrljoint = wrl_joint_origin;
      end
      
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