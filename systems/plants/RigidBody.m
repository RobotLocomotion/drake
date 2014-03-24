classdef RigidBody < RigidBodyElement
  
  properties 
    robotnum = 0;  % this body is associated with a particular robot/object number, named in model.name{objnum} 
    
    % link properties
    linkname=''  % name of the associated link
    dofnum=0     % the index in the state vector corresponding to this joint
    gravity_off=false;
    
    visual_shapes={}; % objects of type RigidBodyGeometry
    contact_shapes={}; % objects of type RigidBodyGeometry

    contact_pts=[];  % a 3xn matrix with [x;y;z] positions of contact points
    collision_group_name={};  % string names of the groups
    collision_group={};       % collision_group{i} is a list of indices into contact_pts which belong to collision_group_name{i}
    contact_shape_group={}; % contact_shape_group{i} is a list of indices into contact_shapes which belong to collision_group_name{i}
    
    % joint properties
    parent=0;       % index (starting at 1) for rigid body parent.  0 means no parent
    jointname='';
    pitch=0;        % for featherstone 3D models
    floating=0; % 0 = not floating base, 1 = rpy floating base, 2 = quaternion floating base
    joint_axis=[1;0;0]; 
    Xtree=eye(6);   % velocity space coordinate transform *from parent to this node*
    X_joint_to_body=eye(6);  % velocity space coordinate transfrom from joint frame (where joint_axis = z-axis) to body frame 
    Ttree=eye(4);   % position space coordinate transform *from this node to parent*
    T_body_to_joint=eye(4);
    wrljoint='';  % tranformation to joint coordinates in wrl syntax
    damping=0; % viscous friction term
    coulomb_friction=0; 
    static_friction=0; % currently not used for simulation
    coulomb_window=eps; % the threshold around zero velocity used for the PWL friction model (See Khalil and Dombre, Fig. 9.2(d))
    joint_limit_min=[];
    joint_limit_max=[];
    effort_limit=[];
    velocity_limit=[];
    has_position_sensor=false;
  end
  
  properties (SetAccess=protected, GetAccess=public)    
    % mass, com, inertia properties
    I=zeros(6);  % spatial mass/inertia
    mass = 0;
    com = [];
    inertia = [];

    % Collision filter properties
    collision_filter = struct('belongs_to',CollisionFilterGroup.DEFAULT_COLLISION_FILTER_GROUP_ID, ...
                             'collides_with',CollisionFilterGroup.ALL_COLLISION_FILTER_GROUPS);
  end
  
  methods    
    
    function varargout = forwardKin(varargin)
      error('forwardKin(body,...) has been replaced by forwardKin(model,body_num,...), because it has a mex version.  please update your kinematics calls');
    end
    
    function [pts,inds] = getContactPoints(body,collision_group)
      % @param collision_group (optional) return only the points in that
      % group.  can be an integer index or a string.
      if (nargin<2) 
        pts = body.contact_pts;
        inds = 1:length(body.contact_pts);
      else
        if isa(collision_group,'char')
          collision_group_ind = find(strcmpi(collision_group,body.collision_group_name));
          if isempty(collision_group_ind)
            error(['cannot find collision group ',collision_group]);
          end
        else
          collision_group_ind = collision_group;
        end
        inds = body.collision_group{collision_group_ind};
        pts = body.contact_pts(:,inds);
      end
    end

    function shapes = getContactShapes(body,collision_group,collision_ind)
      % @param collision_group (optional) return structures for only the
      % contact_shapes in that group.  can be an integer index or a string.
      if (nargin<2) 
        shapes = body.contact_shapes;
      else
        if isa(collision_group,'char')
          collision_group = find(strcmpi(collision_group,body.collision_group_name));
        end
        if (nargin < 2)
          shapes = body.contact_shapes{body.contact_shape_group{collision_group}};
        else
          shapes = body.contact_shapes{body.contact_shape_group{collision_group}(collision_ind)};
        end
      end
    end
     
    function body = replaceContactShapesWithCHull(body,scale_factor)
      pts = body.contact_pts(:,unique(convhull(body.contact_pts')));
      if nargin > 1
        mean_of_pts = mean(pts,2);
        pts = bsxfun(@plus,scale_factor*bsxfun(@minus,pts,mean_of_pts),mean_of_pts);
      end
      body.contact_pts = pts;
      body.contact_shapes = { RigidBodyMeshPts(pts) };
    end
    
    function body = removeCollisionGroups(body,contact_groups)
      if isempty(body.contact_pts), return; end % nothing to do for this body
      if ~iscell(contact_groups), contact_groups={contact_groups}; end
      for i=1:length(contact_groups)
        group_elements = strcmpi(contact_groups{i},body.collision_group_name);
        if ~isempty(group_elements)
          pt_inds = [body.collision_group{group_elements}];
          if ~isempty(pt_inds)
            [keep_pts,ipts] = setdiff(1:size(body.contact_pts,2),pt_inds);
            ripts = repmat(nan,1,size(body.contact_pts));  % reverse index
            ripts(ipts) = 1:length(ipts);
            for j=1:length(body.collision_group)
              body.collision_group{j}=ripts(body.collision_group{j});
            end
            body.contact_pts(:,pt_inds) = [];
          end
          body.collision_group(group_elements) = [];
          body.collision_group_name(group_elements) = [];
          % todo: should actually remove the contact shapes, too
          body.contact_shape_group(group_elements) = [];
        end
      end
    end
    
    function body = removeCollisionGroupsExcept(body,contact_groups)
      if isempty(body.contact_pts), return; end % nothing to do for this body
      if ~iscell(contact_groups) contact_groups={contact_groups}; end
      i=1;
      while i<=length(body.collision_group)
        if ~ismember(body.collision_group_name{i},contact_groups)
          pt_inds = [body.collision_group{i}];
          if ~isempty(pt_inds)
            [keep_pts,ipts] = setdiff(1:size(body.contact_pts,2),pt_inds);
            ripts = repmat(nan,1,size(body.contact_pts));  % reverse index
            ripts(ipts) = 1:length(ipts);
            for j=1:length(body.collision_group)
              body.collision_group{j}=ripts(body.collision_group{j});
            end
            body.contact_pts(:,pt_inds) = [];
          end
          body.collision_group(i) = [];
          body.collision_group_name(i) = [];
          % todo: should actually remove the contact shapes, too
          body.contact_shape_group(i) = [];
        else
          i=i+1;
        end
      end
    end

    function body = makeBelongToNoCollisionFilterGroups(body)
      body.collision_filter.belongs_to = CollisionFilterGroup.NO_COLLISION_FILTER_GROUPS;
    end
    
    function body = makeIgnoreNoCollisionFilterGroups(body)
      body.collision_filter.collides_with = CollisionFilterGroup.ALL_COLLISION_FILTER_GROUPS;
    end

    function body = makeBelongToCollisionFilterGroup(body,collision_fg_id)
      for id = reshape(collision_fg_id,1,[])
        body.collision_filter.belongs_to = ...
          bitor(body.collision_filter.belongs_to,bitshift(1,id-1));
      end
    end

    function body = makeIgnoreCollisionFilterGroup(body,collision_fg_id)
      for id = reshape(collision_fg_id,1,[])
        body.collision_filter.collides_with = ...
          bitand(body.collision_filter.collides_with,bitcmp(bitshift(uint16(1),id-1)));
      end
    end

    function newbody = copy(body)
      % makes a shallow copy of the body
      % note that this functionality is in matlab.mixin.Copyable in newer
      % versions of matlab, but I've done it myself since i want to support
      % < v2011
      
      newbody=RigidBody();
      p=properties(body);
      for i=1:length(p)
        eval(['newbody.',p{i},'=body.',p{i}]);
      end
    end
    
    % Note: bindParams and updateParams are copies of the methods in 
    % RigidBodyElement (yuck!) because the RigidBodyElement version does 
    % not have permissions to do the reflection on the protected properties 
    % in this class.
    function body=bindParams(body,model,pval)
      fr = getParamFrame(model);
      pn = properties(body);
      for i=1:length(pn)
        if isa(body.(pn{i}),'msspoly')
          body.param_bindings.(pn{i}) = body.(pn{i});
          body.(pn{i}) = double(subs(body.(pn{i}),fr.poly,pval));
        end
      end
    end
    
    function body=updateParams(body,poly,pval)
      % this is only intended to be called from the parent manipulator
      % class. (maybe I should move it up to there?)
      fn = fieldnames(body.param_bindings);
      for i=1:length(fn)
        body.(fn{i}) = double(subs(body.param_bindings.(fn{i}),poly,pval));
      end
    end
    
    function body=parseInertial(body,node,model,options)
      mass = 0;
      inertia = eye(3);
      xyz=zeros(3,1); rpy=zeros(3,1);
      origin = node.getElementsByTagName('origin').item(0);  % seems to be ok, even if origin tag doesn't exist
      if ~isempty(origin)
        if origin.hasAttribute('xyz')
          xyz = reshape(parseParamString(model,body.robotnum,char(origin.getAttribute('xyz'))),3,1);
        end
        if origin.hasAttribute('rpy')
          rpy = reshape(parseParamString(model,body.robotnum,char(origin.getAttribute('rpy'))),3,1);
        end
      end
      massnode = node.getElementsByTagName('mass').item(0);
      if ~isempty(massnode)
        if (massnode.hasAttribute('value'))
          mass = parseParamString(model,body.robotnum,char(massnode.getAttribute('value')));
        end
      end
      inode = node.getElementsByTagName('inertia').item(0);
      if ~isempty(inode)
        if inode.hasAttribute('ixx'), ixx = parseParamString(model,body.robotnum,char(inode.getAttribute('ixx'))); else ixx=0; end
        if inode.hasAttribute('ixy'), ixy = parseParamString(model,body.robotnum,char(inode.getAttribute('ixy'))); else ixy=0; end
        if inode.hasAttribute('ixz'), ixz = parseParamString(model,body.robotnum,char(inode.getAttribute('ixz'))); else ixz=0; end
        if inode.hasAttribute('iyy'), iyy = parseParamString(model,body.robotnum,char(inode.getAttribute('iyy'))); else iyy=0; end
        if inode.hasAttribute('iyz'), iyz = parseParamString(model,body.robotnum,char(inode.getAttribute('iyz'))); else iyz=0; end
        if inode.hasAttribute('izz'), izz = parseParamString(model,body.robotnum,char(inode.getAttribute('izz'))); else izz=0; end
        inertia = [ixx, ixy, ixz; ixy, iyy, iyz; ixz, iyz, izz];
      end
      
      % randomly scale inertia
      % keep scale factor positive to ensure positive definiteness
      % x'*I*x > 0 && eta > 0 ==> x'*(eta*I)*x > 0
      eta = 1 + min(1,max(-0.9999,options.inertia_error*randn()));
      inertia = eta*inertia;  
      
      if isnumeric(inertia) && ~all(eig(inertia)>0)
        warning('Drake:RigidBodyManipulator:NonPSDInertia','RigidBody: inertia matrix not positive definite!');
      end
      if any(rpy)
        error([body.linkname,': rpy in inertia block not implemented yet (but would be easy)']);
      end
      body = setInertial(body,mass,xyz,inertia);
    end
    
    function body = setInertial(body,varargin)
      % setInertial(body,mass,com,inertia) or setInertial(body,spatialI)
      % this guards against things getting out of sync
      
      function v = skew(A)
        v = 0.5 * [ A(3,2) - A(2,3);
          A(1,3) - A(3,1);
          A(2,1) - A(1,2) ];
      end
      
      if nargin==2
        sizecheck(varargin{1},[6 6]);
        % extract mass, center of mass, and inertia information from the
        % spatial I matrix
        body.I = varargin{1};
        body.mass = body.I(6,6);
        mC = body.I(1:3,4:6);
        body.com = skew(mC)/body.mass;
        body.inertia = body.I(1:3,1:3) - mC*mC'/body.mass;
      elseif nargin==4
        sizecheck(varargin{1},1);
        sizecheck(varargin{2},[3 1]);
        sizecheck(varargin{3},[3 3]);
        body.mass = varargin{1};
        body.com = varargin{2};
        body.inertia = varargin{3};
        body.I = mcI(body.mass,body.com,body.inertia);
      else
        error('wrong number of arguments');
      end    
    end    
    
    function body = parseVisual(body,node,model,options)
      c = .7*[1 1 1];
      
      xyz=zeros(3,1); rpy=zeros(3,1);
      origin = node.getElementsByTagName('origin').item(0);  % seems to be ok, even if origin tag doesn't exist
      if ~isempty(origin)
        if origin.hasAttribute('xyz')
          xyz = reshape(parseParamString(model,body.robotnum,char(origin.getAttribute('xyz'))),3,1);
        end
        if origin.hasAttribute('rpy')
          rpy = reshape(parseParamString(model,body.robotnum,char(origin.getAttribute('rpy'))),3,1);
        end
      end
        
      matnode = node.getElementsByTagName('material').item(0);
      if ~isempty(matnode)
        c = RigidBodyManipulator.parseMaterial(matnode,options);
      end
      
      geomnode = node.getElementsByTagName('geometry').item(0);
      if ~isempty(geomnode)
        if (options.visual || options.visual_geometry)
          shape = RigidBodyGeometry.parseURDFNode(geomnode,xyz,rpy,model,body.robotnum,options);
          shape.c = c;
          body.visual_shapes = {body.visual_shapes{:},shape};
        end
      end        
    end
    
    function body = parseCollision(body,node,model,options)
      xyz=zeros(3,1); rpy=zeros(3,1);
      origin = node.getElementsByTagName('origin').item(0);  % seems to be ok, even if origin tag doesn't exist
      if ~isempty(origin)
        if origin.hasAttribute('xyz')
          xyz = reshape(parseParamString(model,body.robotnum,char(origin.getAttribute('xyz'))),3,1);
        end
        if origin.hasAttribute('rpy')
          rpy = reshape(parseParamString(model,body.robotnum,char(origin.getAttribute('rpy'))),3,1);
        end
      end
      
      npts=size(body.contact_pts,2);
      geomnode = node.getElementsByTagName('geometry').item(0);
      if ~isempty(geomnode)
        shape = RigidBodyGeometry.parseURDFNode(geomnode,xyz,rpy,model,body.robotnum,options);
        pts = getPoints(shape);
        npts_additional = size(pts,2);
        [body.contact_pts,ind_old,ind_new]=unique([body.contact_pts';pts'],'rows','stable');
        body.contact_pts = body.contact_pts';
        body.contact_shapes = {body.contact_shapes{:},shape};
      end
      if (node.hasAttribute('group'))
        name=char(node.getAttribute('group'));
      else
        name='default';
      end
      ind = find(strcmp(body.collision_group_name,name));
      if isempty(ind)
        body.collision_group_name=horzcat(body.collision_group_name,name);
        ind=length(body.collision_group_name);
        body.collision_group{ind}=ind_new(npts+(1:npts_additional))';
        body.contact_shape_group{ind} = length(body.contact_shapes);
      else
        body.collision_group{ind}=[body.collision_group{ind},ind_new(npts+(1:npts_additional))'];
        body.contact_shape_group{ind} = [body.contact_shape_group{ind},length(body.contact_shapes)];
      end
    end
  end
  
  methods (Static)
    function testMakeBelongToCollisionFilterGroup
      body = RigidBody();
      collision_fg_id = uint16(3);
      belongs_to_ref = '0000000000000101';
      body = makeBelongToCollisionFilterGroup(body,collision_fg_id);
      belongs_to = dec2bin(body.collision_filter.belongs_to,16);
      assert(strcmp(belongs_to,belongs_to_ref), ...
      'Expected ''%s'', found ''%s''',belongs_to_ref,belongs_to);
    end

    function testMakeIgnoreCollisionFilterGroup
      body = RigidBody();
      collision_fg_id = uint16(3);
      collides_with_ref = '1111111111111011';
      body = makeIgnoreCollisionFilterGroup(body,collision_fg_id);
      collides_with = dec2bin(body.collision_filter.collides_with,16);
      assert(strcmp(collides_with,collides_with_ref), ...
      'Expected ''%s'', found ''%s''',collides_with_ref,collides_with);
    end

    function testMakeBelongToNoCollisionFilterGroups
      body = RigidBody();
      collision_fg_id = uint16(3);
      belongs_to_ref = '0000000000000000';
      body = makeBelongToCollisionFilterGroup(body,collision_fg_id);
      body = makeBelongToNoCollisionFilterGroups(body);
      belongs_to = dec2bin(body.collision_filter.belongs_to,16);
      assert(strcmp(belongs_to,belongs_to_ref), ...
      'Expected ''%s'', found ''%s''',belongs_to_ref,belongs_to);
    end

    function testMakeIgnoreNoCollisionFilterGroups
      body = RigidBody();
      collision_fg_id = uint16(3);
      collides_with_ref = '1111111111111111';
      body = makeIgnoreCollisionFilterGroup(body,collision_fg_id);
      body = makeIgnoreNoCollisionFilterGroups(body);
      collides_with = dec2bin(body.collision_filter.collides_with,16);
      assert(strcmp(collides_with,collides_with_ref), ...
      'Expected ''%s'', found ''%s''',collides_with_ref,collides_with);
    end
  end
  
  methods    
    function obj = updateBodyIndices(obj,map_from_old_to_new)
      obj.parent = map_from_old_to_new(obj.parent);
    end
  end
end
