classdef RigidBody
  
  properties 
    robotnum = 0;  % this body is associated with a particular robot/object number, named in model.name{objnum} 
    
    % link properties
    linkname=''  % name of the associated link
    wrlgeometry=''; % geometry (compatible w/ wrl).  see parseVisual below.
    geometry={}  % geometry (compatible w/ patch).  see parseVisual below.
    dofnum=0     % the index in the state vector corresponding to this joint
    gravity_off=false;

    contact_pts=[];  % a 3xn matrix with [x;y;z] positions of contact points
    collision_group_name={};  % string names of the groups
    collision_group={};       % collision_group{i} is a list of indices into contact_pts which belong to collision_group_name{i}
    contact_shapes=[]; %struct('type',[],'T',[],'params',[]);  with params as used by URDF (not necessarily by Bullet)
    contact_shape_group={}; % contact_shape_group{i} is a list of indices into contact_shapes which belong to collision_group_name{i}
    
    % joint properties
    jointname='';
    parent=0;       % index (starting at 1) for rigid body parent.  0 means no parent
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
    
    param_bindings=struct();   % structure containing msspoly parameterized representations of some properties
  end
  
  properties (Constant)
    BOX=1;
    SPHERE=2;
    CYLINDER=3;
    MESH=4;
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
          shapes = body.contact_shapes(:,body.contact_shape_group{collision_group});
        else
          shapes = body.contact_shapes(:,body.contact_shape_group{collision_group}(collision_ind));
        end
      end
    end
     
    function body = replaceContactShapesWithCHull(body)
      pts = body.contact_pts(:,unique(convhull(body.contact_pts')));
      shape = struct('type',RigidBody.MESH,'T',eye(4),'params',pts);
      body.contact_shapes = shape;
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
        inertia = [ parseParamString(model,body.robotnum,char(inode.getAttribute('ixx'))), parseParamString(model,body.robotnum,char(inode.getAttribute('ixy'))), parseParamString(model,body.robotnum,char(inode.getAttribute('ixz'))); ...
                    0, parseParamString(model,body.robotnum,char(inode.getAttribute('iyy'))), parseParamString(model,body.robotnum,char(inode.getAttribute('iyz'))); ...
                    0, 0, parseParamString(model,body.robotnum,char(inode.getAttribute('izz'))) ];
        inertia(2,1) = inertia(1,2); inertia(3,1) = inertia(1,3); inertia(3,2) = inertia(2,3);
      end
      
      % randomly scale inertia
      % keep scale factor positive to ensure positive definiteness
      % x'*I*x > 0 && eta > 0 ==> x'*(eta*I)*x > 0
      eta = 1 + min(1,max(-0.9999,options.inertia_error*randn()));
      inertia = eta*inertia;  
      
      if isnumeric(inertia) && ~all(eig(inertia)>0)
        warning('RigidBody: inertia matrix not positive definite!');
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
      
      wrl_transform_str = '';
      wrl_shape_str='';
      wrl_appearance_str='';
      
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
      if any(xyz)
        wrl_transform_str = [wrl_transform_str,'\ttranslation',sprintf(' %f %f %f\n',xyz)];
      end
      if any(rpy)
        wrl_transform_str = [wrl_transform_str,'\trotation',sprintf(' %f %f %f %f\n',rpy2axis(rpy))];
      end        
        
      matnode = node.getElementsByTagName('material').item(0);
      if ~isempty(matnode)
        c = RigidBodyManipulator.parseMaterial(matnode,options);
      end
      wrl_appearance_str = sprintf('appearance Appearance { material Material { diffuseColor %f %f %f } }\n',c(1),c(2),c(3));
      
      if ~isempty(wrl_transform_str)
        if isempty(body.linkname)
          wrl_transform_str = ['Transform {\n',wrl_transform_str];
        else
            % get rid of dots in link name
            body.linkname = regexprep(body.linkname, '\.', '_', 'preservecase');
            wrl_transform_str = [sprintf('DEF %s Transform {\n',body.linkname),wrl_transform_str];
        end
        wrl_transform_str = [wrl_transform_str,'\tchildren [\n'];
      end
      
      geomnode = node.getElementsByTagName('geometry').item(0);
      if ~isempty(geomnode)
        wrl_shape_str = [wrl_shape_str,RigidBody.parseWRLGeometry(geomnode,wrl_appearance_str,model,body.robotnum,options)];
        if (options.visual_geometry)
          [xpts,ypts,zpts] = RigidBody.parseGeometry(geomnode,xyz,rpy,model,options);
          body.geometry{1}.x = xpts;
          body.geometry{1}.y = ypts;
          body.geometry{1}.z = zpts;
          body.geometry{1}.c = c;
        end
      end        
      
      if isempty(wrl_transform_str)
        body.wrlgeometry = [body.wrlgeometry,wrl_shape_str];
      else
        body.wrlgeometry = [body.wrlgeometry,wrl_transform_str,wrl_shape_str,'\t]\n}'];
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
        [xpts,ypts,zpts,shape] = RigidBody.parseGeometry(geomnode,xyz,rpy,model,body.robotnum,options);
        npts_additional = numel(xpts);
        [body.contact_pts,ind_old,ind_new]=unique([body.contact_pts';xpts(:), ypts(:), zpts(:)],'rows','stable');
        body.contact_pts = body.contact_pts';
        body.contact_shapes = horzcat(body.contact_shapes,shape);
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
          
    function writeWRLJoint(body,fp)
      if isempty(body.jointname)
        fprintf(fp,'Transform {\n');
      else
        % get rid of dots in joint name
        body.jointname = regexprep(body.jointname, '\.', '_', 'preservecase');

        fprintf(fp,'DEF %s Transform {\n',body.jointname); 
      end
      if (body.floating)
        fprintf(fp,'translation 0 0 0\n');
        fprintf(fp,'rotation 0 1 0 0\n'); 
      elseif (body.pitch==0) % then it's a pin joint
        fprintf(fp,'rotation 0 1 0 0\n'); 
      elseif isinf(body.pitch) % then it's a slider
        fprintf(fp,'translation 0 0 0\n');
      end
    end
    
    function td=writeWRLBody(body,fp,td)
      t=''; 
      for i=1:td, t=[t,'\t']; end
      s = regexprep(body.wrlgeometry,'\n',['\n',t]);
      fprintf(fp,[t,s,'\n']);
    end
  end
  
  methods (Static)
    function filename=parseMeshFilename(filename,options)
      if ~isempty(strfind(filename,'package://'))
        filename=strrep(filename,'package://','');
        [package,filename]=strtok(filename,filesep);
        filename=[rospack(package),filename];
      else
        [path,name,ext] = fileparts(filename);
        if (path(1)~=filesep)  % the it's a relative path
          path = fullfile(options.urdfpath,path);
        end
        filename = fullfile(path,[name,ext]);
      end
    end
    
    function wrlstr = parseWRLGeometry(node,wrl_appearance_str,model,robotnum,options)
      % param node DOM node for the geometry block
      % param X coordinate transform for the current body
      if (nargin<3) options=struct(); end
      if ~isfield(options,'urdfpath') options.urdfpath=[]; end
      
      wrlstr='';
      childNodes = node.getChildNodes();
      for i=1:childNodes.getLength()
        thisNode = childNodes.item(i-1);
        switch (lower(char(thisNode.getNodeName())))
          case 'box'
            s = parseParamString(model,robotnum,char(thisNode.getAttribute('size')));
            wrlstr=[wrlstr,sprintf('Shape {\n\tgeometry Box { size %f %f %f }\n\t%s}\n',s(1),s(2),s(3),wrl_appearance_str)];
            
          case 'cylinder'
            r = parseParamString(model,robotnum,char(thisNode.getAttribute('radius')));
            l = parseParamString(model,robotnum,char(thisNode.getAttribute('length')));
            
            % default axis for cylinder in urdf is the z-axis, but
            % the default in vrml is the y-axis. 
            wrlstr=[wrlstr,sprintf('Transform {\n\trotation 1 0 0 1.5708\n\tchildren Shape {\n\t\tgeometry Cylinder {\n\t\t\theight %f\n\t\t\tradius %f\n\t\t}\n\t\t%s\n\t}\n}\n',l,r,wrl_appearance_str)];
            
          case 'sphere'
            r = parseParamString(model,robotnum,char(thisNode.getAttribute('radius')));
            wrlstr=[wrlstr,sprintf('Shape {\n\tgeometry Sphere { radius %f }\n\t%s}\n',r,wrl_appearance_str)];

          case 'mesh'
            filename=char(thisNode.getAttribute('filename'));
            filename=RigidBody.parseMeshFilename(filename,options);
            [path,name,ext] = fileparts(filename);
            if strcmpi(ext,'.stl')
              wrlfile = fullfile(tempdir,[name,'.wrl']);
              stl2vrml(fullfile(path,[name,ext]),tempdir);
              txt=fileread(wrlfile);
              txt = regexprep(txt,'#.*\n','','dotexceptnewline');
              wrlstr=[wrlstr,sprintf('Shape {\n\tgeometry %s\n\t%s}\n',txt,wrl_appearance_str)];
%              wrlstr=[wrlstr,sprintf('Inline { url "%s" }\n',wrlfile)];
            elseif strcmpi(ext,'.wrl')
              txt = fileread(filename);
              txt = regexprep(txt,'#.*\n','','dotexceptnewline');
              wrlstr=[wrlstr,txt];
%              wrlstr=[wrlstr,sprintf('Inline { url "%s" }\n',filename)];
            end
            
          case {'#text','#comment'}
            % intentionally blank
          otherwise
            warning([char(thisNode.getNodeName()),' is not a supported element of robot/link/visual/material.']);
        end
      end
      
    end

    function [x,y,z,shape] = parseGeometry(node,x0,rpy,model,robotnum,options)
      % param node DOM node for the geometry block
      % param X coordinate transform for the current body
      % option twoD true implies that I can safely ignore y.
      x=[];y=[];z=[];
      T= [quat2rotmat(rpy2quat(rpy)),x0; 0 0 0 1];
      
      childNodes = node.getChildNodes();
      for i=1:childNodes.getLength()
        thisNode = childNodes.item(i-1);
        cx=[]; cy=[]; cz=[];
        switch (lower(char(thisNode.getNodeName())))
          case 'box'
            s = parseParamString(model,robotnum,char(thisNode.getAttribute('size')));
            shape = struct('type',RigidBody.BOX,'T',T,'params',s);  % s/2
            
            cx = s(1)/2*[-1 1 1 -1 -1 1 1 -1];
            cy = s(2)/2*[1 1 1 1 -1 -1 -1 -1];
            cz = s(3)/2*[1 1 -1 -1 -1 -1 1 1];
            
            pts = T(1:end-1,:)*[cx;cy;cz;ones(1,8)];
            x=pts(1,:)';y=pts(2,:)'; z=pts(3,:)';
            
          case 'cylinder'
            r = parseParamString(model,robotnum,char(thisNode.getAttribute('radius')));
            l = parseParamString(model,robotnum,char(thisNode.getAttribute('length')));
            shape = struct('type',RigidBody.CYLINDER,'T',T,'params',[r,l]);  % l/2
            
            % treat it as a box, for collisions
            warning('Drake:RigidBody:SimplifiedCollisionGeometry','for efficiency, cylinder geometry will be treated like a box for 3D collisions'); 
            cx = r*[-1 1 1 -1 -1 1 1 -1];
            cy = r*[1 1 1 1 -1 -1 -1 -1];
            cz = l/2*[1 1 -1 -1 -1 -1 1 1];
              
            pts = T(1:end-1,:)*[cx;cy;cz;ones(1,8)];
            x=pts(1,:)';y=pts(2,:)'; z=pts(3,:)';
              
          case 'sphere'
            r = parseParamString(model,robotnum,char(thisNode.getAttribute('radius')));
            shape = struct('type',RigidBody.SPHERE,'T',T,'params',r);
            if (r~=0)
              warning('Drake:RigidBody:SimplifiedCollisionGeometry','for efficiency, 3D sphere geometry will be treated like a point (at the center of the sphere)');
            end
            cx=0; cy=0; cz=0;
            pts = T(1:end-1,:)*[0;0;0;1];
            x=pts(1,:)';y=pts(2,:)'; z=pts(3,:)';

          case 'mesh'
            filename=char(thisNode.getAttribute('filename'));
            filename=RigidBody.parseMeshFilename(filename,options);
            [path,name,ext] = fileparts(filename);
            wrlfile=[];
            if strcmpi(ext,'.stl')
              wrlfile = fullfile(tempdir,[name,'.wrl']);
              stl2vrml(fullfile(path,[name,ext]),tempdir);
            elseif strcmpi(ext,'.wrl')
              wrlfile = filename;
            end
            if ~isempty(wrlfile)
              txt=fileread(wrlfile);

              ind=regexp(txt,'coordIndex[\s\n]*\[([^\]]*)\]','tokens'); ind = ind{1}{1};
              ind=strread(ind,'%d','delimiter',' ,')+1; 
              
              pts=regexp(txt,'point[\s\n]*\[([^\]]*)\]','tokens'); pts = pts{1}{1};
              pts=strread(pts,'%f','delimiter',' ,');
              pts=reshape(pts,3,[]);
              pts=T(1:end-1,:)*[pts;ones(1,size(pts,2))];
              x=pts(1,:)';y=pts(2,:)'; z=pts(3,:)';
            end
            shape = struct('type',RigidBody.MESH,'T',T,'params',pts);
           otherwise
            % intentionally blank
        end
      end
    end

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
end
