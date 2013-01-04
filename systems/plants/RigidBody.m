classdef RigidBody < handle
  
  properties
    % link properties
    linkname=''  % name of the associated link
    wrlgeometry=''; % geometry (compatible w/ wrl).  see parseVisual below.
    geometry={}  % geometry (compatible w/ patch).  see parseVisual below.
    dofnum=0     % the index in the state vector corresponding to this joint
    contact_pts=[];  % a 3xn matrix with [x;y;z] positions of contact points
    gravity_off=false;
    
    % joint properties
    jointname=''
    parent
    pitch=0;        % for featherstone 3D models
    joint_axis=[1;0;0]; 
    Xtree=eye(6);   % velocity space coordinate transform *from parent to this node*
    X_joint_to_body=eye(6);  % velocity space coordinate transfrom from joint frame (where joint_axis = z-axis) to body frame 
    Ttree=eye(4);   % position space coordinate transform *from this node to parent*
    T_body_to_joint=eye(4);
    wrljoint='';  % tranformation to joint coordinates in wrl syntax
    damping=0;
    joint_limit_min=[];
    joint_limit_max=[];
    effort_limit=[];
    velocity_limit=[];
    
    % dynamic properties (e.g. state at runtime)
    cached_q=[];  % the current kinematics were computed using these q and qd values 
    cached_qd=[]
    T = eye(4);  % transformation from this body to world coordinates
    dTdq = [];
    ddTdqdq = [];
    Tdot = [];
    dTdotdqqd = [];
  end

  properties (SetAccess=protected, GetAccess=public)    
    % mass, com, inertia properties
    I=zeros(6);  % spatial mass/inertia
    mass = 0;
    com = [];
    inertia = [];
  end
  
  methods    
    
    function varargout = forwardKin(varargin)
      error('forwardKin(body,...) has been replaced by forwardKin(model,body_num,...), because it has a mex version.  please update your kinematics calls');
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
    
    function body=parseInertial(body,node,options)
      mass = 0;
      inertia = eye(3);
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
      massnode = node.getElementsByTagName('mass').item(0);
      if ~isempty(massnode)
        if (massnode.hasAttribute('value'))
          mass = str2num(char(massnode.getAttribute('value')));
        end
      end
      inode = node.getElementsByTagName('inertia').item(0);
      if ~isempty(inode)
        if inode.hasAttribute('ixx'), inertia(1,1)=str2num(char(inode.getAttribute('ixx'))); end
        if inode.hasAttribute('ixy'), inertia(1,2)=str2num(char(inode.getAttribute('ixy'))); inertia(2,1)=inertia(1,2); end
        if inode.hasAttribute('ixz'), inertia(1,3)=str2num(char(inode.getAttribute('ixz'))); inertia(3,1)=inertia(1,3); end
        if inode.hasAttribute('iyy'), inertia(2,2)=str2num(char(inode.getAttribute('iyy'))); end
        if inode.hasAttribute('iyz'), inertia(2,3)=str2num(char(inode.getAttribute('iyz'))); inertia(3,2)=inertia(2,3); end
        if inode.hasAttribute('izz'), inertia(3,3)=str2num(char(inode.getAttribute('izz'))); end
      end      

      if any(rpy)
        error('rpy in inertia block not implemented yet (but would be easy)');
      end
      setInertial(body,mass,xyz,inertia);
    end
    
    function setInertial(body,varargin)
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
          xyz = reshape(str2num(char(origin.getAttribute('xyz'))),3,1);
        end
        if origin.hasAttribute('rpy')
          rpy = reshape(str2num(char(origin.getAttribute('rpy'))),3,1);
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
        c = parseMaterial(model,matnode,options);
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
        wrl_shape_str = [wrl_shape_str,RigidBody.parseWRLGeometry(geomnode,wrl_appearance_str,options)];
        if (options.visual_geometry)
          [xpts,ypts,zpts] = RigidBody.parseGeometry(geomnode,xyz,rpy,options);
          body.geometry{1}.x = xpts;
          body.geometry{1}.y = ypts;
          body.geometry{1}.z = zpts;
          body.geometry{1}.c = c;
        end
      end        
      
      if isempty(wrl_transform_str)
        body.wrlgeometry = wrl_shape_str;
      else
        body.wrlgeometry = [wrl_transform_str,wrl_shape_str,'\t]\n}'];
      end
    end
    
    function body = parseCollision(body,node,options)
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
      
      % note: could support multiple geometry elements
      geomnode = node.getElementsByTagName('geometry').item(0);
      if ~isempty(geomnode)
        [xpts,ypts,zpts] = RigidBody.parseGeometry(geomnode,xyz,rpy,options);
        body.contact_pts=unique([body.contact_pts';xpts(:), ypts(:), zpts(:)],'rows')';
      end
    end
      
    function b=leastCommonAncestor(body1,body2)
      % recursively searches for the lowest body in the tree that is an
      % ancestor to both body1 and body2

      b=body2;
      if (body1==body2) return; end
      
      % check if body1 is an ancestor to body2
      while (~isempty(b.parent))
        b=b.parent;
        if (body1==b) return; end
      end
      
      % body1 is not an ancestor to body2.  check body1's parent (and
      % recurse)
      b = leastCommonAncestor(body1.parent,body2);
    end
    
    function writeWRLJoint(body,fp)
      if isempty(body.jointname)
        fprintf(fp,'Transform {\n');
      else
          % get rid of dots in joint name
          body.jointname = regexprep(body.jointname, '\.', '_', 'preservecase');

        fprintf(fp,'DEF %s Transform {\n',body.jointname); 
      end
      if (body.pitch==0) % then it's a pin joint 
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
    function wrlstr = parseWRLGeometry(node,wrl_appearance_str,options)
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
            s = str2num(char(thisNode.getAttribute('size')));
            wrlstr=[wrlstr,sprintf('Shape {\n\tgeometry Box { size %f %f %f }\n\t%s}\n',s(1),s(2),s(3),wrl_appearance_str)];
            
          case 'cylinder'
            r = str2num(char(thisNode.getAttribute('radius')));
            l = str2num(char(thisNode.getAttribute('length')));
            
            % default axis for cylinder in urdf is the z-axis, but
            % the default in vrml is the y-axis. 
            wrlstr=[wrlstr,sprintf('Transform {\n\trotation 1 0 0 1.5708\n\tchildren Shape {\n\t\tgeometry Cylinder {\n\t\t\theight %f\n\t\t\tradius %f\n\t\t}\n\t\t%s\n\t}\n}\n',l,r,wrl_appearance_str)];
            
          case 'sphere'
            r = str2num(char(thisNode.getAttribute('radius')));
            wrlstr=[wrlstr,sprintf('Shape {\n\tgeometry Sphere { radius %f }\n\t%s}\n',r,wrl_appearance_str)];

          case 'mesh'
            filename=char(thisNode.getAttribute('filename'));
            [path,name,ext] = fileparts(filename);
            path = strrep(path,'package://','');
            if (path(1)~=filesep)  % the it's a relative path
              path = fullfile(options.urdfpath,path);
            end
            filename = fullfile(path,[name,ext]);
            if strcmpi(ext,'.stl')
              wrlfile = fullfile(tempdir,[name,'.wrl']);
              stl2vrml(fullfile(path,[name,ext]),tempdir);
              txt=fileread(wrlfile);
              [~,txt]=strtok(txt,'DEFI');
              wrlstr=[wrlstr,sprintf('Shape {\n\tgeometry %s\n\t%s}\n',txt,wrl_appearance_str)];
%              wrlstr=[wrlstr,sprintf('Inline { url "%s" }\n',wrlfile)];
            elseif strcmpi(ext,'.wrl')
              txt = fileread(filename);
              [~,txt]=strtok(txt,'DEF');
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

    function [x,y,z] = parseGeometry(node,x0,rpy,options)
      % param node DOM node for the geometry block
      % param X coordinate transform for the current body
      % option twoD true implies that I can safely ignore y.
      x=[];y=[];z=[];
      T= [quat2rotmat(rpy2quat(rpy)),x0]; % intentially leave off the bottom row [0,0,0,1];
      
      childNodes = node.getChildNodes();
      for i=1:childNodes.getLength()
        thisNode = childNodes.item(i-1);
        cx=[]; cy=[]; cz=[];
        switch (lower(char(thisNode.getNodeName())))
          case 'box'
            s = str2num(char(thisNode.getAttribute('size')));
            
            cx = s(1)/2*[-1 1 1 -1 -1 1 1 -1];
            cy = s(2)/2*[1 1 1 1 -1 -1 -1 -1];
            cz = s(3)/2*[1 1 -1 -1 -1 -1 1 1];
            
            pts = T*[cx;cy;cz;ones(1,8)];
            x=pts(1,:)';y=pts(2,:)'; z=pts(3,:)';
            
          case 'cylinder'
            r = str2num(char(thisNode.getAttribute('radius')));
            l = str2num(char(thisNode.getAttribute('length')));
            
            % treat it as a box, for collisions
            warning('for efficiency, cylinder geometry will be treated like a box for 3D collisions'); 
            cx = r*[-1 1 1 -1 -1 1 1 -1];
            cy = r*[1 1 1 1 -1 -1 -1 -1];
            cz = l/2*[1 1 -1 -1 -1 -1 1 1];
              
            pts = T*[cx;cy;cz;ones(1,8)];
            x=pts(1,:)';y=pts(2,:)'; z=pts(3,:)';
              
          case 'sphere'
            r = str2num(char(thisNode.getAttribute('radius')));
            if (r~=0)
              warning('for efficiency, 3D sphere geometry will be treated like a point (at the center of the sphere)');
            end
            cx=0; cy=0; cz=0;
            pts = T*[0;0;0;1];
            x=pts(1,:)';y=pts(2,:)'; z=pts(3,:)';

          case 'mesh'
            filename=char(thisNode.getAttribute('filename'));
            [path,name,ext] = fileparts(filename);
            path = strrep(path,'package://','');
            if (path(1)~=filesep)  % the it's a relative path
              path = fullfile(options.urdfpath,path);
            end
            filename = fullfile(path,[name,ext]);
            if strcmpi(ext,'.stl')
              wrlfile = fullfile(tempdir,[name,'.wrl']);
              stl2vrml(fullfile(path,[name,ext]),tempdir);
              txt=fileread(wrlfile);

              ind=regexp(txt,'coordIndex \[([^\]]*)\]','tokens'); ind = ind{1}{1};
              ind=strread(ind,'%d','delimiter',' ,')+1; 
              
              pts=regexp(txt,'point \[([^\]]*)\]','tokens'); pts = pts{1}{1};
              pts=strread(pts,'%f','delimiter',' ,');
              pts=reshape(pts,3,[]);
              x=pts(1,:)';y=pts(2,:)'; z=pts(3,:)';
            end

           otherwise
            % intentionally blank
        end
      end
    end
  end
end
