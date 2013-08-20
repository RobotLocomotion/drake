classdef PlanarRigidBody < RigidBody
  
  properties 
    jcode=-1;        % for featherstone planar models
    jsign=1;
  end
  
  methods
    function obj = PlanarRigidBody()
      obj = obj@RigidBody();
      obj.I = zeros(3);
      obj.Xtree = eye(3);
      obj.Ttree = eye(3);
    end
    

    function body = parseVisual(body,node,model,options)
      xpts = [];
      ypts = [];
      c = .7*[1 1 1];
      
      xyz=zeros(3,1); rpy=zeros(3,1);
      origin = node.getElementsByTagName('origin').item(0);  % seems to be ok, even if origin tag doesn't exist
      if ~isempty(origin)
        if origin.hasAttribute('xyz')
          xyz = reshape(parseParamString(model,char(origin.getAttribute('xyz'))),3,1);
        end
        if origin.hasAttribute('rpy')
          rpy = reshape(parseParamString(model,char(origin.getAttribute('rpy'))),3,1);
        end
      end
        
      matnode = node.getElementsByTagName('material').item(0);
      if ~isempty(matnode)
        c = RigidBodyManipulator.parseMaterial(matnode,options);
      end
      
      geomnode = node.getElementsByTagName('geometry').item(0);
      if ~isempty(geomnode)
        [xpts,ypts] = PlanarRigidBody.parseGeometry(geomnode,xyz,rpy,model,options);
        % % useful for testing local geometry
        % h=patch(xpts,ypts,.7*[1 1 1]);
        % axis equal
        % pause;
        % delete(h);
      
        body.geometry{1}.x = xpts;
        body.geometry{1}.y = ypts;
        body.geometry{1}.c = c;
      end        
      
      body = parseVisual@RigidBody(body,node,model,options); % also parse wrl geometry
    end
    
    function body=parseInertial(body,node,model,options)
      mass = 0;
      inertia = 0;
      xyz=zeros(3,1); rpy=zeros(3,1);
      origin = node.getElementsByTagName('origin').item(0);  % seems to be ok, even if origin tag doesn't exist
      if ~isempty(origin)
        if origin.hasAttribute('xyz')
          xyz = reshape(parseParamString(model,char(origin.getAttribute('xyz'))),3,1);
        end
        if origin.hasAttribute('rpy')
          rpy = reshape(parseParamString(model,char(origin.getAttribute('rpy'))),3,1);
        end
      end
      massnode = node.getElementsByTagName('mass').item(0);
      if ~isempty(massnode)
        if (massnode.hasAttribute('value'))
          mass = parseParamString(model,char(massnode.getAttribute('value')));
        end
      end
      inode = node.getElementsByTagName('inertia').item(0);
      if ~isempty(inode)
        if isequal(options.view_axis,[1;0;0])
          if inode.hasAttribute('ixx'), inertia=parseParamString(model,char(inode.getAttribute('ixx'))); end
        elseif isequal(options.view_axis,[0;1;0])
          if inode.hasAttribute('iyy'), inertia=parseParamString(model,char(inode.getAttribute('iyy'))); end
        elseif isequal(options.view_axis,[0;0;1])
          if inode.hasAttribute('izz'), inertia=parseParamString(model,char(inode.getAttribute('izz'))); end
        else
          error('view not supported');
        end
      end      

      if any(rpy)
        error('rpy in inertia block not implemented yet (but would be easy)');
      end
      com = [options.x_axis'; options.y_axis']*xyz;
      body = setInertial(body,mass,com,inertia);
    end    
 
    function body=setInertial(body,varargin)  
      % setInertial(body,mass,com,inertia) or setInertial(body,spatialI)
      % this guards against things getting out of sync
      
      if nargin==2
        % extract mass, center of mass, and inertia information from the
        % spatial I matrix
        sizecheck(varargin{1},[3 3]);
        body.I = varargin{1};
        body.mass = body.I(3,3);
        body.com = [body.I(3,1);-body.I(2,1)]/body.mass;
        body.inertia = body.I(1,1) - body.mass*dot(body.com,body.com);
      elseif nargin==4
        sizecheck(varargin{1},1);
        sizecheck(varargin{2},[2 1]);
        sizecheck(varargin{3},1);
        body.mass = varargin{1};
        body.com = varargin{2};
        body.inertia = varargin{3};
        body.I = mcIp(body.mass,body.com,body.inertia);
      else
        error('wrong number of arguments');
      end
    end    
    
    function body = parseCollision(body,node,model,options)
      xyz=zeros(3,1); rpy=zeros(3,1);
      origin = node.getElementsByTagName('origin').item(0);  % seems to be ok, even if origin tag doesn't exist
      if ~isempty(origin)
        if origin.hasAttribute('xyz')
          xyz = reshape(parseParamString(model,char(origin.getAttribute('xyz'))),3,1);
        end
        if origin.hasAttribute('rpy')
          rpy = reshape(parseParamString(model,char(origin.getAttribute('rpy'))),3,1);
        end
      end
      
      npts = size(body.contact_pts,2);
      geomnode = node.getElementsByTagName('geometry').item(0);
      if ~isempty(geomnode)
        options.collision = true; 
        [xpts,ypts] = PlanarRigidBody.parseGeometry(geomnode,xyz,rpy,model,options);
        body.contact_pts=unique([body.contact_pts';xpts(:), ypts(:)],'rows')';
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
        body.collision_group{ind}=npts+1:size(body.contact_pts,2);
        body.contact_shape_group{ind} = [];  % not supported for planar case (yet)
      else
        body.collision_group{ind}=[body.collision_group{ind},npts+1:size(body.contact_pts,2)];
        body.contact_shape_group{ind} = [];  % not supported for planar case (yet)
      end
    end
    
  end
  
  methods (Static)
    
%     function body = extractFrom3D(3Dbody,options)
%       % uses the pose encoded in 3Dbody.T (e.g. from calling doKinematics)
%       % before planarizing
%       
%       error('not implemented correctly yet.  coming soon.');
%       
%       body = PlanarRigidBody();
%       
%       % first just copy over all of the fields
%       fn = fieldnames(3Dbody);
%       for i=1:length(fn)
%         body = setfield(body,fn{i},getfield(3Dbody,fn{i}));
%       end
% 
%       P = [options.x_axis',0; options.y_axis',0; zeros(1,3),1];
%       body.T = P*3Dbody.T;
%       body.Ttree = P*3Dbody.Ttree;
% 
%       P = [options.x_axis'; options.y_axis'];
%       for i=1:length(3Dbody.geometry)
%         x = P*3Dbody.T*[3Dbody.geometry{i}.x; 3Dbody.geometry{i}.y; 3Dbody.geometry{i}.z; ones(1,length(3Dbody.geomtry{i}.x))];
%         body.geometry{i}.x = x(1,:);
%         body.geometry{i}.y = y(1,:);
%         body.geometry{i} = rmfield(body.geometry{i},'z');
%       end
%       
%       X_3D_to_planar = [options.view_axis', zeros(1,3); zeros(1,3), options.x_axis'; zeros(1,3), options.y_axis'];
% 
%       body.I = zeros(3);
%       body.Xtree = body.Xtree;
%     end
    
    function [x,y] = parseGeometry(node,x0,rpy,model,options)
      % param node DOM node for the geometry block
      % param X coordinate transform for the current body
      % option twoD true implies that I can safely ignore y.
      x=[];y=[];
      T3= [quat2rotmat(rpy2quat(rpy)),x0]; % intentially leave off the bottom row [0,0,0,1];
      T = [options.x_axis'; options.y_axis']*T3;
      
      childNodes = node.getChildNodes();
      for i=1:childNodes.getLength()
        thisNode = childNodes.item(i-1);
        cx=[]; cy=[]; cz=[];
        switch (lower(char(thisNode.getNodeName())))
          case 'box'
            s = parseParamString(model,char(thisNode.getAttribute('size')));
            
            cx = s(1)/2*[-1 1 1 -1 -1 1 1 -1];
            cy = s(2)/2*[1 1 1 1 -1 -1 -1 -1];
            cz = s(3)/2*[1 1 -1 -1 -1 -1 1 1];
            
            pts = T*[cx;cy;cz;ones(1,8)];
            i = convhull(pts(1,:),pts(2,:));
            x=pts(1,i)';y=pts(2,i)';
            
          case 'cylinder'
            r = parseParamString(model,char(thisNode.getAttribute('radius')));
            l = parseParamString(model,char(thisNode.getAttribute('length')));
            
            if (abs(options.view_axis'*T3*[0;0;1;0]) < 1e-4 || ... % then it just looks like a box or
                (isfield(options,'collision') && options.collision)) % getting contacts, so use bb corners
              cx = r*[-1 1 1 -1 -1 1 1 -1];
              cy = r*[1 1 1 1 -1 -1 -1 -1];
              cz = l/2*[1 1 -1 -1 -1 -1 1 1];
              
              pts = T*[cx;cy;cz;ones(1,8)];
              i = convhull(pts(1,:),pts(2,:));
              x=pts(1,i)';y=pts(2,i)';
              
            elseif (abs(options.view_axis'*T3*[0;0;1;0]) > (1-1e-4)) % then it just looks like a circle
              theta = 0:0.1:2*pi;
              pts = r*[cos(theta); sin(theta)] + repmat(T*[0;0;0;1],1,length(theta));
              x=pts(1,:)';y=pts(2,:)';
            else  % full cylinder geometry
              error('full cylinder geometry not implemented yet');  % but wouldn't be hard
            end
            
          case 'sphere'
            r = parseParamString(model,char(thisNode.getAttribute('radius')));
            if (r==0)
                cx=0; cy=0; cz=0;
                pts = T*[0;0;0;1];
            elseif (isfield(options,'collision') && options.collision)
                pts = r*[-1 1 1 -1; 1 1 -1 -1] + repmat(T*[0;0;0;1],1,4);
            else
                theta = 0:0.1:2*pi;
                pts = r*[cos(theta); sin(theta)] + repmat(T*[0;0;0;1],1,length(theta));
            end
            x=pts(1,:)';y=pts(2,:)';


          case 'mesh'
            filename=char(thisNode.getAttribute('filename'));
            filename=RigidBody.parseMeshFilename(filename,options);
            [path,name,ext] = fileparts(filename);
            if strcmpi(ext,'.stl')
              wrlfile = fullfile(tempdir,[name,'.wrl']);
              stl2vrml(fullfile(path,[name,ext]),tempdir);
              txt=fileread(wrlfile);

              ind=regexp(txt,'coordIndex \[([^\]]*)\]','tokens'); ind = ind{1}{1};
              ind=strread(ind,'%d','delimiter',' ,')+1; 
              
              pts=regexp(txt,'point \[([^\]]*)\]','tokens'); pts = pts{1}{1};
              pts=strread(pts,'%f','delimiter',' ,');
              pts=reshape(pts,3,[]);

%              save testmesh.mat pts ind T;

              pts=T*[pts(1:3,:);ones(1,size(pts,2))];

              n=max(diff(find(ind==0)));
              if (min(diff(find(ind==0)))~=n), error('need to handle this case'); end
              ind = reshape(ind,n,[]); ind(end,:)=[];

              %% remove repeated indices (from 3D to 2D conversion)
%              [pts,ipts,iind]=unique(pts','rows'); pts=pts';
%              ind = iind(ind);

              x=pts(1,ind); y=pts(2,ind);
              i = convhull(x,y);
              x=x(i);y=y(i);

%              clf
%              h=patch(x,y,.7*[1 1 1]);%,'EdgeColor','none');
%              pause;
%              delete(h);
            end

            if strcmpi(ext,'.wrl')
              txt=fileread(filename);

              ind=regexp(txt,'coordIndex\s+\n*\s*\[([^\]]*)\]','tokens'); ind = ind{1}{1};
              ind=strread(ind,'%d','delimiter',' ,')+1; 
              
              pts=regexp(txt,'point\s+\n*\s*\[([^\]]*)\]','tokens'); pts = pts{1}{1};
              pts=strread(pts,'%f','delimiter',' ,');
              pts=reshape(pts,3,[]);

%              save testmesh.mat pts ind T;

              pts=T*[pts(1:3,:);ones(1,size(pts,2))];

              n=max(diff(find(ind==0)));
              if (min(diff(find(ind==0)))~=n), error('need to handle this case'); end
              ind = reshape(ind,n,[]); ind(end,:)=[];

              %% remove repeated indices (from 3D to 2D conversion)
%              [pts,ipts,iind]=unique(pts','rows'); pts=pts';
%              ind = iind(ind);

              x=pts(1,ind); y=pts(2,ind);
              i = convhull(x,y);
              x=x(i);y=y(i);
              
%              clf
%              h=patch(x,y,.7*[1 1 1]);%,'EdgeColor','none');
%              pause;
%              delete(h);
            end
            
          case {'#text','#comment'}
            % intentionally blank
          otherwise
            warning([char(thisNode.getNodeName()),' is not a supported element of robot/link/visual/material.']);
        end
      end


    end
  end
  
end
