classdef RigidBodyGeometry < RigidBodyElement

  methods % to be implemented in derived classes
    pts = getPoints(obj);             % returned in body coordinates
    pts = getBoundingBoxPoints(obj);  % returned in body coordinates
    lcmt_viewer_geometry_data = serializeToLCM(obj);
  end
  
  methods
    function obj = RigidBodyGeometry(drake_shape_id,varargin)
      % obj = RigidBodyGeometry(drake_shape_id) constructs a
      % RigidBodyGeometry object with the geometry-to-body transform set to
      % identity.
      %
      % obj = RigidBodyGeometry(drake_shape_id,T) constructs a
      % RigidBodyGeometry object with the geometry-to-body transform T.
      % 
      % obj = RigidBodyGeometry(drake_shape_id,xyz,rpy) constructs a
      % RigidBodyGeometry object with the geometry-to-body transform specified
      % by the position, xyz, and Euler angles, rpy.
      %
      % @param drake_shape_id - Integer that tells the DrakeCollision library
      % what type of geometry this is.
      % @param T - 4x4 homogenous transform from geometry-frame to body-frame
      % @param xyz - 3-element vector specifying the position of the geometry
      % in the body-frame
      % @param rpy - 3-element vector of Euler angles specifying the orientation of the
      % geometry in the body-frame
      if nargin < 2
        T_geometry_to_body = eye(4);
      elseif nargin < 3
        typecheck(varargin{1},'numeric');
        sizecheck(varargin{1},[4, 4]);
        T_geometry_to_body = varargin{1};
      else
        typecheck(varargin{1},'numeric');
        typecheck(varargin{2},'numeric');
        sizecheck(varargin{1},3);
        sizecheck(varargin{2},3);
        xyz = reshape(varargin{1},3,1);
        rpy = reshape(varargin{2},3,1);
        T_geometry_to_body = [rpy2rotmat(rpy), xyz; zeros(1,3),1];
      end
      obj.drake_shape_id = drake_shape_id;
      obj.T = T_geometry_to_body;
    end
    
    function [x,y,z,c] = getPatchData(obj,x_axis,y_axis,view_axis)
      % output is compatible with patch for 2D viewing
      % still returns a 3xn, but the z-axis is constant (just meant for
      % depth ordering in the 2d figure)
      
      Rview = [x_axis, y_axis, view_axis]';
      valuecheck(svd(Rview),[1;1;1]);  % assert that it's orthonormal
      
      pts = getPoints(obj);
      
      % project it into 2D frame
      pts = Rview*pts;
      
      % keep only convex hull (todo: do better here)
      ind = convhull(pts(1,:),pts(2,:));
      z = max(pts(3,:));
      
      % take it back out of view coordinates
      pts = Rview'*[pts(1:2,ind); repmat(z,1,length(ind))];
      x = pts(1,:)';
      y = pts(2,:)';
      z = pts(3,:)';
      c = obj.c;
    end

    function pts = getTerrainContactPoints(obj)
      % pts = getTerrainContactPoints(obj) returns the terrain contact points
      % on this object (in body-frame). This default implementation returns an
      % empty array, indicating that the object has no terrain contact points.
      %
      % Each sub-class of RigidBodyGeometry is responsible for overriding this
      % method if the geometry it defines has points which should be considered
      % terrain contact points.
      %
      % What are terrain contact points?
      %   Presently, DrakeCollision, our C++ collision detection library
      %   (powered by Bullet), only handles collisions with terrain for the
      %   case of flat terrain. For other terrain types (or for all terrain
      %   types on systems without Bullet) we look at the interaction between
      %   the terrain and specific set of points on the manipulator. These are
      %   refered to as "terrain contact points". The terrain contact points
      %   are the only ones that can collide with the terrain during
      %   simulation.
      %
      % @param  obj - RigidBodyGeometry object
      % @retval pts - 3xm array of points on this geometry (in body frame) that
      %               can collide with the world.
      %
      % See also RigidBodySphere/getTerrainContactPoints,
      % RigidBodyBox/getTerrainContactPoints
      pts = [];
    end
    
    function obj = setColor(obj, color)
      % Sets the color of the geometry for drawing
      %
      % @param color 3x1 color rgb array (defaults to [.7 .7 .7]
      %
      % @retval obj updated object
      
      obj.c = color;
      
    end
    
    function h = draw(obj,model,kinsol,body_ind)
      % intentionally left blank, can be overloaded
    end
  end
  
  methods (Static)
    function obj = parseURDFNode(node,x0,rpy,model,robotnum,options)
      T= [rpy2rotmat(rpy),x0; 0 0 0 1];
      
      childNodes = node.getChildNodes();
      for i=1:childNodes.getLength()
        thisNode = childNodes.item(i-1);
        switch (lower(char(thisNode.getNodeName())))
          case 'box'
            size = parseParamString(model,robotnum,char(thisNode.getAttribute('size')));
            obj = RigidBodyBox(size(:));
          case 'sphere'
            r = max(1e-7, parseParamString(model,robotnum,char(thisNode.getAttribute('radius'))));
            obj = RigidBodySphere(r);
          case 'cylinder'
            r = parseParamString(model,robotnum,char(thisNode.getAttribute('radius')));
            l = parseParamString(model,robotnum,char(thisNode.getAttribute('length')));
            obj = RigidBodyCylinder(r,l);  % l/2
          case 'mesh'
            filename=char(thisNode.getAttribute('filename'));
            scale = 1;
            if thisNode.hasAttribute('scale')
              scale = parseParamString(model,robotnum,char(thisNode.getAttribute('scale')));
            end
            
            % parse strings with forward and backward slashes
            filename = strrep(filename,'/',filesep);
            filename = strrep(filename,'\',filesep);

            if ~isempty(strfind(filename,['package:',filesep,filesep]))
              filename=strrep(filename,['package:',filesep,filesep],'');
              [package,filename]=strtok(filename,filesep);
              filename=[rospack(package),filename];
            else
              [path,name,ext] = fileparts(filename);
              if (isempty(path) || path(1)~=filesep)  % the it's a relative path
                path = fullfile(options.urdfpath,path);
              end
              filename = fullfile(path,[name,ext]);
            end
            
            obj = RigidBodyMesh(GetFullPath(filename));
            obj.scale = scale;
          case 'capsule'
            r = parseParamString(model,robotnum,char(thisNode.getAttribute('radius')));
            l = parseParamString(model,robotnum,char(thisNode.getAttribute('length')));
            obj = RigidBodyCapsule(r,l);  % l/2
        end
        obj.T = T;
      end
    end

    function obj = parseSDFNode(node,x0,rpy,model,robotnum,options)
      T= [rpy2rotmat(rpy),x0; 0 0 0 1];
      
      obj=[];
      childNodes = node.getChildNodes();
      for i=1:childNodes.getLength()
        thisNode = childNodes.item(i-1);
        switch (lower(char(thisNode.getNodeName())))
          case 'box'
            size_node = thisNode.getElementsByTagName('size').item(0);
            size = parseParamString(model,robotnum,char(getNodeValue(getFirstChild(size_node))));
            obj = RigidBodyBox(size);
          case 'mesh'
            uriNode = thisNode.getElementsByTagName('uri').item(0);
            filename = char(getNodeValue(getFirstChild(uriNode)));

            scale = 1;
            scaleNode = thisNode.getElementsByTagName('scale').item(0);
            if ~isempty(scaleNode)
              scale = parseParamString(model,robotnum,char(getNodeValue(getFirstChild(scaleNode))));
            end
            
            % parse strings with forward and backward slashes
            filename = strrep(filename,'/',filesep);
            filename = strrep(filename,'\',filesep);
            
            if ~isempty(strfind(filename,['model:',filesep,filesep]))
              filename=strrep(filename,['model:',filesep,filesep],'');
              [model,filename]=strtok(filename,filesep);
              filename=[gazeboModelPath(model),filename];
            else
              strrep(filename,'file://','');
              if ~isempty(strfind(filename,'http://'))
                error('urls not supported yet -- but might not be too hard');
              end
              [path,name,ext] = fileparts(filename);
              if (isempty(path) || path(1)~=filesep)  % the it's a relative path
                path = fullfile(options.urdfpath,path);
              end
              filename = fullfile(path,[name,ext]);
            end
            obj = RigidBodyMesh(GetFullPath(filename));
            obj.scale = scale;
          case 'plane'
            size_node = thisNode.getElementsByTagName('size').item(0);
            size = parseParamString(model,robotnum,char(getNodeValue(getFirstChild(size_node))));
            normal_node = thisNode.getElementsByTagName('normal').item(0);
            normal = parseParamString(model,robotnum,char(getNodeValue(getFirstChild(normal_node))));
            
            pts = diag([size/2,0])*[ 1 1 -1 -1; 1 -1 -1 1; 0 0 0 0];
            
            % now rotate [0;0;1] onto the normal
            % http://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
            v = [ -normal(2); normal(1); 0];
            s = norm(v); c = normal(3);
            if abs(s)<eps^2
              R = diag([sign(c),1,sign(c)]);
            else
              vx = [ 0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
              R = eye(3) + vx + vx*vx*(1-c)/(s^2);
            end
            obj = RigidBodyMeshPoints(R*pts);            
          case {'#text','#comment'}
            % intentionally do nothing
          otherwise
            model.warning_manager.warnOnce(['Drake:RigidBodyGeometry:UnsupportedGeometry:',char(thisNode.getNodeName())],['SDF geometry ',char(thisNode.getNodeName()),' not implemented yet']);
        end
      end
      if ~isempty(obj)
        obj.T = T;
      end
    end  

  end
  
  properties  % note: constructModelmex currently depends on these being public
    T = eye(4);  % coordinate transform (from geometry to link coordinates)
                 % look like:
                 % [    R         x  ]
                 % [ zeros(1,3)   1  ]
                 %
                 % where R is a 3x3 rotation matrix
                 % and x is a 3x1 vector denoting position
                 %
                 % if, for example, you want to set the position, you
                 % would do
                 % obstacle1 = RigidBodyCylinder(1, 10);
                 % obstacle1.T(1:3, 4) = [ 10; 0; 0 ];
                 
                 
                 
    c = [.7 .7 .7];  % 3x1 color
    drake_shape_id = 0;  % UNKNOWN
    name % String identifier
  end
end
