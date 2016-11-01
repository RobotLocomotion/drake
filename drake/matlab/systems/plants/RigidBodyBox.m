classdef RigidBodyBox < RigidBodyGeometry
  
  methods 
    function obj = RigidBodyBox(size,varargin)
      % obj = RigidBodyBox(size) constructs a RigidBodyBox object with
      % the geometry-to-body transform set to identity.
      %
      % obj = RigidBodyBox(size,T) constructs a RigidBodyBox object with
      % the geometry-to-body transform T.
      % 
      % obj = RigidBodyBox(size,xyz,rpy) constructs a RigidBodyBox
      % object with the geometry-to-body transform specified by the
      % position, xyz, and Euler angles, rpy.
      %
      % @param size - 3-element vector containing the box dimensions.
      % @param T - 4x4 homogenous transform from geometry-frame to
      %   body-frame
      % @param xyz - 3-element vector specifying the position of the
      %   geometry in the body-frame
      % @param rpy - 3-element vector of Euler angles specifying the
      %   orientation of the geometry in the body-frame
      obj = obj@RigidBodyGeometry(1,varargin{:});
      sizecheck(size,3);
      obj.size = size(:);
    end
    
    function pts = getPoints(obj)
      cx = obj.size(1)/2*[-1 1 1 -1 -1 1 1 -1];
      cy = obj.size(2)/2*[1 1 1 1 -1 -1 -1 -1];
      cz = obj.size(3)/2*[1 1 -1 -1 -1 -1 1 1];
      
      pts = obj.T(1:end-1,:)*[cx;cy;cz;ones(1,8)];
    end

    function pts = getBoundingBoxPoints(obj)
      pts = getPoints(obj);
    end
    
    function pts = getTerrainContactPoints(obj)
      % pts = getTerrainContactPoints(obj) returns the terrain contact points
      % of this geometry. The terrain contact points for a box are its corners. 
      %
      % For a general description of terrain contact points see 
      % <a href="matlab:help RigidBodyGeometry/getTerrainContactPoints">RigidBodyGeometry/getTerrainContactPoints</a>
      %
      % @param  obj - RigidBodySphere object
      % @retval pts - 3xm array of points on this geometry (in link frame) that
      %               can collide with the world.
      pts = getPoints(obj);
    end
    
    function geometry = serializeToLCM(obj)
      geometry = drake.lcmt_viewer_geometry_data();
      geometry.type = geometry.BOX;
      geometry.string_data = '';
      geometry.num_float_data = 3;
      geometry.float_data = obj.size;
      
      geometry.position = obj.T(1:3,4);
      geometry.quaternion = rotmat2quat(obj.T(1:3,1:3));
      geometry.color = [obj.c(:);1.0];
    end
    
    function writeWRLShape(obj,fp,td)
      function tabprintf(fp,varargin), for i=1:td, fprintf(fp,'\t'); end, fprintf(fp,varargin{:}); end

      tabprintf(fp,'Transform {\n'); td=td+1;
      tabprintf(fp,'translation %f %f %f\n',obj.T(1:3,4));
      tabprintf(fp,'rotation %f %f %f %f\n',rotmat2axis(obj.T(1:3,1:3)));

      tabprintf(fp,'children Shape {\n'); td=td+1;
      tabprintf(fp,'geometry Box { size %f %f %f }\n',obj.size);
      tabprintf(fp,'appearance Appearance { material Material { diffuseColor %f %f %f } }\n',obj.c(1),obj.c(2),obj.c(3));
      td=td-1; tabprintf(fp,'}\n');  % end Shape {
      td=td-1; tabprintf(fp,'}\n'); % end Transform {
    end

  end
  
  properties
    size  % 3x1 vector with x size, y size, z size
  end
  
end
