classdef RigidBodyDepthCamera < RigidBodySensor
  % Uses collision detection framework to simulate a laser rangefinder.
  %
  %

  methods
    function obj = RigidBodyDepthCamera(camera_name, frame_id, min_pitch, max_pitch, num_pixel_rows, min_yaw, max_yaw, num_pixel_cols, range)
      % @param name name of this camera (as a string)
      % @param frame_id ID of the RigidBodyFrame
      % @param min_yaw minimum yaw angle (in radians) that the sensor
      %   reads
      % @param max_yaw maximum yaw angle (in radians) that the sensor
      %   reads
      % @param num_pixels number of pixels that the sensor reads
      % @param range maximum range of the laser (in meters).  can be inf.
      
      obj.name = name;
      obj.frame_id = frame_id;
      obj.min_pitch = min_pitch;
      obj.max_pitch = max_pitch;
      obj.num_pixel_rows = num_pixel_rows;
      obj.min_yaw = min_yaw;
      obj.max_yaw = max_yaw;
      obj.num_pixel_cols = num_pixel_cols;
      obj.range = range;
      
      [row_theta,col_theta] = ndgrid(linspace(obj.min_yaw,obj.max_yaw,obj.num_pixel_cols), ...
        linspace(obj.min_pitch,obj.max_pitch,obj.num_pixel_rows));
      obj.body_pts = obj.range*[ cos(col_theta(:))*cos(row_theta(:)), ...
        sin(col_theta(:)), ...
        -cos(col_theta(:))*sin(row_theta(:))]';  % rolled out from roty(row_theta)*rotz(col_theta)*[1;0;0]
    end
    
    function obj = enableLCMGLOutput(obj)
      checkDependency('lcmgl');
      obj.lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), obj.name);
    end
    
    function distance = output(obj,manip,t,x,u)
      % Computes the output of the sensor
      % @param manip RigidBodyManipulator with collision geometries
      % @param t time (unused)
      % @param x system state
      % @param u control input (unused)
      %
      % @retval distance array of distances returned by the sensor, size
      %  will equal obj.num_pixel_rows x obj.num_pixel_cols.
      
      % compute raycasting points
      
      kinsol = doKinematics(manip,x(1:getNumPositions(manip)));
      pts = forwardKin(manip,kinsol,obj.frame_id,[zeros(3,1),obj.body_pts]);
      
      origin = repmat(pts(:,1),1,obj.num_pixels);
      point_on_ray = pts(:,2:end);
      
      distance = collisionRaycast(manip, kinsol, origin, point_on_ray);
      distance( distance<0 ) = obj.range;
      
      if ~isempty(obj.lcmgl)
        obj.lcmgl.glColor3f(1, 0, 0);
        point = forwardKin(manip,kinsol,obj.frame_id,(distance/obj.range).*obj.body_pts);
        
        % todo: vectorize this
        for (i=1:size(point,2))
          obj.lcmgl.sphere(point, .025, 20, 20);
          obj.lcmgl.line3(origin(1), origin(2), origin(3), point(i,1), point(i,2), point(i,3));
        end
      end
      obj.lcmgl.switchBuffers;
    end
    
    function fr = constructFrame(obj,manip)
      fr = CoordinateFrame(obj.name,obj.num_pixels,'d');
    end
    
    function tf = isDirectFeedthrough(obj)
      tf=false;
    end
    
        
  end
  
   properties
    name; % name of this sensor as a string
    frame_id; % ID of the RigidBodyCoordinateFrame we are inside of
    min_pitch; % minimum pitch of the camera FOV in radians
    max_pitch; % maximum pitch of the camera FOV in radians
    min_yaw; % minimum yaw of the sensor FOV in radians
    max_yaw; % maximum yaw of the sensor FOV in radians
    num_pixel_rows; % number of points in the image vertically (pitch)
    num_pixel_cols; % number of points in the image horizontally (yaw)
    range; % range of the sensor in meters
    body_pts; % internal cache of the points to scan (in the sensor frame)
    lcmgl; 
  end
  
end