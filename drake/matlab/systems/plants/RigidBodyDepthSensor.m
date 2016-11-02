classdef RigidBodyDepthSensor < RigidBodySensor
  % Uses collision detection framework to simulate a depth camera rangefinder.
  %
  %

  methods
    function obj = RigidBodyDepthSensor(sensor_name, frame_id, min_pitch, max_pitch, num_pixel_rows, min_yaw, max_yaw, num_pixel_cols, range)
      % @param name name of this camera (as a string)
      % @param frame_id ID of the RigidBodyFrame
      % @param min_pitch minimum pitch angle (in radians) that the sensor
      %   reads
      % @param max_pitch maximum pitch angle (in radians) that the sensor
      %   reads
      % @param num_pixel_rows number of pixels that the sensor reads in the
      %   pitch axis
      % @param min_yaw minimum yaw angle (in radians) that the sensor
      %   reads
      % @param max_yaw maximum yaw angle (in radians) that the sensor
      %   reads
      % @param num_pixel_cols number of pixels that the sensor reads in the
      %   yaw axis
      % @param range maximum range of the sensor (in meters).  can be inf.

      obj.name = sensor_name;
      obj.frame_id = frame_id;
      obj.min_pitch = min_pitch;
      obj.max_pitch = max_pitch;
      obj.num_pixel_rows = num_pixel_rows;
      obj.min_yaw = min_yaw;
      obj.max_yaw = max_yaw;
      obj.num_pixel_cols = num_pixel_cols;
      obj.range = range;
      
      [theta_x,theta_z] = ndgrid(linspace(obj.min_yaw,obj.max_yaw,obj.num_pixel_cols), ...
        linspace(obj.min_pitch,obj.max_pitch,obj.num_pixel_rows));
      obj.body_pts = obj.range*[ cos(theta_x(:)).*cos(theta_z(:)), ...
        sin(theta_x(:)), ...
        -cos(theta_x(:)).*sin(theta_z(:))]';  % rolled out from roty(theta_z)*rotz(theta_x)*[1;0;0]
    end
    
    function obj = enableLCMGL(obj)
      checkDependency('lcmgl');
      obj.lcmgl = drake.matlab.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), obj.name);
    end
    
    function points = output(obj,manip,t,x,u)
      % Computes the output of the sensor
      % @param manip RigidBodyManipulator with collision geometry
      % @param t time (unused)
      % @param x system state
      % @param u control input (unused)
      %
      % @retval distance array of distances returned by the sensor, size
      %  will equal 3 x obj.num_pixel_rows x obj.num_pixel_cols.
      
      % compute raycasting points
      
      kinsol = doKinematics(manip,x(1:getNumPositions(manip)));
      pts = forwardKin(manip,kinsol,obj.frame_id,[zeros(3,1),obj.body_pts]);
      
      origin = repmat(pts(:,1),1,obj.num_pixel_rows*obj.num_pixel_cols);
      point_on_ray = pts(:,2:end);
      
      [distance, normals] = collisionRaycast(manip, kinsol, origin, point_on_ray);
      distance( distance<0 ) = obj.range;
      
      % return the points in the sensor frame
      points = (repmat(distance',3,1)/obj.range).*obj.body_pts;
      
      if ~isempty(obj.lcmgl)
        points_in_world_frame = forwardKin(manip,kinsol,obj.frame_id,points);
        obj.lcmgl.glColor3f(1, 0, 0);
        obj.lcmgl.points(points_in_world_frame(1,:),points_in_world_frame(2,:),points_in_world_frame(3,:));
%          obj.lcmgl.line3(origin(1), origin(2), origin(3), point(1,i), point(2,i), point(3,i));
        obj.lcmgl.switchBuffers;
      end
      
      points = points(:);
    end
    
    function fr = constructFrame(obj,manip)
      fr = CoordinateFrame(obj.name,3*obj.num_pixel_rows*obj.num_pixel_cols,'d');
    end
    
    function tf = isDirectFeedthrough(obj)
      tf=false;
    end
    
        
  end
  
   properties
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
