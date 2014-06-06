classdef RigidBodyLidar < RigidBodySensor
  % Uses collision detection framework to simulate a laser rangefinder.
  %
  %

  methods
    function obj = RigidBodyLidar(lidar_name, frame_id, min_yaw, max_yaw, num_pixels, range)
      % @param lidar_name name of  this lidar (as a string)
      % @param frame_id ID of the RigidBodyFrame
      % @param min_yaw minimum yaw angle (in raidians) that the sensor
      %   reads
      % @param max_yaw maximum yaw angle (in raidians) that the sensor
      %   reads
      % @param num_pixels number of pixels that the sensor reads
      % @param LIDAR range in meters
      
      obj.lidar_name = lidar_name;
      obj.frame_id = frame_id;
      obj.min_yaw = min_yaw;
      obj.max_yaw = max_yaw;
      obj.num_pixels = num_pixels;
      obj.range = range;
      
      checkDependency('lcmgl');
      obj.lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), obj.lidar_name);
      obj.lcmgl.glColor3f(1, 0, 0);
    end
    
    function distance = output(obj,manip,t,x,u)
      % Computes the output of the LIDAR
      % @param manip RigidBodyManipulator with collision geometries
      % @param t time (unused)
      % @param x system state
      % @param u control input (unused)
      %
      % @retval distance array of distances returned by the LIDAR, size
      %  will equal obj.num_pixels.
      
      % compute raycasting points
      
      kinsol = doKinematics(manip,x(1:getNumPositions(manip)));
      
      theta = linspace(obj.min_yaw,obj.max_yaw,obj.num_pixels);
      pts = forwardKin(manip,kinsol,obj.frame_id,[zeros(3,1),obj.range*[cos(theta);sin(theta);0*theta]]);
      
      origin = repmat(pts(:,1),1,obj.num_pixels);
      point_on_ray = pts(:,2:end);
      
      
      % TODO: vectorize
      
      distance = zeros(obj.num_pixels, 1);
      
      for (i=1:obj.num_pixels)
        
        distance(i) = collisionRaycast(manip, kinsol, origin(:,i), point_on_ray(:,i));
        
        point = forwardKin(manip,kinsol,obj.frame_id,distance(i)*[cos(theta(i));sin(theta(i));0]);
        obj.lcmgl.sphere(point, .1, 20, 20);
        obj.lcmgl.line3(origin(1), origin(2), origin(3), point(1), point(2), point(3));
      end
      
      obj.lcmgl.switchBuffers;
    end
    
    function fr = constructFrame(obj,manip)
      fr = CoordinateFrame(obj.lidar_name,numel(obj.num_pixels),'d');
    end
    
    function tf = isDirectFeedthrough(obj)
      tf=false;
    end
    
        
  end
  
   properties
    lidar_name; % name of this LIDAR as a string
    frame_id; % ID of the RigidBodyCoordinateFrame we are inside of
    min_yaw; % minimum yaw of the LIDAR sensor in radians
    max_yaw; % maximum yaw of the LIDAR sensor in radians
    num_pixels; % number of points the LIDAR returns on each run
    range; % range of the LIDAR in meters
    lcmgl; 
  end
  
end