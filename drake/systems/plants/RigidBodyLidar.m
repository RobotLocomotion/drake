classdef RigidBodyLidar < RigidBodyDepthSensor
  
  methods
    
    function obj = RigidBodyLidar(lidar_name, frame_id, min_yaw, max_yaw, num_pixels, range)
      % @param lidar_name name of  this lidar (as a string)
      % @param frame_id ID of the RigidBodyFrame
      % @param min_yaw minimum yaw angle (in radians) that the sensor
      %   reads
      % @param max_yaw maximum yaw angle (in radians) that the sensor
      %   reads
      % @param num_pixels number of pixels that the sensor reads
      % @param range maximum range of the laser (in meters).  can be inf.
      
      obj = obj@RigidBodyDepthSensor(lidar_name,frame_id,0,0,1,min_yaw,max_yaw,num_pixels,range);
    end
    
  end
  
end