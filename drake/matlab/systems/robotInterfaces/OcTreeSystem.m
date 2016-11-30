classdef OcTreeSystem < DrakeSystem
  % Builds a system which takes a state estimate 
  % (of a rigid body manipulator) and a point cloud sensor
  % stream as inputs and writes it into an OcTree object
  
  properties
    manip;
    tree;
    sensor;
    range_percentage_threshold=.95; % we need to discard returns at the max range of the sensor (which are not real hits). this is the threshold percentage of total range above which to discard the points
  end
  
  methods
    function obj = OcTreeSystem(rigid_body_manipulator,octree)
      % @param rigid_body_manipulator a RigidBodyManipulator or
      % TimeSteppingRigidBodyManipulator object with at least one 
      % RigidBodyDepthSensor.
      % @param octree optional input describing the octree. can be 
      %   1) an existing OcTree object
      %   2) a filename of an octomap 
      %   3) the resolution of the new octree
      % @default creates a new octree with a resolution of 10cm

      typecheck(rigid_body_manipulator,{'RigidBodyManipulator','TimeSteppingRigidBodyManipulator'});
      if (rigid_body_manipulator.dirty)
        error('Drake:OcTreeSystem:ManipIsDirty','you must compile the manipulator before calling me');
      end
      
      if nargin<2 || isempty(octree)
        octree = OcTree(.1);
      elseif isa(octree,'char') || isa(octree,'double')
        octree = OcTree(octree);
      end
      typecheck(octree,'OcTree');
      
      sensor={};
      sensor_frame={};
      for i=1:length(rigid_body_manipulator.sensor)
        if isa(rigid_body_manipulator.sensor{i},'RigidBodyDepthSensor')
          sensor = horzcat(sensor,{rigid_body_manipulator.sensor{i}});
          sensor_frame = horzcat(sensor_frame,{rigid_body_manipulator.sensor{i}.coordinate_frame});
        end
      end
      input_frame = MultiCoordinateFrame.constructFrame(horzcat({rigid_body_manipulator.getPositionFrame},sensor_frame));

      % note that this system has hidden state because of the octree handle
      % object
      obj = obj@DrakeSystem(0,0,input_frame.dim,0,true,true);
      obj.manip = rigid_body_manipulator;
      obj.tree = octree;
      obj.sensor = sensor;
      obj = setInputFrame(obj,input_frame);
    end
    
    function enableLCMGL(obj,lcmgl_channel)
      enableLCMGL(obj.tree,lcmgl_channel);
    end
    
    function ts = getSampleTime(obj)
      ts = [-1;1];  % inherited sample time, fixed in minor offset
    end

    function y = output(obj,~,~,u)
      input = splitCoordinates(getInputFrame(obj),u); % {q,depth_sensor_1,depth_sensor_2,...}
      kinsol = doKinematics(obj.manip,input{1});
      for i=1:length(obj.sensor)
        points = reshape(input{i+1},3,[]);

        % don't add points that are at the sensor's max range
        dist = sum(points.^2,1);
        points = points(:,dist<obj.range_percentage_threshold*(obj.sensor{i}.range^2));
        
        % tranform to world coordinates
        points_in_world_frame = forwardKin(obj.manip,kinsol,obj.sensor{i}.frame_id,points);
        
        % add to the tree
        % note that this call produces a persistent change in the tree (even
        % though the object is not returned, because the tree is a handle
        % object)
        insertPointCloud(obj.tree,points_in_world_frame,forwardKin(obj.manip,kinsol,obj.sensor{i}.frame_id,zeros(3,1)));
      end
      publishLCMGL(obj.tree);  % will do nothing unless we've called enableLCMGL
      y=[];
    end
  end
    
end