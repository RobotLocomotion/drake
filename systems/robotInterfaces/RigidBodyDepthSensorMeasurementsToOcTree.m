classdef RigidBodyDepthSensorMeasurementsToOcTree < DrakeSystem
% a dynamical system which takes in depth sensor points and a 
% state estimate, and writes those points to an octree.
  
  properties (SetAccess=private)
    manip  % a RigidBodyManipulator (or TimeSteppingRigidBodyManipulator)
    octree % an OcTree object
  end

  methods
    function obj = RigidBodyDepthSensorMeasurementsToOcTree(manip,sensor_coordinateframe_name,octree)
      sensor_frame = getFrameByName(getOutputFrame(manip),sensor_coordinateframe_name);
      state_frame = getStateFrame(manip);
      
      obj = DrakeSystem(0,0,sensor_frame.dim+state_frame.dim,0,false,true);
      obj = setInputFrame(obj,MultiCoordinateFrame({sensor_frame,state_frame}));
    end
    
    function obj = enableLCMGL(obj,channel_name)
      enableLCMGL(obj.octree,channel_name);
    end
    
    function y=output(obj,t,~,u)
      y=[];
      [points,xhat] = splitCoordinates(getInputFrame(obj),u);
      points = reshape(points,3,[]);
      
      q = xhat(1:getNumPosition(obj.manip));
      kinsol = doKinematics(obj.manip,q);

      % todo: get sensor origin
      % todo: convert points to world frame
%      points = forwardKin(obj.manip,kinsol,obj.frame_id,(repmat(distance',3,1)/obj.range).*obj.body_pts);

      % todo: obj.tree.castRay();
      
      obj.tree.publishLCMGL();  % if it was enabled
    end
  end
  
end