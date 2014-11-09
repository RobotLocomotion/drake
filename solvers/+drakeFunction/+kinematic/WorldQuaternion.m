classdef WorldQuaternion < drakeFunction.kinematic.RelativeQuaternion
  % quaternion in world frame
  methods
    function obj = WorldQuaternion(rbm,frame)
      % compute the quaternion that transforms directions in frameA to directions in world frame
      % @param rbm       -- RigidBodyManipulator or a TimeSteppingRigidBodyManipulator object
      % @param frame     -- Body/frame name or frame id/body id
      obj = obj@drakeFunction.kinematic.RelativeQuaternion(rbm,frame,'world');
    end
    
    function [quat,dquat] = eval(obj,q,kinsol)
      % quat = eval(obj,q) returns the quaternion
      %
      % [quat,dquat] = eval(obj,q) also returns the Jacobian of the
      %   quaternion
      %
      % @param obj    -- drakeFunction.kinematic.RelativeQuaternion object
      % @param q        -- An obj.rbm.getNumPositions x 1 vector. The robot posture.
      % @param kinsol -- A Kinsol struct returned from RigidBodyManipulator.doKinematics function,
      % stores the information about the kinematic tree
      if(nargin<3)
        kinsol = doKinematics(obj.rbm,q);
      end
      [pos,dpos] = forwardKin(obj.rbm,kinsol,obj.frameA,[0;0;0],2);
      quat = pos(4:7);
      dquat = dpos(4:7,:);
    end
  end
end