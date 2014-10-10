classdef WorldEuler<drakeFunction.kinematic.RelativeEuler
  methods
    function obj = WorldEuler(rbm,frame)
      % compute the Euler angles of frame in the World frame
      % @param rbm A RigidBodyManipulator or a TimeSteppingRigidBodyManipulator object
      % @param frame  Body/frame name or frame id/body id
      obj = obj@drakeFunction.kinematic.RelativeEuler(rbm,frame,'world');
    end
    
    function [rpy,drpy] = eval(obj,q,kinsol)
      % rpy = eval(obj,kinsol) returns the euler
      %
      % [rpy,drpy] = eval(obj,kinsol) also returns the jacobian of the euler
      %
      % @param q        -- An obj.rbm.getNumPositions x 1 vector. The robot posture.
      % @param kinsol   -- A struct returned from RigidBodyManipulator.doKinematics function, that
      % stored the kinematics tree information
      [pos,dpos] = forwardKin(obj.rbm,kinsol,obj.frameA,[0;0;0],1);
      rpy = pos(4:6);
      drpy = dpos(4:6,:);
    end
  end
end