classdef RelativeEuler < drakeFunction.kinematic.Kinematic
  % Euler angles roll, pitch and yaw (in intrinsic z-y'-x'' order) in frame A relative to frame B
  properties (SetAccess = private)
    frameA    % Frame id or body index of frame A
    frameB    % Frame id or body index of frame B
  end
  
  methods
    function obj = RelativeEuler(rbm,frameA,frameB)
      % obj = drakeFunction.kinematic.RelativeQuaternion(rbm,frameA,frameB)
      %   returns a RelativeEuler object that computes the
      %   euler angles that transforms directions in frame A to directions
      %   in frame B
      %   of the origin of frame A relative to frame B
      %
      % @param rbm        -- RigidBodyManipulator object
      % @param frameA     -- Body/frame name or frame id/body idx
      % @param frameB     -- Body/frame name or frame id/body idx
      %
      % @retval obj       -- RelativeQuaternion object
      obj = obj@drakeFunction.kinematic.Kinematic(rbm,drakeFunction.frames.realCoordinateSpace(3));
      obj.frameA = obj.rbm.parseBodyOrFrameID(frameA);
      obj.frameB = obj.rbm.parseBodyOrFrameID(frameB);
    end
    
    function [rpy,drpy] = eval(obj,q,kinsol)
      % rpy = eval(obj,kinsol) returns the relative euler
      %
      % [rpy,drpy] = eval(obj,kinsol) also returns the jacobian of the relative euler
      %
      % @param q        -- An obj.rbm.getNumPositions x 1 vector. The robot posture.
      % @param kinsol   -- A struct returned from RigidBodyManipulator.doKinematics function, that
      % stored the kinematics tree information
      [pos_A,J_A] = forwardKin(obj.rbm,kinsol,obj.frameA,[0;0;0],1);
      [pos_B,J_B] = forwardKin(obj.rbm,kinsol,obj.frameB,[0;0;0],1);
      rpy_A = pos_A(4:6);
      rpy_B = pos_B(4:6);
      [rotmat_A,drotmat_A] = rpy2rotmat(rpy_A);
      [rotmat_B,drotmat_B] = rpy2rotmat(rpy_B);
      drotmat_A_dq = drotmat_A*J_A(4:6,:);
      drotmat_B_dq = drotmat_B*J_B(4:6,:);
      R = rotmat_B\rotmat_A;
      drotmat_B_inv = invMatGrad(rotmat_B,drotmat_B_dq);
      dRdq = matGradMultMat(inv(rotmat_B),rotmat_A,drotmat_B_inv,drotmat_A_dq);
      [rpy,drpy] = rotmat2rpy(R,dRdq);
    end
    
  end
  
  methods (Access = protected)
    function joint_idx = kinematicsPathJoints(obj)
      if isempty(obj.frameA) || isempty(obj.frameB)
        joint_idx = kinematicsPathJoints@drakeFunction.kinematic.Kinematic(obj);
      else
        [~,joint_path] = obj.rbm.findKinematicPath(obj.frameA,obj.frameB);
        joint_idx = zeros(size(joint_path));
        for i = 1:numel(joint_path)
          joint_idx(i) = obj.rbm.getBody(joint_path(i)).dofnum;
        end
      end
    end
  end
end