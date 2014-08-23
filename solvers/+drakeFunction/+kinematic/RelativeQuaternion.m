classdef RelativeQuaternion < drakeFunction.kinematic.Kinematic
  % Quaternion from frame A to frame B
  properties (SetAccess = private)
    frame_A    % Frame id or body index of frame A
    frame_B    % Frame id or body index of frame B
  end
  methods
    function obj = RelativeQuaternion(rbm,frame_A,frame_B)
      % obj = drakeFunction.kinematic.RelativeQuaternion(rbm,frameA,frameB)
      %   returns a RelativeQuaternion object that computes the
      %   quaternion that transforms directions in frame A to directions
      %   in frame B
      %   of the origin of frame A relative to frame B
      %
      % @param rbm        -- RigidBodyManipulator object
      % @param frameA     -- Body/frame name or frame id/body idx
      % @param frameB     -- Body/frame name or frame id/body idx
      %
      % @retval obj       -- RelativeQuaternion object
      obj = obj@drakeFunction.kinematic.Kinematic(rbm,drakeFunction.frames.R(4));
      obj.frame_A = obj.rbm.parseBodyOrFrameID(frame_A);
      obj.frame_B = obj.rbm.parseBodyOrFrameID(frame_B);
    end
    function [quat,dquat] = eval(obj,q)
      % quat = eval(obj,q) returns the relative quaternion
      %
      % [quat,dquat] = eval(obj,q) also returns the Jacobian of the relative
      %   quaternion
      %
      % @param obj  -- drakeFunction.kinematic.RelativeQuaternion object
      % @param q    -- Column vector of joint positions
      kinsol = obj.rbm.doKinematics(q);
      [pos_A,J_A] = forwardKin(obj.rbm,kinsol,obj.frame_A,[0;0;0],2);
      [pos_B,J_B] = forwardKin(obj.rbm,kinsol,obj.frame_B,[0;0;0],2);
      quat_a2w = pos_A(4:7,1);
      dquat_a2w = J_A(4:7,:);
      quat_b2w = pos_B(4:7,1);
      dquat_b2w = J_B(4:7,:);
      [quat_w2b,dquat_w2b] = quatConjugate(quat_b2w);
      dquat_w2b = dquat_w2b*dquat_b2w;

      [quat,dquat_a2b] = quatProduct(quat_w2b,quat_a2w);
      dquat = dquat_a2b*[dquat_w2b;dquat_a2w];
    end
  end

  methods (Access = protected)
    function joint_idx = kinematicsPathJoints(obj)
      [~,joint_path] = obj.rbm.findKinematicPath(obj.frameA,obj.frameB);
      joint_idx = zeros(size(joint_path));
      for i = 1:numel(joint_path)
        joint_idx(i) = obj.rbm.getBody(joint_path(i)).dofnum;
      end
    end
  end
end

