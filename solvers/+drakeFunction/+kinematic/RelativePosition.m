classdef RelativePosition < drakeFunction.kinematic.Kinematic
  % Position of points in frame A relative to frame B
  properties (SetAccess = private)
    frameA    % Frame id or body index of frame A
    frameB    % Frame id or body index of frame B
    n_pts     % Number of points
    % pts_in_A - [3 x n_pts] array. 
    % pts_in_A(:,i) gives the coordinates % of the i-th point in frame A
    pts_in_A  
  end

  methods
    function obj = RelativePosition(rbm,frameA,frameB,pts_in_A)
      % obj = drakeFunction.kinematic.RelativePosition(rbm,frameA, ...
      %                                                frameB,pts_in_A)
      %   returns a RelativePosition object that computes the position
      %   of the given points in frame A relative to frame B
      %
      % obj = drakeFunction.kinematic.RelativePosition(rbm,frameA,frameB)
      %   returns a RelativePosition object that computes the position
      %   of the origin of frame A relative to frame B
      %
      % @param rbm        -- RigidBodyManipulator object
      % @param frameA     -- Body/frame name or frame id/body idx
      % @param frameB     -- Body/frame name or frame id/body idx
      % @param pts_in_A   -- [3 x n_pts] array. pts_in_A(:,i) gives the
      %                      coordinates of the i-th point in frame A.
      %                      Optional. @default [0;0;0]
      %
      % @retval obj       -- RelativePosition object

      if nargin < 4
        pts_in_A = zeros(3,1);
      end
      sizecheck(pts_in_A,[3,NaN]);
      n_pts_tmp = size(pts_in_A,2);
      output_frame = MultiCoordinateFrame.constructFrame( ...
        repmat({drakeFunction.frames.realCoordinateSpace(3)},1,n_pts_tmp));
      obj = obj@drakeFunction.kinematic.Kinematic(rbm,output_frame);
      obj.frameA = obj.rbm.parseBodyOrFrameID(frameA);
      if obj.frameA == 0
        valuecheck(pts_in_A,zeros(3,1));
      end
      obj.frameB = obj.rbm.parseBodyOrFrameID(frameB);
      obj.pts_in_A = pts_in_A;
      obj.n_pts = n_pts_tmp;
    end

    function [pos,J] = eval(obj,q)
      % pos = eval(obj,q) returns the relative positions of the points
      %
      % [pos,J] = eval(obj,q) also returns the Jacobian of the relative
      %   positions
      %
      % @param obj  -- drakeFunction.kinematic.RelativePosition object
      % @param q    -- Column vector of joint positions
      kinsol = obj.rbm.doKinematics(q);
      if obj.frameA == 0
        [pts_in_world,JA] = getCOM(obj.rbm,kinsol);
      else
        [pts_in_world,JA] = forwardKin(obj.rbm,kinsol,obj.frameA,obj.pts_in_A,0);
      end
      [T_B_to_world,dT_B_to_world] = forwardKin(obj.rbm,kinsol,obj.frameB,[0;0;0],2);
      [quat_world_to_B,dquat_world_to_B] = quatConjugate(T_B_to_world(4:7));
      dquat_world_to_B = dquat_world_to_B*dT_B_to_world(4:7,:);
      [xyz_world_to_B,dxyz_world_to_B] = quatRotateVec(quat_world_to_B,T_B_to_world(1:3));
      xyz_world_to_B = -xyz_world_to_B;
      dxyz_world_to_B = -dxyz_world_to_B*[dquat_world_to_B;dT_B_to_world(1:3,:)];

      pts_in_B = zeros(3,obj.n_pts)*q(1);
      J = zeros(3*obj.n_pts,obj.rbm.getNumPositions())*q(1);
      for i = 1:obj.n_pts
        [pts_in_B1,dpts_in_B1] = quatRotateVec(quat_world_to_B,pts_in_world(:,i));
        dpts_in_B1 = dpts_in_B1*[dquat_world_to_B;JA(3*(i-1)+(1:3),:)];
        pts_in_B(:,i) = pts_in_B1+xyz_world_to_B;
        J(3*(i-1)+(1:3),:) = dpts_in_B1+dxyz_world_to_B;
      end
      pos = reshape(pts_in_B,[],1);
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
