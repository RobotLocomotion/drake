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

    function [pos,J,dJ] = eval(obj,q)
      % pos = eval(obj,q) returns the relative positions of the points
      %
      % [pos,J] = eval(obj,q) also returns the Jacobian of the relative
      %   positions
      %
      % [pos,J,dJ] = eval(obj,q) also returns the second-derivaties of
      %   the relative positions
      %
      % @param obj  -- drakeFunction.kinematic.RelativePosition object
      % @param q    -- Column vector of joint positions
      %
      % @retval pos -- [3 x n_pts] array containing the positions of
      %                pts_in_A relative to frameB
      % @retval J   -- [3*n_pts x nq] Jacobian of pos w.r.t. q
      % @retval dJ  -- [3*n_pts x nq^2] second-derivatives of pos w.r.t.
      %                q in geval format
      compute_second_derivatives = (nargout > 2);
      if compute_second_derivatives
        kinsol = obj.rbm.doKinematics(q,true,false);
        if obj.frameA == 0
          [pts_in_world,JA,dJA] = getCOM(obj.rbm,kinsol);
        else
          [pts_in_world,JA,dJA] = forwardKin(obj.rbm,kinsol,obj.frameA,obj.pts_in_A,0);
        end
        [pts_in_B,P,JB,dP,dJB] = obj.rbm.bodyKin(kinsol,obj.frameB,pts_in_world);
        dJ = dJB + reshape(matGradMultMat(P,JA,dP,reshape(dJA,numel(JA),[])),size(dJB));
      else
        kinsol = obj.rbm.doKinematics(q);
        if obj.frameA == 0
          [pts_in_world,JA] = getCOM(obj.rbm,kinsol);
        else
          [pts_in_world,JA] = forwardKin(obj.rbm,kinsol,obj.frameA,obj.pts_in_A,0);
        end
        [pts_in_B,P,JB] = obj.rbm.bodyKin(kinsol,obj.frameB,pts_in_world);
      end
      J = JB + P*JA;
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
