classdef RelativePosition < drakeFunction.kinematic.Kinematic
  % Position of points in frame A relative to frame B
  % Class should work with planar systems, where the relavent dimension is
  % 2 instead of 3
  properties (SetAccess = private)
    frameA    % Frame id or body index of frame A
    frameB    % Frame id or body index of frame B
    n_pts     % Number of points
    % pts_in_A - [3 x n_pts] array.
    % pts_in_A(:,i) gives the coordinates % of the i-th point in frame A
    pts_in_A  
    ind % a subset of indices to use
  end

  methods
    function obj = RelativePosition(rbm,frameA,frameB,pts_in_A,indices)
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
        pts_in_A = zeros(rbm.dim,1);
      end
      if nargin < 5
        indices = 1:rbm.dim;
      end
      nC = length(indices);
      
%       sizecheck(pts_in_A,[rbm.dim,NaN]);
      n_pts_tmp = size(pts_in_A,2);
      dim_output = nC*n_pts_tmp;
      obj = obj@drakeFunction.kinematic.Kinematic(rbm, dim_output);
      obj.frameA = obj.rbm.parseBodyOrFrameID(frameA);
      if obj.frameA == 0
        valuecheck(pts_in_A,zeros(size(pts_in_A,1),n_pts_tmp));
      end
      obj.frameB = obj.rbm.parseBodyOrFrameID(frameB);
      obj.pts_in_A = pts_in_A;
      obj.n_pts = n_pts_tmp;
      obj = obj.setSparsityPattern();  % have to call this again because the frames weren't set before the parent class constructor was called
      obj.ind = indices;
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
      options.base_or_frame_id = obj.frameB;
      options.in_terms_of_qdot = true;
      use_mex = true;
      
      if compute_second_derivatives
        if obj.frameB == 1
          kinsol = obj.rbm.doKinematics(q,true,true);
        else
          kinsol = obj.rbm.doKinematics(q,true,false);
        end
        
        if obj.frameA == 0
          [pts_in_world,JA,dJA] = getCOM(obj.rbm,kinsol);
          [pts_in_B,P,JB,dP,dJB] = obj.rbm.bodyKin(kinsol,obj.frameB,pts_in_world);
          J = JB + P*JA;
          dJ = dJB + reshape(matGradMultMat(P,JA,dP,reshape(dJA,numel(JA),[])),size(dJB));
        else
          if obj.rbm.use_new_kinsol
            [pts_in_B,J,dJ] = forwardKin(obj.rbm,kinsol,obj.frameA,obj.pts_in_A,options);
          else
            [pts_in_world,JA,dJA] = forwardKin(obj.rbm,kinsol,obj.frameA,obj.pts_in_A,0);
            if obj.frameB == 1
              nq = length(q);
              pts_in_B = pts_in_world;
              numelpts = numel(pts_in_B);
              P = eye(numelpts);
              JB = zeros(numelpts,nq);
              dP = zeros(numelpts^2,nq);
              dJB = zeros(numelpts,nq^2);
            else
              [pts_in_B,P,JB,dP,dJB] = obj.rbm.bodyKin(kinsol,obj.frameB,pts_in_world);
            end
            J = JB + P*JA;
            dJ = dJB + reshape(matGradMultMat(P,JA,dP,reshape(dJA,numel(JA),[])),size(dJB));
          end
        end
      elseif (nargout>1)
        kinsol = obj.rbm.doKinematics(q,false,use_mex);
        if obj.frameA == 0
          [pts_in_world,JA] = getCOM(obj.rbm,kinsol);
          [pts_in_B,P,JB] = obj.rbm.bodyKin(kinsol,obj.frameB,pts_in_world);
          J = JB + P*JA;
        else
          [pts_in_B,J] = forwardKin(obj.rbm,kinsol,obj.frameA,obj.pts_in_A,options);
        end
      else
        kinsol = obj.rbm.doKinematics(q,false,use_mex);
        if obj.frameA == 0
          pts_in_world = getCOM(obj.rbm,kinsol);
          pts_in_B = obj.rbm.bodyKin(kinsol,obj.frameB,pts_in_world);
        else
          pts_in_B = forwardKin(obj.rbm,kinsol,obj.frameA,obj.pts_in_A,options);
        end
      end
      pos = reshape(pts_in_B,[],1);
      
            
      if obj.rbm.dim == 2 && size(obj.pts_in_A,1) == 3
        % convert to 2-D
        pos = obj.rbm.T_2D_to_3D'*pos;
        T_rep = kron(eye(size(pos,2)),obj.rbm.T_2D_to_3D');
        J = T_rep*J;
        
        if compute_second_derivatives
          dJ = T_rep*dJ;
        end
      end      
      
      pos = pos(obj.ind);
      if nargout > 1
        J = J(obj.ind,:);
      end
      if compute_second_derivatives
        dJ = dJ(obj.ind,:);
      end
    end
    
    function [pos,J,dJ,Jdotv,dJdotv] = evalWithJdot(obj,q,v)
      options.base_or_frame_id = obj.frameB;
      options.in_terms_of_qdot = true;
      
      kinsol = obj.rbm.doKinematics(q,v,struct('compute_gradients',true,'compute_JdotV',true));
      
      
      if obj.frameA == 0
        error('COM version of evalWithJdot is not implemented yet')
      end
%       if ~obj.rbm.use_new_kinsol
%         error('Requires new kinsol')
%       end  
      
      [pts_in_B,J,dJ] = forwardKin(obj.rbm,kinsol,obj.frameA,obj.pts_in_A,options);
      pos = reshape(pts_in_B,[],1);
      
      if obj.rbm.dim == 2 && size(obj.pts_in_A,1) == 2
        [Jdotv,dJdotvdq] = forwardJacDotTimesV(obj.rbm,kinsol,obj.frameA,obj.rbm.T_2D_to_3D*obj.pts_in_A,0,obj.frameB);        
        if obj.rbm.dim == 2
          Jdotv = obj.rbm.T_2D_to_3D'*Jdotv;
          dJdotvdq = obj.rbm.T_2D_to_3D'*dJdotvdq;
        end
      else
        [Jdotv,dJdotvdq] = forwardJacDotTimesV(obj.rbm,kinsol,obj.frameA,obj.pts_in_A,0,obj.frameB);
      end
      
      Jdot = reshape(reshape(dJ, numel(J), []) * kinsol.v, size(J));

%       Jdot = reshape(reshape(dJ',length(q),[])'*v,length(q),[])';
      dJdotvdv = 2*Jdot;
      dJdotv = [dJdotvdq dJdotvdv];
      
      if obj.rbm.dim == 2 && size(obj.pts_in_A,1) == 3
        % convert to 2-D
        pos = obj.rbm.T_2D_to_3D'*pos;
        T_rep = kron(eye(size(pos,2)),obj.rbm.T_2D_to_3D');
        J = T_rep*J;
        dJ = T_rep*dJ;
        Jdotv = T_rep*Jdotv;
        dJdotv = T_rep*dJdotv;
      end
      
      pos = pos(obj.ind);
      J = J(obj.ind,:);
      dJ = dJ(obj.ind,:);
      Jdotv = Jdotv(obj.ind);
      dJdotv = dJdotv(obj.ind,:);     
    end
  end

  methods (Access = protected)
    function joint_idx = kinematicsPathJoints(obj)
      if isempty(obj.frameA) || isempty(obj.frameB) || obj.frameA == 0
        joint_idx = kinematicsPathJoints@drakeFunction.kinematic.Kinematic(obj);
      else
        [~,joint_path] = obj.rbm.findKinematicPath(obj.frameA,obj.frameB);
        joint_idx = [];
        for i = 1:numel(joint_path)
          joint_idx = vertcat(joint_idx,obj.rbm.getBody(joint_path(i)).position_num);
        end
      end
    end
  end
end
