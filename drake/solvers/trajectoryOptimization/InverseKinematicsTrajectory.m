classdef InverseKinematicsTrajectory < NonlinearProgram
% solve IK
%   min_q sum_i
%   qdd(:,i)'*Qa*qdd(:,i)+qd(:,i)'*Qv*qd(:,i)+(q(:,i)-q_nom(:,i))'*Q*(q(:,i)-q_nom(:,i))]+additional_cost1+additional_cost2+...
%   subject to
%          constraint1 at t_samples(i)
%          constraint2 at t_samples(i)
%          ...
%          constraint(k)   at [t_samples(1) t_samples(2) ... t_samples(nT)]
%          constraint(k+1) at [t_samples(1) t_samples(2) ... t_samples(nT)]
%   ....
%
% using q_seed_traj as the initial guess. q(1) would be fixed to
% q_seed_traj.eval(t(1))
% @param robot    -- A RigidBodyManipulator or a TimeSteppingRigidBodyManipulator
% @param t_knot   -- A 1 x nT double vector. The t_knot(i) is the time of the i'th knot
% point
% @param Q        -- The matrix that penalizes the posture error
% @param q_nom_traj    -- The nominal posture trajectory
% @param Qv       -- The matrix that penalizes the velocity
% @param Qa       -- The matrix that penalizes the acceleration
% @param rgc      -- A cell of RigidBodyConstraint
% @param x_name   -- A string cell, x_name{i} is the name of the i'th decision variable
% @param fix_initial_state   -- A boolean flag. Set to true if the initial q(:,1) is hold
% constant at q_seed_traj.eval(t_knot(1)) and qdot(:,1) =
% q_seed_traj.fnder(1).eval(t_knot(1)). Otherwise the initial state is free to change. The default value is false.
% @param q_idx    -- a nq x nT matrix. q(:,i) = x(q_idx(:,i))
% @param qd0_idx  -- a nq x 1 matrix. qdot0 = x(qd0_idx);
% @param qdf_idx  -- a nq x 1 matrix. qdotf = x(qdf_idx);
% @param qsc_weight_idx  -- a cell of vectors. x(qsc_weight_idx{i}) are the weights of the QuasiStaticConstraint at time t(i)
% @param rbm_joint_bnd_cnstr_id  The ID of the BoundingBoxConstraint that enforces the
% posture to be within the joint limits of the RigidBodyManipulator
  properties(SetAccess = protected)
    t_knot
    Q
    q_nom_traj
    Qv
    Qa
    rgc
    fix_initial_state
    q_idx
    qd0_idx
    qdf_idx
    qsc_weight_idx
    robot

    % nT-element vector of indices into the shared_data, where
    % shared_data{kinsol_dataind(i)} is the kinsol for knot point i
    kinsol_dataind
    rbm_joint_bnd_cnstr_id
  end

  properties(Access = protected)
    q_nom   % a nq x nT matrix. q_nom = q_nom_traj.eval(t)

    cpe   % A CubicPostureError object.
    nT    % Number of knot points
    nq    % Number of generalized positions
    t_kinsol % A 1 x nT boolean array. t_kinsol(i) is true if doKinematics should be called at time t_knot(i)
  end

  properties(Access = private)
    bbcon_initial_state_id % The ID of the BoundingBoxConstraint on the initial state
  end
  
  methods
    function obj = InverseKinematicsTrajectory(robot,t,q_nom_traj,fix_initial_state,x0,varargin)
      % obj =
      % InverseKinematicsTrajectory(robot,t,q_nom_traj,RigidBodyConstraint1,RigidBodyConstraint2,...,RigidBodyConstraintN)
      % @param robot    -- A RigidBodyManipulator or a TimeSteppingRigidBodyManipulator
      % @param t   -- A 1 x nT double vector. t(i) is the time of the i'th knot
      % point
      % @param q_nom_traj    -- The nominal posture trajectory
      % @param fix_initial_state    -- A boolean flag, true if the [q(:,1);qdot(:,q)] is
      % fixed to x0, and thus not a decision variable
      % @param x0                 -- A 2*obj.nq x 1 vector. If fix_initial_state = true,
      % then the initial state if fixed to x0, otherwise it is not used.
      % @param RigidBodyConstraint_i    -- A RigidBodyConstraint object
      t = unique(t(:)');
      nT = numel(t);
      nq = robot.getNumPositions();
      obj = obj@NonlinearProgram(nq*nT);
      obj.robot = robot;
      obj.t_knot = t;
      obj.nT = nT;
      obj.nq = nq;

      obj.x_name = cell(obj.nq*obj.nT,1);
      for i = 1:obj.nT
        for j = 1:obj.nq
          obj.x_name{(i-1)*obj.nq+j} = sprintf('q%d[%d]',j,i);
        end
      end

      qd_name = cell(2*obj.nq,1);
      for i = 1:obj.nq
        qd_name{i} = sprintf('qd%d[1]',i);
        qd_name{i+obj.nq} = sprintf('qd%d[%d]',i,obj.nT);
      end
      obj = obj.addDecisionVariable(2*obj.nq,qd_name);
      if(~isa(q_nom_traj,'Trajectory'))
        error('Drake:InverseKinematicsTrajectory:q_nom_traj should be a trajectory');
      end
      obj.q_nom_traj = q_nom_traj;
      obj.q_nom = obj.q_nom_traj.eval(obj.t_knot);
      obj.q_idx = reshape((1:obj.nq*obj.nT),obj.nq,obj.nT);
      obj.qd0_idx = obj.nq*obj.nT+(1:obj.nq)';
      obj.qdf_idx = obj.nq*(obj.nT+1)+(1:obj.nq)';
      sizecheck(fix_initial_state,[1,1]);
      obj = obj.setFixInitialState(fix_initial_state,x0);
      obj.qsc_weight_idx = cell(1,obj.nT);
      num_rbcnstr = nargin-5;
      [q_lb,q_ub] = obj.robot.getJointLimits();
      [obj,obj.rbm_joint_bnd_cnstr_id] = obj.addBoundingBoxConstraint(BoundingBoxConstraint(reshape(bsxfun(@times,q_lb,ones(1,obj.nT)),[],1),...
        reshape(bsxfun(@times,q_ub,ones(1,obj.nT)),[],1)),obj.q_idx(:));
      if(obj.fix_initial_state)
        t_start = 2;
      else
        t_start = 1;
      end

      % create shared data functions to calculate kinematics at the knot
      % points
      kinsol_dataind = zeros(obj.nT,1);
      for i=1:obj.nT,
        [obj,kinsol_dataind(i)] = obj.addSharedDataFunction(@obj.kinematicsData,{obj.q_idx(:,i)});
      end
      obj.kinsol_dataind = kinsol_dataind;

      for i = 1:num_rbcnstr
        if(~isa(varargin{i},'RigidBodyConstraint'))
          error('Drake:InverseKinematicsTrajectory:the input should be a RigidBodyConstraint');
        end
        if(isa(varargin{i},'SingleTimeKinematicConstraint'))
          for j = t_start:obj.nT
            if(varargin{i}.isTimeValid(obj.t_knot(j)))
              cnstr = varargin{i}.generateConstraint(obj.t_knot(j));
              obj = obj.addKinematicConstraint(cnstr{1},j);
            end
          end
        elseif(isa(varargin{i},'PostureConstraint'))
          for j = t_start:obj.nT
            if(varargin{i}.isTimeValid(obj.t_knot(j)))
              cnstr = varargin{i}.generateConstraint(obj.t_knot(j));
              obj = obj.addBoundingBoxConstraint(cnstr{1},obj.q_idx(:,j));
            end
          end
        elseif(isa(varargin{i},'QuasiStaticConstraint'))
          for j = t_start:obj.nT
            if(varargin{i}.isTimeValid(obj.t_knot(j)) && varargin{i}.active)
              if(~isempty(obj.qsc_weight_idx{j}))
                error('Drake:InverseKinematicsTrajectory:currently only support at most one QuasiStaticConstraint at an individual time');
              end
              cnstr = varargin{i}.generateConstraint(obj.t_knot(j));
              qsc_weight_names = cell(varargin{i}.num_pts,1);
              for k = 1:varargin{i}.num_pts
                qsc_weight_names{k} = sprintf('qsc_weight%d',k);
              end
              obj.qsc_weight_idx{j} = obj.num_vars+(1:varargin{i}.num_pts)';
              obj = obj.addDecisionVariable(varargin{i}.num_pts,qsc_weight_names);
              obj = obj.addConstraint(cnstr{1},{obj.q_idx(:,j);obj.qsc_weight_idx{j}},obj.kinsol_dataind(j));
              obj = obj.addLinearConstraint(cnstr{2},obj.qsc_weight_idx{j});
              obj = obj.addBoundingBoxConstraint(cnstr{3},obj.qsc_weight_idx{j});
            end
          end
        elseif(isa(varargin{i},'SingleTimeLinearPostureConstraint'))
          for j = t_start:obj.nT
            if(varargin{i}.isTimeValid(obj.t_knot(j)))
              cnstr = varargin{i}.generateConstraint(obj.t_knot(j));
              obj = obj.addLinearConstraint(cnstr{1},obj.q_idx(:,j));
            end
          end
        elseif(isa(varargin{i},'MultipleTimeKinematicConstraint'))
          valid_t_flag = varargin{i}.isTimeValid(obj.t_knot(t_start:end));
          t_idx = (t_start:obj.nT);
          valid_t_idx = t_idx(valid_t_flag);
          cnstr = varargin{i}.generateConstraint(obj.t_knot(valid_t_idx));
          obj = obj.addKinematicConstraint(cnstr{1},{[valid_t_idx]});
        elseif(isa(varargin{i},'MultipleTimeLinearPostureConstraint'))
          cnstr = varargin{i}.generateConstraint(obj.t_knot(t_start:end));
          obj = obj.addLinearConstraint(cnstr{1},reshape(obj.q_idx(:,t_start:end),[],1));
        end
      end
      obj.Q = eye(obj.nq);
      obj.Qv = 0*eye(obj.nq);
      obj.Qa = 1e-3*eye(obj.nq);
      obj = obj.setCubicPostureError(obj.Q,obj.Qv,obj.Qa);
      obj = obj.setSolverOptions('snopt','majoroptimalitytolerance',1e-4);
      obj = obj.setSolverOptions('snopt','superbasicslimit',2000);
      obj = obj.setSolverOptions('snopt','majorfeasibilitytolerance',1e-6);
      obj = obj.setSolverOptions('snopt','iterationslimit',10000);
      obj = obj.setSolverOptions('snopt','majoriterationslimit',200);
    end

    function data = kinematicsData(obj,q)
      data = doKinematics(obj.robot,q,false,false);
    end

    function obj = setCubicPostureError(obj,Q,Qv,Qa)
      % set the cost sum_i qdd(:,i)'*Qa*qdd(:,i)+qd(:,i)'*Qv*qd(:,i)+(q(:,i)-q_nom(:,i))'*Q*(q(:,i)-q_nom(:,i))]
      obj.Q = (Q+Q')/2;
      obj.Qv = (Qv+Qv')/2;
      obj.Qa = (Qa+Qa')/2;
      obj.cpe = CubicPostureError(obj.t_knot,obj.Q,obj.q_nom,obj.Qv,obj.Qa);
      if(isempty(obj.cost))
        xind = [obj.q_idx(:);obj.qd0_idx;obj.qdf_idx];
        obj = obj.addCost(obj.cpe,xind);
      else
        xind = [obj.q_idx(:);obj.qd0_idx;obj.qdf_idx];
        obj = obj.replaceCost(obj.cpe,1,xind);
      end
    end

    function obj = setFixInitialState(obj,flag,x0)
      % set obj.fix_initial_state = flag. If flag = true, then fix the initial state to x0
      % @param x0   A 2*obj.nq x 1 double vector. x0 = [q0;qdot0]. The initial state
      sizecheck(flag,[1,1]);
      flag = logical(flag);
      if(isempty(obj.bbcon))
        obj.fix_initial_state = flag;
        if(obj.fix_initial_state)
          [obj,obj.bbcon_initial_state_id] = obj.addBoundingBoxConstraint(BoundingBoxConstraint(x0,x0),[obj.q_idx(:,1);obj.qd0_idx]);
        else
          [obj,obj.bbcon_initial_state_id] = obj.addBoundingBoxConstraint(BoundingBoxConstraint(-inf(2*obj.nq,1),inf(2*obj.nq,1)),[obj.q_idx(:,1);obj.qd0_idx]);
        end
      elseif(obj.fix_initial_state ~= flag)
        obj.fix_initial_state = flag;
        if(obj.fix_initial_state)
          [obj,obj.bbcon_initial_state_id] = obj.updateBoundingBoxConstraint(obj.bbcon_initial_state_id,BoundingBoxConstraint(x0,x0),[obj.q_idx(:,1);obj.qd0_idx]);
        else
          [obj,obj.bbcon_initial_state_id] = obj.updateBoundingBoxConstraint(obj.bbcon_initial_state_id,BoundingBoxConstraint(-inf(2*obj.nq,1),inf(2*obj.nq,1)),[obj.q_idx(:,1);obj.qd0_idx]);
        end
      end
    end


    function [xtraj,F,info,infeasible_constraint] = solve(obj,qtraj_seed)
      % @param qtraj_seed.   A Trajectory object. The initial guess of posture trajectory.
      % @retval xtraj.       A Cubic spline trajectory. The solution of state trajectory,
      % x = [q;qdot]
      q_seed = qtraj_seed.eval(obj.t_knot);
      x0 = zeros(obj.num_vars,1);
      x0(obj.q_idx(:)) = q_seed(:);
      qd0 = (obj.x_lb(obj.qd0_idx)+obj.x_ub(obj.qd0_idx))/2;
      qd0(isnan(qd0)) = 0;
      x0(obj.qd0_idx) = qd0;
      qdf = (obj.x_lb(obj.qdf_idx)+obj.x_ub(obj.qdf_idx))/2;
      qdf(isnan(qdf)) = 0;
      x0(obj.qdf_idx) = qdf;
      for i = 1:length(obj.qsc_weight_idx)
        if(~isempty(obj.qsc_weight_idx{i}))
          x0(obj.qsc_weight_idx{i}) = 1/length(obj.qsc_weight_idx{i});
        end
      end
      [x,F,info,infeasible_constraint] = solve@NonlinearProgram(obj,x0);
      [q,qdot,qddot] = obj.cpe.cubicSpline(x([obj.q_idx(:);obj.qd0_idx;obj.qdf_idx]));
      q = max([q(:) reshape(obj.x_lb(obj.q_idx),[],1)],[],2);
      q = min([q(:) reshape(obj.x_ub(obj.q_idx),[],1)],[],2);
      q = reshape(q,obj.robot.getNumPositions,[]);
      xtraj = PPTrajectory(pchipDeriv(obj.t_knot,[q;qdot],[qdot;qddot]));
      xtraj = xtraj.setOutputFrame(obj.robot.getStateFrame);
    end

    function obj = addKinematicConstraint(obj,constraint,time_index)
      % Add a constraint that is a function of the generalized positions
      % at the specified time or times.
      % @param constraint  a Constraint object
      % @param time_index   a cell array of time indices
      %   ex1., time_index = {1, 2, 3} means the constraint is applied
      %   individually to knot points 1, 2, and 3
      %   ex2,. time_index = {[1 2], [3 4]} means the constraint is applied to knot
      %   points 1 and 2 together (taking the combined state as an argument)
      %   and 3 and 4 together.
      if ~iscell(time_index)
        time_index = num2cell(reshape(time_index,1,[]));
      end
      for j=1:length(time_index),
        kinsol_inds = obj.kinsol_dataind(time_index{j});
        cnstr_inds = mat2cell(obj.q_idx(:,time_index{j}),size(obj.q_idx,1),ones(1,length(time_index{j})));

        obj = obj.addConstraint(constraint,cnstr_inds,kinsol_inds);
      end
    end

  end
end
