classdef InverseKin < NonlinearProgramWConstraint
  % solve the inverse kinematics problem
  % min_q 0.5*(q-qnom)'Q(q-qnom)+cost1(q)+cost2(q)+...
  % s.t    lb<= kc(q) <=ub
  %        q_lb<= q <=q_ub
  % where cost1, cost2 are functions defined by user
  % @param robot      -- A RigidBodyManipulator or a TimeSteppingRigidBodyManipulator
  % @param Q          -- A nq x nq double matrix, where nq is the DOF. Penalize the
  % posture error
  % @param q_nom      -- An nq x 1 double vector. The nominal posture.
  properties(SetAccess = protected)
    robot
    Q
    q_nom
    x_name
  end
  
  properties(Access = protected)
    nq
    q_idx   % q=x(q_idx), the robot posture
    qsc_weight_idx   % qsc_weight = x(qsc_weight_idx), the weight used in QuasiStaticConstraint
    pe   % A PostureError object.
    cost_call_kinsol  % A boolean array. cost_call_kinsol(i) is true if cost{i} calls eval with kinsol object
    nlcon_call_kinsol % A boolean array. nlcon_call_kinsol(i) is true if nlcon{i} calls eval with kinsol object
    cost_call_qsc_weight   % A boolean array. cost_call_qsc_weight(i) is true if cost{i} calls eval with qsc_weight
    nlcon_call_qsc_weight  % A boolean array. nlcon_call_qsc_weight(i) is true if nlcon{i} calls eval with qsc_weight
  end
  
  methods
    function obj = InverseKin(robot,q_nom,varargin)
      % InverseKin(robot,q_nom,RigidBodyConstraint1,RigidBodyConstraint2,...)
      % @param robot    -- A RigidBodyManipulator or a TimeSteppingRigidBodyManipulator
      % object
      % @param q_nom    -- A nq x 1 double vector. The nominal posture
      % @param RigidBodyConstraint_i  -- A RigidBodyConstraint object. Support
      % SingleTimeKinematicConstraint, PostureConstraint, QuasiStaticConstraint and
      % SingleTimeLinearPostureConstraint
      if(~isa(robot,'RigidBodyManipulator') && ~isa(robot,'TimeSteppingRigidBodyManipulator'))
        error('Drake:InverseKin:robot should be a RigidBodyManipulator or a TimeSteppingRigidBodyManipulator');
      end
      obj = obj@NonlinearProgramWConstraint(robot.getNumDOF);
      obj.robot = robot;
      obj.nq = obj.robot.getNumDOF;
      if(~isnumeric(q_nom))
        error('Drake:InverseKin:q_nom should be a numeric vector');
      end
      sizecheck(q_nom,[obj.nq,1]);
      obj.q_nom = q_nom;
      num_rbcnstr = nargin-2;
      t = [];
      obj.q_idx = (1:obj.nq)';
      obj.qsc_weight_idx = [];
      obj.x_name = cell(obj.nq,1);
      obj.cost_call_kinsol = [];
      obj.nlcon_call_kinsol = [];
      obj.cost_call_qsc_weight = [];
      obj.nlcon_call_qsc_weight = [];
      for i = 1:obj.nq
        obj.x_name{i} = sprintf('q%d',i);
      end
      [q_lb,q_ub] = obj.robot.getJointLimits();
      obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(q_lb,q_ub),obj.q_idx);
      for i = 1:num_rbcnstr
        if(~isa(varargin{i},'RigidBodyConstraint'))
          error('Drake:InverseKin:the input should be a RigidBodyConstraint');
        end
        if(isa(varargin{i},'SingleTimeKinematicConstraint'))
          cnstr = varargin{i}.generateConstraint(t);
          obj = obj.addNonlinearConstraint(cnstr{1},obj.q_idx);
          obj.nlcon_call_kinsol = [obj.nlcon_call_kinsol;true];
          obj.nlcon_call_qsc_weight = [obj.nlcon_call_qsc_weight;false];
        elseif(isa(varargin{i},'PostureConstraint'))
          cnstr = varargin{i}.generateConstraint(t);
          obj = obj.addBoundingBoxConstraint(cnstr{1},obj.q_idx);
        elseif(isa(varargin{i},'QuasiStaticConstraint'))
          obj.x_name = [obj.x_name;cell(varargin{i}.num_pts,1)];
          obj.qsc_weight_idx = obj.num_vars+(1:varargin{i}.num_pts)';
          for j = 1:varargin{i}.num_pts
            obj.x_name{obj.num_vars+j} = sprintf('qsc_weight%d',j);
          end
          obj = obj.addDecisionVariable(varargin{i}.num_pts);
          cnstr = varargin{i}.generateConstraint(t);
          obj = obj.addNonlinearConstraint(cnstr{1},[obj.q_idx;obj.qsc_weight_idx]);
          obj.nlcon_call_kinsol = [obj.nlcon_call_kinsol;true];
          obj.nlcon_call_qsc_weight = [obj.nlcon_call_qsc_weight;true];
          obj = obj.addLinearConstraint(cnstr{2},obj.qsc_weight_idx);
          obj = obj.addBoundingBoxConstraint(cnstr{3},obj.qsc_weight_idx);
        elseif(isa(varargin{i},'SingleTimeLinearPostureConstraint'))
          cnstr = varargin{i}.generateConstraint(t);
          obj = obj.addLinearConstraint(cnstr{1},obj.q_idx);
        else
          error('Drake:InverseKin:the input RigidBodyConstraint is not accepted');
        end
      end
      obj = obj.setQ(eye(obj.nq));
      obj = obj.setSolverOptions('snopt','majoroptimalitytolerance',1e-4);
      obj = obj.setSolverOptions('snopt','superbasicslimit',2000);
      obj = obj.setSolverOptions('snopt','majorfeasibilitytolerance',1e-6);
      obj = obj.setSolverOptions('snopt','iterationslimit',10000);
      obj = obj.setSolverOptions('snopt','majoriterationslimit',200);
    end
    
    function obj = setQ(obj,Q)
      % set the Q matrix in the cost function 0.5(q-q_nom)'Q(q-q_nom)
      % @param Q    -- An nq x nq double PSD matrix
      sizecheck(Q,[obj.nq,obj.nq]);
      obj.Q = Q;
      obj.pe = PostureError(obj.Q,obj.q_nom);
      if(isempty(obj.cost))
        obj = obj.addCost(obj.pe,obj.q_idx);
        obj.cost_call_kinsol = false;
        obj.cost_call_qsc_weight = false;
      else
        obj = obj.replaceCost(obj.pe,1,obj.q_idx);
        obj.cost_call_kinsol(1) = false;
        obj.cost_call_qsc_weight(1) = false;
      end
    end
    
    function [f,G] = objectiveAndNonlinearConstraints(obj,x)
      f = zeros(1+obj.num_nlcon,1);
      G = zeros(1+obj.num_nlcon,obj.num_vars);
      kinsol = doKinematics(obj.robot,x(obj.q_idx),false,false);
      for i = 1:length(obj.cost)
        if(obj.cost_call_kinsol(i)&&obj.cost_call_qsc_weight(i))
          [fi,dfi] = obj.cost{i}.eval(kinsol,x(obj.qsc_weight_idx));
        elseif(obj.cost_call_kinsol(i))
          [fi,dfi] = obj.cost{i}.eval(kinsol);
        else
          [fi,dfi] = obj.cost{i}.eval(x(obj.cost_xind_cell{i}));
        end
        f(1) = f(1)+fi;
        G(1,obj.cost_xind_cell{i}) = G(1,obj.cost_xind_cell{i})+dfi;
      end
      f_count = 1;
      for i = 1:length(obj.nlcon)
        if(obj.nlcon_call_kinsol(i)&&obj.nlcon_call_qsc_weight(i))
          [f(f_count+(1:obj.nlcon{i}.num_cnstr)),G(f_count+(1:obj.nlcon{i}.num_cnstr),obj.nlcon_xind{i})] = ...
          obj.nlcon{i}.eval(kinsol,x(obj.qsc_weight_idx));
        elseif(obj.nlcon_call_kinsol(i))
          [f(f_count+(1:obj.nlcon{i}.num_cnstr)),G(f_count+(1:obj.nlcon{i}.num_cnstr),obj.nlcon_xind{i})] = ...
          obj.nlcon{i}.eval(kinsol);
        else
          [f(f_count+(1:obj.nlcon{i}.num_cnstr)),G(f_count+(1:obj.nlcon{i}.num_cnstr),obj.nlcon_xind{i})] = ...
            obj.nlcon{i}.eval(x(obj.nlcon_xind{i}));
        end
        f(f_count+obj.nlcon{i}.ceq_idx) = f(f_count+obj.nlcon{i}.ceq_idx)-obj.nlcon{i}.ub(obj.nlcon{i}.ceq_idx);
        f_count = f_count+obj.nlcon{i}.num_cnstr;
      end
      f = [f(1);f(1+obj.nlcon_ineq_idx);f(1+obj.nlcon_eq_idx)];
      G = [G(1,:);G(1+obj.nlcon_ineq_idx,:);G(1+obj.nlcon_eq_idx,:)]; 
    end
    
    function [q,F,info,infeasible_constraint] = solve(obj,q_seed)
      x0 = zeros(obj.num_vars,1);
      x0(obj.q_idx) = q_seed;
      if(~isempty(obj.qsc_weight_idx))
        x0(obj.qsc_weight_idx) = 1/length(obj.qsc_weight_idx);
      end
      [x,F,info] = solve@NonlinearProgramWConstraint(obj,x0);
      q = x(obj.q_idx);
      q = max([obj.x_lb(obj.q_idx) q],[],2);
      q = min([obj.x_ub(obj.q_idx) q],[],2);
      [info,infeasible_constraint] = infeasibleConstraintName(obj,x,info);
    end

    function [info,infeasible_constraint] = infeasibleConstraintName(obj,x,info)
      % return the name of the infeasible nonlinear constraint
      % @retval info     -- change the return info from nonlinear solver based on how much
      % the solution violate the constraint
      % @retval infeasible_constraint  -- A cell of strings.
      infeasible_constraint = {};
      if(strcmp(obj.solver,'snopt'))
        if(info>10)
          fval = obj.objectiveAndNonlinearConstraints(x);
          A = [obj.Ain;obj.Aeq];
          if(~isempty(A))
            fval = [fval;A*x];
          end
          [lb,ub] = obj.bounds();
          ub_err = fval(2:end)-ub(2:end);
          max_ub_err = max(ub_err);
          max_ub_err = max_ub_err*(max_ub_err>0);
          lb_err = lb(2:end)-fval(2:end);
          max_lb_err = max(lb_err);
          max_lb_err = max_lb_err*(max_lb_err>0);
          cnstr_name = [obj.cin_name;obj.ceq_name;obj.Ain_name;obj.Aeq_name];
          if(max_ub_err+max_lb_err>1e-4)
            infeasible_constraint_idx = (ub_err>5e-5) | (lb_err>5e-5);
            infeasible_constraint = cnstr_name(infeasible_constraint_idx);
          elseif(info == 13)
            info = 4;
          elseif(info == 31)
            info = 5;
          elseif(info == 32)
            info = 6;
          end
        end
      else
        error('not implemented yet');
      end
    end
  end
end