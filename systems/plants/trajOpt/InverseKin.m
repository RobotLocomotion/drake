classdef InverseKin < NonlinearProgramWConstraintObjects
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
    q_idx   % q=x(q_idx), the robot posture
    qsc_weight_idx   % qsc_weight = x(qsc_weight_idx), the weight used in QuasiStaticConstraint
  end
  
  properties(Access = protected)
    nq
    pe   % A PostureError object.
    cost_call_kinsol  % A boolean array. cost_call_kinsol(i) is true if cost{i} calls eval with kinsol object
    nlcon_call_kinsol % A boolean array. nlcon_call_kinsol(i) is true if nlcon{i} calls eval with kinsol object
    cost_nonkinsol_idx   % A cell of vectors. cost_nonkinsol_idx{i} is the indices of variables, apart from kinsol, that are used for evaluating obj.cost{i}
    nlcon_nonkinsol_idx  % A cell of vectors. cost_nonkinsol_idx{i} is the indices of variables, apart from kinsol, that are used for evaluating obj.nlcon{i}
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
      obj = obj@NonlinearProgramWConstraintObjects(robot.getNumDOF);
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
      obj.cost_nonkinsol_idx= {};
      obj.nlcon_nonkinsol_idx = {};
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
          obj = obj.addNonlinearConstraint(cnstr{1},true,[],obj.q_idx);
        elseif(isa(varargin{i},'PostureConstraint'))
          cnstr = varargin{i}.generateConstraint(t);
          obj = obj.addBoundingBoxConstraint(cnstr{1},obj.q_idx);
        elseif(isa(varargin{i},'QuasiStaticConstraint'))
          if(varargin{i}.active)
            obj.qsc_weight_idx = obj.num_vars+(1:varargin{i}.num_pts)';
            qsc_weight_names = cell(varargin{i}.num_pts,1);
            for j = 1:varargin{i}.num_pts
              qsc_weight_names{j} = sprintf('qsc_weight%d',j);
            end
            obj = obj.addDecisionVariable(varargin{i}.num_pts,qsc_weight_names);
            cnstr = varargin{i}.generateConstraint(t);
            obj = obj.addNonlinearConstraint(cnstr{1},true,obj.qsc_weight_idx,[obj.q_idx;obj.qsc_weight_idx]);
            obj = obj.addLinearConstraint(cnstr{2},obj.qsc_weight_idx);
            obj = obj.addBoundingBoxConstraint(cnstr{3},obj.qsc_weight_idx);
          end
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
        obj = obj.addCost(obj.pe,false,[],obj.q_idx);
      else
        obj = obj.replaceCost(obj.pe,1,false,[],obj.q_idx);
      end
    end
    
    function [f,G] = objectiveAndNonlinearConstraints(obj,x)
      f = zeros(1+obj.num_nlcon,1);
      G = zeros(1+obj.num_nlcon,obj.num_vars);
      kinsol = doKinematics(obj.robot,x(obj.q_idx),false,false);
      for i = 1:length(obj.cost)
        if(obj.cost_call_kinsol(i))
          if(isempty(obj.cost_nonkinsol_idx{i}))
            [fi,dfi] = obj.cost{i}.eval(kinsol);
          else
            [fi,dfi] = obj.cost{i}.eval(kinsol,x(obj.cost_nonkinsol_idx{i}));
          end
        else
          [fi,dfi] = obj.cost{i}.eval(x(obj.cost_xind_cell{i}));
        end
        f(1) = f(1)+fi;
        G(1,obj.cost_xind_cell{i}) = G(1,obj.cost_xind_cell{i})+dfi;
      end
      f_count = 1;
      for i = 1:length(obj.nlcon)
        if(obj.nlcon_call_kinsol(i))
          if(isempty(obj.nlcon_nonkinsol_idx{i}))
            [f(f_count+(1:obj.nlcon{i}.num_cnstr)),G(f_count+(1:obj.nlcon{i}.num_cnstr),obj.nlcon_xind{i})] = ...
              obj.nlcon{i}.eval(kinsol);
          else
            [f(f_count+(1:obj.nlcon{i}.num_cnstr)),G(f_count+(1:obj.nlcon{i}.num_cnstr),obj.nlcon_xind{i})] = ...
              obj.nlcon{i}.eval(kinsol,x(obj.nlcon_nonkinsol_idx{i}));  
          end
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
      [x,F,info] = solve@NonlinearProgramWConstraintObjects(obj,x0);
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
    
    function obj = addNonlinearConstraint(obj,cnstr,call_kinsol,non_kinsol_idx,xind)
      % @param cnstr     -- A NonlinearConstraint object
      % @param xind      -- Optional argument. The x(xind) is the decision variables used
      % in evaluating the cnstr. Default value is (1:obj.num_vars)
      % @param call_kinsol  -- A boolean variable. True if the new constraint uses 'kinsol'
      % in 'eval' function; false otherwise.
      % @param non_kinsol_idx    -- A vector. The indices of decision variables, apart from kinsol,
      % that are used for evaluating the constraint. If only kinsol is used, then
      % non_kinsol_idx = [];
      if(nargin<5)
        xind = (1:obj.num_vars)';
      end
      obj = addNonlinearConstraint@NonlinearProgramWConstraintObjects(obj,cnstr,xind);
      sizecheck(call_kinsol,[1,1]);
      obj.nlcon_call_kinsol = [obj.nlcon_call_kinsol;logical(call_kinsol)];
      if(~isnumeric(non_kinsol_idx))
        error('Drake:InverseKin:addNonlinearConstraint:non_kinsol_idx should be numeric');
      end
      non_kinsol_idx = floor(non_kinsol_idx(:));
      if(any(non_kinsol_idx<1 | non_kinsol_idx>obj.num_vars))
        error('Drake:InverseKin:addNonlinearConstraint:non_kinsol_idx out of range');
      end
      obj.nlcon_nonkinsol_idx = [obj.nlcon_nonkinsol_idx,{non_kinsol_idx}];
    end
    
    function obj = addCost(obj,cost,call_kinsol,non_kinsol_idx,xind)
      % @param cnstr   -- A NonlinearConstraint or a LinearConstraint
      % @param xind      -- Optional argument. x(xind) is the decision variables used in
      % evaluating the cost. Default value is (1:obj.num_vars)
      % @param call_kinsol  -- A boolean variable. True if the new cost uses 'kinsol'
      % in 'eval' function; false otherwise.
      % @param non_kinsol_idx    -- A vector. The indices of decision variables, apart from kinsol,
      % that are used for evaluating the cost. If only kinsol is used, then
      % non_kinsol_idx = [];
      if(nargin<5)
        xind = (1:obj.num_vars)';
      end
      obj = addCost@NonlinearProgramWConstraintObjects(obj,cost,xind);
      sizecheck(call_kinsol,[1,1]);
      obj.cost_call_kinsol = [obj.cost_call_kinsol;logical(call_kinsol)];
      if(~isnumeric(non_kinsol_idx))
        error('Drake:InverseKin:addCost:non_kinsol_idx should be numeric');
      end
      non_kinsol_idx = floor(non_kinsol_idx(:));
      if(any(non_kinsol_idx<1 | non_kinsol_idx>obj.num_vars))
        error('Drake:InverseKin:addCost:non_kinsol_idx out of range');
      end
      obj.cost_nonkinsol_idx = [obj.cost_nonkinsol_idx,{non_kinsol_idx}];
    end
    
    function obj = replaceCost(obj,cost,cost_idx,call_kinsol,non_kinsol_idx,xind)
      % @param cost     -- A Constraint object, currently accepts NonlinearConstraint and
      % LinearConstraint
      % @param cost_idx -- The index of the original cost to be replaced
      % @param xind     -- Optional argument. x(xind) is the decision variables used in
      % evaluating the cost. Default value is (1:obj.num_vars)
      % @param call_kinsol  -- A boolean variable. True if the new cost uses 'kinsol'
      % in 'eval' function; false otherwise.
      % @param non_kinsol_idx    -- A vector. The indices of decision variables, apart from kinsol,
      % that are used for evaluating the cost. If only kinsol is used, then
      % non_kinsol_idx = [];
      if(nargin<6)
        xind = (1:obj.num_vars)';
      end
      xind = xind(:);
      obj.iFfun = [];
      obj.jFvar = [];
      num_cost = length(obj.cost);
      sizecheck(cost_idx,[1,1]);
      if(cost_idx>num_cost || cost_idx<1)
        error('Drake:InverseKin:replaceCost:cost_idx is out of range');
      end
      cost_tmp = obj.cost;
      cost_tmp{cost_idx} = cost;
      cost_xind_tmp = obj.cost_xind_cell;
      cost_xind_tmp{cost_idx} = xind;
      cost_call_kinsol_tmp = obj.cost_call_kinsol;
      cost_call_kinsol_tmp(cost_idx) = call_kinsol;
      cost_nonkinsol_idx_tmp = obj.cost_nonkinsol_idx;
      cost_nonkinsol_idx_tmp{cost_idx} = non_kinsol_idx;
      obj.cost = {};
      obj.cost_xind_cell = {};
      obj.cost_call_kinsol = [];
      obj.cost_nonkinsol_idx = {};
      for i = 1:num_cost
        obj = obj.addCost(cost_tmp{i},cost_call_kinsol_tmp(i),cost_nonkinsol_idx_tmp{i},cost_xind_tmp{i});
      end      
    end
    
    function obj = addDecisionVariable(obj,num_new_vars,var_names)
      % appending new decision variables to the end of the current decision variables
      % @param num_new_vars      -- An integer. The newly added decision variable is an
      % num_new_vars x 1 double vector.
      % @param var_names         -- A cell of strings. var_names{i} is the name of the
      % i'th new decision variable
      if(nargin<3)
        var_names = cell(num_new_vars,1);
        for i = 1:num_new_vars
          var_names{i} = sprintf('x%d',obj.num_vars+i);
        end
      else
        if(~iscellstr(var_names))
          error('Drake:InverseKin:addDecisionVariable:var_names should be a cell of strings');
        end
      end
      obj = addDecisionVariable@NonlinearProgramWConstraintObjects(obj,num_new_vars);
      obj.x_name = [obj.x_name;var_names];
    end
  end
end