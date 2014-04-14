classdef NonlinearProgramWKinsol < NonlinearProgramWConstraintObjects
  % This class will be used for ik, trajectory optimization, etc. It uses 'kinsol' for
  % evaluating nonlinear constraints
  properties(SetAccess = protected)
    robot
    x_name
  end
  properties(Access=protected)
    nq
    nT
    t_kinsol % A 1 x nT boolean array. t_kinsol(i) is true if doKinematics should be called at time t_knot(i)
    cost_kinsol_idx % A cell. cost_kinsol_idx{i} is the indices of kinsol used for evaluating cost{i}
    cost_nonkinsol_idx % A cell. cost_nonkinsol_idx{i} is the indices of decision variables, aprat from kinsol, that are used in evaluating cost{i}
    nlcon_kinsol_idx % A cell. nlcon_kinsol_idx{i} is the indices of kinsol used for evaluating nlcon{i}
    nlcon_nonkinsol_idx % A cell. nlcon_nonkinsol_idx{i] is the indices of decision variables, apart from kinsol, that are used in evaluating nlcon{i}
  end
  
  methods
    function obj = NonlinearProgramWKinsol(robot,nT)
      if(~isa(robot,'RigidBodyManipulator') && ~isa(robot,'TimeSteppingRigidBodyManipulator'))
        error('Drake:NonlinearProgramWKinsol:robot should be a RigidBodyManipulator or a TimeSteppingRigidBodyManipulator');
      end
      obj = obj@NonlinearProgramWConstraintObjects(robot.getNumDOF*nT);
      obj.robot = robot;
      obj.nq = robot.getNumDOF;
      obj.nT = nT;
      obj.x_name = cell(obj.nq*obj.nT,1);
      for i = 1:obj.nT
        for j = 1:obj.nq
          obj.x_name{(i-1)*obj.nq+j} = sprintf('q%d[%d]',j,i);
        end
      end
      obj.t_kinsol = false(1,obj.nT);
      obj.cost_kinsol_idx = {};
      obj.cost_nonkinsol_idx = {};
      obj.nlcon_kinsol_idx = {};
      obj.nlcon_nonkinsol_idx = {};
    end
    
    function [f,G] = objectiveAndNonlinearConstraints(obj,x)
      kinsol_cell = cell(1,obj.nT);
      for i = 1:obj.nT
        if obj.t_kinsol(i)
          kinsol_cell{i} = obj.robot.doKinematics(x(obj.q_idx(:,i)),false,false);
        end
      end
      f = zeros(1+obj.num_nlcon,1);
      G = zeros(1+obj.num_nlcon,obj.num_vars);
      for i = 1:length(obj.cost)
        if(~isempty(obj.cost_kinsol_idx{i}))
          if(length(obj.cost_kinsol_idx{i}) == 1)
            if(isempty(obj.cost_nonkinsol_idx{i}))
              [fi,dfi] = obj.cost{i}.eval(kinsol_cell{obj.cost_kinsol_idx{i}});
            else
              [fi,dfi] = obj.cost{i}.eval(kinsol_cell{obj.cost_kinsol_idx{i}},x(obj.cost_nonkinsol_idx{i}));
            end
          else
            if(isempty(obj.cost_nonkinsol_idx{i}))
              [fi,dfi] = obj.cost{i}.eval(kinsol_cell(obj.cost_kinsol_idx{i}));
            else
              [fi,dfi] = obj.cost{i}.eval(kinsol_cell(obj.cost_kinsol_idx{i}),x(obj.cost_nonkinsol_idx{i}));
            end
          end
        else
          [fi,dfi] = obj.cost{i}.eval(x(obj.cost_xind_cell{i}));
        end
        f(1) = f(1)+fi;
        G(1,obj.cost_xind_cell{i}) = G(1,obj.cost_xind_cell{i})+dfi;
      end
      f_count = 1;
      for i = 1:length(obj.nlcon)
        if(~isempty(obj.nlcon_kinsol_idx{i}))
          if(length(obj.nlcon_kinsol_idx{i}) == 1)
            if(isempty(obj.nlcon_nonkinsol_idx{i}))
              [f(f_count+(1:obj.nlcon{i}.num_cnstr)),G(f_count+(1:obj.nlcon{i}.num_cnstr),obj.nlcon_xind{i})] = ...
                obj.nlcon{i}.eval(kinsol_cell{obj.nlcon_kinsol_idx{i}});
            else
              [f(f_count+(1:obj.nlcon{i}.num_cnstr)),G(f_count+(1:obj.nlcon{i}.num_cnstr),obj.nlcon_xind{i})] = ...
                obj.nlcon{i}.eval(kinsol_cell{obj.nlcon_kinsol_idx{i}},x(obj.nlcon_nonkinsol_idx{i}));
            end
          else
            if(isempty(obj.nlcon_nonkinsol_idx{i}))
              [f(f_count+(1:obj.nlcon{i}.num_cnstr)),G(f_count+(1:obj.nlcon{i}.num_cnstr),obj.nlcon_xind{i})] = ...
                obj.nlcon{i}.eval(kinsol_cell(obj.nlcon_kinsol_idx{i}));
            else
              [f(f_count+(1:obj.nlcon{i}.num_cnstr)),G(f_count+(1:obj.nlcon{i}.num_cnstr),obj.nlcon_xind{i})] = ...
                obj.nlcon{i}.eval(kinsol_cell(obj.nlcon_kinsol_idx{i}),x(obj.nlcon_nonkinsol_idx{i}));
            end
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
    
    function obj = addNonlinearConstraint(obj,cnstr,kinsol_idx,non_kinsol_idx,xind)
      % @param cnstr     -- A NonlinearConstraint object
      % @param xind      -- Optional argument. The x(xind) is the decision variables used
      % in evaluating the cnstr. Default value is (1:obj.num_vars)
      % @param kinsol_idx  -- If the constraint calls kinsol, then q(:,kinsol_idx) are the
      % postures whose kinsol needs to be evaluated for the new constraint. If the new
      % constraint does not need kinsol for evaluation, then kinsol_idx = [];
      % @param non_kinsol_idx    -- A vector. The indices of decision variables, apart from kinsol,
      % that are used for evaluating the constraint. If only kinsol is used, then
      % non_kinsol_idx = [];
      if(nargin<5)
        xind = (1:obj.num_vars)';
      end
      obj = addNonlinearConstraint@NonlinearProgramWConstraintObjects(obj,cnstr,xind);
      kinsol_idx = floor(kinsol_idx(:));
      obj.nlcon_kinsol_idx = [obj.nlcon_kinsol_idx,{kinsol_idx}];
      if(~isnumeric(non_kinsol_idx))
        error('Drake:NonlinearProgramWKinsol:addNonlinearConstraint:non_kinsol_idx should be numeric');
      end
      non_kinsol_idx = floor(non_kinsol_idx(:));
      if(any(non_kinsol_idx<1 | non_kinsol_idx>obj.num_vars))
        error('Drake:NonlinearProgramWKinsol:addNonlinearConstraintTraj:non_kinsol_idx out of range');
      end
      obj.nlcon_nonkinsol_idx = [obj.nlcon_nonkinsol_idx,{non_kinsol_idx}];
      obj.t_kinsol(kinsol_idx) = true;
    end
    
    function obj = addCost(obj,cost,kinsol_idx,non_kinsol_idx,xind)
      % @param cnstr   -- A NonlinearConstraint or a LinearConstraint
      % @param xind      -- Optional argument. x(xind) is the decision variables used in
      % evaluating the cost. Default value is (1:obj.num_vars)
      % @param kinsol_idx  -- If the cost calls kinsol, then q(:,kinsol_idx) are the
      % postures whose kinsol needs to be evaluated for the new cost. If the new
      % cost does not need kinsol for evaluation, then kinsol_idx = [];
      % @param non_kinsol_idx    -- A vector. The indices of decision variables, apart from kinsol,
      % that are used for evaluating the cost. If only kinsol is used, then
      % non_kinsol_idx = [];
      if(nargin<5)
        xind = (1:obj.num_vars)';
      end
      obj = addCost@NonlinearProgramWConstraintObjects(obj,cost,xind);
      kinsol_idx = floor(kinsol_idx(:));
      obj.cost_kinsol_idx = [obj.cost_kinsol_idx,{kinsol_idx}];
      if(~isnumeric(non_kinsol_idx))
        error('Drake:NonlinearProgramWKinsol:addCost:non_kinsol_idx should be numeric');
      end
      non_kinsol_idx = floor(non_kinsol_idx(:));
      if(any(non_kinsol_idx<1 | non_kinsol_idx>obj.num_vars))
        error('Drake:NonlinearProgramWKinsol:addCost:non_kinsol_idx out of range');
      end
      obj.cost_nonkinsol_idx = [obj.cost_nonkinsol_idx,{non_kinsol_idx}];
      obj.t_kinsol(kinsol_idx) = true;
    end
    
    function obj = replaceCost(obj,cost,cost_idx,kinsol_idx,non_kinsol_idx,xind)
      % @param cost     -- A Constraint object, currently accepts NonlinearConstraint and
      % LinearConstraint
      % @param cost_idx -- The index of the original cost to be replaced
      % @param xind     -- Optional argument. x(xind) is the decision variables used in
      % evaluating the cost. Default value is (1:obj.num_vars)
      % @param kinsol_idx  -- If the cost calls kinsol, then q(:,kinsol_idx) are the
      % postures whose kinsol needs to be evaluated for the new cost. If the new
      % cost does not need kinsol for evaluation, then kinsol_idx = [];
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
        error('Drake:NonlinearProgramWKinsol:replaceCost:cost_idx is out of range');
      end
      cost_tmp = obj.cost;
      cost_tmp{cost_idx} = cost;
      cost_xind_tmp = obj.cost_xind_cell;
      cost_xind_tmp{cost_idx} = xind;
      cost_kinsol_idx_tmp = obj.cost_kinsol_idx;
      cost_kinsol_idx_tmp{cost_idx} = kinsol_idx;
      cost_nonkinsol_idx_tmp = obj.cost_nonkinsol_idx;
      cost_nonkinsol_idx_tmp{cost_idx} = non_kinsol_idx;
      obj.cost = {};
      obj.cost_xind_cell = {};
      obj.cost_kinsol_idx = {};
      obj.cost_nonkinsol_idx = {};
      for i = 1:num_cost
        obj = obj.addCost(cost_tmp{i},cost_kinsol_idx_tmp{i},cost_nonkinsol_idx_tmp{i},cost_xind_tmp{i});
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
          error('Drake:NonlinearProgramWKinsol:addDecisionVariable:var_names should be a cell of strings');
        end
      end
      obj = addDecisionVariable@NonlinearProgramWConstraintObjects(obj,num_new_vars);
      obj.x_name = [obj.x_name;var_names];
    end
  end
end