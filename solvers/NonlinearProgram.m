classdef NonlinearProgram
  % minimize_x objective(x)
  % subject to
  %            nonlinear_equality_constraints(x) = 0
  %            nonlinear_inequality_constraints(x) <= 0
  %            Aeq*x = beq
  %            Ain*x <= bin
  %            lb <= x <= ub
  
  properties 
    num_decision_vars
    num_nonlinear_inequality_constraints
    num_nonlinear_equality_constraints
    Aeq,beq
    Ain,bin
    lb,ub
    iGfun,jGvar  % sparsity pattern in objective and nonlinear constraints
    solver
    solver_options
    grad_method
  end

  methods (Abstract)
    % @param x = current value of decision variables
    % @retval f = [ objective(x); 
    %               nonlinear_equality_constraints(x);
    %               nonlinear_inequality_constraints(x)]
    f = objectiveAndNonlinearConstraints(obj,x);
  end
  
  methods
    function obj = NonlinearProgram(num_vars,num_nin,num_neq)
      obj.num_decision_vars = num_vars;
      obj.num_nonlinear_inequality_constraints = num_nin;
      obj.num_nonlinear_equality_constraints = num_neq;
      obj.lb = -inf(num_vars,1);
      obj.ub = inf(num_vars,1);
      
      % todo : check dependencies and then go through the list
      obj.solver = 'snopt';
    end
    
    function [x,objval,exitflag] = solve(obj,x0)
      switch lower(obj.solver)
        case 'snopt'
          [x,objval,exitflag] = solveWSNOPT(obj,x0);
        case 'fmincon'
          [x,objval,exitflag] = solveWFMINCON(obj,x0);
        otherwise
          error('Drake:NonlinearProgram:UnknownSolver',['The requested solver, ',options.solver,' is not known, or not currently supported']);
      end
    end
    
    function [x,objval,exitflag,execution_time] = compareSolvers(obj,x0,solvers)
      if nargin<3
        solvers={'snopt','fmincon'};
      end
      
      fprintf('   solver       objval       exitflag   execution time\n---------------------------------------------------------------------\n')
      typecheck(solvers,'cell');
      for i=1:length(solvers)
        obj.solver = solvers{i};
        tic;
        [x{i},objval{i},exitflag{i}] = solve(obj,x0);
        execution_time{i} = toc;
        fprintf('%12s%12f%12d%12f\n',solvers{i},objval{i},exitflag{i},execution_time{i});
      end
    end
    
    function [x,objval,exitflag] = solveWSNOPT(obj,x0)
      checkDependency('snopt');

      global SNOPT_USERFUN;
      SNOPT_USERFUN = @snopt_userfun;
      
      A = [obj.Aeq;obj.Ain];
      b = [obj.beq;obj.bin];

      function [f,G] = snopt_userfun(x)
        [f,G] = geval(@obj.objectiveAndNonlinearConstraints,x);
        f = [f;0*b];
        G = [G;zeros(size(A))];
      end      

      if isempty(A)
        [x,objval,exitflag] = snopt(x0, ...
          obj.lb,obj.ub, ...
          [-inf;zeros(obj.num_nonlinear_equality_constraints,1);-inf(obj.num_nonlinear_inequality_constraints,1);obj.beq;-inf(size(obj.bin))],[inf;zeros(obj.num_nonlinear_equality_constraints+obj.num_nonlinear_inequality_constraints,1);obj.beq;obj.bin],...
          'snoptUserfun');
      else
        [iAfun,jAvar] = find(A);
        [x,objval,exitflag] = snopt(x0, ...
          obj.lb,obj.ub, ...
          [-inf;zeros(obj.num_nonlinear_equality_constraints,1);-inf(obj.num_nonlinear_inequality_constraints,1);obj.beq;-inf(size(obj.bin))],[inf;zeros(obj.num_nonlinear_equality_constraints+obj.num_nonlinear_inequality_constraints,1);obj.beq;obj.bin],...
          'snoptUserfun',...
          0,1,...
          A,iAfun,jAvar);%...
        %        ,iGfun,jGvar);
      end
    end
    
    function [x,objval,exitflag] = solveWFMINCON(obj,x0)
      if (obj.num_nonlinear_equality_constraints || obj.num_nonlinear_inequality_constraints)
        error('not implemented yet');
      end
      [x,objval,exitflag] = fmincon(@obj.objectiveAndNonlinearConstraints,x0,obj.Ain,obj.bin,obj.Aeq,obj.beq,obj.lb,obj.ub);
    end
  end
  
end