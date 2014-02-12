classdef NonlinearProgram
  % minimize_x objective(x)
  % subject to
  %            nonlinear_equality_constraints(x) = 0
  %            nonlinear_inequality_constraints(x) <= 0
  %            Aeq*x = beq
  %            Ain*x <= bin
  %            lb <= x <= ub
  
  properties (SetAccess=protected)
    num_decision_vars
    num_nonlinear_inequality_constraints
    num_nonlinear_equality_constraints
    Ain,bin
    Aeq,beq
    lb,ub
    iGfun,jGvar  % sparsity pattern in objective and nonlinear constraints
    solver
    default_options
    grad_method
  end
  
  properties (Access=private)
    objcon_logic = false;
  end

  methods % A subset of these MUST be overloaded
    
    % PLEASE READ THIS
    %  some solvers take the objective and constraints as
    %  separate callbacks.  others (e.g. snopt) takes 
    %  a single callback which computes both.  i want to 
    %  support the "computes both" functionality, because
    %  it can be more efficient for expensive function
    %  evaluations (e.g. often there is computation that
    %  is shared between computing the objective value
    %  and the constraint value).  
    %  so, i've implemented the simple logic below so 
    %  that derived classes can implement the objective
    %  and constraints methods independently, or all 
    %  in one method.  BUT ALL DERIVED CLASSES MUST
    %  OVERLOAD ONE OR THE OTHER VERSIONS.
    
    function [f,df] = objective(obj,x)
      if (obj.objcon_logic) % then i'm getting called from objectiveAndNonlinearConstraints
        error('Drake:NonlinearProgram:AbstractMethod','all derived classes must implement objective or objectiveAndNonlinearConstraints');
      end
      
      % todo: cache the input/output pairs so I only call
      % this once (in objective OR nonlinearConstraints)
      obj.objcon_logic = true;
      if nargout>1
        [fgh,dfgh] = objectiveAndNonlinearConstraints(obj,x);
        df = dfgh(1,:);
      else
        fgh = objectiveAndNonlinearConstraints(obj,x);
      end
      f = fgh(1);
    end
    
    function [g,h,dg,dh] = nonlinearConstraints(obj,x)
      if (obj.objcon_logic) % then i'm getting called from objectiveAndNonlinearConstraints
        error('Drake:NonlinearProgram:AbstractMethod','all derived classes must implement objective or objectiveAndNonlinearConstraints');
      end
      
      % todo: cache the input/output pairs so I only call
      % this once (in objective OR nonlinearConstraints)
      obj.objcon_logic = true;
      if nargout>2
        [fgh,dfgh] = objectiveAndNonlinearConstraints(obj,x);
        dg = dfgh(1 + (1:obj.num_nonlinear_inequality_constraints),:);
        dh = dfgh(1+obj.num_nonlinear_inequality_constraints + (1:obj.num_nonlinear_equality_constraints),:);
      else
        fgh = objectiveAndNonlinearConstraints(obj,x);
      end
      g = fgh(1 + (1:obj.num_nonlinear_inequality_constraints));
      h = fgh(1+obj.num_nonlinear_inequality_constraints + (1:obj.num_nonlinear_equality_constraints));
    end
    
    function [fgh,dfgh] = objectiveAndNonlinearConstraints(obj,x)
      % @param x = current value of decision variables
      % @retval fgh = [ objective(x);
      %               nonlinear_inequality_constraints(x);
      %               nonlinear_equality_constraints(x)]
      
      if (obj.objcon_logic) % then i'm getting called from objective or nonlinearConstraints
        error('Drake:NonlinearProgram:AbstractMethod','all derived classes must implement objective or objectiveAndNonlinearConstraints');
      end

      obj.objcon_logic = true;
      if nargout>1
        [f,df] = objective(obj,x);
        if (obj.num_nonlinear_inequality_constraints + obj.num_nonlinear_equality_constraints)
          [g,h,dg,dh] = nonlinearConstraints(obj,x);
          fgh = [f;g;h];
          dfgh = [df;dg;dh];
        else
          fgh = f;
          dfgh = df;
        end
      else
        f = objective(obj,x);
        if (obj.num_nonlinear_inequality_constraints + obj.num_nonlinear_equality_constraints)
          [g,h] = nonlinearConstraints(obj,x);
          fgh = [f;g;h];
        else
          fgh = f;
        end
      end
    end
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
      obj.default_options.fmincon = optimset('Display','off');
      obj.default_options.snopt = struct();
    end
    
    function obj = addLinearInequalityConstraints(obj,Ain,bin)
      [m,n] = size(Ain);
      assert(n == obj.num_decision_vars);
      sizecheck(bin,[m,1]);
      obj.Ain = vertcat(obj.Ain,Ain);
      obj.bin = vertcat(obj.bin,bin);
    end
    
    function obj = addLinearEqualityConstraints(obj,Aeq,beq)
      [m,n] = size(Aeq);
      assert(n == obj.num_decision_vars);
      sizecheck(beq,[m,1]);
      obj.Ain = vertcat(obj.Ain,Aeq);
      obj.bin = vertcat(obj.bin,beq);
    end

    function obj = setBounds(obj,lb,ub)
      sizecheck(lb,[obj.num_decision_vars,1]);
      sizecheck(ub,[obj.num_decision_vars,1]);
      obj.lb = lb;
      obj.ub = ub;
    end
    
    function obj = setObjectiveGradientSparsity(obj,jGvar)
      error('todo: finish this');
    end
    
    function obj = setNonlinearInequalityConstraintsGradientSparsity(obj,iGfun,jGvar)
      error('todo: finish this');
    end
    
    function obj = setNonlinearEqualityConstraintsGradientSparsity(obj,iGfun,jGvar)
      error('todo: finish this');
    end
    
    function obj = setSolver(obj,solver)
      typecheck(solver,'char');
      obj.solver = solver;
    end
    
    function obj = setSolverOptions(obj,solver,options)
      error('todo: finish this');
    end
    
    function obj = setSolverOption(obj,solver,optionname,optionval)
      error('todo: finish this');
    end
    
    % function setGradMethod?
    
    function [x,objval,exitflag] = solve(obj,x0)
      switch lower(obj.solver)
        case 'snopt'
          [x,objval,exitflag] = snopt(obj,x0);
        case 'fmincon'
          [x,objval,exitflag] = fmincon(obj,x0);
        otherwise
          error('Drake:NonlinearProgram:UnknownSolver',['The requested solver, ',obj.solver,' is not known, or not currently supported']);
      end
    end
    
    function [x,objval,exitflag,execution_time] = compareSolvers(obj,x0,solvers)
      if nargin<3
        solvers={'snopt','fmincon'};
      end
       
      fprintf('    solver        objval        exitflag   execution time\n-------------------------------------------------------------\n')
      typecheck(solvers,'cell');
      for i=1:length(solvers)
        obj.solver = solvers{i};
        try 
          tic;
          [x{i},objval{i},exitflag{i}] = solve(obj,x0);
          execution_time{i} = toc;
        catch ex
          if ((strncmp(ex.identifier,'Drake:MissingDependency',23)))
            continue;
          else
            rethrow(ex);
          end
    	end
        
        fprintf('%12s%12.3f%12d%17.4f\n',solvers{i},objval{i},exitflag{i},execution_time{i});
      end
    end
    
    function [x,objval,exitflag] = snopt(obj,x0,options)
      checkDependency('snopt');

      global SNOPT_USERFUN;
      SNOPT_USERFUN = @snopt_userfun;
      
      A = [obj.Ain;obj.Aeq];
      b = [obj.bin;obj.beq];
      if isempty(obj.lb) obj.lb = -inf(obj.num_decision_vars,1); end
      if isempty(obj.ub) obj.ub = inf(obj.num_decision_vars,1); end

      function setSNOPTParam(paramstring,default)
        str=paramstring(~isspace(paramstring));
        if (isfield(obj.default_options.snopt,str))
          snset([paramstring,'=',num2str(getfield(obj.default_options.snopt,str))]);
        else
          snset([paramstring,'=',num2str(default)]);
        end
      end
      
      setSNOPTParam('Major Iterations Limit',1000);
      setSNOPTParam('Minor Iterations Limit',500);
      setSNOPTParam('Major Optimality Tolerance',1e-6);
      setSNOPTParam('Major Feasibility Tolerance',1e-6);
      setSNOPTParam('Minor Feasibility Tolerance',1e-6);
      setSNOPTParam('Superbasics Limit',200);
      setSNOPTParam('Derivative Option',0);
      setSNOPTParam('Verify Level',0);
      setSNOPTParam('Iterations Limit',10000);

      function [f,G] = snopt_userfun(x)
        [f,G] = geval(@obj.objectiveAndNonlinearConstraints,x);
        f = [f;0*b];
        
        if ~isempty(obj.iGfun) && ~isempty(obj.jGvar)
          G = G(sub2ind(size(G),obj.iGfun,obj.jGvar));
        else
          G = G(:);
        end
      end      

      if isempty(A)
        [x,objval,exitflag] = snopt(x0, ...
          obj.lb,obj.ub, ...
          [-inf;-inf(obj.num_nonlinear_inequality_constraints,1);zeros(obj.num_nonlinear_equality_constraints,1);-inf(size(obj.bin));obj.beq],[inf;zeros(obj.num_nonlinear_inequality_constraints+obj.num_nonlinear_equality_constraints,1);obj.bin;obj.beq],...
          'snoptUserfun');
      else
        [iAfun,jAvar,Avals] = find(A);
        iAfun = iAfun + 1 + obj.num_nonlinear_inequality_constraints + obj.num_nonlinear_equality_constraints;
        
        if isempty(obj.iGfun) || isempty(obj.jGvar)
          s = [1+obj.num_nonlinear_inequality_constraints+obj.num_nonlinear_equality_constraints,obj.num_decision_vars];
          [iGfun,jGvar] = ind2sub(s,1:prod(s));
        else
          iGfun = obj.iGfun;
          jGvar = obj.jGvar;
        end
          
        [x,objval,exitflag] = snopt(x0, ...
          obj.lb,obj.ub, ...
          [-inf;-inf(obj.num_nonlinear_inequality_constraints,1);zeros(obj.num_nonlinear_equality_constraints,1);-inf(size(obj.bin));obj.beq],[inf;zeros(obj.num_nonlinear_inequality_constraints+obj.num_nonlinear_equality_constraints,1);obj.bin;obj.beq],...
          'snoptUserfun',...
          0,1,...
          Avals,iAfun,jAvar,...
          iGfun,jGvar);
      end
      
      objval = objval(1);
      if exitflag~=1, disp(snoptInfo(exitflag)); end
    end
    
    function [x,objval,exitflag] = fmincon(obj,x0,options)
      if (obj.num_nonlinear_equality_constraints || obj.num_nonlinear_inequality_constraints)
        error('not implemented yet');
      end
      if (obj.num_nonlinear_inequality_constraints + obj.num_nonlinear_equality_constraints)
        nonlinearConstraints = @obj.nonlinearConstraint;
      else
        nonlinearConstraints = [];
      end
      [x,objval,exitflag] = fmincon(@obj.objective,x0,obj.Ain,obj.bin,obj.Aeq,obj.beq,obj.lb,obj.ub,nonlinearConstraints,obj.default_options.fmincon);
      objval = full(objval);
    end
  end
  
end
