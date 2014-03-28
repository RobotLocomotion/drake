classdef NonlinearProgram
  % minimize_x objective(x)
  % subject to
  %            cin_lb<=nonlinear_inequality_constraints(x) <= cin_ub
  %            nonlinear_equality_constraints(x) = 0  
  %            Ain*x <= bin
  %            Aeq*x = beq
  %            x_lb <= x <= x_ub
  % @param num_vars     An integer. The number of decision variables
  % @param num_cin      An integer. The number of nonlinear inequality constraints
  % @param num_ceq      An integer. The number of nonlinear equality constraints
  % @param Ain          A double matrix. 
  % @param bin          A double vector. The upper bound for the linear equality
  % constraint
  % @param Aeq          A double matrix
  % @param beq          A double vector
  % @param cin_lb       A double vector, the lower bounds on the nonlinear inequality
  % constraints. The default value is 0
  % @param cin_ub       A double vector, the upper bounds on the nonlinear inequality
  % constraints. The default value is -inf
  % @param check_grad   A boolean. True if the user gradient will be checked against
  % numerical gradient at the begining and end of the nonlinear optimization
  properties (SetAccess=protected)
    num_vars
    num_cin
    num_ceq
    Ain,bin
    Aeq,beq
    cin_lb,cin_ub
    x_lb,x_ub
    solver
    solver_options
    grad_method
    check_grad
  end
  
  properties (Access=private)
    objcon_logic = false;
  end

  properties (Access = protected)
%     iGfun,jGvar  % sparsity pattern in objective and nonlinear constraints
    iFfun,jFvar  % sparsity pattern in the objective function
    iCinfun,jCinvar  % sparsity pattern in the nonlinear inequality constraints
    iCeqfun,jCeqvar  % sparsity pattern in the nonlinear equality constraints
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
        dg = dfgh(1+(1:obj.num_cin),:);
        dh = dfgh(1+obj.num_cin+(1:obj.num_ceq),:);
      else
        fgh = objectiveAndNonlinearConstraints(obj,x);
      end
      g = fgh(1+(1:obj.num_cin));
      h = fgh(1+obj.num_cin+(1:obj.num_ceq));
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
        if (obj.num_cin + obj.num_ceq>0)
          [g,h,dg,dh] = nonlinearConstraints(obj,x);
          fgh = [f;g;h];
          dfgh = [df;dg;dh];
        else
          fgh = f;
          dfgh = df;
        end
      else
        f = objective(obj,x);
        if (obj.num_cin + obj.num_ceq)
          [g,h] = nonlinearConstraints(obj,x);
          fgh = [f;g;h];
        else
          fgh = f;
        end
      end
    end
  end
  
  methods
    function obj = NonlinearProgram(num_vars,num_nonlinear_inequality_constraints,num_nonlinear_equality_constraints)
      % @param num_vars          -- Number of decision variables
      % @param num_nonlinear_inequality_constraints   -- An int scalar. The number of nonlinear inequality
      % constraints
      % @param num_nonlinear_equality_constraints     -- An int scalar. The number of nonlinear equality
      % constraints
      sizecheck(num_vars,[1,1]);
      sizecheck(num_nonlinear_inequality_constraints,[1,1]);
      sizecheck(num_nonlinear_equality_constraints,[1,1]);
      obj.num_vars = num_vars;
      obj.num_cin = num_nonlinear_inequality_constraints;
      obj.num_ceq = num_nonlinear_equality_constraints;
      obj.x_lb = -inf(num_vars,1);
      obj.x_ub = inf(num_vars,1);
      obj.cin_ub = zeros(obj.num_cin,1);
      obj.cin_lb = -inf(obj.num_cin,1);
      obj.iFfun = ones(obj.num_vars,1);
      obj.jFvar = (1:obj.num_vars)';
      obj.iCinfun = reshape(bsxfun(@times,(1:obj.num_cin)',ones(1,obj.num_vars)),[],1);
      obj.jCinvar = reshape(bsxfun(@times,ones(obj.num_cin,1),(1:obj.num_vars)),[],1);
      obj.iCeqfun = reshape(bsxfun(@times,(1:obj.num_ceq)',ones(1,obj.num_vars)),[],1);
      obj.jCeqvar = reshape(bsxfun(@times,ones(obj.num_ceq,1),(1:obj.num_vars)),[],1);
      
      % todo : check dependencies and then go through the list
      obj.solver = 'snopt';
      obj.solver_options.fmincon = optimset('Display','off');
      obj.solver_options.snopt = struct();
      obj.solver_options.snopt.MajorIterationsLimit = 1000;
      obj.solver_options.snopt.MinorIterationsLimit = 500;
      obj.solver_options.snopt.IterationsLimit = 10000;
      obj.solver_options.snopt.MajorOptimalityTolerance = 1e-6;
      obj.solver_options.snopt.MajorFeasibilityTolerance = 1e-6;
      obj.solver_options.snopt.MinorFeasibilityTolerance = 1e-6;
      obj.solver_options.snopt.SuperbasicsLimit = 300;
      obj.solver_options.snopt.VerifyLevel = 0;
      obj.solver_options.snopt.DerivativeOption = 1;
      obj.solver_options.snopt.print = '';
      obj.check_grad = false;
    end
    
    function obj = addLinearInequalityConstraints(obj,Ain,bin)
      % add linear inequality constraint Ain*x<=bin
      [m,n] = size(Ain);
      assert(n == obj.num_vars);
      sizecheck(bin,[m,1]);
      obj.Ain = vertcat(obj.Ain,Ain);
      obj.bin = vertcat(obj.bin,bin);
    end
    
    function obj = addLinearEqualityConstraints(obj,Aeq,beq)
      [m,n] = size(Aeq);
      assert(n == obj.num_vars);
      sizecheck(beq,[m,1]);
      obj.Aeq = vertcat(obj.Aeq,Aeq);
      obj.beq = vertcat(obj.beq,beq);
    end

    function obj = setVarBounds(obj,x_lb,x_ub)
      % set the lower and upper bounds of the decision variables
      sizecheck(x_lb,[obj.num_vars,1]);
      sizecheck(x_ub,[obj.num_vars,1]);
      obj.x_lb = x_lb;
      obj.x_ub = x_ub;
    end
    
    function obj = setObjectiveGradientSparsity(obj,jFvar)
      % set the sparsity pattern in the objective function
      % @param jFvar     -- A column integer vector. The indices of the non-zero entries
      % in the objective gradient
      if(any(jFvar>obj.num_vars) || any(jFvar<1))
        error('Drake:NonlinearProgram:setObjectiveGradientSparsity:jGvar out of bounds');
      end
      obj.iFfun = ones(length(jFvar),1);
      obj.jFvar = jFvar;
    end
    
    function obj = setNonlinearInequalityConstraintsGradientSparsity(obj,iCinfun,jCinvar)
      % set the sparsity patten in the nonlinear inequality constraints, so
      % dg(iCinfun,jCinvar) are the nonzero entries in the gradient matrix dg.
      nnz = numel(iCinfun);
      sizecheck(iCinfun,[nnz,1]);
      sizecheck(jCinvar,[nnz,1]);
      if(any(iCinfun<1 | iCinfun>obj.num_cin) || any(jCinvar<1 | jCinvar>obj.num_vars))
        error('Drake:NonlinearProgram:setNonlinearInequalityConstraintsGradientSparsity:iCinfun or jCinvar is out of bounds');
      end
      obj.iCinfun = iCinfun;
      obj.jCinvar = jCinvar;
    end
    
    function obj = setNonlinearEqualityConstraintsGradientSparsity(obj,iCeqfun,jCeqvar)
      % set the sparsity patten in the nonlinear equality constraints, so
      % dh(iCeqfun,jCeqvar) are the nonzero entries in the gradient matrix dh.
      nnz = numel(iCeqfun);
      sizecheck(iCeqfun,[nnz,1]);
      sizecheck(jCeqvar,[nnz,1]);
      if(any(iCeqfun<1 | iCeqfun>obj.num_cin) || any(jCeqvar<1 | jCeqvar>obj.num_vars))
        error('Drake:NonlinearProgram:setNonlinearInequalityCOnstraintsGradientSparsity:iCinfun or jCinvar is out of bounds');
      end
      obj.iCeqfun = iCeqfun;
      obj.jCeqvar = jCeqvar;
    end
    
    function obj = setCheckGrad(obj,check_grad)
      sizecheck(check_grad,[1,1]);
      obj.check_grad = logical(check_grad);
    end
    
    function obj = setSolver(obj,solver)
      typecheck(solver,'char');
      obj.solver = solver;
    end
    
%     function obj = setSolverOptions(obj,solver,options)
%       if(strcmp(lower(solver),'snopt'))
%         
%       end
%     end
    
    function obj = setSolverOptions(obj,solver,optionname,optionval)
      % @param solver   - string name of the solver
      % @param optionname    -- string name of the option field
      % @param optionval     -- option value
      if(strcmpi(solver,'snopt'))
        if(strcmpi(optionname(~isspace(optionname)),'majorfeasibilitytolerance'))
          sizecheck(optionval,[1,1]);
          if(optionval<=0)
            error('Drake:NonlinearProgram:setSolverOptions:MajorFeasibilityTolerance should be positive');
          end
          obj.solver_options.snopt.MajorFeasibilityTolerance = optionval;
        elseif(strcmpi(optionname(~isspace(optionname)),'minorfeasibilitytolerance'))
          sizecheck(optionval,[1,1]);
          if(optionval<=0)
            error('Drake:NonlinearProgram:setSolverOptions:MinorFeasibilityTolerance should be positive');
          end
          obj.solver_options.snopt.MinorFeasibilityTolerance = optionval;
        elseif(strcmpi(optionname(~isspace(optionname)),'majoroptimalitytolerance'))
          sizecheck(optionval,[1,1]);
          if(optionval<=0)
            error('Drake:NonlinearProgram:setSolverOptions:MajorOptimalityTolerance should be positive');
          end
          obj.solver_options.snopt.MajorOptimalityTolerance = optionval;
        elseif(strcmpi(optionname(~isspace(optionname)),'majoriterationslimit'))
          sizecheck(optionval,[1,1]);
          if(optionval<1)
            error('Drake:NonlinearProgram:setSolverOptions:MajorIterationsLimit should be positive integers');
          end
          obj.solver_options.snopt.MajorIterationsLimit = floor(optionval);
        elseif(strcmpi(optionname(~isspace(optionname)),'minoriterationslimit'))
          sizecheck(optionval,[1,1]);
          if(optionval<1)
            error('Drake:NonlinearProgram:setSolverOptions:MinorIterationsLimit should be positive integers');
          end
          obj.solver_options.snopt.MinorIterationsLimit = floor(optionval);
        elseif(strcmpi(optionname(~isspace(optionname)),'iterationslimit'))
          sizecheck(optionval,[1,1]);
          if(optionval<1)
            error('Drake:NonlinearProgram:setSolverOptions:IterationsLimit should be positive integers');
          end
          obj.solver_options.snopt.IterationsLimit = floor(optionval);
        elseif(strcmpi(optionname(~isspace(optionname)),'superbasicslimit'))
          sizecheck(optionval,[1,1]);
          if(optionval<1)
            error('Drake:NonlinearProgram:setSolverOptions:SuperbasicsLimit should be positive integers');
          end
          obj.solver_options.snopt.SuperbasicsLimit = floor(optionval);
        elseif(strcmpi(optionname(~isspace(optionname)),'derivativeoption'))
          sizecheck(optionval,[1,1]);
          if(optionval ~= 0  && optionval ~= 1)
            error('Drake:NonlinearProgram:setSolverOptions:DerivativeOption can be either 0 or 1');
          end
          obj.solver_options.snopt.DerivativeOption = optionval;
        elseif(strcmpi(optionname(~isspace(optionname)),'verifylevel'))
          sizecheck(optionval,[1,1]);
          if(optionval ~= 0  && optionval ~= 1 && optionval ~= 2 && optionval ~= 3 && optionval ~= -1)
            error('Drake:NonlinearProgram:setSolverOptions:VerifyLevel can be either 0,1,2,3 or -1');
          end
          obj.solver_options.snopt.VerifyLevel = optionval;
        elseif(strcmpi(optionname(~isspace(optionname)),'print'))
          if(~ischar(optionval))
            error('Drake:NonlinearProgram:setSolverOptions:print should be the file name string');
          end
          obj.solver_options.snopt.print = optionval;
        end
      elseif(strcmpi((solver),'fmincon'))
        error('Not implemented yet');
      else
        error('solver %s not supported yet',solver);
      end
    end
    
    function obj = setNonlinearInequalityBounds(obj,cin_lb,cin_ub,cin_idx)
      % change the value of the nonlinear constraints lower and upper bounds
      % @param cin_lb    -- A double column vector. The lower bound of the nonlinear
      % inequality constraints
      % @param cin_ub    -- A double column vector. The upper bound of the nonlinear
      % inequality constraints
      % @param cin_idx   -- A integer column vector. The indices of the nonlinear
      % constraints whose upper and lower bounds are reset through the function. The
      % default value is (1:obj.num_cin)'
      if(nargin<4)
        cin_idx = (1:obj.num_cin)';
      end
      if(any(cin_idx>obj.num_cin) || any(cin_idx<1))
        error('Drake:NonlinearProgram:setNonlinearInequalityBounds:cin_idx is incorrect');
      end
      cin_idx = cin_idx(:);
      num_cin_set = numel(cin_idx);
      sizecheck(cin_ub,[num_cin_set,1]);
      sizecheck(cin_lb,[num_cin_set,1]);
      if(any(cin_ub<cin_lb))
        error('Drake:NonlinearProgram:setNonlinearInequalityBounds:cin_ub cannot be smaller than cin_lb');
      end
      obj.cin_ub(cin_idx) = cin_ub;
      obj.cin_lb(cin_idx) = cin_lb;
    end
    % function setGradMethod?
    
    function [iGfun,jGvar] = getNonlinearGradientSparsity(obj)
      % This function sets the nonlinear sparsity vector iGfun and jGvar based on the
      % nonlinear sparsity of the objective, nonlinear inequality constraints and
      % nonlinear equality constraints
      % @param iGfun,jGvar. G(iGfun,jGvar) are the non-zero entries in the matrix G, which
      % is the gradient of return value f in the objectiveAndNonlinearConstraints function
      iGfun = [obj.iFfun;obj.iCinfun+1;obj.iCeqfun+1+obj.num_cin];
      jGvar = [obj.jFvar;obj.jCinvar;obj.jCeqvar];
    end
    
    function [lb,ub] = bounds(obj)
      % return the bounds for all the objective function, nonlinear constraints and linear
      % constraints
      lb = [-inf;obj.cin_lb;zeros(obj.num_ceq,1);-inf(length(obj.bin),1);obj.beq];
      ub = [inf;obj.cin_ub;zeros(obj.num_ceq,1);obj.bin;obj.beq];
    end
    
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
      x = cell(1,length(solvers));
      objval = cell(1,length(solvers));
      exitflag = cell(1,length(solvers));
      execution_time = cell(1,length(solvers));
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
    
    function [x,objval,exitflag] = snopt(obj,x0)
      checkDependency('snopt');

      global SNOPT_USERFUN;
      SNOPT_USERFUN = @snopt_userfun;
      
      A = [obj.Ain;obj.Aeq];
      
      snseti('Major Iterations Limit',obj.solver_options.snopt.MajorIterationsLimit);
      snseti('Minor Iterations Limit',obj.solver_options.snopt.MinorIterationsLimit);
      snsetr('Major Optimality Tolerance',obj.solver_options.snopt.MajorOptimalityTolerance);
      snsetr('Major Feasibility Tolerance',obj.solver_options.snopt.MajorFeasibilityTolerance);
      snsetr('Minor Feasibility Tolerance',obj.solver_options.snopt.MinorFeasibilityTolerance);
      snseti('Superbasics Limit',obj.solver_options.snopt.SuperbasicsLimit);
      snseti('Derivative Option',obj.solver_options.snopt.DerivativeOption);
      snseti('Verify level',obj.solver_options.snopt.VerifyLevel);
      snseti('Iterations Limit',obj.solver_options.snopt.IterationsLimit);

      function [f,G] = snopt_userfun(x)
        [f,G] = geval(@obj.objectiveAndNonlinearConstraints,x);
        f = [f;zeros(length(obj.bin)+length(obj.beq),1)];
        
        G = G(sub2ind(size(G),iGfun,jGvar));
      end      

      function checkGradient(x)
        num_rows_G = 1+obj.num_ceq+obj.num_cin+length(obj.beq)+length(obj.bin);
        [f,G] = snopt_userfun(x);
        [~,G_numerical] = geval(@snopt_userfun,x,struct('grad_method','numerical'));
        G_user = full(sparse(iGfun,jGvar,G,num_rows_G,obj.num_vars));
        G_err = G_user-G_numerical;
        [max_err,max_err_idx] = max(abs(G_err(:)));
        [max_err_row,max_err_col] = ind2sub([num_rows_G,obj.num_vars],max_err_idx);
        display(sprintf('maximum gradient error is in row %d for x[%d], with error %f\nuser gradient %f, numerical gradient %f',...
          max_err_row,max_err_col,max_err,G_user(max_err_row,max_err_col),G_numerical(max_err_row,max_err_col)));
      end
      
      if isempty(A)
        Avals = [];
        iAfun = [];
        jAvar = [];
      else
        [iAfun,jAvar,Avals] = find(A);
        iAfun = iAfun(:);
        jAvar = jAvar(:);
        Avals = Avals(:);
        iAfun = iAfun + 1 + obj.num_cin + obj.num_ceq;
      end
        
      [iGfun,jGvar] = obj.getNonlinearGradientSparsity();
      [lb,ub] = obj.bounds();
      if(obj.check_grad)
        checkGradient(x0);
      end
      
      if(~isempty(obj.solver_options.snopt.print))
        snprint(obj.solver_options.snopt.print);
      end
      [x,objval,exitflag] = snopt(x0, ...
        obj.x_lb,obj.x_ub, ...
        lb,ub,...
        'snoptUserfun',...
        0,1,...
        Avals,iAfun,jAvar,...
        iGfun,jGvar);
      if(obj.check_grad)
        checkGradient(x);
      end
      
      objval = objval(1);
      if exitflag~=1, disp(snoptInfo(exitflag)); end
    end
    
    function [x,objval,exitflag] = fmincon(obj,x0)
%       if (obj.num_cin + obj.num_ceq)
%         nonlinearConstraints = @obj.nonlinearConstraint;
%       else
%         nonlinearConstraints = [];
%       end
      ub_inf_idx = isinf(obj.cin_ub);
      lb_inf_idx = isinf(obj.cin_lb);
      function [c,ceq,dc,dceq] = fmincon_userfun(x)
        [g,h,dg,dh] = obj.nonlinearConstraints(x);
        ceq = h;
        c = [g(~ub_inf_idx)-obj.cin_ub(~ub_inf_idx);obj.cin_lb(~lb_inf_idx)-g(~lb_inf_idx)];
        dc = [dg(~ub_inf_idx,:);-dg(~lb_inf_idx,:)];
        dceq = dh;
      end
      
      [x,objval,exitflag] = fmincon(@obj.objective,x0,obj.Ain,...
        obj.bin,obj.Aeq,obj.beq,obj.x_lb,obj.x_ub,@fmincon_userfun,obj.solver_options.fmincon);
      objval = full(objval);
    end
  end
  
end
