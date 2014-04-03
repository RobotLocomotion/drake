classdef PolynomialProgram < NonlinearProgram
% Provides a common interface for polynomial optimization.
%  (note: should perhaps be called a SpotPolynomialProgram, to parallel
%  PolynomialSystem/SpotPolynomialSystem)
%  
% Using x to denote the decision variables, attempts to solve the problem
%
%   minimize_x objective(x)
%   subject to 
%         equality_constraints(x) = 0 
%         inequality_constraint(x) <= 0
%         Ain*x <= bin
%         Aeq*x <= beq
%         lb <= x <= ub
%
% @option solver can be 'gloptipoly','snopt' (more coming soon)
% @option x0 initial guess

% todo: implement all of the techniques from http://arxiv.org/pdf/math/0103170.pdf

  properties
    decision_vars     % simple msspoly description of x
    poly_objective              % msspoly
    poly_inequality_constraints % msspoly
    poly_equality_constraints   % msspoly
  end

  methods
    function obj = PolynomialProgram(decision_vars,objective,inequality_constraints,equality_constraints)
      typecheck(decision_vars,'msspoly');
      if ~issimple(decision_vars),
        error('decision_vars must be a simple msspoly');
      end
      typecheck(objective,'msspoly');
      if length(objective)~=1
        error('objective must be a scalar msspoly');
      end
      
      if nargin<3, inequality_constraints = []; end
      if nargin<4, equality_constraints = []; end
      if nargin<5, options=struct(); end
      if ~isfield(options,'solver'),
        % todo: check dependencies here and pick my favorite that is also installed
        options.solver = 'bertini';
      end
      
      if ~isfunction(objective,decision_vars)
        error('objective function must only depend on the decision variables');
      end
      if ~isempty(inequality_constraints)
        sizecheck(inequality_constraints,'colvec');
        if ~isfunction(inequality_constraints,decision_vars)
          error('equality constraints must only depend on the decision variables');
        end
      end
      if ~isempty(equality_constraints)
        sizecheck(equality_constraints,'colvec');
        if ~isfunction(equality_constraints,decision_vars)
          error('equality constraints must only depend on the decision variables');
        end
      end

      obj = obj@NonlinearProgram(length(decision_vars),length(equality_constraints),length(inequality_constraints));
      obj.decision_vars = decision_vars;
      obj.poly_objective = objective;
      obj.poly_inequality_constraints = inequality_constraints;
      obj.poly_equality_constraints = equality_constraints;
    end
    
    function [x,objval,exitflag] = solve(obj,x0)
      switch lower(obj.solver)
        case 'gloptipoly'
          [x,objval,exitflag] = gloptipoly(obj);
        case 'bertini'
          [x,objval,exitflag] = bertini(obj);
        otherwise 
          [x,objval,exitflag] = solve@NonlinearProgram(obj,x0);
      end
    end

    function [x,objval,exitflag,execution_time] = compareSolvers(obj,x0,solvers)
      if nargin<2, x0 = randn(obj.num_vars,1); end
      if nargin<3, solvers = {'gloptipoly','bertini','snopt','fmincon'}; end
      [x,objval,exitflag,execution_time] = compareSolvers@NonlinearProgram(obj,x0,solvers);
    end
    
    function [f,df] = objective(obj,x)
      f = double(msubs(obj.poly_objective,obj.decision_vars,x));

      if (nargout>1)
        df = double(subs(diff(obj.poly_objective,obj.decision_vars),obj.decision_vars,x));
      end
    end
    
    function [g,h,dg,dh] = nonlinearConstraints(obj,x)
      if isempty(obj.poly_inequality_constraints) 
        g=[];
      else
        g = double(msubs(obj.poly_inequality_constraints,obj.decision_vars,x));
      end
      if isempty(obj.poly_equality_constraints)
        h=[];
      else
        h = double(msubs(obj.poly_equality_constraints,obj.decision_vars,x));
      end
      
      if (nargout>2)
        if isempty(obj.poly_inequality_constraints)
          dg=[];
        else
          dg = double(subs(diff(obj.poly_inequality_constraints,obj.decision_vars),obj.decision_vars,x));
        end
        
        if isempty(obj.poly_equality_constraints)
          dh=[];
        else
          dh = double(subs(diff(obj.poly_equality_constraints,obj.decision_vars),obj.decision_vars,x));
        end
      end
    end

  end
  
end