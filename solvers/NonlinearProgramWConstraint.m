classdef NonlinearProgramWConstraint < NonlinearProgram
  % The constraint of this nonlinear program is specified using 'Constraint' class in
  % drake
  % @param nlcon     -- A cell array of NonlinearConstraint
  % @param lcon      -- A cell array of LinearConstraint
  properties(SetAccess = protected)
    nlcon
    lcon
  end
  
  methods
    function obj = NonlinearProgramWConstraint(num_vars)
      % @param num_vars     -- The number of decision variables
      obj = obj@NonlinearProgram(num_vars,0,0);
      nlcon = {};
      lcon = {};
    end
    
    function obj = addNonlinearConstraint(obj,cnstr,xind)
      % add a nonlinear constraint to the object, change the constraint evalation of the
      % program. 
      % @param cnstr     -- A NonlinearConstraint object
      % @param xind      -- Optional argument. The x(xind) is the decision variables used
      % in evaluating the cnstr. Default value is (1:obj.num_vars)
      if(nargin<3)
        xind = (1:obj.num_vars);
      end
      obj.lb(xind) = max([obj.lb(xind) cnstr.x_lb],[],2);
      obj.ub(xind) = min([obj.ub(xind) cnstr.x_ub],[],2);
      
    end
  end
end