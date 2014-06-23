classdef CompositeConstraint
  %CompositeConstraint
  % This is a container class for a set of general constraints
  % and slack variables.
  % The idea is that some intuitive constraints are best formatted as a
  % related set of constraints (and, that there may be multiple ways to
  % describe these, which the user may wish to switch back and forth
  % between)
  
  properties
    constraints = {}
    n_slack = 0
  end
  
  methods
    function obj = ConstraintManager(constraints,n)
      if nargin > 0
        obj.constraints = constraints;
      end
      if nargin > 1
        obj.n_slack = n;
      end
    end

    
    function obj = addConstraints(obj, constraints)
      obj.constraints = [obj.constraints;constraints];
    end
    
    function obj = addSlackVariables(obj, n)
      obj.n_slack = obj.n_slack + n;
    end
  end
  
end

 