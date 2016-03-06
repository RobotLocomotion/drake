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
    function obj = CompositeConstraint(constraints,n_slack)
      % @param constraints  A cell of Constraint objects
      % @param n_slack     The number of slack variables.
      if nargin > 0
        if ~iscell(constraints)
          if ~isa(constraints,'Constraint')
            error('Drake:CompositeConstraint:InvalidArgument','constraints argument must be a cell-array of constraints or a Constraint type');
          end
          constraints = {constraints};
        end
        obj.constraints = constraints;
      end
      if nargin > 1
        obj.n_slack = n_slack;
      end
    end

    
    function obj = addConstraints(obj, constraints)
      obj.constraints = [obj.constraints;constraints];
    end
    
  end
  
end

 