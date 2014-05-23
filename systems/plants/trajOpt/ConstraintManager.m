classdef ConstraintManager
  %CONSTRAINTMANAGER
  % This is a container class for a set of general constraints (linear,
  % nonlinear, bounding box constraints) as well as slack variables.
  %
  % The idea is that some intuitive constraints are best formatted as a
  % related set of constraints (and, that there may be multiple ways to
  % describe these, which the user may wish to switch back and forth
  % between)
  
  properties (Access = protected)
    lincon = {}
    nlcon = {}
    bcon = {}
    n_slack = 0
  end
  
  methods
    function obj = ConstraintManager(lincon,nlcon,bcon,n)
      if nargin > 0
        if ~iscell(lincon)
          lincon = {lincon};
        end
        obj.lincon = lincon;
      end
      if nargin > 1
        if ~iscell(nlcon)
          nlcon = {nlcon};
        end
        obj.nlcon = nlcon;
      end
      if nargin > 2
        if ~iscell(bcon)
          bcon = {bcon};
        end
        obj.bcon = bcon;
      end
      if nargin > 3
        obj.n_slack = n;
      end
    end
    function lincon = getLinearConstraints(obj)
      lincon = obj.lincon;
    end
    
    function nlcon = getNonlinearConstraints(obj)
      nlcon = obj.nlcon;
    end
    
    function bcon = getBoundingBoxConstraints(obj)
      bcon = obj.bcon;
    end
    
    function n = getNumSlackVariables(obj)
      n = obj.n_slack;
    end
  end
  
end

