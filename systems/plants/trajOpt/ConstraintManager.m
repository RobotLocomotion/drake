classdef ConstraintManager
  %TRAJOPTCONSTRAINTGENERATOR Summary of this class goes here
  %   Detailed explanation goes here
  
  properties (SetAccess = protected)
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
        obj.n = n;
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
    
    n = getNumSlackVariables(obj)
  end
  
end

