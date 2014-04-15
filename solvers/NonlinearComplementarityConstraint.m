classdef NonlinearComplementarityConstraint < NonlinearConstraint
  %NONLINEARCOMPLEMENTARITYCONSTRAINT Summary of this class goes here
  %   Detailed explanation goes here
  % z >= 0, f(x) >=0, z'f(x) = 0
  % mode
  properties
    mode
  end
  
  properties (Constant)
    COMP_MODE = 1;
    FB_MODE = 2;
  end
  
  methods
    function obj = NonlinearComplementarityConstraint(dim,eval_handle,mode)
      if nargin < 3
        mode = NonlinearComplementarityConstraint.COMP_MODE;
      end
      switch(mode)
        case NonlinearComplementarityConstraint.COMP_MODE:
          
        case NonlinearComplementarityConstraint.FB_MODE:
          
      end
      obj = obj@NonlinearConstraint;
      
      obj.mode = mode;
    end
  end
  
end

