classdef LTIControl < AffineSystem
% Implements a linear time-invariant control policy, u = u0 - K*(x-x0)
  
  methods 
    function obj=LTIControl(x0,u0,K)
      % Constructor for LTI control.
      %
      % @param x0 the state where error is zero (ie u = u0 at this state)
      % @param u0 control action when error is zero
      % @param K gain matrix
        
      obj = obj@AffineSystem([],[],[],[],[],[],[],-K,u0+K*x0);
      obj.x0 = x0;
      obj.u0 = u0;
      obj.K = K;
    end
    
  end
  
  properties 
    x0=[]; % Fixed point (ie state where error is zero)
    u0=[]; % Nominal control value (control value when error is zero)
    K = []; % Gain matrix
  end
end
