classdef LTIControl < PolynomialSystem
% Implements a linear time-invariant control policy.
  
  methods 
    function obj=LTIControl(x0,u0,K)
      % Constructor for LTI control.
      %
      % @param x0 the state where error is zero (ie u = u0 at this state)
      % @param u0 control action when error is zero
      % @param K gain matrix
        
      obj = obj@PolynomialSystem(0,0,length(x0),length(u0),true,true,[],[],[]);
      obj.x0 = x0;
      obj.u0 = u0;
      obj.K = K;
      obj = pullEmptyPolysFromMethods(obj);
    end
    
    function ts = getSampleTime(obj)
      % make sure that this static system uses an inherited sample time
      ts = [-1;0];  % inherited sample time
    end
    
    function [u,du] = output(obj,~,~,x)
      % Implements the actual control function.
      %
      % u = u0 - K * (x - x0)
      %
      % @param x current state 
      %
      % @retval u control action
      % @retval du gradients (in this case, du = [zeros(1,size(K,1)), -K] )
      
% input wrapping is not smooth.  unfortunately, it doesn't belong here
%      u = obj.u0-obj.K*obj.wrapInput(x-obj.x0);

      u = obj.u0-obj.K*(x-obj.x0);
      if (nargout>1)
        du = [zeros(1,size(K,1)),-K];
      end
    end
    
  end
  
  properties 
    x0=[]; % Fixed point (ie state where error is zero)
    u0=[]; % Nominal control value (control value when error is zero)
    K = []; % Gain matrix
  end
end
