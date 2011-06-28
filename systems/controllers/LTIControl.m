classdef LTIControl < SmoothRobotLibSystem
% Implements a linear time-invariant control policy
  
  methods 
    function obj=LTIControl(x0,u0,K,S)
      obj = obj@SmoothRobotLibSystem(0,0,length(x0),length(u0),true,true);
      obj.x0 = x0;
      obj.u0 = u0;
      obj.K = K;
      if (nargin>3)
        obj.S = S;
      end
    end
    
    function ts = getSampleTime(obj)
      % make sure that this static function uses an inherited sample time
      ts = [-1;0];  % inherited sample time
    end
    
    function [u,du] = output(obj,t,junk,x)
      % implements the actual control function
      %      x = wrap(obj,obj.x0,x);
      u = obj.u0-obj.K*obj.wrapInput(x-obj.x0);
      if (nargout>1)
        du = [zeros(1,size(K,1)),-K];
      end
    end
    
  end
  
  properties 
    x0=[];
    u0=[];
    K = [];
    S = [];
  end
end