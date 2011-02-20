classdef LTIControl < RobotLibSystem
% Implements a linear time-invariant control policy
  
  methods 
    function obj=LTIControl(x0,u0,K,S)
      obj = obj@RobotLibSystem(0,0,length(x0),length(u0),true,true);
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
    
    function u = output(obj,t,junk,x)
      % implements the actual control function
      %      x = wrap(obj,obj.x0,x);
      u = obj.u0-obj.K*obj.wrapInput(x-obj.x0);
    end
    
    function du = controlGradients(obj,t,x,order)
      % Computes Taylor expansion of the control function
      if (nargin<4) order=1; end
      du{1} = [zeros(1,size(K,1)),-K];
      if (order>1) error('not implemented yet');  end % it's all zeros... just need to fill in the struct 
    end
  end
  
  properties 
    x0=[];
    u0=[];
    K = [];
    S = [];
  end
end