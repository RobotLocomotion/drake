classdef LTVControl < RobotLibSystem
% Time-varying linear feedback

  methods 
    function obj=LTVControl(x0,u0,K,S,Sdot)
      obj = obj@RobotLibSystem(0,0,x0.dim,u0.dim,true,true);
      obj.x0 = x0;
      obj.u0 = u0;
      obj.K = K;
      if (nargin>3)
        obj.S = S;
        if (nargin>4)
          obj.Sdot = Sdot;
        end
      end
    end
    
    function ts = getSampleTime(obj)
      % make sure that this static function uses an inherited sample time
      ts = [-1;0];  % inherited sample time
    end
    
    function u = output(obj,t,junk,x)
      % implements the actual control function
      %      x = wrap(obj,obj.x0,x);
      u = obj.u0.eval(t)-obj.K.eval(t)*(x-obj.x0.eval(t));
    end
    
    function du = controlGradients(obj,t,x,order)
      error('not implemented yet');
    end
  end
  
  properties 
    x0=[];
    u0=[];
    K = [];
    S = [];
    Sdot = [];
  end

end
