classdef TimeVaryingLinearSystem < TimeVaryingPolynomialSystem
% dynamics, update, output are linear in x and u, but not necessarily in t

  methods 
    function obj=LTVControl(x0,u0,K,S,Sdot)
      obj = obj@SmoothRobotLibSystem(0,1,x0.dim,u0.dim,true,true);
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
    
    function t0=getInitialState(obj)
      t0=-1e6;
    end
    function t0=getInitialStateWInput(obj,t,t0,x)
      t0=t;  % todo: call function to figure out best initial time
    end
    
    function ts = getSampleTime(obj)
      % make sure that this static function uses an inherited sample time
      ts = [-1;0];  % inherited sample time
    end
    
    function t0n = update(obj,t,t0,x)
      t0n=t0;  % do nothing
    end
    
    function u = output(obj,t,t0,x)
      % implements the actual control function
      %      x = wrap(obj,obj.x0,x);
      
      t = t-t0;
      if (t<obj.x0.tspan(1)) t=obj.x0.tspan(1); end
      if (t>obj.x0.tspan(end)) t=obj.x0.tspan(end); end
      K = obj.K.eval(t);
      if (iscell(K))
        u = obj.u0.eval(t)-K{1}*(x-obj.x0.eval(t))-K{2};
      else
        u = obj.u0.eval(t)-K*(x-obj.x0.eval(t));
      end
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
