classdef FunctionHandleLyapunovFunction < LyapunovFunction
  
  properties
    Vfun
  end
  
  methods
    function obj = FunctionHandleLyapunovFunction(frame,Vfun,time_invariant_flag)
      if (nargin<3) time_invariant_flag = false; end
      obj = obj@LyapunovFunction(frame,time_invariant_flag);
      typecheck(Vfun,'function_handle');
      if (nargin(Vfun)~=2)
        error('Vfun should take two inputs:  Vfun(t,x)');
      end
      obj.Vfun = Vfun;
    end
    
    function V = eval(obj,t,x)
      V = feval(obj.Vfun,t,x);
    end
  end
  
end
