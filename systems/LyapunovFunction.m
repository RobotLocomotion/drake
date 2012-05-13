classdef LyapunovFunction 
% interface class for Lyapunov functions
  
  properties (SetAccess=private,GetAccess=public)
    frame
    time_invariant_flag = false;
  end

  methods (Abstract=true)
    V = eval(t,x);  
%    plotLevelSet();
  end
  
  methods 
    function obj=LyapunovFunction(frame,time_invariant_flag)
      typecheck(frame,'CoordinateFrame');
      obj.frame = frame;
      if (nargin>1)
        typecheck(time_invariant_flag,'logical');
        obj.time_invariant_flag = time_invariant_flag;
      end
    end
  end
end
