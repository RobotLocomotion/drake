classdef LyapunovFunction 
% interface class for Lyapunov functions
  
  properties (SetAccess=protected,GetAccess=private)
    frame
    time_invariant_flag = false;
  end

  methods (Abstract=true)
    V = eval(obj,t,x);  
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
    
    function y = getLevelSet(obj,t,options)
      error('not implemented yet');
    end
    
    function fr=getFrame(obj)
      fr = obj.frame;
    end
    
    function obj=setFrame(obj,fr) 
      typecheck(fr,'CoordinateFrame');
      valuecheck(fr.dim,obj.frame.dim);
      obj.frame = fr;
    end
    
    function V=inFrame(obj,frame)
      if (frame==obj.getFrame)
        V = obj;
      else
        % if obj.getFrame.prefix = x and frame.prefix = y, then 
        % I have V(x) and want to return V(f(y)), where x = f(y) is the 
        % transform *from frame to obj.getFrame*
        tf = findTransform(frame,obj.getFrame,struct('throw_error_if_fail',true));
        if getNumStates(tf)>0 error('not implemented yet'); end
        Vfun = @(t,x) obj.eval(t,tf.output(t,[],x));
        V = FunctionHandleLyapunovFunction(frame, Vfun, obj.isTI() && tf.isTI());
      end
    end
    
    function b = isTI(obj)
      b = obj.time_invariant_flag;
    end
  end
end
