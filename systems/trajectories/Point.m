classdef Point

  properties (SetAccess=private,GetAccess=private)
    p
    frame
  end

  methods
    function obj=Point(frame,p)
      typecheck(frame,'CoordinateFrame');
      typecheck(p,'double');
      sizecheck(p,[frame.dim,1]);
      obj.p = p;
      obj.frame = frame;
    end
    
    function fr = getFrame(obj)
      fr = obj.frame;
    end
    
    function p = double(obj)
      p = obj.p;
    end
    
    function s = size(obj,varargin)
      s = size(obj.p,varargin{:});
    end
    
    function pobj = inFrame(obj,fr,t)
      if (fr == obj.frame)
        pobj = obj;
      else
        tf = findTransform(obj.frame,fr,struct('throw_error_if_fail',true));
        if isTI(tf) t=0; elseif nargin<2, error('you must specify a time'); end
        if getNumStates(tf)>0, error('transform should not have state'); end
        pobj = Point(fr,tf.output(t,[],obj.p));
      end
    end
  end
  
end
