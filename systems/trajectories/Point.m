classdef Point

  properties (SetAccess=private,GetAccess=private)
    p
    frame
  end

  methods
    function obj=Point(frame,p)
      typecheck(frame,'CoordinateFrame');
      if (nargin<2)
        p = zeros(frame.dim,1);
      else
        typecheck(p,'double');
        sizecheck(p,[frame.dim,1]);
      end
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
    
    function n = numel(obj)
      n=1;
    end
    
    function display(obj)
      for i=1:length(obj.p)
        fprintf(1,'%20s = %f\n',obj.frame.coordinates{i},obj.p(i));
      end
    end
    
    function varargout = subsasgn(obj,s,val)
      % support the syntax pt.theta = .5 , where theta is a coordinate name
      if (length(s)==1 && strcmp(s(1).type,'.'))
        tf=strcmp(s(1).subs,obj.frame.coordinates);
        if any(tf)
          ind = find(tf,1);
          obj.p(ind)=val;
          varargout = {obj};
          return;
        end
      end
      % otherwise, call the builting subsasgn
      varargout=cell(1,nargout);
      [varargout{:}] = builtin('subsasgn',obj,s,val);
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
