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
        if isscalar(p), p = repmat(p,frame.dim,1); end
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
    
    function v = end(obj,k,n)
      v = feval('end',obj.p,k,n);
    end
    
    function display(obj)
      coordinates = obj.frame.getCoordinateNames();
      for i=1:length(obj.p)
        fprintf(1,'%20s = %f\n',coordinates{i},obj.p(i));
      end
    end
    
    function varargout = subsasgn(obj,s,val)
      % support the syntax pt.theta = .5 , where theta is a coordinate name
      if (length(s)==1 && strcmp(s(1).type,'.'))
        tf=strcmp(s(1).subs,obj.frame.getCoordinateNames());
        if any(tf)
          ind = find(tf,1);
          obj.p(ind)=val;
          varargout = {obj};
          return;
        else
          error('Drake:Point:BadAssign',sprintf('unknown coordinate name %s',s(1).subs));
        end
      end
      % otherwise, call the builting subsasgn
      varargout = {builtin('subsasgn',obj.p,s,val)};
    end
    
    function varargout = subsref(obj,s)
      % support the syntax pt.theta, where theta is a coordinate name
      if (length(s)==1 && strcmp(s(1).type,'.'))
        tf=strcmp(s(1).subs,obj.frame.getCoordinateNames());
        if any(tf)
          ind = find(tf,1);
          varargout = {obj.p(ind)};
          return;
        end
      elseif length(s)<2 && strcmp(s(1).type,'()')
        varargout{1} = builtin('subsref',obj.p,s);  % result is a double, not a Point (since it might not be the right size, etc)
        return;
      end
      % otherwise, call the builting subsref
      varargout=cell(1,max(nargout,1));  % max w/ 1 to support command line access (with no outputs, but writes to ans)
      [varargout{:}] = builtin('subsref',obj,s);
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
    
    function varargout = valuecheck(val,desired_val,varargin)
      if isa(desired_val,'Point')
        val = val.inFrame(desired_val.frame);
      end
      val = double(val);
      desired_val = double(desired_val);
      if nargout>0
        varargout=cell(1,nargout);
        [varargout{:}] = valuecheck(val,desired_val,varargin{:});
      else
        valuecheck(val,desired_val,varargin{:});
      end
    end
  end
  
end
