classdef DTTrajectory < Trajectory
  
  properties
    ts
    xs
  end
  
  methods
    function obj = DTTrajectory(ts,xs)
      obj = obj@Trajectory(size(xs,1));
      typecheck(ts,'double');
      typecheck(xs,'double');
      if (~isrow(ts)) error('ts should be a 1xn list of times'); end
      if (size(ts,2)~=size(xs,2)) error('xs must have the same number of columns as ts'); end
      if (~issorted(ts)) error('ts should be monotonically increasing'); end
      obj.ts = ts;
      obj.xs = xs;
      obj.tspan = [min(ts) max(ts)];
    end
    function y = eval(obj,t)  
      ind = find(obj.ts==t);  % only return on exact matches.  For interpolation, you should be using zoh or foh to make PPTrajectories
      y = obj.xs(:,ind);
    end
    
    function t = getBreaks(obj)
      t = obj.ts;
    end
    
    function h=fnplt(obj,plotdims)
      if (prod(obj.dim)==1)
        h=stem(obj.ts,obj.xs,'b');
      else
        if (nargin<2) plotdims=[1,2]; end
        h=plot(obj.xs(plotdims(1),:),obj.xs(plotdims(2),:),'b.','MarkerSize',5);
      end
    end
  end
end
