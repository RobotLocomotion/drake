classdef DTTrajectory < Trajectory
  
  properties
    tt
    xx
  end
  
  methods
    function obj = DTTrajectory(tt,xx)
      obj = obj@Trajectory(size(xx,1));
      typecheck(tt,'double');
      typecheck(xx,'double');
      if (~isrow(tt)) error('tt should be a 1xn list of times'); end
      if (size(tt,2)~=size(xx,2)) error('xx must have the same number of columns as tt'); end
      if (~issorted(tt)) error('tt should be monotonically increasing'); end
      obj.tt = tt;
      obj.xx = xx;
      obj.tspan = [min(tt) max(tt)];
    end
    function y = eval(obj,t)  
      ind = find(obj.tt==t);  % only return on exact matches.  For interpolation, you should be using zoh or foh to make PPTrajectories
      y = obj.xx(:,ind);
    end
    
    function t = getBreaks(obj)
      t = obj.tt;
    end
    
    function h=fnplt(obj,plotdims)
      if (prod(obj.dim)==1)
        h=stem(obj.tt,obj.xx,'b');
      else
        if (nargin<2) plotdims=[1,2]; end
        h=plot(obj.xx(plotdims(1),:),obj.xx(plotdims(2),:),'b.','MarkerSize',5);
      end
    end
  end
end
