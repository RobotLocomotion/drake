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

      ts = mean(diff(tt));
      if (max(diff(tt))-ts>1e-6 || ts-min(diff(tt))>1e-6)
        error('tt doesn''t appear to have a fixed sample time'); 
      end
      obj = obj.setSampleTime([ts;0]);
      % touch up times to align perfectly with sample times:
      tt = round(tt/ts)*ts;
      
      obj.tt = tt;
      obj.xx = xx;
      obj.tspan = [min(tt) max(tt)];
    end
    function y = eval(obj,t)  
      ind = find(abs(obj.tt-t)<1e-10);  % only return on exact matches.  For interpolation, you should be using zoh or foh to make PPTrajectories
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
        if (length(plotdims)==1)
          h=stem(obj.tt,obj.xx(plotdims,:),'b','MarkerSize',5);
        else
          h=plot(obj.xx(plotdims(1),:),obj.xx(plotdims(2),:),'b.','MarkerSize',5);
        end
      end
    end
  end
end
