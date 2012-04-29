classdef Trajectory < RobotLibSystem
  
  properties
    dim
    tspan
  end
  
  methods (Abstract=true)
    y = eval(obj,t);
    t = getBreaks(obj); 
  end
 
  methods 
    function obj = Trajectory(dim)
      if (isnumeric(dim))
        numout = prod(dim);
      else
        numout = 1;  % just output nan for now
      end
      obj = obj@RobotLibSystem(0,0,0,numout,false,false);
      obj.dim = dim;
    end
    
    function x0 = getInitialState(obj)
      x0 = []; % no state
    end

    function xcdot = dynamics(obj,t,x,u)
      xcdot = []; % no state
    end
    
    function xdn = update(obj,t,x,u)
      xdn = [];  % no state
    end
    
    function y = output(obj,t,x,u)
      if (isnumeric(obj.dim))
        y = reshape(obj.eval(t),1,[]);
      else 
        y=nan;
      end
    end
    
    function dtraj = fnder(obj)
      error('not implemented yet');
    end

    function ydot = deriv(obj,t)
      ydot = eval(fnder(obj),t);
    end
    
    function obj = shiftTime(obj,offset)
      error('not implemented yet');
    end
    
    function s = size(obj,dim)
      s=obj.dim;
      if (length(s)==1) s=[s,1]; end
      if (nargin>1) s=s(dim); end
    end
    
    function traj = subTrajectory(obj,ind)
      error('not implemented yet');
    end
    
    function h=fnplt(obj,plotdims)
      if (nargin>1 && ~isempty(plotdims) && any(plotdims>obj.dim | plotdims<1)) error('plotdims out of range'); end
      breaks=obj.getBreaks();
      m=5; t=linspace(0,1,m)'; n=length(breaks)-1;
      ts = repmat(1-t,1,n).*repmat(breaks(1:end-1),m,1) + repmat(t,1,n).*repmat(breaks(2:end),m,1);
      ts = ts(:);
      pts = obj.eval(ts);
      if (prod(obj.dim)==1)
        h=plot(ts,squeeze(pts),'b.-','LineWidth',1,'MarkerSize',5);
      elseif (nargin>1 && ~isempty(plotdims) && length(plotdims)==1)
        h=plot(ts,squeeze(pts(plotdims,:)),'b.-','LineWidth',1,'MarkerSize',5);
      else
        if (nargin<2) plotdims=[1,2]; end
        h=plot(pts(plotdims(1),1:m:end),pts(plotdims(2),1:m:end),'b.',pts(plotdims(1),:),pts(plotdims(2),:),'b-','LineWidth',1,'MarkerSize',5);
      end
    end
    
    function [dmin,tmin,xwrapped] = distance(obj,x)
      
      t= obj.getBreaks();
      y = obj.eval(t);
      
      ywrap = y;
      if (~isempty(obj.bWrap))
        for i=find(obj.bWrap)  % implement wrapping
          p = obj.xWrap(i,2)-obj.xWrap(i,1);
          ywrap(i,:) = mod(y(i,:)-x(i)+p/2,p)+x(i)-p/2;
        end
      end
      xbar = repmat(x,1,size(y,2)) - ywrap;
        
      d = sum(xbar.^2,1);
      [dmin,imin] = min(d);

      xwrapped = x+y(:,imin)-ywrap(:,imin);
%      tspan = [t(max(1,imin-1)), t(min(length(t),imin+1))];
      tspan = [t(1),t(end)];
      options = optimset('fminbnd');
      options.TolX = 1e-10;
      [tmin,dmin]=fminbnd(@(t) sum((xwrapped-obj.eval(t)).^2,1),tspan(1),tspan(end),options);
      dmin = sqrt(dmin);
    end
        
    function [v,dvdx] = vectorTo(obj,x)
      [d,tmin,xwrapped] = distance(obj,x);
      xmin = obj.eval(tmin);
      v=xmin-xwrapped;

      if (nargout>1)
        t = obj.getBreaks();
        dtraj = fnder(obj);
        xdmin = dtraj.eval(tmin);
 
        if (min(abs(tmin-t([1,end])))<=1e-6) % then I'm at one of the rails (value should be larger than options.tolX above)
          dtmindx = zeros(1,obj.dim);
        else
          ddtraj = fnder(dtraj);
          xddmin = ddtraj.eval(tmin);
          dtmindx = xdmin'/(xdmin'*xdmin + v'*xddmin);
        end
        
        dvdx = xdmin*dtmindx - eye(obj.dim);
      end
    end

    
    function alpha = getParameters(obj)
      error('parameters are not implemented for this type of trajectory'); 
    end
    
    function alpha = setParameters(obj)
      error('parameters are not implemented for this type of trajetory'); 
    end
    
    
  end
end
