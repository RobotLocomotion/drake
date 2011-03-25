classdef Trajectory% < RobotLibSystem
  
  properties
    dim
    tspan
    bWrap=[]
    xWrap=[]
  end
  
  methods (Abstract=true)
    y = eval(obj,t);
    t = getBreaks(obj); 
  end
 
  methods 
    function obj = Trajectory(dim)
%      obj = obj@RobotLibSystem(0,0,0,dim,false,false);
      obj.dim = dim;
    end
    
    function dtraj = fnder(obj)
      error('not implemented yet');
    end

    function ydot = deriv(obj,t)
      ydot = eval(fnder(obj.pp),t);
    end
    
    function obj = shiftTime(obj,offset)
      error('not implemented yet');
    end
    
    function sizecheck(obj,sizemat)
      s = obj.dim;
      if (length(sizemat)<2)
        if (sizemat==0) sizemat = [0,0]; 
        else sizemat=[sizemat,1]; end 
      end
      if (length(s)<2) s = [s,1]; end
      if (length(s)~=length(sizemat) || any(s~=sizemat))
        error(['Wrong size.  Expected [',num2str(sizemat),'] but got a [', num2str(s), '] instead.']);
      end
    end
      
    function h=fnplt(obj,plotdims)
      if (nargin<2) plotdims=[1,2]; end
      if (obj.dim~=2) error('not implemented yet'); end
      breaks=obj.getBreaks();
      pts = obj.eval(breaks);
      panels = [];
      if (isempty(obj.bWrap) || isempty(obj.xWrap))
        plot(pts(plotdims(1),:),pts(plotdims(2),:),'b.-','LineWidth',1,'MarkerSize',5);
      else
        for i=1:obj.dim %find(obj.bWrap)
          if (obj.bWrap(i))
            panel = unique(floor((pts(i,:)-obj.xWrap(i,1))/(obj.xWrap(i,2)-obj.xWrap(i,1))));
            % plot entire trajectory over, shifted appropriately for each unique panel
            % (the easiest way to get the lines across the
            % boundaries correct)
            panels = [repmat(panels,1,length(panel)); reshape(repmat(panel,max(length(panels),1),1),1,[])];
          else
            panels = [panels;zeros(1,max(1,size(panels,2)))];
          end
        end
        for i=1:size(panels,2)
          shift = panels(:,i).*(obj.xWrap(:,2)-obj.xWrap(:,1));
          shift(isnan(shift)) = 0;
          ppts = pts - repmat(shift,1,size(pts,2));
          plot(ppts(plotdims(1),:),ppts(plotdims(2),:),'b.-','LineWidth',1,'MarkerSize',5);
        end
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
