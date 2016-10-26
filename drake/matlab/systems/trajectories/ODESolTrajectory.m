classdef ODESolTrajectory < Trajectory
  
  properties
    sol=[];
    shape;
  end
  
  methods 
    function obj = ODESolTrajectory(sol,shape)
      obj = obj@Trajectory(size(sol.y,1));
      if (nargin>0)
        obj.sol = sol;
        if (nargin>1)
          obj.shape=shape;
        end
        obj.tspan = [min(obj.sol.x), max(obj.sol.x)];
      end
    end
    
    function y = eval(obj,t)
      if (any(t<obj.tspan(1)) || any(t>obj.tspan(end))) 
        error('outside interval'); 
      end
      y = deval(obj.sol,t);
      if isempty(obj.shape)
        return;
      elseif iscell(obj.shape)
        nt = length(t);
        offset = 0;
        for i=1:length(obj.shape)
          n = prod(obj.shape{i});
          yout{i} = reshape(y(offset + (1:n),:),[obj.shape{i},nt]);
          offset = offset+n;
        end
        y=yout;
      else
        y = reshape(y,obj.shape);
      end
    end
    function ydot = deriv(obj,t)
      ydot = obj.sol.extdata.odefun(t,deval(obj.sol,t));
      if isempty(obj.shape)
        return;
      elseif iscell(obj.shape)
        offset = 0;
        for i=1:length(obj.shape)
          n = prod(obj.shape{i});
          ydotout{i} = reshape(ydot(offset + (1:n)),obj.shape{i});
          offset = offset+n;
        end
        ydot=ydotout;
      else
        ydot = reshape(ydot,obj.shape);
      end
    end
    
    function t = getBreaks(obj)
      if (obj.sol.x(end)<obj.sol.x(1)) % then it was solved backwards in time.  ok to reverse
        t = obj.sol.x(end:-1:1);
      else
        t = obj.sol.x;
      end
    end
    
    function pptraj=flipToPP(obj)
      t = getBreaks(obj);
      y = deval(obj.sol,t); ydot=y;
      for i=1:length(t)  % don't know if odefun was vectorized
        ydot(:,i) = obj.sol.extdata.odefun(t(i),y(:,i));
      end
      if isempty(obj.shape)
        pptraj = PPTrajectory(pchipDeriv(t,y,ydot));
      elseif iscell(obj.shape)
        offset=0;
        for i=1:length(obj.shape)
          n=prod(obj.shape{i});
          pptraj{i} = PPTrajectory(pchipDeriv(t,y(offset + (1:n),:),ydot(offset + (1:n),:),ydot(offset + (1:n),:),obj.shape{i}));
          offset = offset+n;
        end
      else
        pptraj = PPTrajectory(pchipDeriv(t,y,ydot,ydot,obj.shape));
      end
    end
  end
  
end
