classdef LQRControl < Control
% Interface class that wraps TI and TV LQR design and access features
%
% Type <a href="matlab: help lqr_trajectory>lqr_trajectory">help lqr_trajectory>lqr_trajectory</a> for help with the constructor.

  methods 
    function obj = LQRControl(num_x,num_u)
      % construct an LQR controller
      obj = obj@Control(num_x,num_u);
    end
  end
  
  methods (Abstract=true)
    [obj,K,S] = design(obj);
    [obj,rho] = verify(obj);

    J = costToGo(obj,t,x);
    [bVerified,verified_time,confidence] = isVerified(obj,x,time_to_check);

    plotNominal(obj,options);
    plotFunnel(obj,options);
    plotFunnel3(obj,options); 
  end

  methods
    function obj = setWrapping(obj,bWrap,xWrap)
      % sets up wrapping coordinate systems (e.g., trig systems live on the torus)  
      
      % todo: consider yanking this in favor of the
      % dynamics.stateVectorNorm, etc.
      
      % bWrap is a vector of booleans of length(dim(x))
      % xWrap is a matrix of [low,high] values of size (dim(x),2)
      if (length(bWrap)~=size(xWrap,1)) error('xWrap must be the same length as bWrap'); end
      if (size(xWrap,2)~=2) error('xWrap should be dim(x) by 2'); end
      obj.bWrap = bWrap;
      obj.xWrap = xWrap;
    end
    
    function x = wrap(obj,x0,x)
      % implement the wrapping.  
      
      % note: this is not necessarily optimal (e.g., if
      % the pendulum is in theta=0, but has so much velocity that the lower
      % cost-to-go is from theta=2*pi.  but that case seems unlikely to occur and
      % also unlikely to work well, so it's skipped for efficiency.
      
      % note: this should handle the case that bWrap and xWrap have not been
      % set.
      for i=find(obj.bWrap)
        p = obj.xWrap(i,2)-obj.xWrap(i,1);
        x(i,:) = mod(x(i,:)-x0(i)+p/2,p)+x0(i)-p/2;
      end
    end
      
  end
  
  properties (SetAccess=protected,GetAccess=public)
    bWrap=[];  % bWrap(i) = true if x(i) is wrapped
    xWrap=[];  % xWrap(i,1) is lower limit xwrap(i,2) is upper limit
  end
  
end

    
