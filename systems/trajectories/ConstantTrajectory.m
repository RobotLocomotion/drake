classdef ConstantTrajectory < Trajectory
% Trivial instance of a trajectory as a constant.
  
  properties
    pt
  end
  
  methods 
    function obj = ConstantTrajectory(pt)
    % Construct ConstantTrajectory from a pt
      obj = obj@Trajectory(size(pt));
      obj.pt=pt;
      obj.tspan = [-inf,inf];
    end
    
    function a = double(obj)
      a=double(obj.pt);
    end
    
    function dtraj = fnder(obj)
    % Implements the (trivial) derivative.
      dtraj = ConstantTrajectory(0*obj.pt);
    end
    
    function y = eval(obj,t)
    % Return the fixed point for all t.
      y = obj.pt;
    end
    
    function t = getBreaks(obj)
    % Return a single break (at t=0).
      t = 0;
    end
    
    function obj = uminus(obj)
      obj.pt = -obj.pt;
    end
    
    function c = mtimes(a,b)
      if isa(a,'ConstantTrajectory') a=a.pt; end
      if isa(b,'ConstantTrajectory') b=b.pt; end
      c=ConstantTrajectory(a*b);
    end    
    
    function c = plus(a,b)
      if isa(a,'ConstantTrajectory') a=a.pt; end
      if isa(b,'ConstantTrajectory') b=b.pt; end
      c=ConstantTrajectory(a+b);
    end
    
    function a = vertcat(varargin)
      for i=1:length(varargin)
        if isa(varargin{i},'ConstantTrajectory') varargin{i}=varargin{i}.pt; end
      end
      a = ConstantTrajectory(vertcat(varargin{:}));
    end
  end
end
