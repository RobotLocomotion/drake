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
    
    function a = subsasgn(a,s,b)
      if isa(a,'ConstantTrajectory')
        if isnumeric(b)
          subsasgn(a.pt,s,b);
        elseif isa(b,'ConstantTrajectory')
          subsasgn(a.pt,s,b.pt);
        else
          a = subsasgn@Trajectory(a,s,b);
        end
      else % b must be a ConstantTrajectory
        subsasgn(a,s,b.pt);
      end
    end
    
    function obj = uminus(obj)
      obj.pt = -obj.pt;
    end
    
    function a = ctranspose(a)
      a = ConstantTrajectory(a.pt');
    end
    
    function c = mtimes(a,b)
      if isa(a,'ConstantTrajectory') a=a.pt; 
      elseif ~isa(a,'numeric') c = mtimes@Trajectory(a,b); return; end
      if isa(b,'ConstantTrajectory') b=b.pt; 
      elseif ~isa(b,'numeric') c = mtimes@Trajectory(a,b); return; end
      c=ConstantTrajectory(a*b);
    end    
    
    function c = plus(a,b)
      if isa(a,'ConstantTrajectory') a=a.pt; 
      elseif ~isa(a,'numeric') c = plus@Trajectory(a,b); return; end
      if isa(b,'ConstantTrajectory') b=b.pt; 
      elseif ~isa(b,'numeric') c = plus@Trajectory(a,b); return; end
      c=ConstantTrajectory(a+b);
    end
    
    function c = inv(a)
      c = ConstantTrajectory(inv(a.pt));
    end
    
    function a = vertcat(varargin)
      pt = cell(1,nargin);
      for i=1:nargin
        if isa(varargin{i},'ConstantTrajectory') 
          pt{i}=varargin{i}.pt; 
        elseif isa(varargin{i},'numeric')
          pt{i}=varargin{i};
        else
          % abort... not handled here
          a = vertcat@Trajectory(varargin{:});
          return
        end
      end
      a = ConstantTrajectory(vertcat(pt{:}));
    end
  end
end
