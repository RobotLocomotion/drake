classdef ConstantTrajectory < Trajectory
% Trivial instance of a trajectory as a constant.
  
  properties
    pt
  end
  
  methods 
    function obj = ConstantTrajectory(pt)
    % Construct ConstantTrajectory from a pt
      obj = obj@Trajectory(size(pt));
      if isa(pt,'Point')
        obj.pt = double(pt);
        obj = setOutputFrame(obj,getFrame(pt));
      else
        obj.pt=pt;
      end
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
      if isscalar(t)
        y=obj.pt;
      else
        d = size(obj.pt);
        dd = 0*d+1;  % 1 for all values
        if (d(end)==1), 
          dd(end) = length(t);
        else
          dd(end+1) = length(t);
        end
        y = repmat(obj.pt,dd);
      end
    end
    
    function t = getBreaks(obj)
    % Return a single break (at t=0).
      t = 0;
    end
    
    function a = subsasgn(a,s,b)
      if isa(b,'PPTrajectory')
        breaks = getBreaks(b);
        a = PPTrajectory(zoh(breaks,repmat(a.pt,1,length(breaks))));
        a = subsasgn(a,s,b);
        return;
      end
      if isa(a,'ConstantTrajectory')
        if isnumeric(b)
          a = subsasgn(a.pt,s,b);
        elseif isa(b,'ConstantTrajectory')
          a = subsasgn(a.pt,s,b.pt);
        else
          a = subsasgn@Trajectory(a,s,b);
        end
      else % b must be a ConstantTrajectory
        a = subsasgn(a,s,b.pt);
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
