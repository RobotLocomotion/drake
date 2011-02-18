classdef MixedTrajectory < Trajectory
  
  properties
    trajs
    indices
    breaks
  end
  
  methods
    function obj = MixedTrajectory(trajs,indices)
      obj = obj@Trajectory(max([indices{:}]));
      if (~iscell(trajs) || ~iscell(indices) || length(trajs)~=length(indices))
        error('trajs must be a cell array of component trajectories, and indices must be a list of the output indices to be drawn from each component.');
      end
      
      obj.trajs = trajs;
      obj.indices = indices;
      
      obj.breaks = getBreaks(trajs{1});
      for i=2:length(trajs)
        obj.breaks = [obj.breaks,getBreaks(trajs{i})];
      end
      obj.breaks = unique(obj.breaks);
    end
    
    function ydot = deriv(obj,t)
      for i=1:length(obj.trajs)
        ydot(obj.indices{i},:) = deriv(obj.trajs{i},t);
      end
    end

    function y = eval(obj,t)
      for i=1:length(obj.trajs)
        y(obj.indices{i},:) = eval(obj.trajs{i},t);
      end
    end

    function dtraj = fnder(obj)
      for i=1:length(obj.trajs)
        dtrajs{i} = fnder(obj.trajs{i});
      end
      dtraj = MixedTrajectory(dtrajs,obj.indices);
    end
    
    function t = getBreaks(obj)
      t = obj.breaks;
    end
  end
end
