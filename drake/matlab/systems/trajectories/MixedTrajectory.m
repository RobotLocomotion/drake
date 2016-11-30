classdef MixedTrajectory < Trajectory
  % Creates a trajectory that concatenates dynamics.  In other words, if
  % you have two trajectories, one that is 2-dimensional and one that is
  % 1-dimensional, you can create a mixed trajectory that will be three
  % dimensional and will return the value of the first trajectory for the
  % first two dimensions and the value of the second trajectory for the
  % third dimension.
  %
  properties
    trajs % component trajectories
    indices % list of which component trajectories correspond to which output dimensions
    breaks % union of all breaks in the component trajectories
  end
  
  methods
    function obj = MixedTrajectory(trajs,indices)
        % MixedTrajectory constructor.
        % @param trajs trajectories (as a cell array) to concatenate
        % @param indices list of which trajectories should correspond to
        %    which dimensions in the output MixedTrajectory (as a cell
        %    array)
        %
        % @retval obj MixedTrajectory output
        
      obj = obj@Trajectory(max([indices{:}]));
      if (~iscell(trajs) || ~iscell(indices) || length(trajs)~=length(indices))
        error('trajs must be a cell array of component trajectories, and indices must be a list of the output indices to be drawn from each component.');
      end
      
      obj.trajs = trajs;
      obj.indices = indices;

      for i=1:length(trajs)
        typecheck(trajs{i},'Trajectory');
        if (~isvector(trajs{i}) || size(trajs{i},2)~=1) error('individual trajectories must be column vectors'); end
      end
      if (length(unique([indices{:}]))~=obj.dim) % check if 1:obj.dim are all in indices
        error('all indices must be accounted for.  some are missing.');
      end

      obj.breaks = getBreaks(trajs{1});
      for i=2:length(trajs)
        obj.breaks = [obj.breaks,getBreaks(trajs{i})];
      end
      obj.breaks = unique(obj.breaks);
      obj.tspan = [min(obj.breaks),max(obj.breaks)];
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
    
    function traj = subTrajectory(obj,ind)
      % check the case that ind is exactly one of the sub trajectories
      for i=1:length(obj.trajs)
        if (length(ind)==length(obj.indices{i}) && all(ind==obj.indices{i}))
          traj = obj.trajs{i};
          return;
        end
      end
      
      error('not implemented yet');
    end
    
    function obj = shiftTime(obj, tshift)
      for i=1:length(obj.trajs)
        obj.trajs{i} = obj.trajs{i}.shiftTime(tshift);
      end
      obj.tspan = obj.tspan + [tshift tshift];
      obj.breaks = obj.breaks + repmat(tshift, 1, length(obj.breaks));
    end
  end
end
