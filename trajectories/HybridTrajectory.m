classdef HybridTrajectory < Trajectory
% Container class for a set of continuous trajectories punctuated by collision events
  
  properties
    traj
    te
  end
  
  methods
    function obj = HybridTrajectory(trajectories)
      % Construct container from a cell array of Trajectories
      obj = obj@Trajectory(0);  % todo: set DIM better here
      if (nargin>0)
        if ~iscell(trajectories), trajectories = {trajectories}; end
        obj.traj = trajectories;
        % check that one trajectory follows the other sequentially in time
        te = trajectories{1}.getBreaks();  
        obj.tspan = [te(1),te(end)];
        te = te(end);
        for i=2:length(trajectories)
          t = trajectories{i}.getBreaks();
          if (abs(t(1)-te(end))>1e-10) error('trajectories must line up in time'); end
          te = [te,t(end)];
          obj.tspan(2) = t(end);
        end
        obj.te = te(1:(end-1));
      end
    end
    
    function te = getEvents(obj)
      % return the events (times where the trajectory switches)
      te = obj.te;
    end
    
    function t = getBreaks(obj)
      % return the combined break points of all trajectories with event times included.
      t=obj.traj{1}.getBreaks();
      for i=2:length(obj.traj)
        b = obj.traj{i}.getBreaks();
        t = [t,b(2:end)];
      end
    end
    
    function y = eval(obj,t)
      % look into and evaluate correct trajectory based on the time
      if (any(t<obj.tspan(1)) || any(t>obj.tspan(end)))
        error('outside tspan');
      end
      if (length(t)==1)
        trajind=find(t>=[obj.tspan(1),obj.te],1,'last');
        y = obj.traj{trajind}.eval(t);
      else  % vectorized version
        t=sort(t);
        ind = 1;
        for i=1:length(obj.te)+1
          nind = [];
          if(i<=length(obj.te))
            nind = find(t(ind:end)>obj.te(i),1,'first');  % first (relative) index beyond this segment
          end
          if (isempty(nind)) % the rest must belong to this segment
            c = ind:length(t);
          elseif (nind==1) % none in this segment
            continue;
          else
            c = ind:(ind+nind-2);
          end
          y(:,c) = obj.traj{i}.eval(t(c));
          ind=ind+nind-1;
        end
      end
    end
    
  end
end