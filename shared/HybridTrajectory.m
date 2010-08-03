classdef HybridTrajectory < Trajectory
  
  % HybridTrajectory: container class for a set of continuous trajectories 
  % punctuated by collision events
  
  properties
    traj
    te
  end
  
  methods
    function obj = HybridTrajectory(trajectories)
      if (nargin>0)
        if ~iscell(trajectories), trajectories = {trajectories}; end
        obj.traj = trajectories;
        % check that one trajectory follows the other sequentially in time
        te = trajectories{1}.getBreaks();  
        obj.tspan = [te(1),te(end)];
        te = te(end);
        for i=2:length(trajectories)
          t = trajectories{i}.getBreaks();
          if (t(1)~=te(end)) error('trajectories must line up in time'); end
          te = [te,t(end)];
          obj.tspan(2) = t(end);
        end
        obj.te = te(1:(end-1));
      end
    end
    
    function te = getEvents(obj)
      te = obj.te;
    end
    
    function t = getBreaks(obj)
      t=obj.traj{1}.getBreaks();
      for i=2:length(obj.traj)
        b = obj.traj{i}.getBreaks();
        t = [t,b(2:end)];
      end
    end
    
    function y = eval(obj,t)
      if (any(t<obj.tspan(1)) || any(t>obj.tspan(end))) 
        error('outside tspan'); 
      end
      if (length(t)==1)
        trajind=find(t>[obj.tspan(1),obj.te],1,'last');
        y = obj.traj{trajind}.eval(t);
      else  % vectorized version
        t=sort(t);
        ind = 1;
        for i=1:length(obj.te)
          nind = find(t(ind:end)>obj.te(i),1,'first');
          if (isempty(nind)) break; end %then we're done
          if (nind==1) continue; end
          c = ind:(ind+nind-2);
          y(:,c) = obj.traj{i}.eval(t(c));
          ind=ind+(nind-1);
        end
      end
    end
  end
  
end