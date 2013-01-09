classdef TrajectoryTree < TrajectoryLibrary
% a trajectory library with a hieararchy

  properties
    parent=[]
  end
 
  methods
    function obj = TrajectoryTree(frame)
      obj = obj@TrajectoryLibrary(frame);
    end
    
    function obj = addTrajectory(obj,traj,parent)
      typecheck(parent,{'Trajectory','double'});
      if isa(parent,'Trajectory')
        parent = find([parent==obj.traj]);
        if isempty(parent)
          error('couldn''t find parent in the trajectory library');
        end
      else
        parent = ind2sub(size(obj.traj),parent);  % will throw an error if parent is an invalid index
      end
      obj = addTrajectory(obj,traj);
      obj.parent(end+1)=parent;
    end    
  end
end