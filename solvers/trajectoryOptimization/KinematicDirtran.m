classdef KinematicDirtran < KinematicTrajectoryOptimization ...
                            & DirtranTrajectoryOptimization
  properties
    v_inds
  end
  methods
    function obj = KinematicDirtran(robot,N,duration,options)
      plant = KinematicDummyPlant(robot);
      parent_args = {plant,N,duration};
      if nargin > 3, parent_args{end+1} = options; end
      obj = obj@DirtranTrajectoryOptimization(parent_args{:});
      obj = obj@KinematicTrajectoryOptimization(robot);
      %obj.v_inds = obj.u_inds;
      obj.v_inds = obj.x_inds(robot.getNumPositions()+(1:robot.getNumVelocities()),:);
    end
  end
end
