classdef KinematicDirtran < KinematicTrajectoryOptimization ...
                            & DirtranTrajectoryOptimization
  % Trajectory planning problem in which the acceleration of the joints is
  % taken to be directly controllable. The decision variables are the joint
  % postitions (q), velocities (v), and accelerations (u) at each knot point.
  % Linear interpolation is used to enforce the relationships between those
  % quantities as described in DirtranTrajectoryOptimization.
  properties
    v_inds % nv x N array of indices into the decision variable vector
           % corresponding to the velocities at each knot point.
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

    function obj = addConstraint(obj, constraint, varargin)
      if isa(constraint, 'RigidBodyConstraint')
        obj = addRigidBodyConstraint(obj,constraint, varargin{:});
      else
        obj = addConstraint@DirtranTrajectoryOptimization(obj,constraint,varargin{:});
      end
    end

    function [xtraj,z,F,info] = solveTraj(obj,t_init,q_traj_init)
      traj_init.x = [q_traj_init;q_traj_init.fnder()];      
      traj_init.u = q_traj_init.fnder().fnder();
      [xtraj,~,z,F,info] = solveTraj@DirectTrajectoryOptimization(obj,t_init,traj_init);
    end
  end
end
