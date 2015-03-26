classdef GravityCompensationTorqueConstraint < SingleTimeKinematicConstraint
  properties (SetAccess = protected)
    joint_indices
    lb
    ub
  end
  methods
    function obj = GravityCompensationTorqueConstraint(robot, joint_indices, lb, ub, tspan)
      if nargin < 5, tspan = [-Inf, Inf]; end
      obj = obj@SingleTimeKinematicConstraint(robot, tspan);
      obj.joint_indices = reshape(joint_indices, [], 1);
      if isscalar(lb)
        lb = repmat(lb, size(obj.joint_indices));
      else
        assert(numel(lb) == numel(obj.joint_indices), ...
             ['lb must be either a scalar, or a vector with the same number '...
              'of elements as joint_indices']);
      end
      if isscalar(ub)
        ub = repmat(ub, size(obj.joint_indices));
      else
        assert(numel(ub) == numel(obj.joint_indices), ...
             ['ub must be either a scalar, or a vector with the same number '...
              'of elements as joint_indices']);
      end
      obj.lb = lb;
      obj.ub = ub;
      obj.num_constraint = numel(obj.joint_indices);
      if robot.getMexModelPtr~=0 && exist('constructPtrRigidBodyConstraintmex','file')
        obj.mex_ptr = constructPtrRigidBodyConstraintmex(RigidBodyConstraint.GravityCompensationTorqueConstraintType,robot.getMexModelPtr, obj.joint_indices, obj.lb, obj.ub, tspan);
      end
    end

    function [lb, ub] = bounds(obj, ~)
      lb = obj.lb;
      ub = obj.ub;
    end

    function obj = updateRobot(obj,robot)
      obj.robot = robot;
      obj.mex_ptr = updatePtrRigidBodyConstraintmex(obj.mex_ptr,'robot',robot.getMexModelPtr);
    end

    function name_str = name(obj, t)
      joint_names = obj.robot.getPositionFrame().coordinates;
      name_str = cellStrCat({'Gravity compensation torque constraint on  '}, ...
                             joint_names(obj.joint_indices), ...
                            {sprintf(' at time %10.4f',t)})';
    end
  end
  methods (Access = protected)
    function [c, dc] = evalValidTime(obj, kinsol)
      [~, G, ~, ~, dG] = obj.robot.manipulatorDynamics(kinsol.q, zeros(obj.robot.getNumVelocities,1));
      c = G(obj.joint_indices);
      dc = dG(obj.joint_indices, 1:obj.robot.getNumPositions());
    end
  end
end
