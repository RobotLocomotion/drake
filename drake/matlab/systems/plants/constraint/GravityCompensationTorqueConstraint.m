classdef GravityCompensationTorqueConstraint < SingleTimeKinematicConstraint
  % Constraint on the torque required to keep specified joints static in the
  % face of gravity. This constraint considers gravity only - it does not
  % account for other external forces that might need to be resisted.
  properties (SetAccess = protected)
    joint_indices
    lb
    ub
  end
  methods
    function obj = GravityCompensationTorqueConstraint(robot, joint_indices, lb, ub, tspan)
      % obj = GravityCompensationTorqueConstraint(robot, joint_indices, lb, ub, tspan)
      % @param robot          -- RigidBodyManipulator object
      % @param joint_indices  -- column vector containing the indices of the
      %                          joints to which the constraint will be applied
      % @param lb             -- lower bound on the allowable torques. May be a
      %                          scalar, or a column vector with the same number
      %                          of elements as joint_indices
      % @param ub             -- Upper bound on the allowable torques. May be a
      %                          scalar, or a column vector with the same number
      %                          of elements as joint_indices
      % @param tspan

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
      obj.type = RigidBodyConstraint.GravityCompensationTorqueConstraintType;
      if robot.getMexModelPtr~=0 && exist('constructPtrRigidBodyConstraintmex','file')
        obj.mex_ptr = constructPtrRigidBodyConstraintmex(RigidBodyConstraint.GravityCompensationTorqueConstraintType,robot.getMexModelPtr, obj.joint_indices, obj.lb, obj.ub, tspan);
      end
    end

    function [lb, ub] = bounds(obj, t)
      if(obj.isTimeValid(t))
        lb = obj.lb;
        ub = obj.ub;
      else
        lb = [];
        ub = [];
      end
    end

    function obj = updateRobot(obj,robot)
      obj.robot = robot;
      obj.mex_ptr = updatePtrRigidBodyConstraintmex(obj.mex_ptr,'robot',robot.getMexModelPtr);
    end

    function name_str = name(obj, t)
      if(obj.isTimeValid(t))
        joint_names = obj.robot.getPositionFrame().getCoordinateNames();
        name_str = cellStrCat({'Gravity compensation torque constraint on  '}, ...
                               joint_names(obj.joint_indices), ...
                              {sprintf(' at time %10.4f',t)})';
      else
        name_str = {};
      end
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
