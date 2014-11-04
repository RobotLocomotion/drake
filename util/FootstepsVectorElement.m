classdef FootstepsVectorElement < MixedIntegerElement
  properties 
    start_poses;
    nsteps;
    MAX_DISTANCE = 30;
  end

  methods
    function obj = FootstepsVectorElement(start_poses, nsteps)
      obj.start_poses = start_poses;
      obj.nsteps = nsteps;
    end

    function [constraints, objective] = getYalmipForm(obj)
      footsteps = sdpvar(4, obj.nsteps, 'full');
      constraints = [...
        obj.start_poses(1,1) - obj.MAX_DISTANCE <= footsteps(1,:) <= obj.start_poses(1,1) + obj.MAX_DISTANCE,...
        obj.start_poses(2,1) - obj.MAX_DISTANCE <= footsteps(2,:) <= obj.start_poses(2,1) + obj.MAX_DISTANCE,...
        obj.start_poses(3,1) - obj.MAX_DISTANCE <= footsteps(3,:) <= obj.start_poses(3,1) + obj.MAX_DISTANCE,...
        footsteps(:,1) == obj.start_poses(:,1),...
        footsteps(:,2) == obj.start_poses(:,2),...
        ];
        objective = 0;
    end

    function helper = addToHelper(obj, helper)
      lb = [repmat(obj.start_poses(1:3), 1, obj.nsteps) - obj.MAX_DISTANCE;
            -2 * pi + zeros(1, obj.nsteps)];
      ub = [repmat(obj.start_poses(1:3), 1, obj.nsteps) + obj.MAX_DISTANCE;
            2 * pi + zeros(1, obj.nsteps)];
      lb(:,1) = obj.start_poses(:,1);
      ub(:,1) = obj.start_poses(:,1);
      lb(:,2) = obj.start_poses(:,2);
      ub(:,2) = obj.start_poses(:,2);
      helper = helper.addVariable('footsteps', 'C', [4, obj.nsteps], lb, ub);
    end
  end
end


