classdef StandingPlan < QPWalkingPlan
  methods(Static)
    function obj = from_standing_state(x0, biped, support_state)

      if nargin < 3
        support_state = RigidBodySupportState(biped, [biped.foot_body_id.right, biped.foot_body_id.left]);
      end

      obj = StandingPlan(biped);
      obj.x0 = x0;
      obj.support_times = [0, inf];
      obj.duration = inf;
      obj.supports = [support_state, support_state];

      nq = obj.robot.getNumPositions();
      q0 = x0(1:nq);
      kinsol = doKinematics(obj.robot, q0);

      foot_pos = [obj.robot.forwardKin(kinsol, obj.robot.foot_frame_id.right, [0;0;0]),...
                  obj.robot.forwardKin(kinsol, obj.robot.foot_frame_id.left, [0;0;0])];
      comgoal = mean(foot_pos(1:2,:), 2);

      obj.zmptraj = comgoal;
      [~, obj.V, obj.comtraj, limp_height] = obj.robot.planZMPController(comgoal, q0);
      obj.D_ls = -limp_height/9.81*eye(2);

      link_constraints(1).link_ndx = obj.robot.foot_body_id.right;
      link_constraints(1).pt = [0;0;0];
      link_constraints(1).ts = [0, inf];
      link_constraints(1).coefs = cat(3, zeros(6,1,3), reshape(forwardKin(obj.robot,kinsol,obj.robot.foot_body_id.right,[0;0;0],1),[6,1,1]));
      link_constraints(2).link_ndx = obj.robot.foot_body_id.left;
      link_constraints(2).pt = [0;0;0];
      link_constraints(2).ts = [0, inf];
      link_constraints(2).coefs = cat(3, zeros(6,1,3),reshape(forwardKin(obj.robot,kinsol,obj.robot.foot_body_id.left,[0;0;0],1),[6,1,1]));
      pelvis_id = obj.robot.findLinkId('pelvis');
      link_constraints(3).link_ndx = pelvis_id;
      link_constraints(3).pt = [0;0;0];
      link_constraints(3).ts = [0, inf];
      pelvis_current = forwardKin(obj.robot,kinsol,pelvis_id,[0;0;0],1);
      pelvis_target = [mean(foot_pos(1:2,:), 2); pelvis_current(3:6)];
      link_constraints(3).coefs = cat(3, zeros(6,1,3),reshape(pelvis_target,[6,1,1,]));
      obj.link_constraints = link_constraints;

      obj.zmp_final = comgoal;
      obj.qstar = x0(1:nq);
      obj.comtraj = comgoal;
      obj.gain_set = 'standing';
    end
  end

  methods
    function obj = StandingPlan(robot)
      obj = obj@QPWalkingPlan(robot);
    end
  end
end
